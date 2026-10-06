#include "db_history.h"
#include "trip_data_fields.h"
#include "engine_power.h"
#include "db_query.h"
#include "logger.h"

#include "sqlite3.h"

#include <QHash>
#include <algorithm>
#include <functional>

namespace {

// Shared prepare/bind(trip)/step/finalize skeleton for queryLiftoffs() and
// queryTouchdowns() below, which differ only in table/column layout
// (touchdowns append a g_force column after the shared ones, read at
// CONTACT_POINT_COLUMN_COUNT)
// and the target struct type. extractRow runs once per SQLITE_ROW to build
// one T from the current row. callerName/itemsWord name the caller and its
// rows in the log lines (e.g. "queryLiftoffs(trip %d): loading liftoffs" /
// "... loaded %d liftoffs").
template <typename T>
std::vector<T> queryContactPoints(sqlite3* sql, int tripId, const char* stmtText,
	const char* callerName, const char* itemsWord,
	const std::function<T(sqlite3_stmt*)>& extractRow) {
	std::vector<T> items;
	Logger::logf(Logger::Trace, "DB", "%s(trip %d): loading %s", callerName, tripId, itemsWord);

	const QString context = QStringLiteral("%1(trip %2)").arg(QLatin1String(callerName)).arg(tripId);
	sqlite3_stmt* stmt = prepareStatement(sql, stmtText, context);
	if (!stmt)
		return items;
	sqlite3_bind_int(stmt, 1, tripId);
	forEachRow(sql, stmt, context, [&](sqlite3_stmt* row) {
		items.push_back(extractRow(row));
		return true;
	});
	Logger::logf(Logger::Trace, "DB", "%s(trip %d): loaded %d %s", callerName, tripId, (int)items.size(), itemsWord);
	return items;
}

}

std::vector<TripSummary> queryAllTrips(sqlite3* sql, int liveTripId) {
	std::vector<TripSummary> trips;
	Logger::log(Logger::Trace, "DB", QStringLiteral("queryAllTrips: loading trip list"));

	// Trip History opens only after migrate_db() succeeded, so every column
	// exists.
	const QString context = QStringLiteral("queryAllTrips");
	sqlite3_stmt* stmt = prepareStatement(sql,
		"SELECT t.id, t.title, t.atc_airline, t.atc_flight_number, t.departure_icao, t.departure_name, t.departure_region, t.departure_rwy, "
		"t.destination_icao, t.destination_name, t.destination_region, t.destination_rwy, t.departure_zulu_time, t.destination_zulu_time, "
		"t.departure_latitude, t.departure_longitude, t.destination_latitude, t.destination_longitude, "
		"t.group_id, g.name "
		"FROM trips t LEFT JOIN trip_groups g ON g.id = t.group_id ORDER BY t.id DESC", context);
	if (!stmt)
		return trips;

	forEachRow(sql, stmt, context, [&](sqlite3_stmt* row) {
		TripSummary trip;
		trip.id = sqlite3_column_int(row, 0);
		trip.title = columnText(row, 1);
		trip.atcAirline = columnText(row, 2);
		trip.atcFlightNumber = columnText(row, 3);
		trip.departureIcao = columnText(row, 4);
		trip.departureName = columnText(row, 5);
		trip.departureRegion = columnText(row, 6);
		trip.departureRwy = columnText(row, 7);
		trip.destinationIcao = columnText(row, 8);
		trip.destinationName = columnText(row, 9);
		trip.destinationRegion = columnText(row, 10);
		trip.destinationRwy = columnText(row, 11);
		trip.departureZuluTime = columnText(row, 12);
		trip.destinationZuluTime = columnText(row, 13);
		trip.departureLat    = sqlite3_column_double(row, 14);
		trip.departureLng    = sqlite3_column_double(row, 15);
		trip.destinationLat  = sqlite3_column_double(row, 16);
		trip.destinationLng  = sqlite3_column_double(row, 17);
		trip.groupId         = sqlite3_column_int(row, 18); // NULL reads as 0 (ungrouped)
		trip.groupName       = columnText(row, 19);

		if (trip.id == liveTripId)
			trip.status = TripStatus::Live;
		else if (trip.destinationZuluTime.isEmpty())
			trip.status = TripStatus::Open;
		else
			trip.status = TripStatus::Completed;

		trips.push_back(trip);
		return true;
	});
	Logger::logf(Logger::Trace, "DB", "queryAllTrips: loaded %d trips", (int)trips.size());
	return trips;
}

TripDataset queryTripData(sqlite3* sql, int tripId) {
	TripDataset dataset;
	dataset.tripId = tripId;
	Logger::logf(Logger::Trace, "DB", "queryTripData(trip %d): loading sample points", tripId);

	// SELECT * (rather than a curated column list) plus a name -> index map
	// built from the statement's own column metadata so the query doesn't need
	// to enumerate over a hundred columns by hand or care about their exact ordinal
	// positions in the table.
	const QString context = QStringLiteral("queryTripData(trip %1)").arg(tripId);
	sqlite3_stmt* stmt = prepareStatement(sql, "SELECT * FROM trip_data WHERE trip = ? ORDER BY rowid", context);
	if (!stmt)
		return dataset;
	sqlite3_bind_int(stmt, 1, tripId);

	QHash<QString, int> columnIndex;
	int columnCount = sqlite3_column_count(stmt);
	for (int i = 0; i < columnCount; ++i)
		columnIndex.insert(QString::fromUtf8(sqlite3_column_name(stmt, i)), i);
	auto indexOf = [&](const char* name) {
		return columnIndex.value(QString::fromLatin1(name), -1);
	};

	// A column's position is the same on every row of this prepared statement
	// -- resolving names to indices is one-time per-query setup, not per-row
	// work. Doing it here instead of inside the row loop below turns a QHash
	// lookup per field per row into a direct column read.
	const int idxBoolGroup1 = indexOf("bool_group_1");
	const int idxBoolGroup2 = indexOf("bool_group_2");
	const int idxBoolGroup3 = indexOf("bool_group_3");
	const int idxGearHandlePosition = indexOf("gear_handle_position");
	const int idxGearPosition0 = indexOf("gear_position_0");
	const int idxGearPosition1 = indexOf("gear_position_1");
	const int idxGearPosition2 = indexOf("gear_position_2");
	const int idxLatitude = indexOf("plane_latitude");
	const int idxLongitude = indexOf("plane_longitude");
	const int idxAltitude = indexOf("plane_altitude");
	const int idxGroundSpeed = indexOf("ground_velocity");
	const int idxAirspeed = indexOf("airspeed_indicated");
	const int idxVerticalSpeed = indexOf("vertical_speed");
	const int idxEngineType = indexOf("engine_type");
	const int idxEngineSpeed = indexOf("engine_speed");
	const int idxEngineLoad = indexOf("engine_load");
	const int idxBrakeIndicator = indexOf("brake_indicator");
	const int idxFlapsHandleIndex = indexOf("flaps_handle_index");
	const int idxSpoilersHandlePosition = indexOf("spoilers_handle_position");
	const int idxFuelTotalQuantityWeight = indexOf("fuel_total_quantity_weight");
	const int idxPitch = indexOf("plane_pitch_degrees");
	const int idxBank  = indexOf("plane_bank_degrees");
	const int idxZuluTime = indexOf("zulu_time");
	const int idxLocalTime = indexOf("local_time");

	std::vector<int> numFieldIndices;
	numFieldIndices.reserve(128);
#define TRIP_NUM_IDX(dbColumn, memberExpr, sqlType) \
	numFieldIndices.push_back(indexOf(#dbColumn));
	TRIP_DATA_NUM_FIELDS(TRIP_NUM_IDX)
#undef TRIP_NUM_IDX

	auto colDouble = [&](int index) -> double { return index >= 0 ? sqlite3_column_double(stmt, index) : 0.0; };
	auto colInt = [&](int index) -> int { return index >= 0 ? sqlite3_column_int(stmt, index) : 0; };
	auto colText = [&](int index) -> QString { return index >= 0 ? columnText(stmt, index) : QString(); };
	// Unpacks an engine_speed/engine_load BLOB into out; its engine count (0 for NULL).
	auto colEngineValues = [&](int index, std::array<float, MAX_ENGINES>& out) -> int {
		if (index < 0)
			return 0;
		const void* blob = sqlite3_column_blob(stmt, index); // before _bytes(), as SQLite documents
		return unpackEngineValues(blob, sqlite3_column_bytes(stmt, index), out);
	};

	forEachRow(sql, stmt, context, [&](sqlite3_stmt*) {
		TripSamplePoint point;
		point.boolGroups = { 0, (uint32_t)colInt(idxBoolGroup1),
			(uint32_t)colInt(idxBoolGroup2), (uint32_t)colInt(idxBoolGroup3) };
		point.gearHandlePosition = colDouble(idxGearHandlePosition);
		point.gearPosition[0] = colInt(idxGearPosition0);
		point.gearPosition[1] = colInt(idxGearPosition1);
		point.gearPosition[2] = colInt(idxGearPosition2);
		point.gearOnGround[0] = TripBoolBits::gear_is_on_ground_0.isSet(point.boolGroups);
		point.gearOnGround[1] = TripBoolBits::gear_is_on_ground_1.isSet(point.boolGroups);
		point.gearOnGround[2] = TripBoolBits::gear_is_on_ground_2.isSet(point.boolGroups);
		point.latitude = colDouble(idxLatitude);
		point.longitude = colDouble(idxLongitude);
		point.altitude = colInt(idxAltitude);
		point.groundSpeed = colInt(idxGroundSpeed);
		point.airspeed = colInt(idxAirspeed);
		point.verticalSpeed = colInt(idxVerticalSpeed);
		point.engine.engineType = colInt(idxEngineType);
		point.engine.count = qMin(colEngineValues(idxEngineSpeed, point.engine.speed),
			colEngineValues(idxEngineLoad, point.engine.load));
		point.brakeIndicator = colInt(idxBrakeIndicator);
		point.flapsHandleIndex = colDouble(idxFlapsHandleIndex);
		point.spoilersHandlePosition = colDouble(idxSpoilersHandlePosition);
		point.fuelTotalQuantityWeight = colDouble(idxFuelTotalQuantityWeight);
		point.pitchDegrees = colDouble(idxPitch);
		point.bankDegrees  = colDouble(idxBank);
		point.zuluTime = colText(idxZuluTime);
		point.localTime = colText(idxLocalTime);

		point.rawNums.reserve(numFieldIndices.size());
		for (int idx : numFieldIndices)
			point.rawNums.push_back(colDouble(idx));

		dataset.points.push_back(point);
		return true;
	});
	Logger::logf(Logger::Trace, "DB", "queryTripData(trip %d): loaded %d points", tripId, (int)dataset.points.size());

	return dataset;
}

// The trip_liftoffs/trip_touchdowns columns readContactPoint() reads, in
// this order; a touchdown query appends g_force after them.
#define CONTACT_POINT_COLUMNS \
	"id, plane_latitude, plane_longitude, icao, airport_name, runway, runway_heading, airspeed_indicated, " \
	"vertical_speed, plane_pitch_degrees, plane_bank_degrees, heading_indicator, " \
	"distance_length, distance_width, distance_length_percent, distance_width_percent, " \
	"wind_direction, wind_velocity, time_zulu, time_local, analysis_report"

namespace {

// Number of comma-separated names in a column list.
constexpr int columnCount(const char* columns) {
	int count = 1;
	for (; *columns; ++columns)
		if (*columns == ',')
			++count;
	return count;
}

constexpr int CONTACT_POINT_COLUMN_COUNT = columnCount(CONTACT_POINT_COLUMNS);

void readContactPoint(sqlite3_stmt* stmt, RunwayContactPoint& point) {
	point.rowId                 = sqlite3_column_int(stmt, 0);
	point.latitude              = sqlite3_column_double(stmt, 1);
	point.longitude             = sqlite3_column_double(stmt, 2);
	point.icao                  = columnText(stmt, 3);
	point.airportName           = columnText(stmt, 4);
	point.runway                = columnText(stmt, 5);
	point.runwayHeading         = sqlite3_column_type(stmt, 6) == SQLITE_NULL ? -1 : sqlite3_column_int(stmt, 6);
	point.airspeed              = sqlite3_column_int(stmt, 7);
	point.verticalSpeed         = sqlite3_column_int(stmt, 8);
	point.pitchDegrees          = sqlite3_column_double(stmt, 9);
	point.bankDegrees           = sqlite3_column_double(stmt, 10);
	point.headingDegrees        = sqlite3_column_int(stmt, 11);
	point.distanceLength        = sqlite3_column_double(stmt, 12);
	point.distanceWidth         = sqlite3_column_double(stmt, 13);
	point.distanceLengthPercent = sqlite3_column_double(stmt, 14);
	point.distanceWidthPercent  = sqlite3_column_double(stmt, 15);
	point.windDirection         = sqlite3_column_int(stmt, 16);
	point.windVelocity          = sqlite3_column_int(stmt, 17);
	point.zuluTime              = columnText(stmt, 18);
	point.localTime             = columnText(stmt, 19);
	point.analysisReport        = columnText(stmt, 20);
}

}

std::vector<LiftoffPoint> queryLiftoffs(sqlite3* sql, int tripId) {
	// migrate_db() at app startup ensures all columns exist before any query runs.
	const char* stmt_txt =
		"SELECT " CONTACT_POINT_COLUMNS " FROM trip_liftoffs WHERE trip = ? ORDER BY id";
	return queryContactPoints<LiftoffPoint>(sql, tripId, stmt_txt, "queryLiftoffs", "liftoffs",
		[](sqlite3_stmt* stmt) {
			LiftoffPoint point;
			readContactPoint(stmt, point);
			return point;
		});
}

std::vector<TouchdownPoint> queryTouchdowns(sqlite3* sql, int tripId) {
	// migrate_db() at app startup ensures all columns exist before any query runs.
	const char* stmt_txt =
		"SELECT " CONTACT_POINT_COLUMNS ", g_force FROM trip_touchdowns WHERE trip = ? ORDER BY id";
	return queryContactPoints<TouchdownPoint>(sql, tripId, stmt_txt, "queryTouchdowns", "touchdowns",
		[](sqlite3_stmt* stmt) {
			TouchdownPoint point;
			readContactPoint(stmt, point);
			point.gForce = sqlite3_column_double(stmt, CONTACT_POINT_COLUMN_COUNT);
			return point;
		});
}

void resolveEventPositions(TripDataset& dataset) {
	for (TripEvent& event : dataset.events) {
		auto it = std::lower_bound(dataset.points.begin(), dataset.points.end(), event.zuluTime,
			[](const TripSamplePoint& point, const QString& time) { return point.zuluTime < time; });
		if (it == dataset.points.end() && !dataset.points.empty())
			--it;
		if (it != dataset.points.end()) {
			event.latitude = it->latitude;
			event.longitude = it->longitude;
			event.sampleIndex = (int)(it - dataset.points.begin());
		}
	}
}

TripDataset tripSamples(sqlite3* sql, int tripId, const QString& aircraftTitle, const QString& departureZuluTime) {
	TripDataset dataset = sql ? queryTripData(sql, tripId) : TripDataset();
	dataset.tripId = tripId;
	dataset.aircraftTitle = aircraftTitle;
	dataset.departureZuluTime = departureZuluTime;
	return dataset;
}

void completeTripDataset(TripDataset& dataset, std::vector<LiftoffPoint> liftoffPoints,
	std::vector<TouchdownPoint> touchdowns, std::vector<TripEvent> events) {
	dataset.liftoffPoints = std::move(liftoffPoints);
	dataset.touchdowns = std::move(touchdowns);
	dataset.events = std::move(events);
	resolveEventPositions(dataset);
}

bool deleteTripData(sqlite3* sql, int tripId) {
	// Child tables first (trip_data is largest), then the trip row itself.
	const char* stmts[] = {
		"DELETE FROM trip_data WHERE trip = ?",
		"DELETE FROM trip_events WHERE trip = ?",
		"DELETE FROM trip_liftoffs WHERE trip = ?",
		"DELETE FROM trip_touchdowns WHERE trip = ?",
		"DELETE FROM trips WHERE id = ?",
	};
	const QString context = QStringLiteral("deleteTripData(trip %1)").arg(tripId);
	Logger::logf(Logger::Trace, "DB", "deleteTripData(trip %d): starting delete", tripId);
	const bool ok = inTransaction(sql, context, [&]() {
		for (const char* stmt_txt : stmts)
			if (!execStatement(sql, stmt_txt, context, [&](sqlite3_stmt* stmt) { sqlite3_bind_int(stmt, 1, tripId); }))
				return false;
		return true;
	});
	if (ok)
		Logger::logf(Logger::Trace, "DB", "deleteTripData(trip %d): delete committed", tripId);
	return ok;
}

std::vector<TripEvent> queryEvents(sqlite3* sql, int tripId) {
	std::vector<TripEvent> events;
	Logger::logf(Logger::Trace, "DB", "queryEvents(trip %d): loading events", tripId);

	// BRAKES fires continuously while brakes are applied (taxi/landing roll) and
	// would swamp the map with noise -- excluded here rather than at recording
	// time so the raw data stays in the DB if ever needed.
	// PARKING_BRAKES fires exactly once on set and once on release, so it is
	// kept; the spatial grouping in map.html coalesces the two events into one
	// marker since the aircraft is stationary between set and release.
	const QString context = QStringLiteral("queryEvents(trip %1)").arg(tripId);
	sqlite3_stmt* stmt = prepareStatement(sql,
		"SELECT event, time_zulu FROM trip_events WHERE trip = ? AND event != 'BRAKES' ORDER BY rowid", context);
	if (!stmt)
		return events;
	sqlite3_bind_int(stmt, 1, tripId);
	forEachRow(sql, stmt, context, [&](sqlite3_stmt* row) {
		TripEvent event;
		event.event = columnText(row, 0);
		event.zuluTime = columnText(row, 1);
		events.push_back(event);
		return true;
	});
	Logger::logf(Logger::Trace, "DB", "queryEvents(trip %d): loaded %d events", tripId, (int)events.size());
	return events;
}

bool saveAnalysisReport(sqlite3* sql, CONTACT_TABLE table, int rowId, const QString& report) {
	const char* what = contactLabel(table);
	if (rowId <= 0) {
		Logger::logf(Logger::Trace, "DB", "saveAnalysisReport: ignoring invalid %s id %d", what, rowId);
		return false;
	}
	const std::string stmt_txt = std::string("UPDATE ") + contactTableName(table) + " SET analysis_report = ? WHERE id = ?";
	const bool ok = execStatement(sql, stmt_txt.c_str(), QStringLiteral("saveAnalysisReport(%1 %2)").arg(QLatin1String(what)).arg(rowId),
		[&](sqlite3_stmt* stmt) {
			bindText(stmt, 1, report);
			sqlite3_bind_int(stmt, 2, rowId);
		});
	if (ok)
		Logger::logf(Logger::Trace, "DB", "saveAnalysisReport(%s %d): analysis report saved (%d bytes)", what, rowId, (int)report.toUtf8().size());
	return ok;
}
