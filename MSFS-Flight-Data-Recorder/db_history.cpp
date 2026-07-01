#include "db_history.h"
#include "trip_data_fields.h"

#include "sqlite3.h"

#include <QHash>

namespace {

QString columnTextOrEmpty(sqlite3_stmt* stmt, int column) {
	const unsigned char* text = sqlite3_column_text(stmt, column);
	return text ? QString::fromUtf8(reinterpret_cast<const char*>(text)) : QString();
}

}

std::vector<TripSummary> queryAllTrips(sqlite3* sql, int liveTripId) {
	std::vector<TripSummary> trips;

	// departure_region/destination_region are a recent addition to trips --
	// only the recording feature's write connection ever changes the schema
	// (and only for new databases via CREATE TABLE), so a database from
	// before they existed legitimately lacks these columns. Try the query
	// with them first and fall back to a region-less query (leaving those
	// fields blank) rather than touching the schema from this read-only path.
	const char* stmt_txt_with_region =
		"SELECT id, title, atc_airline, atc_flight_number, departure_icao, departure_region, departure_rwy, "
		"destination_icao, destination_region, destination_rwy, departure_zulu_time, destination_zulu_time "
		"FROM trips ORDER BY id DESC";
	const char* stmt_txt_no_region =
		"SELECT id, title, atc_airline, atc_flight_number, departure_icao, departure_rwy, "
		"destination_icao, destination_rwy, departure_zulu_time, destination_zulu_time "
		"FROM trips ORDER BY id DESC";

	sqlite3_stmt* stmt = nullptr;
	bool hasRegion = sqlite3_prepare_v2(sql, stmt_txt_with_region, -1, &stmt, nullptr) == SQLITE_OK;
	if (!hasRegion) {
		if (sqlite3_prepare_v2(sql, stmt_txt_no_region, -1, &stmt, nullptr) != SQLITE_OK)
			return trips;
	}

	while (sqlite3_step(stmt) == SQLITE_ROW) {
		TripSummary trip;
		trip.id = sqlite3_column_int(stmt, 0);
		trip.title = columnTextOrEmpty(stmt, 1);
		trip.atcAirline = columnTextOrEmpty(stmt, 2);
		trip.atcFlightNumber = columnTextOrEmpty(stmt, 3);
		trip.departureIcao = columnTextOrEmpty(stmt, 4);
		if (hasRegion) {
			trip.departureRegion = columnTextOrEmpty(stmt, 5);
			trip.departureRwy = columnTextOrEmpty(stmt, 6);
			trip.destinationIcao = columnTextOrEmpty(stmt, 7);
			trip.destinationRegion = columnTextOrEmpty(stmt, 8);
			trip.destinationRwy = columnTextOrEmpty(stmt, 9);
			trip.departureZuluTime = columnTextOrEmpty(stmt, 10);
			trip.destinationZuluTime = columnTextOrEmpty(stmt, 11);
		} else {
			trip.departureRwy = columnTextOrEmpty(stmt, 5);
			trip.destinationIcao = columnTextOrEmpty(stmt, 6);
			trip.destinationRwy = columnTextOrEmpty(stmt, 7);
			trip.departureZuluTime = columnTextOrEmpty(stmt, 8);
			trip.destinationZuluTime = columnTextOrEmpty(stmt, 9);
		}

		if (trip.id == liveTripId)
			trip.status = TripStatus::Live;
		else if (trip.destinationZuluTime.isEmpty())
			trip.status = TripStatus::Open;
		else
			trip.status = TripStatus::Completed;

		trips.push_back(trip);
	}
	sqlite3_finalize(stmt);
	return trips;
}

TripDataset queryTripData(sqlite3* sql, int tripId) {
	TripDataset dataset;
	dataset.tripId = tripId;

	// SELECT * (rather than a curated column list) plus a name -> index map
	// built from the statement's own column metadata so DataTablePanel can
	// show every trip_data field (TripSamplePoint::allFields, populated below
	// via the shared X-macro list in trip_data_fields.h) without this query
	// needing to enumerate ~140 columns by hand or care about their exact
	// ordinal position in the table.
	const char* stmt_txt = "SELECT * FROM trip_data WHERE trip = ? ORDER BY rowid";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK)
		return dataset;
	sqlite3_bind_int(stmt, 1, tripId);

	QHash<QString, int> columnIndex;
	int columnCount = sqlite3_column_count(stmt);
	for (int i = 0; i < columnCount; ++i)
		columnIndex.insert(QString::fromUtf8(sqlite3_column_name(stmt, i)), i);
	auto indexOf = [&](const char* name) {
		return columnIndex.value(QString::fromLatin1(name), -1);
	};

	// A column's position (and a field's label text) is the same on every row
	// of this prepared statement -- resolving names to indices and building
	// label strings is one-time per-query setup, not per-row work. Doing it
	// here instead of inside the row loop below turns ~236 QHash lookups +
	// label rebuilds per row into a handful of direct array reads (and QString
	// refcount bumps, via Qt's copy-on-write) per row.
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
	const int idxN1_1 = indexOf("turb_eng_n1_1");
	const int idxN1_2 = indexOf("turb_eng_n1_2");
	const int idxN2_1 = indexOf("turb_eng_n2_1");
	const int idxN2_2 = indexOf("turb_eng_n2_2");
	const int idxBrakeIndicator = indexOf("brake_indicator");
	const int idxFlapsHandleIndex = indexOf("flaps_handle_index");
	const int idxSpoilersHandlePosition = indexOf("spoilers_handle_position");
	const int idxFuelTotalQuantityWeight = indexOf("fuel_total_quantity_weight");
	const int idxZuluTime = indexOf("zulu_time");
	const int idxLocalTime = indexOf("local_time");

	struct NumFieldMeta { QString label; int index; };
	std::vector<NumFieldMeta> numFields;
#define TRIP_NUM_META(dbColumn, memberExpr) \
	numFields.push_back({ tripFieldLabel(#dbColumn), indexOf(#dbColumn) });
	TRIP_DATA_NUM_FIELDS(TRIP_NUM_META)
#undef TRIP_NUM_META

	struct BoolFieldMeta { QString label; int group; int bit; };
	std::vector<BoolFieldMeta> boolFields;
#define TRIP_BOOL_META(name, group, bit) \
	boolFields.push_back({ tripFieldLabel(#name), group, bit });
	TRIP_DATA_BOOL_FIELDS(TRIP_BOOL_META)
#undef TRIP_BOOL_META

	auto colDouble = [&](int index) -> double { return index >= 0 ? sqlite3_column_double(stmt, index) : 0.0; };
	auto colInt = [&](int index) -> int { return index >= 0 ? sqlite3_column_int(stmt, index) : 0; };
	auto colText = [&](int index) -> QString { return index >= 0 ? columnTextOrEmpty(stmt, index) : QString(); };

	while (sqlite3_step(stmt) == SQLITE_ROW) {
		TripSamplePoint point;
		int boolGroup1 = colInt(idxBoolGroup1);
		int boolGroup2 = colInt(idxBoolGroup2);
		int boolGroup3 = colInt(idxBoolGroup3);
		point.gearHandlePosition = colDouble(idxGearHandlePosition);
		point.gearPosition[0] = colInt(idxGearPosition0);
		point.gearPosition[1] = colInt(idxGearPosition1);
		point.gearPosition[2] = colInt(idxGearPosition2);
		point.gearOnGround[0] = (boolGroup1 & (0x1 << 26)) != 0;
		point.gearOnGround[1] = (boolGroup1 & (0x1 << 27)) != 0;
		point.gearOnGround[2] = (boolGroup1 & (0x1 << 28)) != 0;
		point.latitude = colDouble(idxLatitude);
		point.longitude = colDouble(idxLongitude);
		point.altitude = colInt(idxAltitude);
		point.groundSpeed = colInt(idxGroundSpeed);
		point.airspeed = colInt(idxAirspeed);
		point.verticalSpeed = colInt(idxVerticalSpeed);
		point.n1_1 = colDouble(idxN1_1);
		point.n1_2 = colDouble(idxN1_2);
		point.n2_1 = colDouble(idxN2_1);
		point.n2_2 = colDouble(idxN2_2);
		point.brakeIndicator = colInt(idxBrakeIndicator);
		point.flapsHandleIndex = colDouble(idxFlapsHandleIndex);
		point.spoilersHandlePosition = colDouble(idxSpoilersHandlePosition);
		point.fuelTotalQuantityWeight = colDouble(idxFuelTotalQuantityWeight);
		point.zuluTime = colText(idxZuluTime);
		point.localTime = colText(idxLocalTime);

		point.allFields.reserve(numFields.size() + boolFields.size());
		for (const NumFieldMeta& f : numFields)
			point.allFields.push_back({ f.label, QString::number(colDouble(f.index), 'g', 6) });
		for (const BoolFieldMeta& f : boolFields) {
			int group = f.group == 1 ? boolGroup1 : f.group == 2 ? boolGroup2 : boolGroup3;
			point.allFields.push_back({ f.label, (group & (0x1 << f.bit)) != 0 ? QStringLiteral("Yes") : QStringLiteral("No") });
		}

		dataset.points.push_back(point);
	}
	sqlite3_finalize(stmt);
	return dataset;
}

std::vector<TouchdownPoint> queryTouchdowns(sqlite3* sql, int tripId) {
	std::vector<TouchdownPoint> touchdowns;

	const char* stmt_txt =
		"SELECT plane_latitude, plane_longitude, icao, runway, airspeed_indicated, "
		"vertical_speed, g_force, plane_pitch_degrees, plane_bank_degrees, heading_indicator, time_zulu "
		"FROM trip_touchdowns WHERE trip = ? ORDER BY id";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK)
		return touchdowns;
	sqlite3_bind_int(stmt, 1, tripId);

	while (sqlite3_step(stmt) == SQLITE_ROW) {
		TouchdownPoint point;
		point.latitude = sqlite3_column_double(stmt, 0);
		point.longitude = sqlite3_column_double(stmt, 1);
		point.icao = columnTextOrEmpty(stmt, 2);
		point.runway = columnTextOrEmpty(stmt, 3);
		point.airspeed = sqlite3_column_int(stmt, 4);
		point.verticalSpeed = sqlite3_column_int(stmt, 5);
		point.gForce = sqlite3_column_double(stmt, 6);
		point.pitchDegrees = sqlite3_column_double(stmt, 7);
		point.bankDegrees = sqlite3_column_double(stmt, 8);
		point.headingDegrees = sqlite3_column_int(stmt, 9);
		point.zuluTime = columnTextOrEmpty(stmt, 10);
		touchdowns.push_back(point);
	}
	sqlite3_finalize(stmt);
	return touchdowns;
}

bool deleteTripData(sqlite3* sql, int tripId) {
	// Child tables first (trip_data is largest), then the trip row itself.
	const char* stmts[] = {
		"DELETE FROM trip_data WHERE trip = ?",
		"DELETE FROM trip_events WHERE trip = ?",
		"DELETE FROM trip_touchdowns WHERE trip = ?",
		"DELETE FROM trips WHERE id = ?",
	};
	for (const char* stmt_txt : stmts) {
		sqlite3_stmt* stmt = nullptr;
		if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK) {
			if (stmt) sqlite3_finalize(stmt);
			return false;
		}
		sqlite3_bind_int(stmt, 1, tripId);
		sqlite3_step(stmt);
		sqlite3_finalize(stmt);
	}
	return true;
}

std::vector<TripEvent> queryEvents(sqlite3* sql, int tripId) {
	std::vector<TripEvent> events;

	// Brake/autobrake events (BRAKES, PARKING_BRAKES, AUTOBRAKE_*) fire
	// constantly during taxi/landing roll and swamp the event list with noise
	// that isn't useful on the trajectory map -- excluded by name rather than
	// at recording time so the raw data stays in the DB if ever needed.
	const char* stmt_txt = "SELECT event, time_zulu FROM trip_events WHERE trip = ? AND event NOT LIKE '%BRAKE%' ORDER BY rowid";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK)
		return events;
	sqlite3_bind_int(stmt, 1, tripId);

	while (sqlite3_step(stmt) == SQLITE_ROW) {
		TripEvent event;
		event.event = columnTextOrEmpty(stmt, 0);
		event.zuluTime = columnTextOrEmpty(stmt, 1);
		events.push_back(event);
	}
	sqlite3_finalize(stmt);
	return events;
}
