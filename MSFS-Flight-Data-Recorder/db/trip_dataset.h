#pragma once

#include <QDateTime>
#include <QString>
#include <QtGlobal>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "trip_data_fields.h"

// One row of trip_data, with its engines' trip_engine_data rows, as
// db_history::queryTripData() reads it, decoded into engineering
// units/booleans.
struct TripSamplePoint {
	double latitude = 0;
	double longitude = 0;
	int altitude = 0;
	int airspeed = 0;
	int groundSpeed = 0;
	int verticalSpeed = 0;
	int engineType = -1;  // ENGINE TYPE (enginePowerSpec()); -1 if not recorded
	double gearHandlePosition = 0;
	int gearPosition[3] = { 0, 0, 0 };
	bool gearOnGround[3] = { false, false, false };
	int brakeIndicator = 0;
	double flapsHandleIndex = 0;
	double spoilersHandlePosition = 0;
	double fuelTotalQuantityWeight = 0;
	double pitchDegrees = 0;
	double bankDegrees = 0;
	QString zuluTime;
	QString localTime;

	// Raw values for all TRIP_DATA_NUM_FIELDS entries, in macro-list order
	// (TripNumField); NaN for a NULL (a column added after the row was written).
	// Raw bool groups (bit-packed ints) for all TRIP_DATA_BOOL_FIELDS entries,
	// indexed by group number as tripBoolGroups() returns them ([0] unused).
	// DataTablePanel formats these to strings on demand for the one point it
	// is currently displaying, avoiding ~7M QString constructions per load.
	std::vector<double> rawNums;
	std::array<uint32_t, 4> boolGroups{};
	// The sample's trip_engine_data values: TRIP_ENGINE_FIELD_COUNT per
	// engine, engine 1 first, through its highest recorded engine. NaN where
	// nothing was recorded (a NULL, or an engine without a row).
	std::vector<double> engineValues;

	// field's value from rawNums; NaN if not recorded.
	double rawNum(TripNumField field) const {
		return field < (int)rawNums.size() ? rawNums[field] : std::nan("");
	}
	// Engines 1..engineCount() have room in engineValues.
	int engineCount() const { return (int)(engineValues.size() / TRIP_ENGINE_FIELD_COUNT); }
	// field of engine (1-based); NaN if not recorded.
	double engineValue(int engine, TripEngineField field) const {
		return engine >= 1 && engine <= engineCount()
			? engineValues[(size_t)(engine - 1) * TRIP_ENGINE_FIELD_COUNT + field] : std::nan("");
	}
};

// A liftoff's/touchdown's conditions, from the trip's samples at that moment
// (analyzeContacts() in contact_analysis.h; derived when the trip loads,
// never stored). Every value is NaN (a name empty) when its sample didn't
// record it, or when no sample was found (fromSample false).
struct ContactConditions {
	bool fromSample = false;
	double headingTrue = std::nan("");      // degrees
	double headingMagnetic = std::nan("");  // degrees
	double trackTrue = std::nan("");        // ground track, degrees
	double magneticVariation = std::nan(""); // headingTrue - headingMagnetic, -180..180 (+ = east)
	double crabAngle = std::nan("");        // headingTrue - trackTrue, -180..180 (+ = nose right of the track)
	double sunElevation = std::nan("");     // degrees above the horizon (sunElevationDegrees())
	double temperatureC = std::nan("");     // outside air
	double stationPressureHg = std::nan(""); // ambient pressure, inHg
	double altimeterHg = std::nan("");      // Kohlsman setting
	double altimeterMb = std::nan("");
	double pressureAltitudeFt = std::nan("");
	double densityAltitudeFt = std::nan(""); // densityAltitudeFeet()
	double visibilityM = std::nan("");
	double seaLevelPressureMb = std::nan(""); // the actual QNH, so a wrong altimeter setting shows
	QString precipitation;                   // AMBIENT PRECIP STATE ("rain"), empty if unknown
	double precipitationRate = std::nan(""); // AMBIENT PRECIP RATE, millimeters of water
	// 1 in cloud, 0 not; NaN unless the sample also recorded the precipitation
	// state: older samples, recorded before this bit, have it cleared, not unknown.
	double inCloud = std::nan("");
	double fieldElevationFt = std::nan(""); // ground under the aircraft, ft MSL
	// Wind over the 30 s up to the contact: lowest and highest speed (kt), and
	// the direction range (degrees true, From..To clockwise).
	double windMinKt = std::nan("");
	double windMaxKt = std::nan("");
	double windDirectionFrom = std::nan("");
	double windDirectionTo = std::nan("");
	QString surfaceType;       // SURFACE TYPE ("asphalt"), empty if unknown
	QString surfaceCondition;  // SURFACE CONDITION ("wet"), empty if unknown
};

// One sample of a contact's final trajectory (ContactProfile). Heights are
// measured from the contact, not from the ground below: radio height follows
// the terrain under the aircraft (a valley, cliff or the sea before the
// runway), height above the touchdown/liftoff point doesn't.
struct ProfilePoint {
	QString gate;                 // "500 ft", "2 nm"; empty in the dense series
	double seconds = 0;           // from the contact (negative = before it)
	double heightFt = std::nan(""); // MSL altitude minus the on-ground sample's at the contact
	double distanceNm = std::nan(""); // great-circle distance to the contact position
	double terrainFt = std::nan("");  // the ground below, relative to the ground at the contact
	double airspeed = std::nan("");   // kt indicated
	double verticalSpeed = std::nan(""); // ft/min
	double pitch = std::nan("");      // degrees, + = nose up
	double bank = std::nan("");       // degrees
	double headingTrue = std::nan("");
	double trackTrue = std::nan("");
	// Along the runway (ft; from the marked threshold for a touchdown, from the
	// physical start for a liftoff, like distanceLength; negative = before it)
	// and from its centerline (ft, +right/-left); NaN without the contact's
	// ContactRunway.
	double thresholdFt = std::nan("");
	double centerlineFt = std::nan("");
	// Distance gates only: the descent angle (degrees) from this gate to the
	// next closer one, or to the contact after the last.
	double pathAngle = std::nan("");
};

// A liftoff's/touchdown's final trajectory (analyzeContacts()): the approach
// before a touchdown, the rotation and climb-out around a liftoff. Never
// reaches past the trip's previous or next contact.
struct ContactProfile {
	std::vector<ProfilePoint> gates;   // in time order
	std::vector<ProfilePoint> series;  // every sample near the contact (capped), in time order
	// The highest ground (relative to the ground at the contact) under the
	// last 5 nm of the approach / the first 5 nm of the climb-out, and its
	// distance from the contact; NaN without samples there.
	double highestTerrainFt = std::nan("");
	double highestTerrainNm = std::nan("");
};

// The used runway end's facility data stored with a contact (the runway_*
// columns); every value NaN when no runway was matched or the contact was
// recorded before these were.
struct ContactRunway {
	double lengthFt = std::nan("");
	double widthFt = std::nan("");
	double thresholdOffsetFt = std::nan("");  // displaced threshold of the used end
	double startLatitude = std::nan("");      // the used end's physical start
	double startLongitude = std::nan("");
	double trueHeading = std::nan("");        // exact, degrees true
	double slopeDegrees = std::nan("");       // in the direction of travel, + = uphill
};

// What a liftoff and a touchdown both record (trip_liftoffs/trip_touchdowns
// share these columns), shown as a map marker and its popup. conditions and
// profile are derived from the trip's samples when it loads.
struct RunwayContactPoint {
	double latitude = 0;
	double longitude = 0;
	QString icao;
	QString runway;
	int runwayHeading = -1;  // real facility heading (1-360) of runway used, -1 = unknown
	int airspeed = 0;
	int verticalSpeed = 0;
	double pitchDegrees = 0;
	double bankDegrees = 0;
	int headingDegrees = 0;
	QString airportName;
	// The four distances below only mean something when runway is set: with no
	// runway matched their columns are NULL and read back as 0.
	double distanceLength = 0;    // feet from threshold (negative = before it)
	double distanceWidth = 0;     // feet from centerline (+right/-left)
	double distanceLengthPercent = 0;   // distanceLength as a fraction of runway length (may be < 0 or > 1)
	double distanceWidthPercent = 0;    // distanceWidth as a signed fraction of runway half-width
	int windDirection = 0;   // degrees true, 0 if unknown
	int windVelocity = 0;    // knots
	QString zuluTime;
	QString localTime;
	int rowId = 0;           // the row's id, used to UPDATE analysis_report
	QString analysisReport;  // stored AI analysis text, empty if never analyzed
	ContactRunway runwayData;
	ContactConditions conditions;
	ContactProfile profile;
};

// One moment the aircraft became airborne during the trip (trip_liftoffs):
// the trip's departure plus any subsequent touch-and-go liftoffs. No gForce
// field: unlike a landing, there's no meaningful "how hard" to becoming
// airborne.
struct LiftoffPoint : RunwayContactPoint {};

// One landing logged during the trip (trip_touchdowns). A trip can have more
// than one touchdown (go-arounds, bounces).
struct TouchdownPoint : RunwayContactPoint {
	double gForce = 0;
};

// One discrete cockpit event (gear/flaps/spoilers/etc. toggled) logged during
// the trip (trip_events). trip_events only stores a timestamp, not a
// position -- latitude/longitude/sampleIndex are resolved after loading from
// the first TripSamplePoint at or after zuluTime, or the last point for an
// event after the final sample (sampleAtOrAfter(); see
// resolveEventPositions() in db_history.h).
struct TripEvent {
	QString event;
	QString zuluTime;
	double latitude = 0;
	double longitude = 0;
	int sampleIndex = -1;
};

// Parses a DATETIME::format_date_time() string (types.h), which is ISO8601
// with milliseconds and a UTC offset, plus a non-standard trailing
// "_<day-of-week>" suffix that QDateTime doesn't understand -- strip it
// before handing the rest to Qt's ISO parser.
inline QDateTime parseZuluTime(const QString& value) {
	const QString isoPart = value.section('_', 0, 0);
	return QDateTime::fromString(isoPart, Qt::ISODateWithMs);
}

// The index of the first of points at or after zuluTime, the last point for
// a time after the final one, or -1 without points. Compares zuluTime
// strings lexicographically: every row shares the same
// "%04d-%02d-%02dT..." format and timezone. points are in time order.
inline int sampleAtOrAfter(const std::vector<TripSamplePoint>& points, const QString& zuluTime) {
	auto it = std::lower_bound(points.begin(), points.end(), zuluTime,
		[](const TripSamplePoint& point, const QString& time) { return point.zuluTime < time; });
	if (it == points.end() && !points.empty())
		--it;
	return it == points.end() ? -1 : (int)(it - points.begin());
}

// "ICAO (Name)", or just "ICAO" without a name; empty without an ICAO. The
// one airport label shared by the Trip History table, its delete
// confirmation and kml_export.cpp's descriptions (ui/map/map.html's
// contactAirportLabel() builds the same format in JavaScript for its popup
// and AI prompt).
inline QString airportLabel(const QString& icao, const QString& airportName) {
	if (icao.isEmpty())
		return QString();
	return airportName.isEmpty() ? icao : QStringLiteral("%1 (%2)").arg(icao, airportName);
}

// "<departure>-<destination>" / "<departure>" / "<destination>" / fallback --
// shared by MapWidget's default Save Image/KML file name, TripHistoryPanel's
// row-menu KML file name, and kml_export.cpp's KML document title.
inline QString airportPairName(const QString& departureIcao, const QString& destinationIcao, const QString& fallback) {
	if (!departureIcao.isEmpty() && !destinationIcao.isEmpty())
		return QStringLiteral("%1-%2").arg(departureIcao, destinationIcao);
	if (!departureIcao.isEmpty())
		return departureIcao;
	if (!destinationIcao.isEmpty())
		return destinationIcao;
	return fallback;
}

// Same, deriving the ICAO pair from a trip's first liftoff and last touchdown.
inline QString airportPairName(const std::vector<LiftoffPoint>& liftoffPoints,
	const std::vector<TouchdownPoint>& touchdowns, const QString& fallback) {
	return airportPairName(!liftoffPoints.empty() ? liftoffPoints.front().icao : QString(),
		!touchdowns.empty() ? touchdowns.back().icao : QString(), fallback);
}

// Appends "_yyyyMMddHHmmss" derived from departureZulu to base, if it parses
// -- so re-saving/re-exporting the same trip later (or two trips between the
// same airports) doesn't silently overwrite the earlier file. Deliberately
// shared by MapWidget's Save Image *and* Export to KML file names (not just
// KML) and TripHistoryPanel's Export to KML file name, so all three use the
// same collision-avoidance scheme.
inline QString appendDepartureTimestamp(const QString& base, const QString& departureZulu) {
	const QDateTime departureTime = parseZuluTime(departureZulu);
	return departureTime.isValid()
		? base + QStringLiteral("_%1").arg(departureTime.toString(QStringLiteral("yyyyMMddHHmmss")))
		: base;
}

// Sample indices lo..hi thinned by a whole-number stride to at most
// maxPoints, plus hi itself -- so the last sample (e.g. the touchdown) is
// never dropped.
// Shared by MapWidget's trajectory and ChartsPanel's series: rendering every
// sample of a long flight is too slow for both. Empty if hi < lo.
inline std::vector<int> decimatedIndices(int lo, int hi, int maxPoints) {
	std::vector<int> indices;
	if (hi < lo)
		return indices;
	const int count = hi - lo + 1;
	// qMax, not std::max: <Windows.h> (included before this header in some
	// files) defines a max() macro.
	const int stride = qMax(1, (count + maxPoints - 1) / maxPoints);
	indices.reserve(count / stride + 2);
	for (int i = lo; i <= hi; i += stride)
		indices.push_back(i);
	if ((hi - lo) % stride != 0)
		indices.push_back(hi);
	return indices;
}

struct TripDataset {
	int tripId = -1;
	QString aircraftTitle;  // trips.title -- human-readable name like "Airbus A320neo FlyByWire"
	// trips.departure_zulu_time -- set at trip creation, so unlike
	// liftoffPoints.front().zuluTime it's available even when the trip has no
	// detected liftoff. The authoritative source for the Save Image and KML
	// file names' timestamp (appendDepartureTimestamp()).
	QString departureZuluTime;
	std::vector<TripSamplePoint> points;
	std::vector<LiftoffPoint> liftoffPoints;
	std::vector<TouchdownPoint> touchdowns;
	std::vector<TripEvent> events;
};

enum class TripStatus {
	Completed,
	Open,
	Live,
};

struct TripSummary {
	int id = 0;
	QString title;
	QString atcAirline;
	QString atcFlightNumber;
	QString departureIcao;
	QString departureName;
	QString departureRegion;
	QString departureRwy;
	QString destinationIcao;
	QString destinationName;
	QString destinationRegion;
	QString destinationRwy;
	QString departureZuluTime;
	QString destinationZuluTime;
	double departureLat = 0;
	double departureLng = 0;
	double destinationLat = 0;
	double destinationLng = 0;
	TripStatus status = TripStatus::Completed;
	// 0 = ungrouped (trips.group_id is NULL); sqlite3_column_int already
	// reads NULL as 0, so this needs no separate "has group" flag.
	int groupId = 0;
	QString groupName;
	// This trip's group's 1-based position in the Manage Groups list order
	// (trip_groups.sort_order, ties broken by name -- see queryAllGroups()).
	// 0 for ungrouped or if not yet resolved. Populated by TripHistoryPanel
	// (not by queryAllTrips() itself, which has no reason to know group
	// order) so the map can key each group's color/shape off its list
	// position rather than its immutable but arbitrary creation id.
	int groupRank = 0;
};

// A user-defined label trips can be assigned to (trip_groups table). Managed
// entirely via db_groups.h/.cpp and ManageGroupsDialog.
struct TripGroup {
	int id = 0;
	QString name;
	int tripCount = 0;
};
