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
	// Whether the sample recorded the actual vertical speed (VELOCITY WORLD Y):
	// one recorded before it was added didn't.
	bool hasActualVerticalSpeed() const { return !std::isnan(rawNum(TRIP_NUM_velocity_world_y)); }
	// The vertical speed (ft/min) the charts and the AI analysis show: the
	// actual one when recorded, else the indicated one (VERTICAL SPEED), which
	// lags it -- a liftoff can still read negative.
	double verticalSpeed() const {
		return hasActualVerticalSpeed() ? rawNum(TRIP_NUM_velocity_world_y) : rawNum(TRIP_NUM_vertical_speed);
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
	// The runway's true heading (RunwayContactPoint::runwayTrueHeading())
	// converted to magnetic with magneticVariation, in whole degrees 1..360;
	// NaN without either. Runway numbers come from the magnetic heading, so
	// this is the one the number matches.
	double runwayHeadingMagnetic = std::nan("");
	// Degrees above the horizon (sunElevationDegrees()); NaN when the
	// contact's time can't be read.
	double sunElevation = std::nan("");
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
	// the direction range (degrees true, From..To clockwise). The range stops
	// at a sample whose time can't be read; when the contact's can't, it is
	// the contact's sample alone.
	double windMinKt = std::nan("");
	double windMaxKt = std::nan("");
	double windDirectionFrom = std::nan("");
	double windDirectionTo = std::nan("");
	QString surfaceType;       // SURFACE TYPE ("asphalt"), empty if unknown
	QString surfaceCondition;  // SURFACE CONDITION ("wet"), empty if unknown
	double totalWeightLb = std::nan("");  // TOTAL WEIGHT
	double fuelWeightLb = std::nan("");   // FUEL TOTAL QUANTITY WEIGHT
};

// One sample of a contact's final trajectory (ContactProfile). Heights are
// measured from the contact, not from the ground below: radio height follows
// the terrain under the aircraft (a valley, cliff or the sea before the
// runway), height above the touchdown/liftoff point doesn't.
struct ProfilePoint {
	QString gate;                 // "500 ft", "2 nm"; empty in the series
	double seconds = 0;           // from the contact (negative = before it)
	double heightFt = std::nan(""); // MSL altitude minus the on-ground sample's at the contact
	double distanceNm = std::nan(""); // great-circle distance to the contact position
	double terrainFt = std::nan("");  // the ground below, relative to the ground at the contact
	double airspeed = std::nan("");   // kt indicated
	double groundSpeedKt = std::nan(""); // GROUND VELOCITY
	double verticalSpeed = std::nan(""); // ft/min, TripSamplePoint::verticalSpeed()
	double gForce = std::nan("");     // G FORCE
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
	// next closer one, or to the contact after the last, over the distance
	// flown between them (not the straight line, shorter on a curved path).
	double pathAngle = std::nan("");
	bool onGround = false;  // SIM ON GROUND
	// The weather at this point: the wind (from, degrees true; kt) and its
	// components along the contact's runway (runwayTrueHeading(); + = headwind, - =
	// tailwind) and across it (+ = from the right; both NaN without a
	// runway), the vertical wind (AMBIENT WIND Y, ft/min; + = up), the outside
	// air (°C), inCloud like ContactConditions', the precipitation like its
	// (precipitationName()) and the structural ice (STRUCTURAL ICE PCT, 0..1).
	double windDirection = std::nan("");
	double windSpeedKt = std::nan("");
	double verticalWindFpm = std::nan("");
	double headwindKt = std::nan("");
	double crosswindKt = std::nan("");
	double temperatureC = std::nan("");
	double inCloud = std::nan("");
	QString precipitation;
	double structuralIce = std::nan("");
	// Per engine, engine 1 first (NaN where not recorded): the throttle lever
	// position as the aircraft reports it (%, negative = reverse range), and
	// the value ContactProfile::powerName names (empty without one), and
	// ENG FAILED and ENG ON FIRE (1 or 0).
	std::vector<double> throttle;
	std::vector<double> power;
	std::vector<double> engineFailed;
	std::vector<double> engineOnFire;
};

// The weather and the ride through a contact's final segment
// (ContactProfile::segment): the approach's samples from its last crossing
// of 1000 ft above the touchdown point (or, never that high, its first since
// the previous contact) down to the touchdown, or the climb-out's from the
// liftoff up to its first crossing of 1000 ft (or the last before the next
// contact); either only within 5 nm of the contact, stopping at the first
// sample beyond it, so one that stays low isn't the whole flight. A value is NaN when none of the samples recorded it; the wind
// components also without the runway's heading.
struct SegmentWeather {
	int samples = 0;                      // 0 when the segment is empty: then nothing else is set
	double windMinKt = std::nan("");
	double windMaxKt = std::nan("");
	// The lowest and highest headwind component, in time order: from
	// headwindFromKt at headwindFromHeightFt to headwindToKt at
	// headwindToHeightFt, the largest change of the headwind through the
	// segment (wind shear). Where several samples share an extreme, the two
	// closest in time (the first such pair), so the heights show where the
	// change happened; a steady headwind is its first sample at both ends.
	double headwindFromKt = std::nan("");
	double headwindFromHeightFt = std::nan("");
	double headwindToKt = std::nan("");
	double headwindToHeightFt = std::nan("");
	double maxTailwindKt = std::nan("");  // the most negative headwind, negated (<= 0: no tailwind)
	double maxCrosswindKt = std::nan(""); // the strongest crosswind, signed like ProfilePoint's
	// Whether any sample recorded the cloud state (ContactConditions::inCloud),
	// and the lowest height at which the aircraft was in cloud: on an approach
	// where it broke out of the cloud base, on a climb-out where it entered it
	// (NaN when never in cloud).
	bool cloudKnown = false;
	double lowestInCloudFt = std::nan("");
	// 1 when a sample was in visible moisture (cloud or precipitation) at an
	// outside air temperature of -20..+10 °C, 0 when none was; NaN when no
	// sample recorded both.
	double icingConditions = std::nan("");
	double maxStructuralIce = std::nan("");  // STRUCTURAL ICE PCT, 0..1
	double maxPitotIce = std::nan("");       // PITOT ICE PCT, 0..1
	// The share (0..1) of the samples with ENG ANTI ICE on for any engine,
	// among those that recorded it.
	double engineAntiIceShare = std::nan("");
	// The lowest and highest G FORCE among the airborne samples: their spread
	// shows turbulence, but each is the instant of its sample, so a bump
	// between two samples is missed. Like the vertical wind, only up to the
	// stall sample below when it stalled: after it, the aircraft may be out of
	// control (a crash's tumble), and neither shows the air any more.
	double gForceMin = std::nan("");
	double gForceMax = std::nan("");
	// The lowest and highest vertical wind (ProfilePoint::verticalWindFpm): a
	// downdraft or updraft through the segment, each at the height of the
	// first sample that reached it.
	double verticalWindMinFpm = std::nan("");
	double verticalWindMinHeightFt = std::nan("");
	double verticalWindMaxFpm = std::nan("");
	double verticalWindMaxHeightFt = std::nan("");
	// The airborne sample closest to the stall: the smallest margin of its
	// angle of attack (INCIDENCE ALPHA, degrees) below the stall's (STALL
	// ALPHA, the angle of the maximum lift), at its height, airspeed (kt
	// indicated) and bank (degrees); when it stalled (no margin left), the
	// first such sample, where the stall began.
	double stallMarginAlphaDeg = std::nan("");
	double stallMarginStallAlphaDeg = std::nan("");
	double stallMarginHeightFt = std::nan("");
	double stallMarginAirspeedKt = std::nan("");
	double stallMarginBankDeg = std::nan("");
};

// A liftoff's/touchdown's final trajectory (contactProfile() in
// contact_analysis.h, built when its AI analysis asks for it): the approach,
// touchdown and rollout around a touchdown, the liftoff and climb-out
// around a liftoff. The gates never reach past the trip's previous or next
// contact; the series covers fixed seconds around the contact, whatever
// happened in them (a bounce, a go-around), and the highest terrain its 5 nm.
struct ContactProfile {
	std::vector<ProfilePoint> gates;   // in time order
	// Every sample near the contact (capped), and one per second further out
	// on the ground side (the rollout, the takeoff roll); in time order.
	std::vector<ProfilePoint> series;
	SegmentWeather segment;            // the approach's or climb-out's weather and ride
	// What each point's power values are, by the contact sample's ENGINE TYPE,
	// and their unit: "N1" for a jet and "torque" for a turboprop or helicopter
	// turbine, in "%"; "MP" for a piston, in " inHg" (manifold pressure: it
	// shows power with a fixed-pitch or a constant-speed propeller, where RPM
	// is held by the governor). powerTitle is the full name an abbreviated
	// powerName stands for ("manifold pressure" for "MP"), spelled out once
	// in the prompt so each point can use the short one; empty when powerName
	// is the name itself. All empty for any other type.
	QString powerName;
	QString powerUnit;
	QString powerTitle;
	// The highest ground (relative to the ground at the contact) under the
	// last 5 nm of the approach / the first 5 nm of the climb-out, and its
	// distance from the contact; NaN without samples there. terrainRangeNm is
	// the farthest from the contact the search reached when the samples ended
	// within those 5 nm, at a reset or the trip's start or end; NaN when it
	// covered them.
	double highestTerrainFt = std::nan("");
	double highestTerrainNm = std::nan("");
	double terrainRangeNm = std::nan("");
	// Whether the points' verticalSpeed is the indicated one: the contact's
	// sample didn't record the actual one (TripSamplePoint::hasActualVerticalSpeed()).
	bool indicatedVerticalSpeed = false;
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
// share these columns), shown as a map marker and its popup. conditions are
// derived from the trip's samples when it loads.
struct RunwayContactPoint {
	double latitude = 0;
	double longitude = 0;
	QString icao;
	QString runway;
	int runwayHeading = -1;  // real facility heading (1-360) of runway used, -1 = unknown
	int airspeed = 0;
	int verticalSpeed = 0;   // ft/min indicated (VERTICAL SPEED), which lags the actual rate
	// ft/min actual (VELOCITY WORLD Y) in the frame of the contact; NaN for a
	// contact recorded before it was.
	double actualVerticalSpeed = std::nan("");
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

	// The used runway's true heading in degrees: the exact one when stored
	// (runwayData), else the whole-degree runwayHeading; NaN when unknown.
	double runwayTrueHeading() const {
		if (!std::isnan(runwayData.trueHeading)) return runwayData.trueHeading;
		return runwayHeading >= 0 ? runwayHeading : std::nan("");
	}
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
	// The sink rate (ft/min, + = into the surface) at which the wheels met the
	// runway, perpendicular to it: PLANE TOUCHDOWN NORMAL VELOCITY, derived
	// when the trip loads (analyzeContacts()); NaN when it can't be.
	double touchdownRate = std::nan("");
	// Whether the aircraft crashed here: the simulator's reset after a crash
	// followed it, derived when the trip loads (analyzeContacts()).
	bool crashed = false;
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
// maxPoints, plus hi itself -- so the last sample (e.g. the end of the
// trajectory) is never dropped.
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
	// The aircraft's ATC MODEL SimVar as recorded, MSFS's wrapping included
	// (e.g. "ATCCOM.AC_MODEL A320.0.text"); Trip History unwraps it.
	QString atcModel;
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
