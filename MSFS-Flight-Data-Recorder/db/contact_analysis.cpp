#include "contact_analysis.h"
#include "runway_match.h"
#include "types.h"

#include <cmath>
#include <optional>

namespace {

constexpr double kDegToRad = V_PI / 180;
constexpr double kFeetPerNm = 6076.12;

// The touchdown gates: the last crossing before the contact of each height
// above it (ft) and each distance from it (nm). A liftoff's gates are the
// first crossing after it of each height.
constexpr double kTouchdownHeightGates[] = { 1000, 500, 300, 200, 100, 50 };
constexpr double kTouchdownDistanceGates[] = { 3, 2, 1 };
constexpr double kLiftoffHeightGates[] = { 35, 100, 200, 500, 1000 };

// The dense series: every sample from 15 s before a touchdown, or from 10 s
// before to 10 s after a liftoff, thinned to at most kMaxSeriesPoints.
constexpr double kTouchdownSeriesSeconds = 15;
constexpr double kLiftoffSeriesSeconds = 10;
constexpr int kMaxSeriesPoints = 60;

constexpr double kTerrainRangeNm = 5;     // ContactProfile's highest terrain
constexpr double kWindRangeSeconds = 30;  // ContactConditions' wind range

// a - b in degrees, wrapped to -180..180; NaN if either is.
double signedAngle(double a, double b) {
	return std::remainder(a - b, 360.0);
}

double wrap360(double degrees) {
	const double wrapped = std::fmod(degrees, 360.0);
	return wrapped < 0 ? wrapped + 360 : wrapped;
}

// Great-circle distance (nm) between two positions.
double distanceNm(double lat1, double lon1, double lat2, double lon2) {
	COORDINATE from;
	from.latitude = lat1;
	from.longitude = lon1;
	COORDINATE to;
	to.latitude = lat2;
	to.longitude = lon2;
	return from.distanceInKm2Coordinate(to) * 1000 * M_2_FT / kFeetPerNm;
}

// The ground (ft MSL) under a sample: its altitude minus its height above the
// ground, both measured at the aircraft's reference point.
double groundElevation(const TripSamplePoint& point) {
	return point.rawNum(TRIP_NUM_plane_altitude) - point.rawNum(TRIP_NUM_plane_alt_above_ground);
}

// The samples a contact is measured from: [begin, end) is its window (from
// the previous contact to the next), at is the sample at or after the
// contact, and ground the on-ground sample its heights are measured from (at
// for a touchdown; the last one before a liftoff, since at is airborne).
struct ContactSamples {
	int begin = 0;
	int end = 0;
	int at = 0;
	int ground = 0;
	QDateTime time;
};

// Turns samples into ProfilePoints relative to one contact.
class ProfileBuilder {
public:
	ProfileBuilder(const std::vector<TripSamplePoint>& points, const RunwayContactPoint& contact, const ContactSamples& samples,
		bool touchdown)
		: points_(points), contact_(contact), samples_(samples), touchdown_(touchdown),
		groundAltitude_(points[samples.ground].rawNum(TRIP_NUM_plane_altitude)),
		groundElevation_(groundElevation(points[samples.ground])) {}

	ProfilePoint at(int index, const QString& gate = QString()) const {
		const TripSamplePoint& p = points_[index];
		ProfilePoint out;
		out.gate = gate;
		out.seconds = seconds(index);
		out.heightFt = height(index);
		out.distanceNm = distance(index);
		out.terrainFt = groundElevation(p) - groundElevation_;
		out.airspeed = p.rawNum(TRIP_NUM_airspeed_indicated);
		out.verticalSpeed = p.rawNum(TRIP_NUM_vertical_speed);
		out.pitch = p.pitchDegrees;
		out.bank = p.bankDegrees;
		out.headingTrue = p.rawNum(TRIP_NUM_plane_heading_degrees_true);
		out.trackTrue = p.rawNum(TRIP_NUM_gps_ground_true_track);
		const ContactRunway& runway = contact_.runwayData;
		if (!std::isnan(runway.startLatitude) && !std::isnan(runway.startLongitude) && !std::isnan(runway.trueHeading)) {
			COORDINATE start, point;
			start.latitude = runway.startLatitude;
			start.longitude = runway.startLongitude;
			point.latitude = p.latitude;
			point.longitude = p.longitude;
			const RUNWAY_OFFSETS offsets = runway_offsets(start, runway.trueHeading, point);
			// Measured like the contact's distanceLength (match_runways()).
			out.thresholdFt = offsets.along_ft - (touchdown_ ? runway.thresholdOffsetFt : 0);
			out.centerlineFt = offsets.cross_ft;
		}
		return out;
	}
	double height(int index) const { return points_[index].rawNum(TRIP_NUM_plane_altitude) - groundAltitude_; }
	double distance(int index) const {
		return distanceNm(points_[index].latitude, points_[index].longitude, contact_.latitude, contact_.longitude);
	}
	// Seconds from the contact (negative before it); NaN for an unparsable time.
	double seconds(int index) const {
		const QDateTime time = parseZuluTime(points_[index].zuluTime);
		return time.isValid() && samples_.time.isValid() ? samples_.time.msecsTo(time) / 1000.0 : std::nan("");
	}
	// The highest terrain among indices from..to (either order) within
	// kTerrainRangeNm of the contact, stopping at the first sample beyond it.
	void highestTerrain(ContactProfile& profile, int from, int to) const {
		const int step = to >= from ? 1 : -1;
		for (int i = from; i != to + step && distance(i) <= kTerrainRangeNm; i += step) {
			const double terrain = groundElevation(points_[i]) - groundElevation_;
			if (!std::isnan(terrain) && (std::isnan(profile.highestTerrainFt) || terrain > profile.highestTerrainFt)) {
				profile.highestTerrainFt = terrain;
				profile.highestTerrainNm = distance(i);
			}
		}
	}

private:
	const std::vector<TripSamplePoint>& points_;
	const RunwayContactPoint& contact_;
	const ContactSamples& samples_;
	const bool touchdown_;
	double groundAltitude_;
	double groundElevation_;
};

QString feetGate(double feet) { return QStringLiteral("%1 ft").arg(feet); }
QString nmGate(double nm) { return QStringLiteral("%1 nm").arg(nm); }

// Indices lo..hi thinned to kMaxSeriesPoints, as ProfilePoints.
std::vector<ProfilePoint> series(const ProfileBuilder& builder, int lo, int hi) {
	std::vector<ProfilePoint> out;
	// decimatedIndices() may add hi to its maxPoints.
	for (int i : decimatedIndices(lo, hi, kMaxSeriesPoints - 1))
		out.push_back(builder.at(i));
	return out;
}

ContactProfile touchdownProfile(const ProfileBuilder& builder, const ContactSamples& s) {
	// Scanning back from the contact, the first sample at or above a gate is
	// its last crossing: an earlier descent (a go-around's) doesn't count.
	auto lastAtOrAbove = [&](auto value, double gate) {
		for (int i = s.ground - 1; i >= s.begin; --i)
			if (value(i) >= gate)
				return i;
		return -1;
	};
	std::vector<std::pair<int, ProfilePoint>> gates;
	for (double feet : kTouchdownHeightGates) {
		const int i = lastAtOrAbove([&](int j) { return builder.height(j); }, feet);
		if (i >= 0)
			gates.push_back({ i, builder.at(i, feetGate(feet)) });
	}
	// Each distance gate's path angle is down to the next closer one, the
	// closest's down to the contact (height 0, distance 0).
	double nextHeight = 0, nextDistance = 0;
	for (int k = (int)std::size(kTouchdownDistanceGates) - 1; k >= 0; --k) {
		const double nm = kTouchdownDistanceGates[k];
		const int i = lastAtOrAbove([&](int j) { return builder.distance(j); }, nm);
		if (i < 0)
			continue;
		ProfilePoint gate = builder.at(i, nmGate(nm));
		gate.pathAngle = std::atan2(gate.heightFt - nextHeight, (gate.distanceNm - nextDistance) * kFeetPerNm) / kDegToRad;
		nextHeight = gate.heightFt;
		nextDistance = gate.distanceNm;
		gates.push_back({ i, gate });
	}
	std::stable_sort(gates.begin(), gates.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

	ContactProfile profile;
	for (auto& gate : gates)
		profile.gates.push_back(gate.second);
	int first = s.at;
	// A sample whose time can't be read is taken in, so contactProfile()
	// drops the trajectory rather than cutting it short there.
	while (first - 1 >= s.begin && !(builder.seconds(first - 1) < -kTouchdownSeriesSeconds))
		--first;
	profile.series = series(builder, first, s.at);
	builder.highestTerrain(profile, s.ground, s.begin);
	return profile;
}

ContactProfile liftoffProfile(const ProfileBuilder& builder, const ContactSamples& s) {
	ContactProfile profile;
	for (double feet : kLiftoffHeightGates) {
		for (int i = s.at; i < s.end; ++i) {
			if (builder.height(i) >= feet) {
				profile.gates.push_back(builder.at(i, feetGate(feet)));
				break;
			}
		}
	}
	// From the on-ground sample its heights are measured from, even one past
	// the 10 s (a gap in the recording).
	int first = s.ground, last = s.at;
	// As in touchdownProfile(), a sample without a time is taken in.
	while (first - 1 >= s.begin && !(builder.seconds(first - 1) < -kLiftoffSeriesSeconds))
		--first;
	while (last + 1 < s.end && !(builder.seconds(last + 1) > kLiftoffSeriesSeconds))
		++last;
	profile.series = series(builder, first, last);
	if (s.ground < s.end)
		builder.highestTerrain(profile, s.ground, s.end - 1);
	return profile;
}

ContactConditions conditionsAt(const std::vector<TripSamplePoint>& points, const RunwayContactPoint& contact, const ContactSamples& s) {
	ContactConditions c;
	c.fromSample = true;
	const TripSamplePoint& p = points[s.at];
	c.headingTrue = p.rawNum(TRIP_NUM_plane_heading_degrees_true);
	c.headingMagnetic = p.rawNum(TRIP_NUM_plane_heading_degrees_magnetic);
	c.trackTrue = p.rawNum(TRIP_NUM_gps_ground_true_track);
	c.magneticVariation = signedAngle(c.headingTrue, c.headingMagnetic);
	c.crabAngle = signedAngle(c.headingTrue, c.trackTrue);
	if (s.time.isValid())
		c.sunElevation = sunElevationDegrees(contact.latitude, contact.longitude, s.time);
	c.temperatureC = p.rawNum(TRIP_NUM_ambient_temperature);
	c.stationPressureHg = p.rawNum(TRIP_NUM_ambient_pressure);
	c.altimeterHg = p.rawNum(TRIP_NUM_kohlsman_setting_hg);
	c.altimeterMb = p.rawNum(TRIP_NUM_kohlsman_setting_mb);
	c.pressureAltitudeFt = p.rawNum(TRIP_NUM_pressure_altitude) * M_2_FT;  // recorded in meters
	c.densityAltitudeFt = densityAltitudeFeet(c.pressureAltitudeFt, c.temperatureC);
	c.visibilityM = p.rawNum(TRIP_NUM_ambient_visibility);
	c.seaLevelPressureMb = p.rawNum(TRIP_NUM_sea_level_pressure);
	const double precipitationState = p.rawNum(TRIP_NUM_ambient_precip_state);
	c.precipitation = precipitationName(precipitationState);
	c.precipitationRate = p.rawNum(TRIP_NUM_ambient_precip_rate);
	if (!std::isnan(precipitationState))
		c.inCloud = TripBoolBits::ambient_in_cloud.isSet(p.boolGroups) ? 1 : 0;
	c.fieldElevationFt = groundElevation(points[s.ground]);
	c.surfaceType = surfaceTypeName(p.rawNum(TRIP_NUM_surface_type));
	c.surfaceCondition = surfaceConditionName(p.rawNum(TRIP_NUM_surface_condition));

	// The wind range, from the contact's sample back kWindRangeSeconds. The
	// directions are offsets from the contact's own, so a range across north
	// stays one range.
	const double direction = p.rawNum(TRIP_NUM_ambient_wind_direction);
	double lowOffset = 0, highOffset = 0;
	for (int i = s.at; i >= 0; --i) {
		if (i != s.at) {
			const QDateTime time = parseZuluTime(points[i].zuluTime);
			if (!time.isValid() || !s.time.isValid() || time.msecsTo(s.time) > kWindRangeSeconds * 1000)
				break;
		}
		const double speed = points[i].rawNum(TRIP_NUM_ambient_wind_velocity);
		if (!std::isnan(speed)) {
			c.windMinKt = std::isnan(c.windMinKt) ? speed : (std::min)(c.windMinKt, speed);
			c.windMaxKt = std::isnan(c.windMaxKt) ? speed : (std::max)(c.windMaxKt, speed);
		}
		const double offset = signedAngle(points[i].rawNum(TRIP_NUM_ambient_wind_direction), direction);
		if (!std::isnan(offset)) {
			lowOffset = (std::min)(lowOffset, offset);
			highOffset = (std::max)(highOffset, offset);
		}
	}
	// Unknown (NaN) without the contact's own direction.
	c.windDirectionFrom = wrap360(direction + lowOffset);
	c.windDirectionTo = wrap360(direction + highOffset);
	return c;
}

// Whether every point of profile has its time from the contact.
bool allTimesKnown(const ContactProfile& profile) {
	for (const std::vector<ProfilePoint>* points : { &profile.gates, &profile.series })
		for (const ProfilePoint& p : *points)
			if (std::isnan(p.seconds))
				return false;
	return true;
}

// contact's samples in dataset; nullopt without a sample at or after it.
std::optional<ContactSamples> contactSamples(const TripDataset& dataset, const RunwayContactPoint& contact, bool touchdown) {
	const std::vector<TripSamplePoint>& points = dataset.points;
	const int at = sampleAtOrAfter(points, contact.zuluTime);
	if (at < 0)
		return std::nullopt;
	std::vector<QString> contactTimes;
	for (const LiftoffPoint& p : dataset.liftoffPoints)
		contactTimes.push_back(p.zuluTime);
	for (const TouchdownPoint& p : dataset.touchdowns)
		contactTimes.push_back(p.zuluTime);
	std::sort(contactTimes.begin(), contactTimes.end());
	ContactSamples s;
	// The neighboring contacts: strictly before and after this one's time.
	const auto previous = std::lower_bound(contactTimes.begin(), contactTimes.end(), contact.zuluTime);
	const auto next = std::upper_bound(contactTimes.begin(), contactTimes.end(), contact.zuluTime);
	s.begin = previous == contactTimes.begin() ? 0 : sampleAtOrAfter(points, *(previous - 1));
	s.end = next == contactTimes.end() ? (int)points.size() : sampleAtOrAfter(points, *next);
	s.at = at;
	s.ground = !touchdown && at > s.begin ? at - 1 : at;
	s.time = parseZuluTime(contact.zuluTime);
	return s;
}

}

double sunElevationDegrees(double latitude, double longitude, const QDateTime& utc) {
	const double julianDay = utc.toMSecsSinceEpoch() / 86400000.0 + 2440587.5;
	const double t = (julianDay - 2451545) / 36525;  // Julian centuries since J2000
	const double meanLongitude = wrap360(280.46646 + t * (36000.76983 + t * 0.0003032));
	const double meanAnomaly = 357.52911 + t * (35999.05029 - 0.0001537 * t);
	const double eccentricity = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
	const double m = meanAnomaly * kDegToRad;
	const double center = std::sin(m) * (1.914602 - t * (0.004817 + 0.000014 * t))
		+ std::sin(2 * m) * (0.019993 - 0.000101 * t) + std::sin(3 * m) * 0.000289;
	const double omega = (125.04 - 1934.136 * t) * kDegToRad;
	const double apparentLongitude = (meanLongitude + center - 0.00569 - 0.00478 * std::sin(omega)) * kDegToRad;
	const double meanObliquity = 23 + (26 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60) / 60;
	const double obliquity = (meanObliquity + 0.00256 * std::cos(omega)) * kDegToRad;
	const double declination = std::asin(std::sin(obliquity) * std::sin(apparentLongitude));
	const double y = std::pow(std::tan(obliquity / 2), 2);
	const double l0 = meanLongitude * kDegToRad;
	const double equationOfTimeMinutes = 4 / kDegToRad * (y * std::sin(2 * l0) - 2 * eccentricity * std::sin(m)
		+ 4 * eccentricity * y * std::sin(m) * std::cos(2 * l0) - 0.5 * y * y * std::sin(4 * l0)
		- 1.25 * eccentricity * eccentricity * std::sin(2 * m));
	const double utcMinutes = utc.toUTC().time().msecsSinceStartOfDay() / 60000.0;
	const double hourAngle = ((utcMinutes + equationOfTimeMinutes + 4 * longitude) / 4 - 180) * kDegToRad;
	const double lat = latitude * kDegToRad;
	const double cosZenith = std::sin(lat) * std::sin(declination) + std::cos(lat) * std::cos(declination) * std::cos(hourAngle);
	return 90 - std::acos(std::clamp(cosZenith, -1.0, 1.0)) / kDegToRad;
}

double densityAltitudeFeet(double pressureAltitudeFt, double temperatureC) {
	return pressureAltitudeFt + 118.8 * (temperatureC - (15 - 1.98 * pressureAltitudeFt / 1000));
}

namespace {

// names[value], or empty for a value that isn't one of its indices.
template <size_t N>
QString enumName(const char* const (&names)[N], double value) {
	return value >= 0 && value < N && value == std::floor(value) ? QString::fromLatin1(names[(int)value]) : QString();
}

}

QString surfaceTypeName(double value) {
	static const char* const names[] = { "concrete", "grass", "water", "bumpy grass", "asphalt", "short grass",
		"long grass", "hard turf", "snow", "ice", "urban", "forest", "dirt", "coral", "gravel", "oil-treated",
		"steel mats", "bituminous", "brick", "macadam", "planks", "sand", "shale", "tarmac", "Wright Flyer track" };
	return enumName(names, value);
}

QString surfaceConditionName(double value) {
	static const char* const names[] = { "normal", "wet", "icy", "snow" };
	return enumName(names, value);
}

QString precipitationName(double mask) {
	if (std::isnan(mask))
		return QString();
	const int bits = (int)mask;
	if ((bits & 4) && (bits & 8))
		return QStringLiteral("rain and snow");
	if (bits & 4)
		return QStringLiteral("rain");
	if (bits & 8)
		return QStringLiteral("snow");
	return bits & 2 ? QStringLiteral("none") : QString();
}

void analyzeContacts(TripDataset& dataset) {
	for (LiftoffPoint& p : dataset.liftoffPoints)
		if (const std::optional<ContactSamples> s = contactSamples(dataset, p, false))
			p.conditions = conditionsAt(dataset.points, p, *s);
	for (TouchdownPoint& p : dataset.touchdowns)
		if (const std::optional<ContactSamples> s = contactSamples(dataset, p, true))
			p.conditions = conditionsAt(dataset.points, p, *s);
}

std::optional<ContactProfile> contactProfile(const TripDataset& dataset, const RunwayContactPoint& contact, bool touchdown) {
	const std::optional<ContactSamples> s = contactSamples(dataset, contact, touchdown);
	if (!s)
		return std::nullopt;
	const ProfileBuilder builder(dataset.points, contact, *s, touchdown);
	ContactProfile profile = touchdown ? touchdownProfile(builder, *s) : liftoffProfile(builder, *s);
	if (!allTimesKnown(profile))
		return std::nullopt;
	return profile;
}
