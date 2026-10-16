#include "contact_analysis.h"
#include "runway_match.h"
#include "types.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace {

constexpr double kFeetPerNm = 6076.12;

// The touchdown gates: the last crossing before the contact of each height
// above it (ft) and each distance from it (nm). A liftoff's gates are the
// first crossing after it of each height.
constexpr double kTouchdownHeightGates[] = { 1000, 500, 300, 200, 100, 50 };
constexpr double kTouchdownDistanceGates[] = { 3, 2, 1 };
constexpr double kLiftoffHeightGates[] = { 35, 100, 200, 500, 1000 };

// The dense series: every sample from 15 s before to 10 s after a
// touchdown, or from 10 s before to 10 s after a liftoff, thinned to at most
// kMaxSeriesPoints.
constexpr double kTouchdownSeriesSeconds = 15;
constexpr double kLiftoffSeriesSeconds = 10;
constexpr double kSeriesSecondsAfter = 10;
constexpr int kMaxSeriesPoints = 60;

constexpr double kTerrainRangeNm = 5;     // ContactProfile's highest terrain and SegmentWeather's segment
constexpr double kWindRangeSeconds = 30;  // ContactConditions' wind range
constexpr double kSegmentTopFt = 1000;    // SegmentWeather's segment

// The outside air temperatures (°C) at which visible moisture is taken as
// icing conditions (SegmentWeather::icingConditions).
constexpr double kIcingWarmestC = 10;
constexpr double kIcingColdestC = -20;

// AMBIENT PRECIP STATE's rain and snow bits (precipitationName()).
constexpr int kPrecipitationRain = 4;
constexpr int kPrecipitationSnow = 8;

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

// Lowers lowest to value / raises highest to it, ignoring a NaN value; a NaN
// lowest/highest takes any value.
void keepLowest(double& lowest, double value) {
	if (!std::isnan(value) && !(lowest <= value))
		lowest = value;
}
void keepHighest(double& highest, double value) {
	if (!std::isnan(value) && !(highest >= value))
		highest = value;
}

// point's in-cloud bit as 1 or 0; NaN unless point also recorded the
// precipitation state: samples recorded before the bit have it cleared, not
// unknown.
double inCloud(const TripSamplePoint& point) {
	if (std::isnan(point.rawNum(TRIP_NUM_ambient_precip_state)))
		return std::nan("");
	return TripBoolBits::ambient_in_cloud.isSet(point.boolGroups) ? 1 : 0;
}

// 1 when point was in visible moisture (in cloud, or in rain or snow) at an
// outside air temperature of kIcingColdestC..kIcingWarmestC, else 0; NaN
// without its cloud state or temperature.
double icingConditions(const TripSamplePoint& point) {
	const double cloud = inCloud(point);
	const double temperature = point.rawNum(TRIP_NUM_ambient_temperature);
	if (std::isnan(cloud) || std::isnan(temperature))
		return std::nan("");
	const bool moisture = cloud == 1
		|| ((int)point.rawNum(TRIP_NUM_ambient_precip_state) & (kPrecipitationRain | kPrecipitationSnow)) != 0;
	return moisture && temperature >= kIcingColdestC && temperature <= kIcingWarmestC ? 1 : 0;
}

// 1 when point has ENG ANTI ICE on for any engine, 0 when off for every
// engine that recorded it; NaN when none did.
double engineAntiIce(const TripSamplePoint& point) {
	double on = std::nan("");
	for (int engine = 1; engine <= point.engineCount(); ++engine) {
		const double value = point.engineValue(engine, TRIP_ENGINE_eng_anti_ice);
		if (!std::isnan(value))
			keepHighest(on, value != 0 ? 1 : 0);
	}
	return on;
}

// The samples a contact is measured from: [begin, end) is its gates' and
// terrain's window (from the previous contact to the next), at is the
// sample at or after the contact, and ground the on-ground sample its
// heights are measured from (at for a touchdown; the last one before a
// liftoff, since at is airborne).
struct ContactSamples {
	int begin = 0;
	int end = 0;
	int at = 0;
	int ground = 0;
	QDateTime time;
};

// The engine value ContactProfile::powerName/powerUnit describe.
struct PowerField {
	TripEngineField field;
	QString name;
	QString unit;
};

// The power value for an ENGINE TYPE, or nullopt for a type without one.
std::optional<PowerField> powerField(int engineType) {
	switch (engineType) {
	case 0: return PowerField{ TRIP_ENGINE_recip_eng_manifold_pressure, QStringLiteral("manifold pressure"), QStringLiteral(" inHg") };  // piston
	case 1: return PowerField{ TRIP_ENGINE_turb_eng_n1, QStringLiteral("N1"), QStringLiteral("%") };  // jet
	case 3:  // helicopter turbine
	case 5: return PowerField{ TRIP_ENGINE_turb_eng_max_torque_percent, QStringLiteral("torque"), QStringLiteral("%") };  // turboprop
	default: return std::nullopt;
	}
}

// Turns samples into ProfilePoints relative to one contact.
class ProfileBuilder {
public:
	ProfileBuilder(const std::vector<TripSamplePoint>& points, const RunwayContactPoint& contact, const ContactSamples& samples,
		bool touchdown)
		: points_(points), contact_(contact), samples_(samples), touchdown_(touchdown),
		groundAltitude_(points[samples.ground].rawNum(TRIP_NUM_plane_altitude)),
		groundElevation_(groundElevation(points[samples.ground])),
		power_(powerField(points[samples.at].engineType)) {}

	ProfilePoint at(int index, const QString& gate = QString()) const {
		const TripSamplePoint& p = points_[index];
		ProfilePoint out;
		out.gate = gate;
		out.seconds = seconds(index);
		out.heightFt = height(index);
		out.distanceNm = distance(index);
		out.terrainFt = groundElevation(p) - groundElevation_;
		out.airspeed = p.rawNum(TRIP_NUM_airspeed_indicated);
		out.verticalSpeed = p.verticalSpeed();
		out.pitch = p.pitchDegrees;
		out.bank = p.bankDegrees;
		out.headingTrue = p.rawNum(TRIP_NUM_plane_heading_degrees_true);
		out.trackTrue = p.rawNum(TRIP_NUM_gps_ground_true_track);
		// Without the runway's start or heading (a contact recorded before
		// they were stored) runway_offsets() gives NaN, so the point isn't
		// placed on the runway.
		const ContactRunway& runway = contact_.runwayData;
		COORDINATE start, point;
		start.latitude = runway.startLatitude;
		start.longitude = runway.startLongitude;
		point.latitude = p.latitude;
		point.longitude = p.longitude;
		const RUNWAY_OFFSETS offsets = runway_offsets(start, runway.trueHeading, point);
		// Measured like the contact's distanceLength (match_runways()).
		out.thresholdFt = offsets.along_ft - (touchdown_ ? runway.thresholdOffsetFt : 0);
		out.centerlineFt = offsets.cross_ft;
		out.onGround = TripBoolBits::sim_on_ground.isSet(p.boolGroups);
		out.windDirection = p.rawNum(TRIP_NUM_ambient_wind_direction);
		out.windSpeedKt = p.rawNum(TRIP_NUM_ambient_wind_velocity);
		const double windAngle = (out.windDirection - contact_.runwayTrueHeading()) * DEG_2_RAD;
		out.headwindKt = out.windSpeedKt * std::cos(windAngle);
		out.crosswindKt = out.windSpeedKt * std::sin(windAngle);
		out.temperatureC = p.rawNum(TRIP_NUM_ambient_temperature);
		out.inCloud = inCloud(p);
		out.precipitation = precipitationName(p.rawNum(TRIP_NUM_ambient_precip_state));
		out.structuralIce = p.rawNum(TRIP_NUM_structural_ice_pct);
		for (int engine = 1; engine <= p.engineCount(); ++engine) {
			out.throttle.push_back(p.engineValue(engine, TRIP_ENGINE_general_eng_throttle_lever_position));
			if (power_)
				out.power.push_back(p.engineValue(engine, power_->field));
		}
		return out;
	}
	// Names the power values on profile (ContactProfile::powerName/powerUnit).
	void describePower(ContactProfile& profile) const {
		if (power_) {
			profile.powerName = power_->name;
			profile.powerUnit = power_->unit;
		}
	}
	int size() const { return (int)points_.size(); }
	const TripSamplePoint& sample(int index) const { return points_[index]; }
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
	std::optional<PowerField> power_;
};

QString feetGate(double feet) { return QStringLiteral("%1 ft").arg(feet); }
QString nmGate(double nm) { return QStringLiteral("%1 nm").arg(nm); }

// The samples from secondsBefore before the contact to kSeriesSecondsAfter
// after it, thinned to kMaxSeriesPoints, as ProfilePoints. Always from the
// on-ground sample the heights are measured from through the contact's,
// even one past those seconds (a gap in the recording), and both are kept
// however the rest is thinned. Unlike the gates,
// not stopped at the neighboring contacts: whatever happened in those
// seconds (a bounce, a go-around) is evidence for judging this one.
std::vector<ProfilePoint> series(const ProfileBuilder& builder, const ContactSamples& s, double secondsBefore) {
	int first = s.ground, last = s.at;
	// A sample whose time can't be read is taken in, so contactProfile()
	// drops the trajectory rather than cutting it short there.
	while (first - 1 >= 0 && !(builder.seconds(first - 1) < -secondsBefore))
		--first;
	while (last + 1 < builder.size() && !(builder.seconds(last + 1) > kSeriesSecondsAfter))
		++last;
	// decimatedIndices() may add hi to its maxPoints; s.ground and s.at may
	// add two more.
	std::vector<int> indices = decimatedIndices(first, last, kMaxSeriesPoints - 3);
	for (int keep : { s.ground, s.at })
		if (!std::binary_search(indices.begin(), indices.end(), keep))
			indices.insert(std::upper_bound(indices.begin(), indices.end(), keep), keep);
	std::vector<ProfilePoint> out;
	for (int i : indices)
		out.push_back(builder.at(i));
	return out;
}

// The weather and the ride (SegmentWeather) through the samples from..to
// (inclusive, from first); empty when to is from - 1 (a liftoff whose next
// contact falls before its sample after it).
SegmentWeather segmentWeather(const ProfileBuilder& builder, int from, int to) {
	SegmentWeather w;
	w.samples = to - from + 1;
	// Each sample's headwind and height, in time order, for the change.
	std::vector<std::pair<double, double>> headwinds;
	double lowestHeadwind = std::nan(""), highestHeadwind = std::nan("");
	int antiIceOn = 0, antiIceKnown = 0;
	for (int i = from; i <= to; ++i) {
		const ProfilePoint p = builder.at(i);
		const TripSamplePoint& sample = builder.sample(i);
		keepLowest(w.windMinKt, p.windSpeedKt);
		keepHighest(w.windMaxKt, p.windSpeedKt);
		if (!std::isnan(p.headwindKt))
			headwinds.emplace_back(p.headwindKt, p.heightFt);
		keepLowest(lowestHeadwind, p.headwindKt);
		keepHighest(highestHeadwind, p.headwindKt);
		keepHighest(w.maxTailwindKt, -p.headwindKt);
		if (!std::isnan(p.crosswindKt) && !(std::abs(w.maxCrosswindKt) >= std::abs(p.crosswindKt)))
			w.maxCrosswindKt = p.crosswindKt;
		if (!std::isnan(p.inCloud)) {
			w.cloudKnown = true;
			if (p.inCloud == 1)
				keepLowest(w.lowestInCloudFt, p.heightFt);
		}
		keepHighest(w.icingConditions, icingConditions(sample));
		keepHighest(w.maxStructuralIce, p.structuralIce);
		keepHighest(w.maxPitotIce, sample.rawNum(TRIP_NUM_pitot_ice_pct));
		const double antiIce = engineAntiIce(sample);
		if (!std::isnan(antiIce)) {
			++antiIceKnown;
			antiIceOn += antiIce == 1;
		}
		keepLowest(w.verticalWindMinFpm, sample.rawNum(TRIP_NUM_ambient_wind_y));
		keepHighest(w.verticalWindMaxFpm, sample.rawNum(TRIP_NUM_ambient_wind_y));
		if (!p.onGround) {
			keepLowest(w.gForceMin, sample.rawNum(TRIP_NUM_g_force));
			keepHighest(w.gForceMax, sample.rawNum(TRIP_NUM_g_force));
			const double alpha = sample.rawNum(TRIP_NUM_incidence_alpha);
			const double stallAlpha = sample.rawNum(TRIP_NUM_stall_alpha);
			const double margin = stallAlpha - alpha;
			if (!std::isnan(margin) && !(w.stallMarginStallAlphaDeg - w.stallMarginAlphaDeg <= margin)) {
				w.stallMarginAlphaDeg = alpha;
				w.stallMarginStallAlphaDeg = stallAlpha;
				w.stallMarginHeightFt = p.heightFt;
			}
		}
	}
	if (!headwinds.empty()) {
		// The closest pair in time with one at each extreme, so the change's
		// heights show where it happened: the last sample at the earlier
		// extreme and the first at the later one. A steady headwind is its
		// first sample at both ends.
		std::size_t first = 0, second = 0;
		if (lowestHeadwind < highestHeadwind) {
			std::optional<std::size_t> lastLowest, lastHighest;
			std::size_t gap = headwinds.size();
			for (std::size_t k = 0; k < headwinds.size(); ++k) {
				const double headwind = headwinds[k].first;
				const std::optional<std::size_t> other = headwind == lowestHeadwind ? lastHighest
					: headwind == highestHeadwind ? lastLowest : std::nullopt;
				if (other && k - *other < gap) {
					first = *other;
					second = k;
					gap = k - *other;
				}
				if (headwind == lowestHeadwind)
					lastLowest = k;
				if (headwind == highestHeadwind)
					lastHighest = k;
			}
		}
		w.headwindFromKt = headwinds[first].first;
		w.headwindFromHeightFt = headwinds[first].second;
		w.headwindToKt = headwinds[second].first;
		w.headwindToHeightFt = headwinds[second].second;
	}
	if (antiIceKnown > 0)
		w.engineAntiIceShare = (double)antiIceOn / antiIceKnown;
	return w;
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
		gate.pathAngle = std::atan2(gate.heightFt - nextHeight, (gate.distanceNm - nextDistance) * kFeetPerNm) / DEG_2_RAD;
		nextHeight = gate.heightFt;
		nextDistance = gate.distanceNm;
		gates.push_back({ i, gate });
	}
	std::stable_sort(gates.begin(), gates.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

	ContactProfile profile;
	for (auto& gate : gates)
		profile.gates.push_back(gate.second);
	profile.series = series(builder, s, kTouchdownSeriesSeconds);
	// Scanning back from the touchdown, the segment starts at the last
	// crossing of the top, but no further out than kTerrainRangeNm.
	const int top = lastAtOrAbove([&](int j) { return builder.height(j); }, kSegmentTopFt);
	int from = s.ground;
	while (from > (top >= 0 ? top : s.begin) && builder.distance(from - 1) <= kTerrainRangeNm)
		--from;
	profile.segment = segmentWeather(builder, from, s.ground);
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
	profile.series = series(builder, s, kLiftoffSeriesSeconds);
	int top = s.at;
	while (top < s.end - 1 && !(builder.height(top) >= kSegmentTopFt) && builder.distance(top + 1) <= kTerrainRangeNm)
		++top;
	profile.segment = segmentWeather(builder, s.at, (std::min)(top, s.end - 1));
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
	if (!std::isnan(contact.runwayTrueHeading()) && !std::isnan(c.magneticVariation)) {
		const int magnetic = qRound(wrap360(contact.runwayTrueHeading() - c.magneticVariation));
		c.runwayHeadingMagnetic = magnetic == 0 ? 360 : magnetic;
	}
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
	c.precipitation = precipitationName(p.rawNum(TRIP_NUM_ambient_precip_state));
	c.precipitationRate = p.rawNum(TRIP_NUM_ambient_precip_rate);
	c.inCloud = inCloud(p);
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
		keepLowest(c.windMinKt, speed);
		keepHighest(c.windMaxKt, speed);
		const double offset = signedAngle(points[i].rawNum(TRIP_NUM_ambient_wind_direction), direction);
		keepLowest(lowOffset, offset);
		keepHighest(highOffset, offset);
	}
	// Unknown (NaN) without the contact's own direction.
	c.windDirectionFrom = wrap360(direction + lowOffset);
	c.windDirectionTo = wrap360(direction + highOffset);
	return c;
}

// The touchdown's PLANE TOUCHDOWN NORMAL VELOCITY. The simulator holds each
// touchdown's value until the next one's, and can set it a sample after its
// on-ground flag, so the touchdown's own sample may still hold the previous
// value: the touchdown's is the first value, from its sample through the next
// contact's, that differs from the sample's before it. NaN without a sample
// before it, or when none differs.
double touchdownRate(const std::vector<TripSamplePoint>& points, const ContactSamples& s) {
	if (s.at == 0)
		return std::nan("");
	const double previous = points[s.at - 1].rawNum(TRIP_NUM_plane_touchdown_normal_velocity);
	for (int i = s.at; i <= s.end && i < (int)points.size(); ++i) {
		const double value = points[i].rawNum(TRIP_NUM_plane_touchdown_normal_velocity);
		if (!std::isnan(value) && value != previous)
			return value;
	}
	return std::nan("");
}

// Whether every point of profile has its time from the contact.
bool allTimesKnown(const ContactProfile& profile) {
	for (const std::vector<ProfilePoint>* points : { &profile.gates, &profile.series })
		for (const ProfilePoint& p : *points)
			if (std::isnan(p.seconds))
				return false;
	return true;
}

// contact's samples in dataset; nullopt without any samples.
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
	const double m = meanAnomaly * DEG_2_RAD;
	const double center = std::sin(m) * (1.914602 - t * (0.004817 + 0.000014 * t))
		+ std::sin(2 * m) * (0.019993 - 0.000101 * t) + std::sin(3 * m) * 0.000289;
	const double omega = (125.04 - 1934.136 * t) * DEG_2_RAD;
	const double apparentLongitude = (meanLongitude + center - 0.00569 - 0.00478 * std::sin(omega)) * DEG_2_RAD;
	const double meanObliquity = 23 + (26 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60) / 60;
	const double obliquity = (meanObliquity + 0.00256 * std::cos(omega)) * DEG_2_RAD;
	const double declination = std::asin(std::sin(obliquity) * std::sin(apparentLongitude));
	const double y = std::pow(std::tan(obliquity / 2), 2);
	const double l0 = meanLongitude * DEG_2_RAD;
	const double equationOfTimeMinutes = 4 / DEG_2_RAD * (y * std::sin(2 * l0) - 2 * eccentricity * std::sin(m)
		+ 4 * eccentricity * y * std::sin(m) * std::cos(2 * l0) - 0.5 * y * y * std::sin(4 * l0)
		- 1.25 * eccentricity * eccentricity * std::sin(2 * m));
	const double utcMinutes = utc.toUTC().time().msecsSinceStartOfDay() / 60000.0;
	const double hourAngle = ((utcMinutes + equationOfTimeMinutes + 4 * longitude) / 4 - 180) * DEG_2_RAD;
	const double lat = latitude * DEG_2_RAD;
	const double cosZenith = std::sin(lat) * std::sin(declination) + std::cos(lat) * std::cos(declination) * std::cos(hourAngle);
	return 90 - std::acos(std::clamp(cosZenith, -1.0, 1.0)) / DEG_2_RAD;
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
	const bool rain = bits & kPrecipitationRain, snow = bits & kPrecipitationSnow;
	if (rain && snow)
		return QStringLiteral("rain and snow");
	if (rain)
		return QStringLiteral("rain");
	if (snow)
		return QStringLiteral("snow");
	return bits & 2 ? QStringLiteral("none") : QString();
}

void analyzeContacts(TripDataset& dataset) {
	for (LiftoffPoint& p : dataset.liftoffPoints)
		if (const std::optional<ContactSamples> s = contactSamples(dataset, p, false))
			p.conditions = conditionsAt(dataset.points, p, *s);
	for (TouchdownPoint& p : dataset.touchdowns) {
		if (const std::optional<ContactSamples> s = contactSamples(dataset, p, true)) {
			p.conditions = conditionsAt(dataset.points, p, *s);
			p.touchdownRate = touchdownRate(dataset.points, *s);
		}
	}
}

std::optional<ContactProfile> contactProfile(const TripDataset& dataset, const RunwayContactPoint& contact, bool touchdown) {
	const std::optional<ContactSamples> s = contactSamples(dataset, contact, touchdown);
	if (!s)
		return std::nullopt;
	const ProfileBuilder builder(dataset.points, contact, *s, touchdown);
	ContactProfile profile = touchdown ? touchdownProfile(builder, *s) : liftoffProfile(builder, *s);
	builder.describePower(profile);
	profile.indicatedVerticalSpeed = !dataset.points[s->at].hasActualVerticalSpeed();
	if (!allTimesKnown(profile))
		return std::nullopt;
	return profile;
}
