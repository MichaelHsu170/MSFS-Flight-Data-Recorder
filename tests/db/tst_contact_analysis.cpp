// contact_analysis.h: the conditions and final trajectory each liftoff and
// touchdown gets from the trip's samples. Every expected value is worked out
// by hand from the fixture (heights, times, a 3-degree path, the solstice sun).
#include "contact_analysis.h"
#include "trip_data_fields.h"
#include "trip_dataset.h"

#include <QtTest>

#include <algorithm>
#include <cmath>
#include <optional>

namespace {

const QDateTime kBase(QDate(2026, 6, 21), QTime(12, 0), QTimeZone::UTC);

QString zulu(double seconds) {
	return kBase.addMSecs(qRound64(seconds * 1000)).toString(QStringLiteral("yyyy-MM-ddTHH:mm:ss.zzz")) + QStringLiteral("+00:00_1");
}

// A sample `seconds` from kBase at altitude/height above the ground (ft),
// nmNorth nautical miles north of 0/0 (one nm being about 1/60 degree of
// latitude). Every other raw value is not recorded.
TripSamplePoint sample(double seconds, double altitude, double agl, double nmNorth = 0) {
	TripSamplePoint p;
	p.latitude = nmNorth / 60;
	p.zuluTime = zulu(seconds);
	p.rawNums.assign(TRIP_NUM_FIELD_COUNT, std::nan(""));
	p.rawNums[TRIP_NUM_plane_altitude] = altitude;
	p.rawNums[TRIP_NUM_plane_alt_above_ground] = agl;
	return p;
}

template <class Contact>
Contact contactAt(double seconds) {
	Contact c;
	c.zuluTime = zulu(seconds);
	return c;
}

// A touchdown at 0 s after samples 1 s apart at these heights (ft) above it,
// the last at 0 s on the ground.
TripDataset touchdownAfter(const std::vector<double>& heights) {
	TripDataset dataset;
	for (size_t i = 0; i < heights.size(); ++i)
		dataset.points.push_back(sample(double(i) - double(heights.size() - 1), 1000 + heights[i], heights[i]));
	dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
	return dataset;
}

QStringList gateNames(const ContactProfile& profile) {
	QStringList names;
	for (const ProfilePoint& p : profile.gates)
		names.append(p.gate);
	return names;
}

const ProfilePoint* gate(const ContactProfile& profile, const QString& name) {
	for (const ProfilePoint& p : profile.gates)
		if (p.gate == name)
			return &p;
	return nullptr;
}

}

class TstContactAnalysis : public QObject {
	Q_OBJECT

private slots:
	void densityAltitude() {
		// 5000 + 118.8 x (30 - (15 - 9.9)) = 7958.12
		QCOMPARE(densityAltitudeFeet(5000, 30), 7958.12);
		QCOMPARE(densityAltitudeFeet(0, 15), 0.0);
	}

	// At the June solstice (declination 23.44) the sun culminates at
	// 90 - lat + 23.44 and is lowest at -(90 - lat - 23.44).
	void sunElevationFollowsLatitudeAndTime() {
		const QDateTime noon(QDate(2026, 6, 21), QTime(12, 0), QTimeZone::UTC);
		const QDateTime midnight(QDate(2026, 6, 21), QTime(0, 0), QTimeZone::UTC);
		QVERIFY(qAbs(sunElevationDegrees(64, 0, noon) - 49.44) < 0.1);
		// Local noon 90 degrees east is 06:00 UTC.
		QVERIFY(qAbs(sunElevationDegrees(64, 90, noon.addSecs(-6 * 3600)) - 49.44) < 0.1);
		QVERIFY(qAbs(sunElevationDegrees(64, 0, midnight) - -2.56) < 0.1);
		QVERIFY(qAbs(sunElevationDegrees(30, 0, midnight) - -36.56) < 0.1);
		// On 3 November the sun runs 16.4 min ahead of the clock (the equation
		// of time; declination -15.2): at 06:00 UTC on the equator at longitude
		// 0 it is 4.1 degrees of hour angle past rising, so at
		// asin(cos 15.2 x sin 4.1) = 3.97 degrees.
		const QDateTime sixAm(QDate(2026, 11, 3), QTime(6, 0), QTimeZone::UTC);
		QVERIFY2(qAbs(sunElevationDegrees(0, 0, sixAm) - 3.97) < 0.1, qPrintable(QString::number(sunElevationDegrees(0, 0, sixAm))));
	}

	// The SDK's SURFACE TYPE and SURFACE CONDITION values, in order.
	void surfaceNames() {
		const char* const types[] = { "concrete", "grass", "water", "bumpy grass", "asphalt", "short grass",
			"long grass", "hard turf", "snow", "ice", "urban", "forest", "dirt", "coral", "gravel", "oil-treated",
			"steel mats", "bituminous", "brick", "macadam", "planks", "sand", "shale", "tarmac", "Wright Flyer track" };
		for (int i = 0; i < (int)std::size(types); ++i)
			QCOMPARE(surfaceTypeName(i), QString::fromLatin1(types[i]));
		const char* const conditions[] = { "normal", "wet", "icy", "snow" };
		for (int i = 0; i < (int)std::size(conditions); ++i)
			QCOMPARE(surfaceConditionName(i), QString::fromLatin1(conditions[i]));
		for (double unknown : { 25.0, 255.0, -1.0, 1.5, std::nan("") })
			QVERIFY2(surfaceTypeName(unknown).isEmpty(), qPrintable(QString::number(unknown)));
		for (double unknown : { 4.0, 255.0, -1.0 })
			QVERIFY2(surfaceConditionName(unknown).isEmpty(), qPrintable(QString::number(unknown)));
	}

	// AMBIENT PRECIP STATE: 2 none, 4 rain, 8 snow.
	void precipitationNames() {
		QCOMPARE(precipitationName(2), QStringLiteral("none"));
		QCOMPARE(precipitationName(4), QStringLiteral("rain"));
		QCOMPARE(precipitationName(8), QStringLiteral("snow"));
		QCOMPARE(precipitationName(12), QStringLiteral("rain and snow"));
		QVERIFY(precipitationName(0).isEmpty());
		QVERIFY(precipitationName(std::nan("")).isEmpty());
	}

	// Precipitation, sea-level pressure and cloud come from the contact's
	// sample; in-cloud only when that sample also recorded the precipitation
	// state, since older samples have its bit cleared rather than unknown.
	void precipitationAndCloudComeFromTheContactSample() {
		const TripBoolBit cloud = TripBoolBits::ambient_in_cloud;
		TripDataset dataset;
		TripSamplePoint p = sample(0, 1017, 17);
		p.rawNums[TRIP_NUM_ambient_precip_state] = 4;
		p.rawNums[TRIP_NUM_ambient_precip_rate] = 2.5;
		p.rawNums[TRIP_NUM_sea_level_pressure] = 1008;
		p.boolGroups[cloud.group] = 1u << cloud.bit;
		dataset.points = { p };
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		analyzeContacts(dataset);
		const ContactConditions& c = dataset.touchdowns[0].conditions;
		QCOMPARE(c.precipitation, QStringLiteral("rain"));
		QCOMPARE(c.precipitationRate, 2.5);
		QCOMPARE(c.seaLevelPressureMb, 1008.0);
		QCOMPARE(c.inCloud, 1.0);

		dataset.points[0].boolGroups[cloud.group] = 0;
		analyzeContacts(dataset);
		QCOMPARE(dataset.touchdowns[0].conditions.inCloud, 0.0);

		// An older sample: no precipitation state, so its cleared or even set
		// bit says nothing.
		dataset.points[0].rawNums[TRIP_NUM_ambient_precip_state] = std::nan("");
		dataset.points[0].boolGroups[cloud.group] = 1u << cloud.bit;
		analyzeContacts(dataset);
		QVERIFY(std::isnan(dataset.touchdowns[0].conditions.inCloud));
		QVERIFY(dataset.touchdowns[0].conditions.precipitation.isEmpty());
	}

	// The runway's magnetic heading, the one its number matches: its true
	// heading minus the sample's magnetic variation, in whole degrees 1..360.
	void runwayHeadingMagneticUsesTheSampleVariation() {
		auto magneticRunway = [](int runwayTrue, double headingTrue, double headingMagnetic) {
			TripDataset dataset;
			TripSamplePoint p = sample(0, 0, 0);
			p.rawNums[TRIP_NUM_plane_heading_degrees_true] = headingTrue;
			p.rawNums[TRIP_NUM_plane_heading_degrees_magnetic] = headingMagnetic;
			dataset.points = { p };
			TouchdownPoint td = contactAt<TouchdownPoint>(0);
			td.runwayHeading = runwayTrue;
			dataset.touchdowns = { td };
			analyzeContacts(dataset);
			return dataset.touchdowns[0].conditions.runwayHeadingMagnetic;
		};
		QCOMPARE(magneticRunway(62, 65, 77.4), 74.0);   // 12.4° W (Iceland): 74.4 rounds down
		QCOMPARE(magneticRunway(62, 65, 77.6), 75.0);   // 74.6 rounds up
		QCOMPARE(magneticRunway(2, 2, 358), 358.0);     // 4° E across north: not -2
		QCOMPARE(magneticRunway(358, 358, 2), 2.0);     // 4° W across north: not 362
		QCOMPARE(magneticRunway(4, 4, 0), 360.0);       // north is 360, as runway headings are
		QCOMPARE(magneticRunway(4, 4, 359.6), 360.0);   // 359.6 rounds to 360, not 0
		QVERIFY(std::isnan(magneticRunway(-1, 65, 77)));  // no runway heading
		QVERIFY(std::isnan(magneticRunway(62, std::nan(""), 77)));  // no variation
	}

	void conditionsComeFromTheContactSample() {
		TripDataset dataset;
		TripSamplePoint p = sample(0, 1017, 17);
		p.rawNums[TRIP_NUM_plane_heading_degrees_true] = 2;
		p.rawNums[TRIP_NUM_plane_heading_degrees_magnetic] = 358;
		p.rawNums[TRIP_NUM_gps_ground_true_track] = 6;
		p.rawNums[TRIP_NUM_pressure_altitude] = 1524;  // m = 5000 ft
		p.rawNums[TRIP_NUM_ambient_temperature] = 30;
		p.rawNums[TRIP_NUM_ambient_pressure] = 24.9;
		p.rawNums[TRIP_NUM_kohlsman_setting_hg] = 29.8;
		p.rawNums[TRIP_NUM_kohlsman_setting_mb] = 1009;
		p.rawNums[TRIP_NUM_ambient_visibility] = 8000;
		p.rawNums[TRIP_NUM_surface_type] = 4;
		p.rawNums[TRIP_NUM_surface_condition] = 1;
		dataset.points = { p };
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		analyzeContacts(dataset);

		const ContactConditions& c = dataset.touchdowns[0].conditions;
		QVERIFY(c.fromSample);
		QCOMPARE(c.headingTrue, 2.0);
		QCOMPARE(c.headingMagnetic, 358.0);
		QCOMPARE(c.trackTrue, 6.0);
		QCOMPARE(c.magneticVariation, 4.0);  // across north: east, not -356
		QCOMPARE(c.crabAngle, -4.0);         // nose left of the track
		QVERIFY(qAbs(c.pressureAltitudeFt - 5000) < 0.01);
		QVERIFY(qAbs(c.densityAltitudeFt - 7958.12) < 0.01);
		QCOMPARE(c.temperatureC, 30.0);
		QCOMPARE(c.stationPressureHg, 24.9);
		QCOMPARE(c.altimeterHg, 29.8);
		QCOMPARE(c.altimeterMb, 1009.0);
		QCOMPARE(c.visibilityM, 8000.0);
		QCOMPARE(c.fieldElevationFt, 1000.0);
		QCOMPARE(c.surfaceType, QStringLiteral("asphalt"));
		QCOMPARE(c.surfaceCondition, QStringLiteral("wet"));
		QVERIFY(qAbs(c.sunElevation - 66.56) < 0.1);  // noon on the equator at the solstice
	}

	// The sun and every distance are taken at the contact's own position:
	// 12:00 UTC is midnight at longitude 180, where at 30°N at the solstice
	// the sun is 90 - 30 - 23.44 below the horizon (with latitude and
	// longitude swapped it would be about 52.6 below).
	void theContactsPositionPlacesTheSunAndTheDistances() {
		TripDataset dataset = touchdownAfter({ 100, 0 });
		for (TripSamplePoint& p : dataset.points) {
			p.latitude = 30;
			p.longitude = 180;
		}
		dataset.points[0].latitude = 30 + 1.0 / 60;  // 1 arcminute north: 1.0007 nm
		dataset.touchdowns[0].latitude = 30;
		dataset.touchdowns[0].longitude = 180;
		analyzeContacts(dataset);
		QVERIFY(qAbs(dataset.touchdowns[0].conditions.sunElevation - -36.56) < 0.1);

		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		QCOMPARE(profile->series.size(), size_t(2));
		QVERIFY2(qAbs(profile->series[0].distanceNm - 1.0) < 0.01, qPrintable(QString::number(profile->series[0].distanceNm)));
		QVERIFY2(profile->series[1].distanceNm < 0.01, qPrintable(QString::number(profile->series[1].distanceNm)));
	}

	// Each gate and series point carries its own sample's flight values.
	void pointsCarryTheirSamplesFlightValues() {
		TripDataset dataset = touchdownAfter({ 50, 0 });
		TripSamplePoint& p = dataset.points[0];
		p.rawNums[TRIP_NUM_airspeed_indicated] = 132;
		p.rawNums[TRIP_NUM_vertical_speed] = -700;
		p.pitchDegrees = 3.5;
		p.bankDegrees = -2;
		p.rawNums[TRIP_NUM_plane_heading_degrees_true] = 271;
		p.rawNums[TRIP_NUM_gps_ground_true_track] = 268;
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);

		const ProfilePoint* fifty = gate(*profile, QStringLiteral("50 ft"));
		QVERIFY(fifty);
		for (const ProfilePoint* point : { fifty, &profile->series.at(0) }) {
			QCOMPARE(point->airspeed, 132.0);
			QCOMPARE(point->verticalSpeed, -700.0);
			QCOMPARE(point->pitch, 3.5);
			QCOMPARE(point->bank, -2.0);
			QCOMPARE(point->headingTrue, 271.0);
			QCOMPARE(point->trackTrue, 268.0);
		}
		QVERIFY(std::isnan(profile->series.at(1).airspeed));  // the touchdown sample's, not recorded
		// Recorded before VELOCITY WORLD Y: the V/S above is the indicated one.
		QVERIFY(profile->indicatedVerticalSpeed);
	}

	// With VELOCITY WORLD Y recorded, the points' V/S is the actual one.
	void pointsShowTheActualVerticalSpeedWhenRecorded() {
		TripDataset dataset = touchdownAfter({ 50, 0 });
		dataset.points[0].rawNums[TRIP_NUM_vertical_speed] = -700;
		dataset.points[0].rawNums[TRIP_NUM_velocity_world_y] = -760;
		dataset.points[1].rawNums[TRIP_NUM_vertical_speed] = -650;
		dataset.points[1].rawNums[TRIP_NUM_velocity_world_y] = -20;
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		QCOMPARE(gate(*profile, QStringLiteral("50 ft"))->verticalSpeed, -760.0);
		QCOMPARE(profile->series.at(0).verticalSpeed, -760.0);
		QCOMPARE(profile->series.at(1).verticalSpeed, -20.0);
		QVERIFY(!profile->indicatedVerticalSpeed);
	}

	// PLANE TOUCHDOWN NORMAL VELOCITY holds the previous touchdown's value
	// until the simulator sets the new one, which can be a sample after the
	// touchdown's: the rate is the first value that differs from the
	// sample's before the touchdown.
	void touchdownRateIsTheFirstNewValueAfterTheTouchdown() {
		TripDataset dataset = touchdownAfter({ 20, 10, 0 });
		dataset.points.push_back(sample(1, 1000, 0));
		dataset.points.push_back(sample(2, 1000, 0));
		const double rates[] = { 198, 198, 198, 262, 262 };
		for (size_t i = 0; i < dataset.points.size(); ++i)
			dataset.points[i].rawNums[TRIP_NUM_plane_touchdown_normal_velocity] = rates[i];
		analyzeContacts(dataset);
		QCOMPARE(dataset.touchdowns[0].touchdownRate, 262.0);

		// Set in the touchdown's own sample.
		dataset.points[2].rawNums[TRIP_NUM_plane_touchdown_normal_velocity] = 305;
		analyzeContacts(dataset);
		QCOMPARE(dataset.touchdowns[0].touchdownRate, 305.0);

		// A sample without the value is skipped, not taken as the new one.
		dataset.points[2].rawNums[TRIP_NUM_plane_touchdown_normal_velocity] = std::nan("");
		analyzeContacts(dataset);
		QCOMPARE(dataset.touchdowns[0].touchdownRate, 262.0);
	}

	// Unknown when nothing changed, or with no sample before the touchdown to
	// tell the previous value.
	void touchdownRateUnknownWithoutANewValue() {
		TripDataset dataset = touchdownAfter({ 10, 0 });
		dataset.points.push_back(sample(1, 1000, 0));
		for (TripSamplePoint& p : dataset.points)
			p.rawNums[TRIP_NUM_plane_touchdown_normal_velocity] = 198;
		analyzeContacts(dataset);
		QVERIFY(std::isnan(dataset.touchdowns[0].touchdownRate));

		dataset.points.erase(dataset.points.begin());
		dataset.points[1].rawNums[TRIP_NUM_plane_touchdown_normal_velocity] = 262;
		analyzeContacts(dataset);
		QVERIFY(std::isnan(dataset.touchdowns[0].touchdownRate));
	}

	// The search ends at the next contact's sample: a later touchdown's value
	// isn't taken for this one's.
	void touchdownRateStopsAtTheNextContact() {
		TripDataset dataset;
		const double rates[] = { 198, 198, 262, 262, 300 };
		for (int i = 0; i < 5; ++i) {
			dataset.points.push_back(sample(i - 1, 1000, 0));
			dataset.points.back().rawNums[TRIP_NUM_plane_touchdown_normal_velocity] = rates[i];
		}
		dataset.touchdowns = { contactAt<TouchdownPoint>(0), contactAt<TouchdownPoint>(3) };
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(0.6) };
		analyzeContacts(dataset);
		QCOMPARE(dataset.touchdowns[0].touchdownRate, 262.0);  // set in the liftoff's sample
		QCOMPARE(dataset.touchdowns[1].touchdownRate, 300.0);

		dataset.points[2].rawNums[TRIP_NUM_plane_touchdown_normal_velocity] = 198;
		dataset.points[3].rawNums[TRIP_NUM_plane_touchdown_normal_velocity] = 198;
		analyzeContacts(dataset);
		QVERIFY(std::isnan(dataset.touchdowns[0].touchdownRate));
		QCOMPARE(dataset.touchdowns[1].touchdownRate, 300.0);
	}

	// The wind range covers the 30 s up to the contact (30 s before it still
	// in, 31 s out), its directions one range across north.
	void windRangeCoversTheLast30Seconds() {
		struct Wind { double seconds, direction, speed; };
		auto windsBefore = [](const std::vector<Wind>& winds) {
			TripDataset dataset;
			for (const Wind& w : winds) {
				TripSamplePoint p = sample(w.seconds, 1000, 0);
				p.rawNums[TRIP_NUM_ambient_wind_direction] = w.direction;
				p.rawNums[TRIP_NUM_ambient_wind_velocity] = w.speed;
				dataset.points.push_back(p);
			}
			dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
			analyzeContacts(dataset);
			return dataset.touchdowns[0].conditions;
		};

		const ContactConditions c = windsBefore({ { -31, 90, 30 }, { -30, 350, 8 }, { -10, 20, 14 }, { 0, 0, 10 } });
		QCOMPARE(c.windMinKt, 8.0);
		QCOMPARE(c.windMaxKt, 14.0);
		QCOMPARE(c.windDirectionFrom, 350.0);
		QCOMPARE(c.windDirectionTo, 20.0);

		// The range's end past north, from the contact's 350.
		const ContactConditions past = windsBefore({ { -10, 10, 12 }, { 0, 350, 10 } });
		QCOMPARE(past.windDirectionFrom, 350.0);
		QCOMPARE(past.windDirectionTo, 10.0);
	}

	// Each gate is its last crossing before the touchdown: the climb back to
	// 700 ft (a go-around) makes the 500 ft gate the second descent's.
	void touchdownGatesAreTheLastCrossing() {
		TripDataset dataset = touchdownAfter({ 600, 400, 700, 550, 450, 250, 150, 80, 40, 0 });
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);

		// No 1000 ft gate (never that high) and no distance gates (never 1 nm away).
		QCOMPARE(gateNames(*profile), QStringList({ "500 ft", "300 ft", "200 ft", "100 ft", "50 ft" }));
		const double heights[] = { 550, 450, 250, 150, 80 };
		const double seconds[] = { -6, -5, -4, -3, -2 };
		for (int i = 0; i < 5; ++i) {
			QCOMPARE(profile->gates[i].heightFt, heights[i]);
			QCOMPARE(profile->gates[i].seconds, seconds[i]);
		}
	}

	// A touchdown's gates and terrain start at the previous contact (a
	// touch-and-go's liftoff), so the circuit before it doesn't count; its
	// series still covers the 15 s before it, the liftoff included.
	void touchdownWindowStartsAtThePreviousContact() {
		TripDataset dataset = touchdownAfter({ 600, 0, 300, 100, 40, 0 });
		dataset.points[0].rawNums[TRIP_NUM_plane_alt_above_ground] = 100;  // ground +500 ft, before the liftoff
		dataset.points[2].rawNums[TRIP_NUM_plane_alt_above_ground] = 200;  // ground +100 ft, after it
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(-4) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);

		QCOMPARE(gateNames(*profile), QStringList({ "300 ft", "200 ft", "100 ft", "50 ft" }));
		QCOMPARE(profile->series.size(), size_t(6));
		QCOMPARE(profile->series.front().seconds, -5.0);
		QCOMPARE(profile->series.front().heightFt, 600.0);
		QCOMPARE(profile->series.back().seconds, 0.0);
		QCOMPARE(profile->highestTerrainFt, 100.0);
	}

	// A liftoff's gates are each height's first crossing after it, measured
	// from the last on-ground sample: the dip back to 30 ft doesn't move the
	// 35 ft gate.
	void liftoffGatesAreTheFirstCrossing() {
		TripDataset dataset;
		const double heights[] = { 0, 20, 60, 30, 150, 250, 600, 1200 };
		for (int i = 0; i < 8; ++i)
			dataset.points.push_back(sample(i, 500 + heights[i], heights[i]));
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(0.5) };
		analyzeContacts(dataset);
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);

		QCOMPARE(gateNames(*profile), QStringList({ "35 ft", "100 ft", "200 ft", "500 ft", "1000 ft" }));
		const double gateHeights[] = { 60, 150, 250, 600, 1200 };
		const double gateSeconds[] = { 1.5, 3.5, 4.5, 5.5, 6.5 };
		for (int i = 0; i < 5; ++i) {
			QCOMPARE(profile->gates[i].heightFt, gateHeights[i]);
			QCOMPARE(profile->gates[i].seconds, gateSeconds[i]);
		}
		// The series runs from the on-ground sample through the climb-out.
		QCOMPARE(profile->series.size(), size_t(8));
		QCOMPARE(profile->series.front().seconds, -0.5);
		QCOMPARE(profile->series.front().heightFt, 0.0);
		QCOMPARE(dataset.liftoffPoints[0].conditions.fieldElevationFt, 500.0);
	}

	// On a downhill takeoff roll the runway behind is higher than the liftoff
	// point: the gates are still each height's first crossing after the
	// liftoff, a sample right at a gate's height crosses it, and the field
	// elevation is the liftoff point's, not the ground under the first
	// airborne sample (10 ft lower here).
	void liftoffGatesStartAtTheLiftoff() {
		TripDataset dataset;
		dataset.points = { sample(0, 560, 0), sample(1, 530, 0), sample(2, 500, 0), sample(3, 560, 70),
			sample(4, 600, 110), sample(5, 700, 210) };
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(2.5) };
		analyzeContacts(dataset);
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);

		QCOMPARE(gateNames(*profile), QStringList({ "35 ft", "100 ft", "200 ft" }));
		const double gateSeconds[] = { 0.5, 1.5, 2.5 };
		for (int i = 0; i < 3; ++i)
			QCOMPARE(profile->gates[i].seconds, gateSeconds[i]);
		QCOMPARE(dataset.liftoffPoints[0].conditions.fieldElevationFt, 500.0);
	}

	// A liftoff's gates end at the next contact in time, though the trip
	// lists its liftoffs before its touchdowns; so does its terrain. Its
	// series runs on through the 10 s after it: the touchdown and the rest.
	void liftoffWindowEndsAtTheNextContact() {
		TripDataset dataset;
		const double heights[] = { 0, 60, 0, 150, 0 };
		for (int i = 0; i < 5; ++i)
			dataset.points.push_back(sample(i, heights[i], heights[i]));
		dataset.points[1].rawNums[TRIP_NUM_plane_alt_above_ground] = 40;  // ground +20 ft, before the touchdown
		dataset.points[3].rawNums[TRIP_NUM_plane_alt_above_ground] = 50;  // ground +100 ft, after it
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(0.5), contactAt<LiftoffPoint>(2.5) };
		dataset.touchdowns = { contactAt<TouchdownPoint>(2), contactAt<TouchdownPoint>(4) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);

		QCOMPARE(gateNames(*profile), QStringList({ "35 ft" }));
		QCOMPARE(profile->series.size(), size_t(5));
		QCOMPARE(profile->series.back().seconds, 3.5);
		QCOMPARE(profile->series.back().heightFt, 0.0);
		QCOMPARE(profile->highestTerrainFt, 20.0);
	}

	// A touch-and-go's liftoff series reaches back past its touchdown, less
	// than 10 s before, to the approach.
	void liftoffSeriesReachesPastThePreviousContact() {
		TripDataset dataset;
		const double heights[] = { 50, 20, 0, 0, 0, 60, 150 };
		for (int i = 0; i < 7; ++i)
			dataset.points.push_back(sample(i, 500 + heights[i], heights[i]));
		dataset.touchdowns = { contactAt<TouchdownPoint>(2) };
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(4.5) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);

		QCOMPARE(profile->series.size(), size_t(7));
		QCOMPARE(profile->series.front().seconds, -4.5);
		QCOMPARE(profile->series.front().heightFt, 50.0);
	}

	// A liftoff's series starts at its on-ground sample, the 0 ft its heights
	// are measured from, even when a gap in the recording put that more than
	// 10 s before.
	void liftoffSeriesKeepsItsOnGroundSample() {
		TripDataset dataset;
		dataset.points = { sample(0, 500, 0), sample(20, 560, 60), sample(21, 650, 150) };
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(19.5) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);

		QCOMPARE(profile->series.size(), size_t(3));
		QCOMPARE(profile->series.front().seconds, -19.5);
		QCOMPARE(profile->series.front().heightFt, 0.0);
	}

	// A bounce whose touchdown, liftoff and next touchdown all fall between
	// two samples: the liftoff's gate window is just the sample after them,
	// so it has no gates and no climb-out to find terrain under; its series
	// still has every sample of its 10 s each side, measured from that one.
	void bounceWithinOneSampleHasNoGates() {
		TripDataset dataset;
		const double heights[] = { 40, 5, 3, 0 };
		for (int i = 0; i < 4; ++i)
			dataset.points.push_back(sample(i, 500 + heights[i], heights[i]));
		dataset.touchdowns = { contactAt<TouchdownPoint>(1.3), contactAt<TouchdownPoint>(1.8) };
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(1.6) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);

		QVERIFY(profile->gates.empty());
		QCOMPARE(profile->series.size(), size_t(4));
		QCOMPARE(profile->series.front().seconds, -1.6);
		QCOMPARE(profile->series.front().heightFt, 37.0);
		QCOMPARE(profile->series.back().seconds, 1.4);
		QVERIFY(std::isnan(profile->highestTerrainFt));
	}

	// A bounce like trip 25's (a jet airborne again for 3.5 s, samples 0.5 s
	// apart): the first touchdown's series runs on through the bounce and the
	// second touchdown to 10 s after it, the on-ground state and height
	// showing the bounce; the second's reaches back 15 s past the liftoff and
	// the first touchdown, but its gates stop at that liftoff (no height
	// gate, the bounce being 8 ft high).
	void bounceShowsInBothTouchdownsSeries() {
		TripDataset dataset;
		// Samples every 0.5 s from -20 s to +12 s; the first touchdown at 0,
		// airborne from 1.5 to 4.5 s (peaking at 8 ft at 3 s), the second
		// touchdown at 5 s.
		for (int i = -40; i <= 24; ++i) {
			const double t = i / 2.0;
			const double height = t < 0 ? -t * 15 : t >= 1.5 && t <= 4.5 ? 8 - qAbs(t - 3) * 4 : 0;
			TripSamplePoint p = sample(t, 1000 + height, height);
			if (!(t >= 1.5 && t <= 4.5) && t >= 0)
				p.boolGroups[TripBoolBits::sim_on_ground.group] |= 1u << TripBoolBits::sim_on_ground.bit;
			dataset.points.push_back(p);
		}
		dataset.touchdowns = { contactAt<TouchdownPoint>(0), contactAt<TouchdownPoint>(5) };
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(1.2) };

		const std::optional<ContactProfile> first = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(first);
		QCOMPARE(first->series.size(), size_t(51));  // -15 s to +10 s
		QCOMPARE(first->series.front().seconds, -15.0);
		QCOMPARE(first->series.back().seconds, 10.0);
		QStringList states;
		for (const ProfilePoint& p : first->series)
			if (p.seconds >= 0 && p.seconds <= 5.5)
				states.append(QStringLiteral("%1%2").arg(p.onGround ? "G" : "A").arg(p.heightFt));
		QCOMPARE(states, QStringList({ "G0", "G0", "G0", "A2", "A4", "A6", "A8", "A6", "A4", "A2", "G0", "G0" }));
		QCOMPARE(gateNames(*first), QStringList({ "300 ft", "200 ft", "100 ft", "50 ft" }));

		const std::optional<ContactProfile> second = contactProfile(dataset, dataset.touchdowns[1], true);
		QVERIFY(second);
		QCOMPARE(second->series.front().seconds, -15.0);
		QCOMPARE(second->series.front().heightFt, 150.0);  // the approach, at -10 s from the first
		QCOMPARE(second->series.back().seconds, 7.0);       // the recording's end
		QVERIFY(second->gates.empty());
	}

	// An old trip recorded a bounce with no airborne sample as two
	// touchdowns 0.3 s apart and no liftoff: each one's series covers the
	// other's.
	void touchdownsCloserThanASampleShareTheirSeries() {
		TripDataset dataset;
		for (int i = -20; i <= 12; ++i)
			dataset.points.push_back(sample(i, 1000 + (i < 0 ? -i * 10 : 0), i < 0 ? -i * 10 : 0));
		dataset.touchdowns = { contactAt<TouchdownPoint>(0), contactAt<TouchdownPoint>(0.3) };

		const std::optional<ContactProfile> first = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(first);
		QCOMPARE(first->series.front().seconds, -15.0);
		QCOMPARE(first->series.back().seconds, 10.0);
		const std::optional<ContactProfile> second = contactProfile(dataset, dataset.touchdowns[1], true);
		QVERIFY(second);
		QCOMPARE(second->series.front().seconds, -14.3);
		QCOMPARE(second->series.back().seconds, 9.7);
	}

	// Each point carries the on-ground state and, per engine, the throttle
	// lever (negative in the reverse range) and the power value for the
	// contact sample's ENGINE TYPE: N1 (%) for a jet, torque (%) for a
	// turboprop or helicopter turbine, manifold pressure (inHg) for a piston,
	// none for another type. An engine without its value stays unknown.
	void pointsCarryOnGroundAndEngineValues() {
		auto withEngines = [](int engineType) {
			TripDataset dataset = touchdownAfter({ 50, 0 });
			for (TripSamplePoint& p : dataset.points) {
				p.engineType = engineType;
				p.engineValues.assign(2 * TRIP_ENGINE_FIELD_COUNT, std::nan(""));
				auto set = [&](int engine, TripEngineField field, double value) {
					p.engineValues[(size_t)(engine - 1) * TRIP_ENGINE_FIELD_COUNT + field] = value;
				};
				set(1, TRIP_ENGINE_general_eng_throttle_lever_position, -4);
				set(1, TRIP_ENGINE_turb_eng_n1, 53);
				set(1, TRIP_ENGINE_turb_eng_max_torque_percent, 20);
				set(1, TRIP_ENGINE_general_eng_rpm, 1800);
				set(1, TRIP_ENGINE_recip_eng_manifold_pressure, 15);
			}
			dataset.points[1].boolGroups[TripBoolBits::sim_on_ground.group] |= 1u << TripBoolBits::sim_on_ground.bit;
			const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
			return *profile;
		};

		const ContactProfile jet = withEngines(1);
		QCOMPARE(jet.powerName, QStringLiteral("N1"));
		QCOMPARE(jet.powerUnit, QStringLiteral("%"));
		QCOMPARE(jet.series.size(), size_t(2));
		QVERIFY(!jet.series[0].onGround);
		QVERIFY(jet.series[1].onGround);
		const ProfilePoint& p = jet.series[1];
		QCOMPARE(p.throttle.size(), size_t(2));
		QCOMPARE(p.throttle[0], -4.0);
		QVERIFY(std::isnan(p.throttle[1]));
		QCOMPARE(p.power.size(), size_t(2));
		QCOMPARE(p.power[0], 53.0);
		QVERIFY(std::isnan(p.power[1]));
		QCOMPARE(gate(jet, QStringLiteral("50 ft"))->power.at(0), 53.0);

		const ContactProfile turboprop = withEngines(5);
		QCOMPARE(turboprop.powerName, QStringLiteral("torque"));
		QCOMPARE(turboprop.powerUnit, QStringLiteral("%"));
		QCOMPARE(turboprop.series[1].power.at(0), 20.0);

		const ContactProfile helicopter = withEngines(3);
		QCOMPARE(helicopter.powerName, QStringLiteral("torque"));
		QCOMPARE(helicopter.powerUnit, QStringLiteral("%"));
		QCOMPARE(helicopter.series[1].power.at(0), 20.0);

		const ContactProfile piston = withEngines(0);
		QCOMPARE(piston.powerName, QStringLiteral("manifold pressure"));
		QCOMPARE(piston.powerUnit, QStringLiteral(" inHg"));
		QCOMPARE(piston.series[1].power.at(0), 15.0);

		const ContactProfile none = withEngines(2);  // ENGINE TYPE 2: none
		QVERIFY(none.powerName.isEmpty());
		QVERIFY(none.powerUnit.isEmpty());
		QVERIFY(none.series[1].power.empty());
		QCOMPARE(none.series[1].throttle.at(0), -4.0);
	}

	// A liftoff's series runs from 10 s before it to 10 s after, both ends
	// included: of samples 1 s apart, -9.5 s to +9.5 s for a liftoff between
	// two (the next ones out, -10.5 and +10.5, left out), -10 s to +10 s for
	// one on a sample.
	void liftoffSeriesCoversTenSecondsEachSide() {
		TripDataset dataset;
		for (int i = 0; i <= 30; ++i) {
			const double height = i <= 15 ? 0 : (i - 15) * 50;
			dataset.points.push_back(sample(i, 500 + height, height));
		}
		const struct { double liftoff; size_t count; double end; } cases[] = { { 15.5, 20, 9.5 }, { 15, 21, 10 } };
		for (const auto& c : cases) {
			dataset.liftoffPoints = { contactAt<LiftoffPoint>(c.liftoff) };
			const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
			QVERIFY(profile);

			QCOMPARE(profile->series.size(), c.count);
			QCOMPARE(profile->series.front().seconds, -c.end);
			QCOMPARE(profile->series.back().seconds, c.end);
		}
	}

	// A touchdown's gates are crossings before it: a sample after it (a gap in
	// the recording put it 1.5 nm down the runway, 30 s later) is never one.
	void touchdownGatesComeBeforeTheContact() {
		TripDataset dataset;
		dataset.points = { sample(-10, 1500, 500, -1.5), sample(30, 1000, 0, 1.5) };
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);

		const ProfilePoint* oneNm = gate(*profile, QStringLiteral("1 nm"));
		QVERIFY(oneNm);
		QCOMPARE(oneNm->seconds, -10.0);
		for (const ProfilePoint& p : profile->gates)
			QVERIFY(p.seconds < 0);
	}

	// The touchdown series runs from 15 s before it to 10 s after, both ends
	// included (-16 and +11 s left out); when the recording stops sooner
	// after, at its last sample.
	void touchdownSeriesCovers15SecondsBeforeTo10After() {
		std::vector<double> heights;
		for (int i = 20; i >= 0; --i)
			heights.push_back(i * 10);
		TripDataset dataset = touchdownAfter(heights);  // samples at -20..0 s
		for (int i = 1; i <= 12; ++i)
			dataset.points.push_back(sample(i, 1000, 0));
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		QCOMPARE(profile->series.size(), size_t(26));
		QCOMPARE(profile->series.front().seconds, -15.0);
		QCOMPARE(profile->series.back().seconds, 10.0);

		dataset.points.resize(dataset.points.size() - 9);  // the recording ends at +3 s
		const std::optional<ContactProfile> cut = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(cut);
		QCOMPARE(cut->series.size(), size_t(19));
		QCOMPARE(cut->series.back().seconds, 3.0);
	}

	// A touchdown after the trip's last sample (the recording stopped first)
	// is measured from that sample, as sampleAtOrAfter() gives: its
	// conditions and a trajectory ending before it.
	void contactAfterTheLastSampleUsesThatSample() {
		TripDataset dataset;
		dataset.points = { sample(-2, 1100, 100), sample(-1, 1050, 50) };
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		QCOMPARE(profile->series.size(), size_t(2));
		QCOMPARE(profile->series.front().seconds, -2.0);
		QCOMPARE(profile->series.back().seconds, -1.0);
		analyzeContacts(dataset);
		QVERIFY(dataset.touchdowns[0].conditions.fromSample);
	}

	// However dense the samples, the series is at most 60 lines and keeps
	// both ends.
	void seriesIsCappedAt60() {
		TripDataset dataset;
		for (int i = 119; i >= 0; --i)
			dataset.points.push_back(sample(-i / 10.0, 1000 + i, i));
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		const std::vector<ProfilePoint>& series = profile->series;

		QVERIFY2(series.size() <= 60, qPrintable(QString::number(series.size())));
		QCOMPARE(series.front().seconds, -11.9);
		QCOMPARE(series.back().seconds, 0.0);
	}

	// Thinned, the series still has the contact's own sample and the
	// on-ground one its heights are measured from, both mid-series and off
	// the stride: samples 0.17 s apart give a touchdown's 25 s 147 samples,
	// every third kept, the touchdown 88 in; a liftoff at 0.1 s has 118, its
	// on-ground sample (0 s) 58 in and its own (0.17 s) 59 in.
	void seriesKeepsTheContactAndOnGroundSamples() {
		auto hasPointAt = [](const std::vector<ProfilePoint>& series, double seconds) {
			return std::any_of(series.begin(), series.end(), [&](const ProfilePoint& p) { return std::abs(p.seconds - seconds) < 1e-6; });
		};
		TripDataset dataset;
		for (int k = -117; k <= 70; ++k)
			dataset.points.push_back(sample(k * 0.17, 1000 + std::abs(k) * 10, std::abs(k) * 10));
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		QVERIFY2(profile->series.size() <= 60, qPrintable(QString::number(profile->series.size())));
		QVERIFY(hasPointAt(profile->series, 0.0));

		dataset.touchdowns.clear();
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(0.1) };
		profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);
		QVERIFY2(profile->series.size() <= 60, qPrintable(QString::number(profile->series.size())));
		QVERIFY(hasPointAt(profile->series, -0.1));  // the on-ground sample at 0 s
		QVERIFY(hasPointAt(profile->series, 0.07));  // the liftoff's, at 0.17 s
	}

	// Over a 400 ft cliff edge the radio height drops while the descent stays
	// smooth: the gates follow the height above the touchdown, not the
	// ground below, and the cliff shows as terrain.
	void gatesIgnoreTheTerrainBelow() {
		TripDataset dataset;
		// Field at 100 ft. Terrain +400 ft for the first two samples, then
		// down to the field's level.
		dataset.points = { sample(-4, 900, 400, 1.0), sample(-3, 700, 200, 0.75), sample(-2, 550, 450, 0.5),
			sample(-1, 300, 200, 0.25), sample(0, 100, 0) };
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);

		const ProfilePoint* fiveHundred = gate(*profile, QStringLiteral("500 ft"));
		QVERIFY(fiveHundred);
		QCOMPARE(fiveHundred->seconds, -3.0);  // 600 ft above the touchdown, 200 ft above the ground
		QCOMPARE(fiveHundred->heightFt, 600.0);
		QCOMPARE(fiveHundred->terrainFt, 400.0);
		QCOMPARE(profile->highestTerrainFt, 400.0);
		QVERIFY(qAbs(profile->highestTerrainNm - 0.75) < 0.01);  // the closer of the two cliff-top samples
	}

	// A liftoff's highest terrain is ahead of it, under the climb-out, and
	// measured from the on-ground sample before it, not the first airborne one.
	void liftoffHighestTerrainIsUnderTheClimbOut() {
		TripDataset dataset;
		// Field at 500 ft; the ground under the first airborne sample at 520
		// ft, a 300 ft rise 1 nm out, then lower ground.
		dataset.points = { sample(0, 500, 0), sample(1, 700, 180, 0.5), sample(2, 1100, 300, 1.0),
			sample(3, 1300, 700, 1.5) };
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(0.5) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);

		QCOMPARE(profile->highestTerrainFt, 300.0);
		QVERIFY(qAbs(profile->highestTerrainNm - 1.0) < 0.01);
	}

	// Highest terrain is searched only within 5 nm of the contact, stopping
	// at the first sample beyond: a hill farther out, or one the path passes
	// back within 5 nm of before that, isn't reported.
	void highestTerrainStopsBeyond5Nm() {
		TripDataset dataset;
		// Field at 100 ft. Walking back from the touchdown: level, +200 ft at
		// 4.9 nm, +900 ft at 5.5 nm, then +800 ft back at 3 nm.
		dataset.points = { sample(-4, 1300, 400, 3.0), sample(-3, 1500, 500, 5.5), sample(-2, 1000, 700, 4.9),
			sample(-1, 500, 400, 2.0), sample(0, 100, 0) };
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);

		QCOMPARE(profile->highestTerrainFt, 200.0);
		QVERIFY(qAbs(profile->highestTerrainNm - 4.9) < 0.01);
	}

	// On an exact 3-degree path (318.4 ft per nm) to the touchdown point, each
	// distance gate's path angle is 3.0, the closest's too although the
	// touchdown's sample (the first after it) has rolled 0.1 nm on.
	void distanceGatesGiveThePathAngle() {
		TripDataset dataset;
		const double feetPerNm = 6076.12 * std::tan(3 * std::acos(-1.0) / 180);
		for (int i = 7; i >= 1; --i) {
			const double nm = i * 0.5;
			dataset.points.push_back(sample(-i * 10.0, 100 + nm * feetPerNm, nm * feetPerNm, nm));
		}
		dataset.points.push_back(sample(1, 100, 0, -0.1));
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);

		// The distance gates fall between the height gates, all in time order.
		for (size_t i = 1; i < profile->gates.size(); ++i)
			QVERIFY2(profile->gates[i - 1].seconds <= profile->gates[i].seconds, qPrintable(gateNames(*profile).join('|')));
		for (int nm = 1; nm <= 3; ++nm) {
			const ProfilePoint* g = gate(*profile, QStringLiteral("%1 nm").arg(nm));
			QVERIFY2(g, qPrintable(gateNames(*profile).join('|')));
			QVERIFY2(qAbs(g->distanceNm - nm) < 0.01, qPrintable(QString::number(g->distanceNm)));
			QVERIFY2(qAbs(g->pathAngle - 3.0) < 0.01, qPrintable(QString::number(g->pathAngle)));
		}
		// Height gates have no path angle.
		QVERIFY(std::isnan(gate(*profile, QStringLiteral("500 ft"))->pathAngle));
	}

	// With the contact's runway end stored, each point is placed along and
	// across the runway: from the marked threshold for a touchdown, from the
	// physical start for a liftoff. One arcminute on the 6371 km sphere is
	// 6080.2 ft; 0.001 degree of longitude at the equator 364.8 ft.
	void pointsArePlacedAlongAndAcrossTheRunway() {
		ContactRunway runway;
		runway.startLatitude = 0.5 / 60;  // the runway's start 0.5 arcminute north of the contact
		runway.startLongitude = 0;
		runway.trueHeading = 180;          // southbound
		runway.thresholdOffsetFt = 1000;

		TripDataset landing;
		TripSamplePoint wide = sample(-10, 1300, 300, 1.5);
		wide.longitude = 0.001;            // east: left of a southbound runway
		landing.points = { wide, sample(0, 1000, 0) };
		landing.touchdowns = { contactAt<TouchdownPoint>(0) };
		landing.touchdowns[0].runwayData = runway;
		const std::optional<ContactProfile> landingProfile = contactProfile(landing, landing.touchdowns[0], true);
		QVERIFY(landingProfile);
		const std::vector<ProfilePoint>& approach = landingProfile->series;
		QCOMPARE(approach.size(), size_t(2));
		// 1 arcminute before the start, 1000 ft more before the marked threshold.
		QVERIFY2(qAbs(approach[0].thresholdFt - -7080.2) < 1, qPrintable(QString::number(approach[0].thresholdFt)));
		QVERIFY2(qAbs(approach[0].centerlineFt - -364.8) < 0.5, qPrintable(QString::number(approach[0].centerlineFt)));
		QVERIFY2(qAbs(approach[1].thresholdFt - 2040.1) < 1, qPrintable(QString::number(approach[1].thresholdFt)));
		QVERIFY2(qAbs(approach[1].centerlineFt) < 0.01, qPrintable(QString::number(approach[1].centerlineFt)));

		TripDataset departure;
		departure.points = { sample(0, 1000, 0), sample(1, 1060, 60, -0.5) };
		departure.liftoffPoints = { contactAt<LiftoffPoint>(0.5) };
		departure.liftoffPoints[0].runwayData = runway;
		const std::optional<ContactProfile> departureProfile = contactProfile(departure, departure.liftoffPoints[0], false);
		QVERIFY(departureProfile);
		const ProfilePoint* climb = gate(*departureProfile, QStringLiteral("35 ft"));
		QVERIFY(climb);
		QVERIFY2(qAbs(climb->thresholdFt - 6080.2) < 1, qPrintable(QString::number(climb->thresholdFt)));
	}

	// Every trajectory point needs its time from the contact: a sample in the
	// trajectory whose time can't be read, or the contact's own, leaves no
	// trajectory (the analysis goes without one) rather than one cut short
	// there or with points it can't place. The conditions are unaffected.
	void unreadableTimeLeavesNoTrajectory() {
		// Minute or second 60: unreadable, yet in order among the samples'
		// times (which sampleAtOrAfter() compares as strings).
		const QString beforeTouchdown = QStringLiteral("2026-06-21T11:59:60.000+00:00_1");
		const QString afterLiftoff = QStringLiteral("2026-06-21T12:00:60.000+00:00_1");

		TripDataset approach = touchdownAfter({ 300, 200, 100, 50, 0 });  // samples at -4..0 s
		TripSamplePoint unreadable = sample(0, 1020, 20);
		unreadable.zuluTime = beforeTouchdown;
		approach.points.insert(approach.points.begin() + 4, unreadable);
		QVERIFY(!contactProfile(approach, approach.touchdowns[0], true));
		analyzeContacts(approach);
		QVERIFY(approach.touchdowns[0].conditions.fromSample);

		// A liftoff's, after it and then before it; the late sample crosses no
		// new gate, so only the series takes it in.
		TripDataset departure;
		departure.points = { sample(0, 500, 0), sample(1, 560, 60), sample(2, 650, 150) };
		TripSamplePoint late = sample(3, 650, 150);
		late.zuluTime = afterLiftoff;
		departure.points.push_back(late);
		departure.liftoffPoints = { contactAt<LiftoffPoint>(0.5) };
		QVERIFY(!contactProfile(departure, departure.liftoffPoints[0], false));
		TripDataset rolling = departure;
		rolling.points.back().zuluTime = sample(3, 0, 0).zuluTime;
		TripSamplePoint early = sample(0, 500, 0);
		early.zuluTime = beforeTouchdown;  // 11:59:60, before the liftoff
		rolling.points.insert(rolling.points.begin(), early);
		QVERIFY(!contactProfile(rolling, rolling.liftoffPoints[0], false));

		// The contact's own: the conditions are its sample's (at 0 s), except
		// the sun, which needs the time, and the wind range, its sample alone.
		TripDataset contact = touchdownAfter({ 50, 0 });
		contact.points[0].rawNums[TRIP_NUM_ambient_wind_velocity] = 30;
		contact.points[1].rawNums[TRIP_NUM_ambient_wind_velocity] = 10;
		contact.touchdowns[0].zuluTime = beforeTouchdown;
		QVERIFY(!contactProfile(contact, contact.touchdowns[0], true));
		analyzeContacts(contact);
		const ContactConditions& c = contact.touchdowns[0].conditions;
		QVERIFY(c.fromSample);
		QVERIFY(std::isnan(c.sunElevation));
		QCOMPARE(c.windMinKt, 10.0);
		QCOMPARE(c.windMaxKt, 10.0);
	}

	// The wind range stops at a sample whose time can't be read: the 30 kt
	// at -40 s is outside it anyway, the 20 kt one is unreadable.
	void unreadableTimeEndsTheWindRange() {
		TripDataset dataset;
		TripSamplePoint unreadable = sample(0, 1000, 0);
		unreadable.zuluTime = QStringLiteral("2026-06-21T11:59:60.000+00:00_1");  // between -40 and 0 s
		dataset.points = { sample(-40, 1000, 0), unreadable, sample(0, 1000, 0) };
		const double speeds[] = { 30, 20, 10 };
		for (size_t i = 0; i < dataset.points.size(); ++i)
			dataset.points[i].rawNums[TRIP_NUM_ambient_wind_velocity] = speeds[i];
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		analyzeContacts(dataset);
		QCOMPARE(dataset.touchdowns[0].conditions.windMinKt, 10.0);
		QCOMPARE(dataset.touchdowns[0].conditions.windMaxKt, 10.0);
	}

	// A sample whose time can't be read outside the trajectory (before its
	// 15 s series, and not a gate) doesn't matter; one that is a gate does.
	void unreadableTimeOutsideTheTrajectoryIsIgnored() {
		std::vector<double> heights;
		for (int i = 20; i >= 0; --i)
			heights.push_back(i * 10);
		TripDataset dataset = touchdownAfter(heights);  // samples at -20..0 s
		TripSamplePoint early = sample(0, 1000, 0);  // on the ground: no gate
		early.zuluTime = QStringLiteral("2026-06-21T11:58:60.000+00:00_1");
		dataset.points.insert(dataset.points.begin(), early);
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		QCOMPARE(profile->series.front().seconds, -15.0);

		// 1100 ft above the touchdown point: the 1000, 500 and 300 ft gates.
		dataset.points.front() = sample(0, 2100, 1100);
		dataset.points.front().zuluTime = early.zuluTime;
		QVERIFY(!contactProfile(dataset, dataset.touchdowns[0], true));
	}

	// Each gate carries its sample's weather, the wind split along and
	// across the runway's true heading: 20 kt from 120° on a 090° runway is
	// 20 cos 30° = 17.32 kt of headwind and 20 sin 30° = 10 kt from the
	// right; 10 kt from 270° is all tailwind, 10 kt from 360° all from the
	// left. Without a runway only the wind itself is known.
	void gatesCarryTheirWeather() {
		TripDataset dataset = touchdownAfter({ 600, 400, 250, 0 });
		auto setWind = [&](int index, double direction, double speed) {
			dataset.points[index].rawNums[TRIP_NUM_ambient_wind_direction] = direction;
			dataset.points[index].rawNums[TRIP_NUM_ambient_wind_velocity] = speed;
		};
		setWind(0, 120, 20);
		setWind(1, 270, 10);
		setWind(2, 360, 10);
		TripSamplePoint& p = dataset.points[0];
		p.rawNums[TRIP_NUM_ambient_temperature] = -5;
		p.rawNums[TRIP_NUM_ambient_precip_state] = 8;
		p.rawNums[TRIP_NUM_structural_ice_pct] = 0.03;
		p.boolGroups[TripBoolBits::ambient_in_cloud.group] = 1u << TripBoolBits::ambient_in_cloud.bit;
		dataset.touchdowns[0].runwayHeading = 90;
		std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);

		const ProfilePoint* g500 = gate(*profile, QStringLiteral("500 ft"));
		QVERIFY(g500);
		QCOMPARE(g500->windDirection, 120.0);
		QCOMPARE(g500->windSpeedKt, 20.0);
		QVERIFY(std::abs(g500->headwindKt - 17.3205) < 1e-4);
		QVERIFY(std::abs(g500->crosswindKt - 10) < 1e-9);
		QCOMPARE(g500->temperatureC, -5.0);
		QCOMPARE(g500->inCloud, 1.0);
		QCOMPARE(g500->precipitation, QStringLiteral("snow"));
		QCOMPARE(g500->structuralIce, 0.03);
		const ProfilePoint* g300 = gate(*profile, QStringLiteral("300 ft"));
		QVERIFY(g300);
		QVERIFY(std::abs(g300->headwindKt + 10) < 1e-9);
		QVERIFY(std::abs(g300->crosswindKt) < 1e-9);
		const ProfilePoint* g200 = gate(*profile, QStringLiteral("200 ft"));
		QVERIFY(g200);
		QVERIFY(std::abs(g200->headwindKt) < 1e-9);
		QVERIFY(std::abs(g200->crosswindKt + 10) < 1e-9);

		dataset.touchdowns[0].runwayHeading = -1;
		profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		g500 = gate(*profile, QStringLiteral("500 ft"));
		QVERIFY(g500);
		QCOMPARE(g500->windSpeedKt, 20.0);
		QVERIFY(std::isnan(g500->headwindKt));
		QVERIFY(std::isnan(g500->crosswindKt));
	}

	// The approach's wind runs from its last crossing of 1000 ft down to the
	// touchdown (the 40 kt at 1200 ft before it doesn't count). On a 360°
	// runway the headwinds are 20, 18, 0 (15 kt from 090°, all from the
	// right), 4 and -6 kt (6 kt from 180°): the largest change is from 20 kt
	// at 1000 ft to the 6 kt tailwind at the touchdown.
	void approachSegmentWind() {
		TripDataset dataset = touchdownAfter({ 1200, 1000, 700, 400, 100, 0 });
		const double winds[][2] = { { 360, 40 }, { 360, 20 }, { 360, 18 }, { 90, 15 }, { 360, 4 }, { 180, 6 } };
		for (int i = 0; i < 6; ++i) {
			dataset.points[i].rawNums[TRIP_NUM_ambient_wind_direction] = winds[i][0];
			dataset.points[i].rawNums[TRIP_NUM_ambient_wind_velocity] = winds[i][1];
		}
		dataset.touchdowns[0].runwayHeading = 360;
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		const SegmentWeather& w = profile->segment;
		QCOMPARE(w.samples, 5);
		QCOMPARE(w.windMinKt, 4.0);
		QCOMPARE(w.windMaxKt, 20.0);
		QCOMPARE(w.headwindFromKt, 20.0);
		QCOMPARE(w.headwindFromHeightFt, 1000.0);
		QCOMPARE(w.headwindToKt, -6.0);
		QCOMPARE(w.headwindToHeightFt, 0.0);
		QCOMPARE(w.maxTailwindKt, 6.0);
		QVERIFY(std::abs(w.maxCrosswindKt - 15) < 1e-9);

		// A headwind that grows toward the ground runs the other way: from
		// its lowest, earlier, to its highest.
		dataset.points[5].rawNums[TRIP_NUM_ambient_wind_direction] = 360;
		dataset.points[5].rawNums[TRIP_NUM_ambient_wind_velocity] = 30;
		dataset.points[3].rawNums[TRIP_NUM_ambient_wind_direction] = 360;
		const SegmentWeather grows = contactProfile(dataset, dataset.touchdowns[0], true)->segment;
		QCOMPARE(grows.headwindFromKt, 4.0);
		QCOMPARE(grows.headwindFromHeightFt, 100.0);
		QCOMPARE(grows.headwindToKt, 30.0);
		QCOMPARE(grows.headwindToHeightFt, 0.0);
		QCOMPARE(grows.maxTailwindKt, -4.0);  // never a tailwind

		// The strongest crosswind is the largest from either side: 18 kt from
		// the left (from 270°) over 15 kt from the right (from 090°).
		dataset.points[2].rawNums[TRIP_NUM_ambient_wind_direction] = 270;
		dataset.points[3].rawNums[TRIP_NUM_ambient_wind_direction] = 90;
		dataset.points[3].rawNums[TRIP_NUM_ambient_wind_velocity] = 15;
		QVERIFY(std::abs(contactProfile(dataset, dataset.touchdowns[0], true)->segment.maxCrosswindKt + 18) < 1e-9);

		// Where the wind holds at an extreme, the change is between the two
		// closest samples: 20, 20, 5, 5 and 5 kt drop from 700 ft to 400 ft;
		// 5, 20, 20, 5 and 5 kt rise from 1000 ft to 700 ft (the first of the
		// two equally close pairs); a steady 7 kt is its first sample at both
		// ends.
		auto change = [&](std::initializer_list<double> headwinds) {
			int i = 1;
			for (double headwind : headwinds) {
				dataset.points[i].rawNums[TRIP_NUM_ambient_wind_direction] = 360;
				dataset.points[i++].rawNums[TRIP_NUM_ambient_wind_velocity] = headwind;
			}
			const SegmentWeather s = contactProfile(dataset, dataset.touchdowns[0], true)->segment;
			return QList<double>({ s.headwindFromKt, s.headwindFromHeightFt, s.headwindToKt, s.headwindToHeightFt });
		};
		QCOMPARE(change({ 20, 20, 5, 5, 5 }), QList<double>({ 20, 700, 5, 400 }));
		QCOMPARE(change({ 5, 20, 20, 5, 5 }), QList<double>({ 5, 1000, 20, 700 }));
		QCOMPARE(change({ 7, 7, 7, 7, 7 }), QList<double>({ 7, 1000, 7, 1000 }));
	}

	// Through the same approach: in cloud down to 700 ft (the 1200 ft
	// sample's cloud is before it), icing conditions at 700 ft (in cloud at
	// +5 °C) though not at 100 ft (dry at 0 °C), the most structural and
	// pitot ice, engine anti-ice on (in any engine) in two of the three
	// samples that recorded it, and the G-force spread of the
	// airborne samples only (not the touchdown's 1.9 G, nor 2.5 G before the
	// segment).
	void approachSegmentCloudIceAndRide() {
		TripDataset dataset = touchdownAfter({ 1200, 1000, 700, 400, 100, 0 });
		const TripBoolBit cloud = TripBoolBits::ambient_in_cloud;
		const double gForces[] = { 2.5, 1.0, 1.3, 0.8, 1.1, 1.9 };
		for (int i = 0; i < 6; ++i) {
			TripSamplePoint& p = dataset.points[i];
			p.rawNums[TRIP_NUM_ambient_precip_state] = 2;
			p.rawNums[TRIP_NUM_g_force] = gForces[i];
			if (i <= 2)
				p.boolGroups[cloud.group] = 1u << cloud.bit;
		}
		dataset.points[0].rawNums[TRIP_NUM_structural_ice_pct] = 0.5;
		dataset.points[2].rawNums[TRIP_NUM_ambient_temperature] = 5;
		dataset.points[2].rawNums[TRIP_NUM_structural_ice_pct] = 0.02;
		dataset.points[3].rawNums[TRIP_NUM_structural_ice_pct] = 0.05;
		dataset.points[4].rawNums[TRIP_NUM_ambient_temperature] = 0;
		dataset.points[3].rawNums[TRIP_NUM_pitot_ice_pct] = 0.04;
		dataset.points[4].rawNums[TRIP_NUM_pitot_ice_pct] = 0.1;
		// The 1000 ft sample's anti-ice is on: engine 1's, though engine 2's is off.
		dataset.points[1].engineValues.assign(2 * TRIP_ENGINE_FIELD_COUNT, std::nan(""));
		dataset.points[1].engineValues[TRIP_ENGINE_eng_anti_ice] = 1;
		dataset.points[1].engineValues[TRIP_ENGINE_FIELD_COUNT + TRIP_ENGINE_eng_anti_ice] = 0;
		dataset.points[2].engineValues.assign(TRIP_ENGINE_FIELD_COUNT, std::nan(""));
		dataset.points[2].engineValues[TRIP_ENGINE_eng_anti_ice] = 0;
		dataset.points[3].engineValues.assign(TRIP_ENGINE_FIELD_COUNT, std::nan(""));
		dataset.points[3].engineValues[TRIP_ENGINE_eng_anti_ice] = 1;
		const TripBoolBit ground = TripBoolBits::sim_on_ground;
		dataset.points[5].boolGroups[ground.group] |= 1u << ground.bit;
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		const SegmentWeather& w = profile->segment;
		QVERIFY(w.cloudKnown);
		QCOMPARE(w.lowestInCloudFt, 700.0);
		QCOMPARE(w.icingConditions, 1.0);
		QCOMPARE(w.maxStructuralIce, 0.05);
		QCOMPARE(w.maxPitotIce, 0.1);
		QCOMPARE(w.engineAntiIceShare, 2.0 / 3);
		QCOMPARE(w.gForceMin, 0.8);
		QCOMPARE(w.gForceMax, 1.3);

		// Rain at -20 °C, snow out of cloud at -5 °C and cloud at +10 °C
		// are icing too; in cloud at +11 °C or -21 °C, or dry at 0 °C, isn't;
		// in cloud without the temperature is unknown; out of cloud throughout is
		// known, with no height.
		auto icing = [&](double temperature, double precipitation, bool inCloud) {
			for (TripSamplePoint& p : dataset.points) {
				p.rawNums[TRIP_NUM_ambient_temperature] = temperature;
				p.rawNums[TRIP_NUM_ambient_precip_state] = precipitation;
				p.boolGroups[cloud.group] = inCloud ? 1u << cloud.bit : 0;
			}
			return contactProfile(dataset, dataset.touchdowns[0], true)->segment.icingConditions;
		};
		QCOMPARE(icing(-20, 4, false), 1.0);
		QCOMPARE(icing(-5, 8, false), 1.0);
		QCOMPARE(icing(10, 2, true), 1.0);
		QCOMPARE(icing(11, 2, true), 0.0);
		QCOMPARE(icing(-21, 2, true), 0.0);
		QVERIFY(std::isnan(icing(std::nan(""), 2, true)));
		QCOMPARE(icing(0, 2, false), 0.0);
		const SegmentWeather clear = contactProfile(dataset, dataset.touchdowns[0], true)->segment;
		QVERIFY(clear.cloudKnown);
		QVERIFY(std::isnan(clear.lowestInCloudFt));
	}

	// The approach's vertical wind range, and its airborne sample closest to
	// the stall: the 700 ft one (9° vs 14°), though the 100 ft one has the
	// highest angle of attack (11° vs 18°). Neither the 1200 ft sample (before
	// the segment), a sample without its stall angle, nor the touchdown's on
	// the ground counts.
	void approachSegmentVerticalWindAndStallMargin() {
		TripDataset dataset = touchdownAfter({ 1200, 1000, 700, 400, 100, 0 });
		const double verticalWinds[] = { -900, -300, 150, -450, 50, std::nan("") };
		const double alphas[] = { 14, 5, 9, 14.9, 11, 16 };
		const double stallAlphas[] = { 15, 15, 14, std::nan(""), 18, 16 };
		for (int i = 0; i < 6; ++i) {
			dataset.points[i].rawNums[TRIP_NUM_ambient_wind_y] = verticalWinds[i];
			dataset.points[i].rawNums[TRIP_NUM_incidence_alpha] = alphas[i];
			dataset.points[i].rawNums[TRIP_NUM_stall_alpha] = stallAlphas[i];
		}
		const TripBoolBit ground = TripBoolBits::sim_on_ground;
		dataset.points[5].boolGroups[ground.group] |= 1u << ground.bit;
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		const SegmentWeather& w = profile->segment;
		QCOMPARE(w.verticalWindMinFpm, -450.0);
		QCOMPARE(w.verticalWindMaxFpm, 150.0);
		QCOMPARE(w.stallMarginAlphaDeg, 9.0);
		QCOMPARE(w.stallMarginStallAlphaDeg, 14.0);
		QCOMPARE(w.stallMarginHeightFt, 700.0);
	}

	// An approach never 1000 ft high starts at the previous contact (a
	// touch-and-go's liftoff at -4 s): its 50 kt before that doesn't count.
	void approachSegmentStartsAtThePreviousContact() {
		TripDataset dataset = touchdownAfter({ 600, 0, 300, 100, 40, 0 });
		for (int i = 0; i < 6; ++i)
			dataset.points[i].rawNums[TRIP_NUM_ambient_wind_velocity] = i == 0 ? 50 : 10 + i;
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(-4) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		QCOMPARE(profile->segment.samples, 5);
		QCOMPARE(profile->segment.windMinKt, 11.0);
		QCOMPARE(profile->segment.windMaxKt, 15.0);
	}

	// A climb-out runs from the liftoff up to its first crossing of 1000 ft:
	// neither the 30 kt on the runway before it nor the 50 kt above counts.
	// One cut short by the next contact (a landing at 2 s) ends before it, and
	// has no samples when that contact comes before the liftoff's next sample.
	void climbOutSegmentEndsAt1000Ft() {
		TripDataset dataset;
		const double heights[] = { 0, 300, 1000, 1500 };
		const double speeds[] = { 30, 10, 12, 50 };
		for (int i = 0; i < 4; ++i) {
			dataset.points.push_back(sample(i, 1000 + heights[i], heights[i]));
			dataset.points[i].rawNums[TRIP_NUM_ambient_wind_velocity] = speeds[i];
		}
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(1) };
		std::optional<ContactProfile> profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);
		QCOMPARE(profile->segment.samples, 2);
		QCOMPARE(profile->segment.windMinKt, 10.0);
		QCOMPARE(profile->segment.windMaxKt, 12.0);

		dataset.touchdowns = { contactAt<TouchdownPoint>(2) };
		profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);
		QCOMPARE(profile->segment.samples, 1);
		QCOMPARE(profile->segment.windMaxKt, 10.0);

		// A landing before the sample after the liftoff leaves no climb-out.
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(0.5) };
		dataset.touchdowns = { contactAt<TouchdownPoint>(0.8) };
		profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);
		QCOMPARE(profile->segment.samples, 0);

		// 950 ft is still below the top, so the climb-out runs on to 1500 ft;
		// cut short by a landing at 3 s, it keeps both samples before it.
		TripDataset circuit;
		const double circuitHeights[] = { 0, 300, 600, 950, 1500 };
		const double circuitSpeeds[] = { 30, 10, 20, 40, 50 };
		for (int i = 0; i < 5; ++i) {
			circuit.points.push_back(sample(i, 1000 + circuitHeights[i], circuitHeights[i]));
			circuit.points[i].rawNums[TRIP_NUM_ambient_wind_velocity] = circuitSpeeds[i];
		}
		circuit.liftoffPoints = { contactAt<LiftoffPoint>(1) };
		profile = contactProfile(circuit, circuit.liftoffPoints[0], false);
		QVERIFY(profile);
		QCOMPARE(profile->segment.samples, 4);
		QCOMPARE(profile->segment.windMaxKt, 50.0);
		circuit.touchdowns = { contactAt<TouchdownPoint>(3) };
		profile = contactProfile(circuit, circuit.liftoffPoints[0], false);
		QVERIFY(profile);
		QCOMPARE(profile->segment.samples, 2);
		QCOMPARE(profile->segment.windMaxKt, 20.0);
	}

	// A segment stays within 5 nm of its contact, stopping at the first sample
	// beyond it: an approach whose 1000 ft crossing is at 4 nm but which went
	// out to 5.5 nm since starts after that (4.9 nm, 2 nm and the touchdown),
	// and a climb-out that stays low ends before 5.5 nm (2 nm and 4.9 nm).
	void segmentStaysWithin5Nm() {
		const double speeds[] = { 60, 50, 20, 10, 5 };
		TripDataset approach;
		approach.points = { sample(-4, 2200, 1200, 4.0), sample(-3, 1500, 500, 5.5), sample(-2, 1400, 400, 4.9),
			sample(-1, 1200, 200, 2), sample(0, 1000, 0, 0) };
		for (int i = 0; i < 5; ++i)
			approach.points[i].rawNums[TRIP_NUM_ambient_wind_velocity] = speeds[i];
		approach.touchdowns = { contactAt<TouchdownPoint>(0) };
		std::optional<ContactProfile> profile = contactProfile(approach, approach.touchdowns[0], true);
		QVERIFY(profile);
		QCOMPARE(profile->segment.samples, 3);
		QCOMPARE(profile->segment.windMaxKt, 20.0);

		TripDataset climb;
		climb.points = { sample(0, 1000, 0, 0), sample(1, 1200, 200, 2), sample(2, 1300, 300, 4.9),
			sample(3, 1400, 400, 5.5), sample(4, 1500, 500, 4.0) };
		for (int i = 0; i < 5; ++i)
			climb.points[i].rawNums[TRIP_NUM_ambient_wind_velocity] = speeds[4 - i];
		climb.liftoffPoints = { contactAt<LiftoffPoint>(1) };
		profile = contactProfile(climb, climb.liftoffPoints[0], false);
		QVERIFY(profile);
		QCOMPARE(profile->segment.samples, 2);
		QCOMPARE(profile->segment.windMaxKt, 20.0);
	}

	// With no samples nothing is derived; with values not recorded, those
	// stay unknown rather than 0.
	void missingDataStaysUnknown() {
		TripDataset empty;
		empty.touchdowns = { contactAt<TouchdownPoint>(0) };
		analyzeContacts(empty);
		QVERIFY(!empty.touchdowns[0].conditions.fromSample);
		QVERIFY(!contactProfile(empty, empty.touchdowns[0], true));

		TripDataset dataset;
		TripSamplePoint p;
		p.zuluTime = zulu(0);
		dataset.points = { p };
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		analyzeContacts(dataset);
		const ContactConditions& c = dataset.touchdowns[0].conditions;
		QVERIFY(c.fromSample);
		for (double value : { c.headingTrue, c.magneticVariation, c.crabAngle, c.densityAltitudeFt, c.fieldElevationFt,
				 c.windMinKt, c.windDirectionFrom })
			QVERIFY(std::isnan(value));
		QVERIFY(c.surfaceType.isEmpty());
		QVERIFY(c.surfaceCondition.isEmpty());
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		QVERIFY(std::isnan(profile->highestTerrainFt));
		QVERIFY(std::isnan(profile->highestTerrainNm));
		// Without the runway's stored end, no point is placed on it.
		const ProfilePoint& at = profile->series.at(0);
		QVERIFY(std::isnan(at.thresholdFt));
		QVERIFY(std::isnan(at.centerlineFt));
		for (double value : { at.windSpeedKt, at.headwindKt, at.temperatureC, at.inCloud, at.structuralIce })
			QVERIFY(std::isnan(value));
		QVERIFY(at.precipitation.isEmpty());
		const SegmentWeather& w = profile->segment;
		QCOMPARE(w.samples, 1);
		for (double value : { w.windMinKt, w.headwindFromKt, w.maxTailwindKt, w.maxCrosswindKt, w.lowestInCloudFt,
				 w.icingConditions, w.maxStructuralIce, w.maxPitotIce, w.engineAntiIceShare, w.gForceMin, w.gForceMax,
				 w.verticalWindMinFpm, w.stallMarginAlphaDeg, w.stallMarginHeightFt })
			QVERIFY(std::isnan(value));
		QVERIFY(!w.cloudKnown);
	}
};

QTEST_APPLESS_MAIN(TstContactAnalysis)
#include "tst_contact_analysis.moc"
