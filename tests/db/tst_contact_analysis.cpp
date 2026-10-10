// contact_analysis.h: the conditions and final trajectory each liftoff and
// touchdown gets from the trip's samples. Every expected value is worked out
// by hand from the fixture (heights, times, a 3-degree path, the solstice sun).
#include "contact_analysis.h"
#include "trip_data_fields.h"
#include "trip_dataset.h"

#include <QtTest>

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
	// 12:00 UTC is midnight at longitude 180, where on the equator at the
	// solstice the sun is 90 - 23.44 below the horizon.
	void theContactsPositionPlacesTheSunAndTheDistances() {
		TripDataset dataset = touchdownAfter({ 100, 0 });
		for (TripSamplePoint& p : dataset.points)
			p.longitude = 180;
		dataset.points[0].latitude = 1.0 / 60;  // 1 arcminute north: 1.0007 nm
		dataset.touchdowns[0].longitude = 180;
		analyzeContacts(dataset);
		QVERIFY(qAbs(dataset.touchdowns[0].conditions.sunElevation - -66.56) < 0.1);

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

	// A touchdown's approach starts at the previous contact (a touch-and-go's
	// liftoff), so the circuit before it doesn't count.
	void touchdownWindowStartsAtThePreviousContact() {
		TripDataset dataset = touchdownAfter({ 600, 0, 300, 100, 40, 0 });
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(-4) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);

		QCOMPARE(gateNames(*profile), QStringList({ "300 ft", "200 ft", "100 ft", "50 ft" }));
		QCOMPARE(profile->series.front().seconds, -4.0);
		QCOMPARE(profile->series.back().seconds, 0.0);
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

	// A liftoff's climb-out ends at the next contact in time, though the trip
	// lists its liftoffs before its touchdowns.
	void liftoffWindowEndsAtTheNextContact() {
		TripDataset dataset;
		const double heights[] = { 0, 60, 0, 150, 0 };
		for (int i = 0; i < 5; ++i)
			dataset.points.push_back(sample(i, heights[i], heights[i]));
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(0.5), contactAt<LiftoffPoint>(2.5) };
		dataset.touchdowns = { contactAt<TouchdownPoint>(2), contactAt<TouchdownPoint>(4) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);

		QCOMPARE(gateNames(*profile), QStringList({ "35 ft" }));
		QCOMPARE(profile->series.back().seconds, 0.5);
	}

	// A touch-and-go's liftoff series starts at its touchdown, though that
	// was less than 10 s before.
	void liftoffSeriesStartsAtThePreviousContact() {
		TripDataset dataset;
		const double heights[] = { 50, 20, 0, 0, 0, 60, 150 };
		for (int i = 0; i < 7; ++i)
			dataset.points.push_back(sample(i, 500 + heights[i], heights[i]));
		dataset.touchdowns = { contactAt<TouchdownPoint>(2) };
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(4.5) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);

		QCOMPARE(profile->series.front().seconds, -2.5);
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
	// two samples: the liftoff's window is just the sample after them, so
	// nothing from before the bounce or after it is taken in, and there is no
	// climb-out to find terrain under.
	void bounceWithinOneSampleTakesOnlyItsSample() {
		TripDataset dataset;
		const double heights[] = { 40, 5, 3, 0 };
		for (int i = 0; i < 4; ++i)
			dataset.points.push_back(sample(i, 500 + heights[i], heights[i]));
		dataset.touchdowns = { contactAt<TouchdownPoint>(1.3), contactAt<TouchdownPoint>(1.8) };
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(1.6) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);

		QVERIFY(profile->gates.empty());
		QCOMPARE(profile->series.size(), size_t(1));
		QCOMPARE(profile->series[0].seconds, 0.4);
		QVERIFY(std::isnan(profile->highestTerrainFt));
	}

	// A liftoff's series runs from 10 s before it to 10 s after: of samples
	// 1 s apart, -9.5 s to +9.5 s, the next ones out (-10.5, +10.5) left out.
	void liftoffSeriesCoversTenSecondsEachSide() {
		TripDataset dataset;
		for (int i = 0; i <= 30; ++i) {
			const double height = i <= 15 ? 0 : (i - 15) * 50;
			dataset.points.push_back(sample(i, 500 + height, height));
		}
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(15.5) };
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.liftoffPoints[0], false);
		QVERIFY(profile);

		QCOMPARE(profile->series.size(), size_t(20));
		QCOMPARE(profile->series.front().seconds, -9.5);
		QCOMPARE(profile->series.back().seconds, 9.5);
	}

	// The touchdown series is the last 15 s.
	void touchdownSeriesCoversTheLast15Seconds() {
		std::vector<double> heights;
		for (int i = 20; i >= 0; --i)
			heights.push_back(i * 10);
		TripDataset dataset = touchdownAfter(heights);
		const std::optional<ContactProfile> profile = contactProfile(dataset, dataset.touchdowns[0], true);
		QVERIFY(profile);
		const std::vector<ProfilePoint>& series = profile->series;

		QCOMPARE(series.size(), size_t(16));
		QCOMPARE(series.front().seconds, -15.0);
		QCOMPARE(series.back().seconds, 0.0);
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
		// Without the runway's stored end, no point is placed on it.
		const ProfilePoint& at = profile->series.at(0);
		QVERIFY(std::isnan(at.thresholdFt));
		QVERIFY(std::isnan(at.centerlineFt));
	}
};

QTEST_APPLESS_MAIN(TstContactAnalysis)
#include "tst_contact_analysis.moc"
