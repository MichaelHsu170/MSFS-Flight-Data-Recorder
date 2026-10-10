// contact_analysis.h: the conditions and final trajectory each liftoff and
// touchdown gets from the trip's samples. Every expected value is worked out
// by hand from the fixture (heights, times, a 3-degree path, the solstice sun).
#include "contact_analysis.h"
#include "trip_data_fields.h"
#include "trip_dataset.h"

#include <QtTest>

#include <cmath>

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
	}

	void surfaceNames() {
		QCOMPARE(surfaceTypeName(0), QStringLiteral("concrete"));
		QCOMPARE(surfaceTypeName(4), QStringLiteral("asphalt"));
		QCOMPARE(surfaceTypeName(24), QStringLiteral("Wright Flyer track"));
		QCOMPARE(surfaceConditionName(1), QStringLiteral("wet"));
		QCOMPARE(surfaceConditionName(3), QStringLiteral("snow"));
		for (double unknown : { 25.0, 255.0, -1.0, 1.5, std::nan("") })
			QVERIFY2(surfaceTypeName(unknown).isEmpty(), qPrintable(QString::number(unknown)));
		QVERIFY(surfaceConditionName(4).isEmpty());
		QVERIFY(surfaceConditionName(255).isEmpty());
	}

	void conditionsComeFromTheContactSample() {
		TripDataset dataset;
		TripSamplePoint p = sample(0, 1017, 17);
		p.rawNums[TRIP_NUM_plane_heading_degrees_true] = 2;
		p.rawNums[TRIP_NUM_plane_heading_degrees_magnetic] = 358;
		p.rawNums[TRIP_NUM_gps_ground_true_track] = 6;
		p.rawNums[TRIP_NUM_pressure_altitude] = 1524;  // m = 5000 ft
		p.rawNums[TRIP_NUM_ambient_temperature] = 30;
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
		QCOMPARE(c.visibilityM, 8000.0);
		QCOMPARE(c.fieldElevationFt, 1000.0);
		QCOMPARE(c.surfaceType, QStringLiteral("asphalt"));
		QCOMPARE(c.surfaceCondition, QStringLiteral("wet"));
		QVERIFY(qAbs(c.sunElevation - 66.56) < 0.1);  // noon on the equator at the solstice
	}

	// The wind range covers the 30 s up to the contact, its directions one
	// range across north.
	void windRangeCoversTheLast30Seconds() {
		TripDataset dataset;
		const struct { double seconds, direction, speed; } winds[] = {
			{ -40, 90, 30 }, { -20, 350, 8 }, { -10, 20, 14 }, { 0, 0, 10 } };
		for (const auto& w : winds) {
			TripSamplePoint p = sample(w.seconds, 1000, 0);
			p.rawNums[TRIP_NUM_ambient_wind_direction] = w.direction;
			p.rawNums[TRIP_NUM_ambient_wind_velocity] = w.speed;
			dataset.points.push_back(p);
		}
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		analyzeContacts(dataset);

		const ContactConditions& c = dataset.touchdowns[0].conditions;
		QCOMPARE(c.windMinKt, 8.0);
		QCOMPARE(c.windMaxKt, 14.0);
		QCOMPARE(c.windDirectionFrom, 350.0);
		QCOMPARE(c.windDirectionTo, 20.0);
	}

	// Each gate is its last crossing before the touchdown: the climb back to
	// 700 ft (a go-around) makes the 500 ft gate the second descent's.
	void touchdownGatesAreTheLastCrossing() {
		TripDataset dataset = touchdownAfter({ 600, 400, 700, 550, 450, 250, 150, 80, 40, 0 });
		analyzeContacts(dataset);
		const ContactProfile& profile = dataset.touchdowns[0].profile;

		// No 1000 ft gate (never that high) and no distance gates (never 1 nm away).
		QCOMPARE(gateNames(profile), QStringList({ "500 ft", "300 ft", "200 ft", "100 ft", "50 ft" }));
		const double heights[] = { 550, 450, 250, 150, 80 };
		const double seconds[] = { -6, -5, -4, -3, -2 };
		for (int i = 0; i < 5; ++i) {
			QCOMPARE(profile.gates[i].heightFt, heights[i]);
			QCOMPARE(profile.gates[i].seconds, seconds[i]);
		}
	}

	// A touchdown's approach starts at the previous contact (a touch-and-go's
	// liftoff), so the circuit before it doesn't count.
	void touchdownWindowStartsAtThePreviousContact() {
		TripDataset dataset = touchdownAfter({ 600, 0, 300, 100, 40, 0 });
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(-4) };
		analyzeContacts(dataset);
		const ContactProfile& profile = dataset.touchdowns[0].profile;

		QCOMPARE(gateNames(profile), QStringList({ "300 ft", "200 ft", "100 ft", "50 ft" }));
		QCOMPARE(profile.series.front().seconds, -4.0);
		QCOMPARE(profile.series.back().seconds, 0.0);
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
		const LiftoffPoint& liftoff = dataset.liftoffPoints[0];

		QCOMPARE(gateNames(liftoff.profile), QStringList({ "35 ft", "100 ft", "200 ft", "500 ft", "1000 ft" }));
		const double gateHeights[] = { 60, 150, 250, 600, 1200 };
		const double gateSeconds[] = { 1.5, 3.5, 4.5, 5.5, 6.5 };
		for (int i = 0; i < 5; ++i) {
			QCOMPARE(liftoff.profile.gates[i].heightFt, gateHeights[i]);
			QCOMPARE(liftoff.profile.gates[i].seconds, gateSeconds[i]);
		}
		// The series runs from the on-ground sample through the climb-out.
		QCOMPARE(liftoff.profile.series.size(), size_t(8));
		QCOMPARE(liftoff.profile.series.front().seconds, -0.5);
		QCOMPARE(liftoff.profile.series.front().heightFt, 0.0);
		QCOMPARE(liftoff.conditions.fieldElevationFt, 500.0);
	}

	// A liftoff's climb-out ends at the next contact.
	void liftoffWindowEndsAtTheNextContact() {
		TripDataset dataset;
		const double heights[] = { 0, 60, 0, 150 };
		for (int i = 0; i < 4; ++i)
			dataset.points.push_back(sample(i, heights[i], heights[i]));
		dataset.liftoffPoints = { contactAt<LiftoffPoint>(0.5) };
		dataset.touchdowns = { contactAt<TouchdownPoint>(2) };
		analyzeContacts(dataset);

		QCOMPARE(gateNames(dataset.liftoffPoints[0].profile), QStringList({ "35 ft" }));
		QCOMPARE(dataset.liftoffPoints[0].profile.series.back().seconds, 0.5);
	}

	// The touchdown series is the last 15 s.
	void touchdownSeriesCoversTheLast15Seconds() {
		std::vector<double> heights;
		for (int i = 20; i >= 0; --i)
			heights.push_back(i * 10);
		TripDataset dataset = touchdownAfter(heights);
		analyzeContacts(dataset);
		const std::vector<ProfilePoint>& series = dataset.touchdowns[0].profile.series;

		QCOMPARE(series.size(), size_t(16));
		QCOMPARE(series.front().seconds, -15.0);
		QCOMPARE(series.back().seconds, 0.0);
	}

	// However dense the samples, the series is at most 60 lines and keeps
	// both ends.
	void seriesIsCappedAt60() {
		TripDataset dataset;
		for (int i = 119; i >= 0; --i)
			dataset.points.push_back(sample(-i / 10.0, 1000 + i, i));
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		analyzeContacts(dataset);
		const std::vector<ProfilePoint>& series = dataset.touchdowns[0].profile.series;

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
		analyzeContacts(dataset);
		const ContactProfile& profile = dataset.touchdowns[0].profile;

		const ProfilePoint* fiveHundred = gate(profile, QStringLiteral("500 ft"));
		QVERIFY(fiveHundred);
		QCOMPARE(fiveHundred->seconds, -3.0);  // 600 ft above the touchdown, 200 ft above the ground
		QCOMPARE(fiveHundred->heightFt, 600.0);
		QCOMPARE(fiveHundred->terrainFt, 400.0);
		QCOMPARE(profile.highestTerrainFt, 400.0);
		QVERIFY(qAbs(profile.highestTerrainNm - 0.75) < 0.01);  // the closer of the two cliff-top samples
	}

	// On an exact 3-degree path (318.4 ft per nm), each distance gate's path
	// angle is 3.0.
	void distanceGatesGiveThePathAngle() {
		TripDataset dataset;
		const double feetPerNm = 6076.12 * std::tan(3 * std::acos(-1.0) / 180);
		for (int i = 7; i >= 0; --i) {
			const double nm = i * 0.5;
			dataset.points.push_back(sample(-i * 10.0, 100 + nm * feetPerNm, nm * feetPerNm, nm));
		}
		dataset.touchdowns = { contactAt<TouchdownPoint>(0) };
		analyzeContacts(dataset);
		const ContactProfile& profile = dataset.touchdowns[0].profile;

		for (int nm = 1; nm <= 3; ++nm) {
			const ProfilePoint* g = gate(profile, QStringLiteral("%1 nm").arg(nm));
			QVERIFY2(g, qPrintable(gateNames(profile).join('|')));
			QVERIFY2(qAbs(g->distanceNm - nm) < 0.01, qPrintable(QString::number(g->distanceNm)));
			QVERIFY2(qAbs(g->pathAngle - 3.0) < 0.01, qPrintable(QString::number(g->pathAngle)));
		}
		// Height gates have no path angle.
		QVERIFY(std::isnan(gate(profile, QStringLiteral("500 ft"))->pathAngle));
	}

	// With no samples nothing is derived; with values not recorded, those
	// stay unknown rather than 0.
	void missingDataStaysUnknown() {
		TripDataset empty;
		empty.touchdowns = { contactAt<TouchdownPoint>(0) };
		analyzeContacts(empty);
		QVERIFY(!empty.touchdowns[0].conditions.fromSample);
		QVERIFY(empty.touchdowns[0].profile.series.empty());

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
		QVERIFY(std::isnan(dataset.touchdowns[0].profile.highestTerrainFt));
	}
};

QTEST_APPLESS_MAIN(TstContactAnalysis)
#include "tst_contact_analysis.moc"
