// Runway matching module (runway_match.cpp) on its own: made-up airports and
// points in, candidate runways and distances out.
#include "test_support.h"

#include "runway_match.h"

#include <QtTest>

using namespace TestSupport;

namespace {

const double kFeetPerMeter = 3.2808399;

RunwaySpec eastWest() {
	RunwaySpec r;
	r.latitude = 43.0;
	r.longitude = 1.0;
	r.heading = 90;
	return r;
}

// An AIRPORT holding the given runways, as FACILITY_DATA would fill it.
struct TestAirport {
	AIRPORT airport;
	explicit TestAirport(const std::vector<RunwaySpec>& specs) {
		airport.n_runways = (int)specs.size();
		airport.runways = (RUNWAY*)calloc(specs.size(), sizeof(RUNWAY));
		for (size_t i = 0; i < specs.size(); ++i) {
			RUNWAY& r = airport.runways[i];
			r.coordinate.latitude = specs[i].latitude;
			r.coordinate.longitude = specs[i].longitude;
			r.heading = specs[i].heading;
			r.length = specs[i].lengthM;
			r.width = specs[i].widthM;
			r.numbers[0] = specs[i].primaryNumber;
			r.numbers[1] = specs[i].secondaryNumber;
			r.designators[0] = specs[i].primaryDesignator;
			r.designators[1] = specs[i].secondaryDesignator;
			r.primary_threshold_offset_m = specs[i].primaryThresholdM;
			r.secondary_threshold_offset_m = specs[i].secondaryThresholdM;
			r.primary_threshold_enable = specs[i].thresholdEnable;
			r.secondary_threshold_enable = specs[i].thresholdEnable;
			r.slope = specs[i].slope;
		}
	}
};

RUNWAY_MATCH match(AIRPORT& airport, const COORDINATE& point, double bearing, bool touchdown, QStringList* lines = nullptr) {
	return match_runways(airport, point, bearing, touchdown, [lines](const char* line) {
		if (lines)
			lines->append(QString::fromUtf8(line));
	});
}

}

class TstRunwayMatch : public QObject {
	Q_OBJECT

private slots:
	void pointOnRunwayInDirectionOfPrimary() {
		TestAirport a({ eastWest() });
		const RUNWAY_MATCH m = match(a.airport, pointOnRunway(eastWest(), 1200, 5), 90, false);
		QCOMPARE(m.candidates.size(), size_t(1));
		const RUNWAY_OPERATION& op = m.candidates[0];
		QCOMPARE(op.index, 0);
		QVERIFY(op.is_primary);
		QCOMPARE(op.heading, 90);
		QVERIFY(qAbs(op.diff_bearing_tra) < 1e-9);
		QVERIFY(qAbs(op.distances[0] - 1200 * kFeetPerMeter) < 2);
		QVERIFY(qAbs(op.distances[1] - 5 * kFeetPerMeter) < 0.5);
		QVERIFY(qAbs(op.distances_percent[0] - 0.4) < 0.001);
		QVERIFY(qAbs(op.distances_percent[1] - 5 / 22.5) < 0.01);
		QVERIFY(m.any_margin_hit);
	}

	void reverseDirectionUsesTheSecondaryEnd() {
		TestAirport a({ eastWest() });
		const RUNWAY_MATCH m = match(a.airport, pointOnRunway(eastWest(), 1000, 5), 275, false);
		QCOMPARE(m.candidates.size(), size_t(1));
		QVERIFY(!m.candidates[0].is_primary);
		QCOMPARE(m.candidates[0].heading, 270);
		QVERIFY(qAbs(m.candidates[0].diff_bearing_tra - 5) < 1e-9);
		QVERIFY(qAbs(m.candidates[0].distances[0] - 2000 * kFeetPerMeter) < 2);
		// Now 5 m to the LEFT of the direction of travel. Measured from the far
		// end, it comes out ~0.2 m (0.7 ft) short, hence the wider tolerance:
		// match_runways() measures along the runway's nominal heading there,
		// but the centerline's great circle bears slightly off it at either
		// end (meridian convergence: ~0.01 degrees at 43N over 1.5 km).
		QVERIFY2(qAbs(m.candidates[0].distances[1] + 5 * kFeetPerMeter) < 1.0,qPrintable(QString::number(m.candidates[0].distances[1])));
	}

	void pointExactlyOnTheCenterlineIsMeasuredFromTheThreshold() {
		TestAirport a({ eastWest() });
		const RUNWAY_MATCH m = match(a.airport, pointOnRunway(eastWest(), 1200, 0), 90, false);
		QCOMPARE(m.candidates.size(), size_t(1));
		QVERIFY2(qAbs(m.candidates[0].distances[0] - 1200 * kFeetPerMeter) < 2, qPrintable(QString::number(m.candidates[0].distances[0])));
		QVERIFY2(qAbs(m.candidates[0].distances[1]) < 0.5, qPrintable(QString::number(m.candidates[0].distances[1])));
		const RUNWAY_MATCH atThreshold = match(a.airport, pointOnRunway(eastWest(), 0, 0), 90, false);
		QCOMPARE(atThreshold.candidates.size(), size_t(1));
		QVERIFY2(qAbs(atThreshold.candidates[0].distances[0]) < 2, qPrintable(QString::number(atThreshold.candidates[0].distances[0])));
	}

	void offsetsAreAlongAndAcrossTheRunwayFromTheGivenEnd() {
		const RunwaySpec r = eastWest();
		const COORDINATE threshold = pointOnRunway(r, 0, 0);
		const RUNWAY_OFFSETS on = runway_offsets(threshold, 90, pointOnRunway(r, 1200, 5));
		QVERIFY2(qAbs(on.along_ft - 1200 * kFeetPerMeter) < 2, qPrintable(QString::number(on.along_ft)));
		QVERIFY2(qAbs(on.cross_ft - 5 * kFeetPerMeter) < 0.5, qPrintable(QString::number(on.cross_ft)));
		// Short of the threshold and left of the extended centerline: both negative.
		const RUNWAY_OFFSETS before = runway_offsets(threshold, 90, pointOnRunway(r, -900, -10));
		QVERIFY2(qAbs(before.along_ft + 900 * kFeetPerMeter) < 2, qPrintable(QString::number(before.along_ft)));
		QVERIFY2(qAbs(before.cross_ft + 10 * kFeetPerMeter) < 0.5, qPrintable(QString::number(before.cross_ft)));
	}

	void usedEndsFacilityDataIsCaptured() {
		RunwaySpec r = eastWest();
		r.primaryThresholdM = 300;
		r.secondaryThresholdM = 200;
		r.thresholdEnable = 1;
		r.slope = 0.5f;
		TestAirport a({ r });
		// A liftoff from 09 still gets its end's threshold offset (its distance
		// is measured from the physical end).
		const RUNWAY_MATCH primary = match(a.airport, pointOnRunway(r, 700, 5), 90, false);
		const RUNWAY_OPERATION& p = primary.candidates[0];
		QCOMPARE(p.true_heading, 90.0);
		QCOMPARE(p.length_m, 3000.0);
		QCOMPARE(p.width_m, 45.0);
		QCOMPARE(p.threshold_offset_m, 300.0);
		QCOMPARE(p.slope, 0.5);
		QCOMPARE(p.start.latitude, a.airport.runways[0].start_points[0].latitude);
		QCOMPARE(p.start.longitude, a.airport.runways[0].start_points[0].longitude);
		// Landing on 27: the other end's start and offset, the slope downhill.
		const RUNWAY_MATCH secondary = match(a.airport, pointOnRunway(r, 1000, 5), 270, true);
		const RUNWAY_OPERATION& s = secondary.candidates[0];
		QCOMPARE(s.true_heading, 270.0);
		QCOMPARE(s.threshold_offset_m, 200.0);
		QCOMPARE(s.slope, -0.5);
		QCOMPARE(s.start.latitude, a.airport.runways[0].start_points[1].latitude);
		QCOMPARE(s.start.longitude, a.airport.runways[0].start_points[1].longitude);
	}

	void runwayEndsAreStored() {
		TestAirport a({ eastWest() });
		match(a.airport, pointOnRunway(eastWest(), 100, 5), 90, false);
		RUNWAY& r = a.airport.runways[0];
		QVERIFY(qAbs(r.start_points[0].distanceInKm2Coordinate(r.start_points[1]) - 3.0) < 0.001);
		QVERIFY(r.start_points[0].longitude < r.start_points[1].longitude); // [0] is the west (09) end
	}

	void pastTheEndIsOnlyAMarginHit() {
		TestAirport a({ eastWest() });
		const RUNWAY_MATCH m = match(a.airport, pointOnRunway(eastWest(), 3150, 5), 90, false);
		QVERIFY(m.candidates.empty());
		QVERIFY(m.any_margin_hit);
		const RUNWAY_MATCH beside = match(a.airport, pointOnRunway(eastWest(), 1500, 70), 90, false);
		QVERIFY(beside.candidates.empty());
		QVERIFY(beside.any_margin_hit);
	}

	void farAwayIsNoHitAtAll() {
		TestAirport a({ eastWest() });
		const RUNWAY_MATCH m = match(a.airport, pointOnRunway(eastWest(), 1500, 500), 90, false);
		QVERIFY(m.candidates.empty());
		QVERIFY(!m.any_margin_hit);
		const RUNWAY_MATCH shortOf = match(a.airport, pointOnRunway(eastWest(), -300, 0), 90, false);
		QVERIFY(!shortOf.any_margin_hit);
	}

	void noRunwaysNoResult() {
		TestAirport a({});
		const RUNWAY_MATCH m = match(a.airport, pointOnRunway(eastWest(), 0, 0), 90, false);
		QVERIFY(m.candidates.empty());
		QVERIFY(!m.any_margin_hit);
	}

	void crossingRunwaysBothMatchWithTheirAlignment() {
		RunwaySpec ns = eastWest();
		ns.heading = 360;
		ns.primaryNumber = 36;
		ns.secondaryNumber = 18;
		TestAirport a({ ns, eastWest() });
		COORDINATE center;
		center.latitude = 43.00001;
		center.longitude = 1.00001;
		const RUNWAY_MATCH m = match(a.airport, center, 88, false);
		QCOMPARE(m.candidates.size(), size_t(2));
		QCOMPARE(m.candidates[0].index, 0);
		QCOMPARE(m.candidates[1].index, 1);
		QVERIFY(m.candidates[1].diff_bearing_tra < m.candidates[0].diff_bearing_tra);
	}

	void northRunwayHeadingIs360NotZero() {
		RunwaySpec south = eastWest();
		south.heading = 180;
		TestAirport a({ south });
		const RUNWAY_MATCH m = match(a.airport, pointOnRunway(south, 1500, 5), 0.5, false);
		QCOMPARE(m.candidates.size(), size_t(1));
		QVERIFY(!m.candidates[0].is_primary);
		QCOMPARE(m.candidates[0].heading, 360);
	}

	void displacedThresholdAppliesToTouchdownsOnly() {
		RunwaySpec r = eastWest();
		r.primaryThresholdM = 300;
		r.secondaryThresholdM = 200;
		r.thresholdEnable = 1;
		TestAirport a({ r });
		const COORDINATE p = pointOnRunway(r, 700, 5);
		const RUNWAY_MATCH liftoff = match(a.airport, p, 90, false);
		const RUNWAY_MATCH touchdown = match(a.airport, p, 90, true);
		QVERIFY(qAbs(liftoff.candidates[0].distances[0] - 700 * kFeetPerMeter) < 2);
		QVERIFY(qAbs(liftoff.candidates[0].distances_percent[0] - 700.0 / 3000) < 0.001);
		QVERIFY(qAbs(touchdown.candidates[0].distances[0] - 400 * kFeetPerMeter) < 2);
		QVERIFY(qAbs(touchdown.candidates[0].distances_percent[0] - 400.0 / 2500) < 0.001);
		// Landing the other way is measured from the secondary end's threshold:
		// 2000 m from that end, less its 200 m offset.
		const RUNWAY_MATCH reverse = match(a.airport, pointOnRunway(r, 1000, 5), 270, true);
		QVERIFY(!reverse.candidates[0].is_primary);
		QVERIFY(qAbs(reverse.candidates[0].distances[0] - 1800 * kFeetPerMeter) < 2);
		QVERIFY(qAbs(reverse.candidates[0].distances_percent[0] - 1800.0 / 2500) < 0.001);
	}

	void northPrimaryEndStoredAsZeroIsReportedAs360() {
		RunwaySpec north = eastWest();
		north.heading = 0;
		TestAirport a({ north });
		const RUNWAY_MATCH m = match(a.airport, pointOnRunway(north, 1500, 5), 0.5, false);
		QCOMPARE(m.candidates.size(), size_t(1));
		QVERIFY(m.candidates[0].is_primary);
		QCOMPARE(m.candidates[0].heading, 360);
	}

	// Offsets that leave no landing distance (3500 m of a 3000 m runway):
	// the percentage falls back to the full physical length.
	void thresholdsLongerThanTheRunwayFallBackToItsLength() {
		RunwaySpec r = eastWest();
		r.primaryThresholdM = 2000;
		r.secondaryThresholdM = 1500;
		r.thresholdEnable = 1;
		TestAirport a({ r });
		const RUNWAY_MATCH m = match(a.airport, pointOnRunway(r, 2500, 5), 90, true);
		QVERIFY(qAbs(m.candidates[0].distances[0] - 500 * kFeetPerMeter) < 2);
		QVERIFY(qAbs(m.candidates[0].distances_percent[0] - 500.0 / 3000) < 0.001);
	}

	// At either end.
	void thresholdDataIgnoredWhenDisabled() {
		RunwaySpec r = eastWest();
		r.primaryThresholdM = 300;
		r.secondaryThresholdM = 200;
		r.thresholdEnable = 0;
		TestAirport a({ r });
		const RUNWAY_MATCH m = match(a.airport, pointOnRunway(r, 700, 5), 90, true);
		QVERIFY(qAbs(m.candidates[0].distances[0] - 700 * kFeetPerMeter) < 2);
		QCOMPARE(m.candidates[0].threshold_offset_m, 0.0);
		const RUNWAY_MATCH reverse = match(a.airport, pointOnRunway(r, 1000, 5), 270, true);
		QVERIFY(!reverse.candidates[0].is_primary);
		QVERIFY(qAbs(reverse.candidates[0].distances[0] - 2000 * kFeetPerMeter) < 2);
		QCOMPARE(reverse.candidates[0].threshold_offset_m, 0.0);
	}

	void traceDescribesEachDecision() {
		RunwaySpec r = eastWest();
		r.primaryDesignator = 1;
		r.secondaryDesignator = 2;
		TestAirport a({ r });
		// The trace's key facts (which runway, which end, the verdict), not its
		// exact wording.
		QStringList lines;
		const auto hasLine = [&lines](const QStringList& parts) { return anyLineWith(lines, parts); };
		match(a.airport, pointOnRunway(r, 1000, 5), 90, true, &lines);
		QVERIFY(hasLine({ "09L/27R", "hit" }));
		QVERIFY(hasLine({ "09L/27R", "pass" }));
		QVERIFY(hasLine({ "threshold", "primary" }));
		QVERIFY(hasLine({ "accepted", "is_primary=1" }));
		QVERIFY(!hasLine({ "fail" }));
		lines.clear();
		match(a.airport, pointOnRunway(r, 1000, 500), 90, false, &lines);
		QVERIFY(hasLine({ "09L/27R", "fail", "outside" }));
		QVERIFY(!hasLine({ "accepted" }));
	}
};

QTEST_MAIN(TstRunwayMatch)
#include "tst_runway_match.moc"
