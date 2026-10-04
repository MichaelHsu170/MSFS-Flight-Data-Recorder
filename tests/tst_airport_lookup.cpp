// Liftoff/touchdown detection and the airport + runway lookup behind them:
// runway matching, threshold/centerline distances, displaced thresholds,
// touch-and-go markers, queued lookups and every fallback outcome.
#include "db.h"
#include "test_support.h"

#include <QtTest>

using namespace TestSupport;

namespace {

const double kFeetPerMeter = 3.2808399;

RunwaySpec eastWestRunway() {
	RunwaySpec r;
	r.latitude = 43.0;
	r.longitude = 1.0;
	r.heading = 90;
	r.lengthM = 3000;
	r.widthM = 45;
	r.primaryNumber = 9;
	r.secondaryNumber = 27;
	return r;
}

AirportSpec testAirport(const RunwaySpec& runway) {
	AirportSpec a;
	a.ident = "TEST";
	a.region = "XX";
	a.name = "Test Field";
	a.latitude = runway.latitude;
	a.longitude = runway.longitude;
	a.runways = { runway };
	return a;
}

AirportSpec airportAt(const char* ident, const char* name, const COORDINATE& position) {
	AirportSpec a;
	a.ident = ident;
	a.region = "YY";
	a.name = name;
	a.latitude = position.latitude;
	a.longitude = position.longitude;
	return a;
}

// A liftoff/touchdown point distanceM down the runway, rightM (default 3 m)
// right of the centerline, as a real touchdown rarely sits exactly on it
// (tst_runway_match covers the exact-centerline case).
COORDINATE onRunway(const RunwaySpec& runway, double distanceM, double rightM = 3) {
	return pointOnRunway(runway, distanceM, rightM);
}

// Engines on, parked distanceM down the runway, heading along it.
int startOnRunway(FlightDriver& sim, const RunwaySpec& runway, double distanceM = 50, double heading = 90) {
	sim.moveTo(pointOnRunway(runway, distanceM));
	sim.setHeading(heading);
	return sim.startTrip();
}

void liftOff(FlightDriver& sim, const COORDINATE& at) {
	sim.moveTo(at);
	sim.setOnGround(false);
	sim.tick();
}

void touchDown(FlightDriver& sim, const COORDINATE& at) {
	sim.moveTo(at);
	sim.setOnGround(true);
	sim.tick();
}

QVariantMap trip(int id) {
	return queryRows(QStringLiteral("SELECT * FROM trips WHERE id=%1").arg(id)).value(0);
}

QList<QVariantMap> liftoffs(int tripId) {
	return queryRows(QStringLiteral("SELECT * FROM trip_liftoffs WHERE trip=%1 ORDER BY id").arg(tripId));
}

QList<QVariantMap> touchdowns(int tripId) {
	return queryRows(QStringLiteral("SELECT * FROM trip_touchdowns WHERE trip=%1 ORDER BY id").arg(tripId));
}

// Runs sql on the recorder's connection. The sample writer thread shares it,
// so hold its lock like every recorder write does (db_insert_update_table()
// in db.cpp).
void execOnRecorder(FlightDriver& sim, const QString& sql) {
	std::lock_guard<std::mutex> lock(sim.status().mutex_db_commit);
	QCOMPARE(sqlite3_exec(sim.status().sql, sql.toUtf8().constData(), nullptr, nullptr, nullptr), SQLITE_OK);
}

// Every later UPDATE of table fails, until stopFailingUpdates(); inserts and
// other tables still work.
void failUpdates(FlightDriver& sim, const char* table) {
	execOnRecorder(sim, QStringLiteral("CREATE TRIGGER fail_update_%1 BEFORE UPDATE ON %1 BEGIN SELECT RAISE(ABORT, 'forced'); END").arg(table));
}

void stopFailingUpdates(FlightDriver& sim, const char* table) {
	execOnRecorder(sim, QStringLiteral("DROP TRIGGER fail_update_%1").arg(table));
}

// Answers the pending facility-list request with these airports, leaving the
// facility data for the test to send itself.
void answerAirportList(FlightDriver& sim, const std::vector<AirportSpec>& airports) {
	FakeSim::queue(airportListPacket(airports, 0, 1));
	sim.pump();
}

bool isNear(const QVariant& actual, double expected, double tolerance) {
	return qAbs(actual.toDouble() - expected) <= tolerance;
}

#define VERIFY_NEAR(actual, expected, tolerance) \
	QVERIFY2(isNear((actual), (expected), (tolerance)), \
		qPrintable(QStringLiteral("actual %1, expected %2 +/- %3").arg((actual).toString()).arg(expected).arg(tolerance)))

}

class TstAirportLookup : public QObject {
	Q_OBJECT

private slots:
	void initTestCase() { isolateFiles(); }
	void init() { removeDatabase(); }

	// --- Departure ---

	void liftoffInsertsRowAndRequestsLookup() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		const int tripId = startOnRunway(sim, rwy);
		QVERIFY(FakeSim::state().facilitiesListRequests.empty());
		sim.record.airspeed_indicated = 150;
		sim.record.vertical_speed = 500;
		sim.record.plane_pitch_degrees = -8; // stored as +8 (nose up)
		sim.record.ambient_wind_direction = 270;
		sim.record.ambient_wind_velocity = 12;
		liftOff(sim, onRunway(rwy, 1800));
		QCOMPARE(FakeSim::state().facilitiesListRequests.size(), size_t(1));
		const QList<QVariantMap> rows = liftoffs(tripId);
		QCOMPARE(rows.size(), 1);
		QCOMPARE(rows[0]["airspeed_indicated"].toInt(), 150);
		QCOMPARE(rows[0]["vertical_speed"].toInt(), 500);
		QCOMPARE(rows[0]["plane_pitch_degrees"].toDouble(), 8.0);
		QCOMPARE(rows[0]["heading_indicator"].toInt(), 90);
		QCOMPARE(rows[0]["wind_direction"].toInt(), 270);
		QCOMPARE(rows[0]["wind_velocity"].toInt(), 12);
		QVERIFY(rows[0]["icao"].isNull());
		QVERIFY(rows[0]["runway"].isNull());
	}

	void departureResolvesAirportAndRunway() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		QSignalSpy updated(&sim.bridge(), &RecorderBridge::tripUpdated);
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		QCOMPARE(FakeSim::state().facilityDataRequests.size(), size_t(1));
		QCOMPARE(QString::fromStdString(FakeSim::state().facilityDataRequests[0].icao), QStringLiteral("TEST"));
		QCOMPARE(QString::fromStdString(FakeSim::state().facilityDataRequests[0].region), QStringLiteral("XX"));

		const QVariantMap t = trip(tripId);
		QCOMPARE(t["departure_icao"].toString(), QStringLiteral("TEST"));
		QCOMPARE(t["departure_rwy"].toString(), QStringLiteral("09"));
		QCOMPARE(t["departure_region"].toString(), QStringLiteral("XX"));
		QCOMPARE(t["departure_name"].toString(), QStringLiteral("Test Field"));

		const QVariantMap lo = liftoffs(tripId).value(0);
		QCOMPARE(lo["icao"].toString(), QStringLiteral("TEST"));
		QCOMPARE(lo["airport_name"].toString(), QStringLiteral("Test Field"));
		QCOMPARE(lo["runway"].toString(), QStringLiteral("09"));
		QCOMPARE(lo["runway_heading"].toInt(), 90);
		VERIFY_NEAR(lo["distance_length"], 1800 * kFeetPerMeter, 2);
		VERIFY_NEAR(lo["distance_length_percent"], 0.6, 0.001);
		VERIFY_NEAR(lo["distance_width"], 3 * kFeetPerMeter, 0.5);
		QVERIFY(updated.count() >= 1);
	}

	void departureIsLoggedWithItsLiftoffTime() {
		// The lookup resolves after the aircraft has flown on for 10 s: the
		// log line says when the liftoff happened, not when the lookup ended.
		for (const bool withAirport : { true, false }) {
			FlightDriver sim;
			const RunwaySpec rwy = eastWestRunway();
			if (withAirport)
				sim.airports = { testAirport(rwy) };
			startOnRunway(sim, rwy);
			liftOff(sim, onRunway(rwy, 1800));
			const QString liftoffTime = QString::fromStdString(sim.record.time_local.format_date_time());
			sim.ticks(20);
			const QString laterTime = QString::fromStdString(sim.record.time_local.format_date_time());
			QVERIFY(laterTime != liftoffTime);
			QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
			sim.serviceLookups();
			const QString line = lastLogWith(log, { QStringLiteral("Liftoff from") });
			QVERIFY2(line.contains(withAirport ? QStringLiteral("runway 09 at ") : QStringLiteral(" at ")), qPrintable(line));
			QVERIFY2(line.endsWith(QStringLiteral(" at ") + liftoffTime), qPrintable(line));
		}
	}

	void touchdownWhoseRowWasNeverInsertedIsLoggedAndSkipped() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		// The touchdown's immediate INSERT fails, so it has no row.
		execOnRecorder(sim, QStringLiteral("DROP TABLE trip_touchdowns"));
		touchDown(sim, onRunway(rwy, 500));
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		// A touch-and-go: the waiting touchdown is looked up first, then the liftoff.
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		QVERIFY(!lastLogWith(log, { QStringLiteral("(TEST) runway 09"), QStringLiteral("trip_touchdowns"),
			QStringLiteral("never inserted") }).isEmpty());
		// The trip still gets its destination, and the lookups carry on.
		QCOMPARE(trip(tripId)["destination_icao"].toString(), QStringLiteral("TEST"));
		QCOMPARE(liftoffs(tripId).value(1)["icao"].toString(), QStringLiteral("TEST"));
	}

	void touchdownOffTheRunwayWhoseRowWasNeverInsertedIsLogged() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		execOnRecorder(sim, QStringLiteral("DROP TABLE trip_touchdowns"));
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		// 100 m past the runway end: the airport resolves without a runway.
		// As in the test above, the touch-and-go's liftoff picks up the
		// touchdown's lookup.
		touchDown(sim, onRunway(rwy, 3100));
		liftOff(sim, onRunway(rwy, 3150));
		sim.serviceLookups();
		const QString warning = lastLogWith(log, { QStringLiteral("(TEST)"), QStringLiteral("trip_touchdowns"),
			QStringLiteral("never inserted") });
		QVERIFY(!warning.isEmpty());
		QVERIFY2(!warning.contains(QStringLiteral("runway 09")), qPrintable(warning)); // resolved without a runway
		QCOMPARE(trip(tripId)["destination_icao"].toString(), QStringLiteral("TEST"));
		QVERIFY(!sim.status().lookup.pending);
	}

	// Once the touchdown row is in, a failing trips UPDATE must not stop the
	// touchdown's lookup from being requested.
	void failedLandingDestinationWriteIsLoggedAndTheLookupStillRuns() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		failUpdates(sim, "trips");
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		touchDown(sim, onRunway(rwy, 500));
		QVERIFY(!lastLogWith(log, { QStringLiteral("trip %1").arg(tripId), QStringLiteral("failed"),
			QStringLiteral("destination") }).isEmpty());
		QCOMPARE(touchdowns(tripId).size(), 1);
		QCOMPARE(FakeSim::state().facilitiesListRequests.size(), size_t(2));
		QVERIFY(trip(tripId)["destination_latitude"].isNull());
		// Served with the trigger gone, so the touchdown's own row gets its
		// airport (a still-failing trips UPDATE throws before that write).
		stopFailingUpdates(sim, "trips");
		sim.serviceLookups();
		QCOMPARE(touchdowns(tripId).value(0)["icao"].toString(), QStringLiteral("TEST"));
	}

	// A trips UPDATE that keeps failing ends the touchdown's lookup once,
	// rather than asking for the same touchdown again on every answer.
	void persistentlyFailingTripWriteEndsTheTouchdownLookupOnce() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		failUpdates(sim, "trips");
		touchDown(sim, onRunway(rwy, 500));
		sim.serviceLookups();
		QCOMPARE(FakeSim::state().facilitiesListRequests.size(), size_t(2));
		QVERIFY(!sim.status().lookup.pending);
		// The next liftoff gets its own lookup.
		liftOff(sim, onRunway(rwy, 1800));
		QCOMPARE(FakeSim::state().facilitiesListRequests.size(), size_t(3));
	}

	// A db_exception thrown by a resolved lookup's writes reaches the
	// dispatch callback's catch; the lookup must still end, not stay pending.
	void failedLookupWriteIsLoggedByDispatchAndTheLookupEnds() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		touchDown(sim, onRunway(rwy, 500));
		failUpdates(sim, "trip_touchdowns");
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		sim.serviceLookups();
		QVERIFY(!lastLogWith(log, { QStringLiteral("Database error") }).isEmpty());
		QVERIFY(!sim.status().lookup.pending);
		// The trip's own destination write ran before the failing one.
		QCOMPARE(trip(tripId)["destination_icao"].toString(), QStringLiteral("TEST"));
		QVERIFY(touchdowns(tripId).value(0)["icao"].isNull());
		// The slot is free: the next touchdown gets its own lookup.
		liftOff(sim, onRunway(rwy, 1800));
		QCOMPARE(FakeSim::state().facilitiesListRequests.size(), size_t(3));
	}

	// A failing trips UPDATE when the touchdown has no airport at all, so the
	// lookup ends inside the AIRPORT_LIST handler instead of
	// FACILITY_DATA_END.
	void failedTripWriteOnACoordinateOnlyTouchdownStillFinishesTheLookup() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		failUpdates(sim, "trips");
		touchDown(sim, onRunway(rwy, 500));
		sim.serviceLookups();
		QCOMPARE(FakeSim::state().facilitiesListRequests.size(), size_t(2));
		QVERIFY(!sim.status().lookup.pending);
		// The next liftoff gets its own lookup.
		liftOff(sim, onRunway(rwy, 1800));
		QCOMPARE(FakeSim::state().facilitiesListRequests.size(), size_t(3));
	}

	void liftoffMarkerWhoseInsertFailsIsLoggedAndSkipped() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		touchDown(sim, onRunway(rwy, 500));
		sim.serviceLookups();
		// The touch-and-go's liftoff marker INSERT fails, so it has no row.
		execOnRecorder(sim, QStringLiteral("DROP TABLE trip_liftoffs"));
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		liftOff(sim, onRunway(rwy, 1800));
		// The next touchdown's lookup runs the marker's first; it's skipped.
		touchDown(sim, onRunway(rwy, 500));
		sim.serviceLookups();
		QVERIFY(!lastLogWith(log, { QStringLiteral("trip %1").arg(tripId), QStringLiteral("trip_liftoffs insert failed") }).isEmpty());
		QVERIFY(!lastLogWith(log, { QStringLiteral("(TEST) runway 09"), QStringLiteral("trip_liftoffs"),
			QStringLiteral("never inserted") }).isEmpty());
		QCOMPARE(trip(tripId)["destination_icao"].toString(), QStringLiteral("TEST"));
	}

	void departureInsertFailureIsLoggedAndRetriedOnNextLiftoff() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		// The departure's own immediate INSERT fails (distinct code path from
		// record_contact(), which the sibling tests above exercise for
		// subsequent markers/touchdowns).
		execOnRecorder(sim, QStringLiteral("DROP TABLE trip_liftoffs"));
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		liftOff(sim, onRunway(rwy, 1800));
		QVERIFY(FakeSim::state().facilitiesListRequests.empty());
		QVERIFY(!lastLogWith(log, { QStringLiteral("trip %1").arg(tripId), QStringLiteral("trip_liftoffs insert failed"),
			QStringLiteral("retry") }).isEmpty());
		// Land, restore the table, then take off again: since the first attempt
		// never flipped departure_lookup_initiated TRUE, this liftoff is
		// retried as the trip's departure, not recorded as a touch-and-go marker.
		touchDown(sim, onRunway(rwy, 500));
		migrate_db();
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		QCOMPARE(trip(tripId)["departure_icao"].toString(), QStringLiteral("TEST"));
		QCOMPARE(trip(tripId)["departure_rwy"].toString(), QStringLiteral("09"));
		QCOMPARE(liftoffs(tripId).size(), 1);
	}

	void departureInOppositeDirectionUsesSecondaryEnd() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy, 2950, 270);
		liftOff(sim, onRunway(rwy, 1000)); // 2000 m from the 27 end
		sim.serviceLookups();
		QCOMPARE(trip(tripId)["departure_rwy"].toString(), QStringLiteral("27"));
		const QVariantMap lo = liftoffs(tripId).value(0);
		QCOMPARE(lo["runway"].toString(), QStringLiteral("27"));
		QCOMPARE(lo["runway_heading"].toInt(), 270);
		VERIFY_NEAR(lo["distance_length"], 2000 * kFeetPerMeter, 2);
	}

	void runwayDesignatorsAppearInCodes() {
		FlightDriver sim;
		RunwaySpec rwy = eastWestRunway();
		rwy.primaryDesignator = 1;   // L
		rwy.secondaryDesignator = 2; // R
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1500));
		sim.serviceLookups();
		QCOMPARE(trip(tripId)["departure_rwy"].toString(), QStringLiteral("09L"));
	}

	void offsetFromCenterlineIsSignedRightPositive() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1500, 10)); // 10 m right of centerline
		sim.serviceLookups();
		const QVariantMap lo = liftoffs(tripId).value(0);
		VERIFY_NEAR(lo["distance_width"], 10 * kFeetPerMeter, 0.5);
		VERIFY_NEAR(lo["distance_width_percent"], 10 / 22.5, 0.01);
	}

	void magneticVariationIsAppliedToHeading() {
		// Aircraft heading is magnetic; runway heading true. With 10°E
		// variation the aircraft's magnetic 80° is true 90° -- primary end.
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		AirportSpec airport = testAirport(rwy);
		airport.magvar = -10;
		sim.airports = { airport };
		const int tripId = startOnRunway(sim, rwy, 50, 100);
		liftOff(sim, onRunway(rwy, 1500));
		sim.serviceLookups();
		QCOMPARE(trip(tripId)["departure_rwy"].toString(), QStringLiteral("09"));
	}

	void bestAlignedOfCrossingRunwaysIsChosen() {
		FlightDriver sim;
		RunwaySpec ew = eastWestRunway();
		RunwaySpec ns = eastWestRunway();
		ns.heading = 360;
		ns.primaryNumber = 36;
		ns.secondaryNumber = 18;
		AirportSpec airport = testAirport(ew);
		airport.runways = { ns, ew };
		sim.airports = { airport };
		// Liftoff at the crossing point (both runways' footprints), heading east.
		COORDINATE center;
		center.latitude = 43.0;
		center.longitude = 1.0;
		const int tripId = startOnRunway(sim, ew);
		liftOff(sim, center);
		sim.serviceLookups();
		QCOMPARE(trip(tripId)["departure_rwy"].toString(), QStringLiteral("09"));
	}

	// --- Touchdown ---

	void touchdownRecordsRowAndDestination() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		sim.ticks(10);
		sim.record.airspeed_indicated = 130;
		sim.record.vertical_speed = -180;
		sim.record.g_force = 1.3;
		const COORDINATE tdz = pointOnRunway(rwy, 450);
		touchDown(sim, tdz);
		const QList<QVariantMap> rows = touchdowns(tripId);
		QCOMPARE(rows.size(), 1);
		QCOMPARE(rows[0]["airspeed_indicated"].toInt(), 130);
		QCOMPARE(rows[0]["vertical_speed"].toInt(), -180);
		QCOMPARE(rows[0]["g_force"].toDouble(), 1.3);
		const QVariantMap before = trip(tripId);
		QCOMPARE(before["destination_latitude"].toDouble(), tdz.latitude);
		QCOMPARE(before["destination_longitude"].toDouble(), tdz.longitude);

		sim.serviceLookups();
		const QVariantMap after = trip(tripId);
		QCOMPARE(after["destination_icao"].toString(), QStringLiteral("TEST"));
		QCOMPARE(after["destination_rwy"].toString(), QStringLiteral("09"));
		QCOMPARE(after["destination_name"].toString(), QStringLiteral("Test Field"));
		QCOMPARE(after["destination_region"].toString(), QStringLiteral("XX"));
		const QVariantMap td = touchdowns(tripId).value(0);
		QCOMPARE(td["runway"].toString(), QStringLiteral("09"));
		VERIFY_NEAR(td["distance_length"], 450 * kFeetPerMeter, 2);
		VERIFY_NEAR(td["distance_length_percent"], 0.15, 0.001);
	}

	void displacedThresholdShortensTouchdownDistance() {
		FlightDriver sim;
		RunwaySpec rwy = eastWestRunway();
		rwy.primaryThresholdM = 300;
		rwy.secondaryThresholdM = 200;
		rwy.thresholdEnable = 1;
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		touchDown(sim, onRunway(rwy, 700));
		sim.serviceLookups();
		const QVariantMap td = touchdowns(tripId).value(0);
		// 700 m from the physical end is 400 m past the displaced threshold,
		// out of 3000 - 300 - 200 = 2500 m landing distance available.
		VERIFY_NEAR(td["distance_length"], 400 * kFeetPerMeter, 2);
		VERIFY_NEAR(td["distance_length_percent"], 400.0 / 2500, 0.001);
		// Liftoffs are measured from the physical end regardless.
		VERIFY_NEAR(liftoffs(tripId).value(0)["distance_length"], 1800 * kFeetPerMeter, 2);
	}

	void touchdownBeforeDisplacedThresholdIsNegative() {
		FlightDriver sim;
		RunwaySpec rwy = eastWestRunway();
		rwy.primaryThresholdM = 300;
		rwy.thresholdEnable = 1;
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		touchDown(sim, onRunway(rwy, 200));
		sim.serviceLookups();
		QVERIFY2(isNear(touchdowns(tripId).value(0)["distance_length"], -100 * kFeetPerMeter, 2),
			qPrintable(touchdowns(tripId).value(0)["distance_length"].toString()));
	}

	void thresholdIgnoredWhenNotEnabled() {
		FlightDriver sim;
		RunwaySpec rwy = eastWestRunway();
		rwy.primaryThresholdM = 300;
		rwy.thresholdEnable = 0;
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		touchDown(sim, onRunway(rwy, 700));
		sim.serviceLookups();
		VERIFY_NEAR(touchdowns(tripId).value(0)["distance_length"], 700 * kFeetPerMeter, 2);
	}

	void approachTrackDecidesLandingDirection() {
		// Heading says west, but the final-approach track (the 50-100 ft
		// radio-height position -> touchdown point) is eastbound: the
		// touchdown is attributed to runway 09.
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		sim.record.radio_height = 500;
		sim.tick();
		sim.moveTo(pointOnRunway(rwy, -800));
		sim.record.radio_height = 75;
		sim.tick();
		sim.record.radio_height = 20;
		sim.setHeading(270);
		touchDown(sim, onRunway(rwy, 500));
		sim.serviceLookups();
		QCOMPARE(touchdowns(tripId).value(0)["runway"].toString(), QStringLiteral("09"));
	}

	void withoutApproachTrackHeadingDecidesLandingDirection() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		sim.setHeading(270);
		touchDown(sim, onRunway(rwy, 2500));
		sim.serviceLookups();
		QCOMPARE(touchdowns(tripId).value(0)["runway"].toString(), QStringLiteral("27"));
	}

	void climbingAboveTheBandForgetsTheApproachPosition() {
		// An eastbound low pass (75 ft) is followed by a climb above 100 ft,
		// then a westbound landing that skips the band: the stale eastbound
		// position must not decide the direction.
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		sim.moveTo(pointOnRunway(rwy, -800));
		sim.record.radio_height = 75;
		sim.tick();
		sim.record.radio_height = 1500;
		sim.tick();
		sim.record.radio_height = 20;
		sim.setHeading(270);
		touchDown(sim, onRunway(rwy, 2500));
		sim.serviceLookups();
		QCOMPARE(touchdowns(tripId).value(0)["runway"].toString(), QStringLiteral("27"));
	}

	// --- Touch-and-go and queued lookups ---

	void touchAndGoRecordsMarkersWithoutChangingDeparture() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		touchDown(sim, onRunway(rwy, 500));
		sim.serviceLookups();
		liftOff(sim, onRunway(rwy, 1500));
		sim.serviceLookups();
		touchDown(sim, onRunway(rwy, 600));
		sim.serviceLookups();

		const QList<QVariantMap> los = liftoffs(tripId);
		QCOMPARE(los.size(), 2);
		QCOMPARE(los[1]["runway"].toString(), QStringLiteral("09"));
		VERIFY_NEAR(los[1]["distance_length"], 1500 * kFeetPerMeter, 2);
		QCOMPARE(touchdowns(tripId).size(), 2);
		VERIFY_NEAR(touchdowns(tripId)[1]["distance_length"], 600 * kFeetPerMeter, 2);
		// The departure stays the first liftoff.
		QCOMPARE(trip(tripId)["departure_rwy"].toString(), QStringLiteral("09"));
	}

	void lookupsQueuedWhileOneIsPendingResolveInOrder() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		// Nothing is answered until all four events have happened.
		liftOff(sim, onRunway(rwy, 1800));
		touchDown(sim, onRunway(rwy, 500));
		liftOff(sim, onRunway(rwy, 1500));
		touchDown(sim, onRunway(rwy, 700));
		QCOMPARE(FakeSim::state().facilitiesListRequests.size(), size_t(1));
		sim.serviceLookups();
		QCOMPARE(FakeSim::state().facilitiesListRequests.size(), size_t(4));

		const QList<QVariantMap> los = liftoffs(tripId);
		const QList<QVariantMap> tds = touchdowns(tripId);
		QCOMPARE(los.size(), 2);
		QCOMPARE(tds.size(), 2);
		VERIFY_NEAR(los[0]["distance_length"], 1800 * kFeetPerMeter, 2);
		VERIFY_NEAR(tds[0]["distance_length"], 500 * kFeetPerMeter, 2);
		VERIFY_NEAR(los[1]["distance_length"], 1500 * kFeetPerMeter, 2);
		VERIFY_NEAR(tds[1]["distance_length"], 700 * kFeetPerMeter, 2);
	}

	void runwayDefinitionIsRegisteredOncePerConnection() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		const size_t fields = FakeSim::state().facilityDefinitionFields.size();
		QVERIFY(fields > 0);
		touchDown(sim, onRunway(rwy, 500));
		sim.serviceLookups();
		QCOMPARE(FakeSim::state().facilityDefinitionFields.size(), fields);
	}

	// --- Fallbacks: no runway, no airport ---

	void noAirportsGivesCoordinateOnlyResult() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		QVERIFY(FakeSim::state().facilityDataRequests.empty());
		QVERIFY(trip(tripId)["departure_icao"].isNull());
		QVERIFY(liftoffs(tripId).value(0)["icao"].isNull());
		QCOMPARE(sim.status().departure.runway_act.index, -2);
		QVERIFY(!sim.status().lookup.pending);
		// The lookup slot is free again: the next touchdown gets its own lookup.
		touchDown(sim, onRunway(rwy, 500));
		QCOMPARE(FakeSim::state().facilitiesListRequests.size(), size_t(2));
	}

	void touchdownWithoutAirportClearsDestinationAirport() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		touchDown(sim, onRunway(rwy, 500));
		sim.serviceLookups();
		const QVariantMap t = trip(tripId);
		QVERIFY(t["destination_icao"].isNull());
		QVERIFY(t["destination_rwy"].isNull());
		QVERIFY(!t["destination_latitude"].isNull());
		QVERIFY(touchdowns(tripId).value(0)["icao"].isNull());
	}

	void nearRunwayButOffItGivesAirportWithoutRunway() {
		// 100 m past the runway end: inside the 200 m margin rectangle.
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 3100));
		sim.serviceLookups();
		const QVariantMap t = trip(tripId);
		QCOMPARE(t["departure_icao"].toString(), QStringLiteral("TEST"));
		QCOMPARE(t["departure_name"].toString(), QStringLiteral("Test Field"));
		QVERIFY(t["departure_rwy"].isNull());
		// The departure's own trip_liftoffs row gets the airport too.
		const QVariantMap lo = liftoffs(tripId).value(0);
		QCOMPARE(lo["icao"].toString(), QStringLiteral("TEST"));
		QCOMPARE(lo["airport_name"].toString(), QStringLiteral("Test Field"));
		QVERIFY(lo["runway"].isNull());
		QVERIFY(lo["distance_length"].isNull());
	}

	void touchAndGoNearRunwayGivesMarkerAirportWithoutRunway() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		touchDown(sim, onRunway(rwy, 500));
		liftOff(sim, onRunway(rwy, 3100));
		touchDown(sim, onRunway(rwy, 3150));
		sim.serviceLookups();
		const QList<QVariantMap> los = liftoffs(tripId);
		QCOMPARE(los[1]["icao"].toString(), QStringLiteral("TEST"));
		QCOMPARE(los[1]["airport_name"].toString(), QStringLiteral("Test Field"));
		QVERIFY(los[1]["runway"].isNull());
		const QList<QVariantMap> tds = touchdowns(tripId);
		QCOMPARE(tds[1]["icao"].toString(), QStringLiteral("TEST"));
		QVERIFY(tds[1]["runway"].isNull());
		QCOMPARE(trip(tripId)["destination_icao"].toString(), QStringLiteral("TEST"));
		QVERIFY(trip(tripId)["destination_rwy"].isNull());
	}

	void offAirportWithin5kmUsesNearestAirportIdentity() {
		// 1.5 km north of the runway: outside every margin, but the nearest
		// airport is within 5 km.
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1500, -1500));
		sim.serviceLookups();
		const QVariantMap t = trip(tripId);
		QCOMPARE(t["departure_icao"].toString(), QStringLiteral("TEST"));
		QVERIFY(t["departure_rwy"].isNull());
		const QVariantMap lo = liftoffs(tripId).value(0);
		QCOMPARE(lo["icao"].toString(), QStringLiteral("TEST"));
		QVERIFY(lo["runway"].isNull());
	}

	void offAirportBeyond5kmIsCoordinateOnly() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1500, -8000));
		sim.serviceLookups();
		QVERIFY(trip(tripId)["departure_icao"].isNull());
		QVERIFY(liftoffs(tripId).value(0)["icao"].isNull());
	}

	void walksToFartherCandidateWhenNearestHasNoMatchingRunway() {
		// The nearest airport reference point is closer, but only the second
		// airport's runway actually contains the liftoff point.
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		AirportSpec target = testAirport(rwy);
		target.latitude = 43.02; // reference point ~2 km away from the runway
		AirportSpec decoy = airportAt("NEAR", "Decoy Field", pointOnRunway(rwy, 1500, 300));
		decoy.runways = { eastWestRunway() };
		decoy.runways[0].latitude = 42.9; // its runway is ~11 km south
		sim.airports = { decoy, target };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1500));
		sim.serviceLookups();
		QCOMPARE(FakeSim::state().facilityDataRequests.size(), size_t(2));
		QCOMPARE(QString::fromStdString(FakeSim::state().facilityDataRequests[0].icao), QStringLiteral("NEAR"));
		QCOMPARE(trip(tripId)["departure_icao"].toString(), QStringLiteral("TEST"));
		QCOMPARE(trip(tripId)["departure_rwy"].toString(), QStringLiteral("09"));
	}

	void airportListSplitAcrossPacketsIsMergedBeforeDeciding() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		COORDINATE far1 = pointOnRunway(rwy, 1500, 20000);
		COORDINATE far2 = pointOnRunway(rwy, 1500, -30000);
		sim.airports = { airportAt("FARA", "Far A", far1), testAirport(rwy), airportAt("FARB", "Far B", far2) };
		sim.airportListChunkSize = 1;
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1500));
		sim.serviceLookups();
		QCOMPARE(QString::fromStdString(FakeSim::state().facilityDataRequests.at(0).icao), QStringLiteral("TEST"));
		QCOMPARE(trip(tripId)["departure_rwy"].toString(), QStringLiteral("09"));
	}

	void nonAirportFacilitiesAreIgnored() {
		// Idents that aren't 4 letters (vertiports, heliports) never match.
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { airportAt("VOLC2", "Vertiport", pointOnRunway(rwy, 1500)), testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1500));
		sim.serviceLookups();
		QCOMPARE(QString::fromStdString(FakeSim::state().facilityDataRequests.at(0).icao), QStringLiteral("TEST"));
		QCOMPARE(trip(tripId)["departure_icao"].toString(), QStringLiteral("TEST"));
	}

	// --- Failure and staleness handling ---

	void rejectedLookupRequestIsAbandonedAndQueueContinues() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		touchDown(sim, onRunway(rwy, 500)); // queued behind the departure
		const DWORD rejected = FakeSim::state().facilitiesListRequests.at(0);
		sim.rejectedRequests.insert(rejected);
		sim.send(exceptionPacket(3 /* UNRECOGNIZED_ID */, rejected));
		QCOMPARE(sim.status().departure.runway_act.index, -2);
		// The queued touchdown lookup was sent right away.
		QCOMPARE(FakeSim::state().facilitiesListRequests.size(), size_t(2));
		sim.serviceLookups();
		QVERIFY(trip(tripId)["departure_icao"].isNull());
		QCOMPARE(touchdowns(tripId).value(0)["runway"].toString(), QStringLiteral("09"));
	}

	void unrelatedExceptionDoesNotAbandonLookup() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.send(exceptionPacket(3, 1)); // SendID of an early registration call
		QVERIFY(sim.status().lookup.pending);
		sim.serviceLookups();
		QCOMPARE(trip(tripId)["departure_rwy"].toString(), QStringLiteral("09"));
	}

	void responseArrivingAfterTripEndedIsDropped() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		const int first = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		touchDown(sim, onRunway(rwy, 500));
		sim.endTrip(); // departure lookup still unanswered
		const int second = startOnRunway(sim, rwy);
		sim.serviceLookups();
		QVERIFY(trip(first)["departure_icao"].isNull());
		QVERIFY(trip(second)["departure_icao"].isNull());
		QVERIFY(!sim.status().lookup.pending);
	}

	void departureDeferredBehindStaleLookupStillResolves() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		touchDown(sim, onRunway(rwy, 500));
		sim.endTrip(); // first trip's departure lookup still in flight
		const int second = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1700));
		QVERIFY(sim.status().flight.departure_lookup_needed);
		sim.serviceLookups();
		QCOMPARE(trip(second)["departure_rwy"].toString(), QStringLiteral("09"));
		VERIFY_NEAR(liftoffs(second).value(0)["distance_length"], 1700 * kFeetPerMeter, 2);
	}

	// --- Malformed facility data, fed packet by packet ---

	void negativeRunwayCountIsTreatedAsNone() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		const AirportSpec airport = testAirport(rwy);
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		answerAirportList(sim, { airport });
		QCOMPARE(FakeSim::state().facilityDataRequests.size(), size_t(1));
		std::vector<char> header = facilityAirportPacket(airport);
		const int negative = -1;
		memcpy(header.data() + header.size() - sizeof(int), &negative, sizeof(int)); // N_RUNWAYS is the last field
		sim.send(header);
		sim.send(facilityRunwayPacket(0, 100, rwy));
		sim.send(facilityEndPacket());
		QVERIFY(!lastLogWith(log, { QStringLiteral("n_runways=-1") }).isEmpty());
		QVERIFY(!lastLogWith(log, { QStringLiteral("FACILITY_DATA_RUNWAY"), QStringLiteral("ItemIndex=0") }).isEmpty());
		// No runways, but the airport is right here: its identity, no runway.
		QCOMPARE(trip(tripId)["departure_icao"].toString(), QStringLiteral("TEST"));
		QVERIFY(trip(tripId)["departure_rwy"].isNull());
		QVERIFY(!sim.status().lookup.pending);
	}

	void runwayIndexBeyondTheAnnouncedCountIsDropped() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		const AirportSpec airport = testAirport(rwy); // announces 1 runway
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		answerAirportList(sim, { airport });
		sim.send(facilityAirportPacket(airport));
		sim.send(facilityRunwayPacket(1, 101, rwy)); // slot 0 never arrives
		sim.send(facilityEndPacket());
		QVERIFY(!lastLogWith(log, { QStringLiteral("FACILITY_DATA_RUNWAY"), QStringLiteral("ItemIndex=1") }).isEmpty());
		QCOMPARE(trip(tripId)["departure_icao"].toString(), QStringLiteral("TEST"));
		QVERIFY(trip(tripId)["departure_rwy"].isNull());
	}

	void orphanAndExtraPavementRecordsDoNotMoveTheThreshold() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		const AirportSpec airport = testAirport(rwy);
		sim.airports = { airport };
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		touchDown(sim, onRunway(rwy, 700));
		answerAirportList(sim, { airport });
		QCOMPARE(FakeSim::state().facilityDataRequests.size(), size_t(2));
		sim.send(facilityAirportPacket(airport));
		sim.send(facilityRunwayPacket(0, 100, rwy));
		sim.send(facilityPavementPacket(999, 900, 45, 1)); // no runway has this id
		sim.send(facilityPavementPacket(100, 300, 45, 1)); // primary
		sim.send(facilityPavementPacket(100, 200, 45, 1)); // secondary
		sim.send(facilityPavementPacket(100, 900, 45, 1)); // a third one
		sim.send(facilityEndPacket());
		// Same numbers as displacedThresholdShortensTouchdownDistance: 400 m
		// past the 300 m threshold, of 2500 m available.
		const QVariantMap td = touchdowns(tripId).value(0);
		QCOMPARE(td["runway"].toString(), QStringLiteral("09"));
		VERIFY_NEAR(td["distance_length"], 400 * kFeetPerMeter, 2);
		VERIFY_NEAR(td["distance_length_percent"], 400.0 / 2500, 0.001);
	}

	void facilityDataRejectedMidwayFreesTheRunwaysAndEndsTheLookup() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		const AirportSpec airport = testAirport(rwy);
		const int tripId = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		answerAirportList(sim, { airport });
		sim.send(facilityAirportPacket(airport));
		QVERIFY(sim.status().departure.runways != nullptr);
		sim.send(exceptionPacket(3, sim.status().lookup.send_id));
		QCOMPARE(sim.status().departure.runway_act.index, -2);
		QVERIFY(sim.status().departure.runways == nullptr);
		QVERIFY(!sim.status().lookup.pending);
		QVERIFY(trip(tripId)["departure_icao"].isNull());
	}

	// The first trip's facility data arrives after the next trip has already
	// lifted off and touched down. A new trip retargets the in-flight lookup
	// at touchdowns, so if the stale response weren't dropped it would resolve
	// the new trip's touchdown from the first trip's 1800 m position.
	void facilityDataArrivingAfterTripEndedIsDropped() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		const AirportSpec airport = testAirport(rwy);
		sim.airports = { airport };
		const int first = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		answerAirportList(sim, { airport });
		QCOMPARE(FakeSim::state().facilityDataRequests.size(), size_t(1));
		sim.endTrip(); // facility data still on its way
		const int second = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1700));
		touchDown(sim, onRunway(rwy, 600));
		sim.send(facilityAirportPacket(airport));
		sim.send(facilityRunwayPacket(0, 100, rwy));
		sim.send(facilityEndPacket());
		sim.serviceLookups();
		QVERIFY(trip(first)["departure_icao"].isNull());
		VERIFY_NEAR(liftoffs(second).value(0)["distance_length"], 1700 * kFeetPerMeter, 2);
		VERIFY_NEAR(touchdowns(second).value(0)["distance_length"], 600 * kFeetPerMeter, 2);
	}

	// A new SimConnect connection starts with no lookup in flight and an empty
	// facility definition table, so after a reconnect the next liftoff must
	// get its own lookup and register the runway definition again.
	void reconnectResetsLookupState() {
		FlightDriver sim;
		const RunwaySpec rwy = eastWestRunway();
		sim.airports = { testAirport(rwy) };
		startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1800));
		sim.serviceLookups();
		const size_t definitionFields = FakeSim::state().facilityDefinitionFields.size();
		QVERIFY(definitionFields > 0);
		touchDown(sim, onRunway(rwy, 500)); // left in flight
		QVERIFY(sim.status().lookup.pending);
		const size_t listRequests = FakeSim::state().facilitiesListRequests.size();

		sim.send(recvPacket(SIMCONNECT_RECV_ID_QUIT, sizeof(SIMCONNECT_RECV)));
		sim.pump();
		QVERIFY(waitFor([] { return FakeSim::state().openCalls == 2; }, 5000));
		QVERIFY(!sim.status().lookup.pending);
		QVERIFY(!sim.status().lookup.runway_definition_added);

		sim.send(recvPacket(SIMCONNECT_RECV_ID_OPEN, sizeof(SIMCONNECT_RECV)));
		sim.simEvent(EVENT_SIM, 1);
		const int second = startOnRunway(sim, rwy);
		liftOff(sim, onRunway(rwy, 1700));
		QCOMPARE(FakeSim::state().facilitiesListRequests.size(), listRequests + 1);
		sim.serviceLookups();
		QCOMPARE(FakeSim::state().facilityDefinitionFields.size(), 2 * definitionFields);
		QCOMPARE(trip(second)["departure_icao"].toString(), QStringLiteral("TEST"));
	}
};

QTEST_MAIN(TstAirportLookup)
#include "tst_airport_lookup.moc"
