// Cockpit event recording (trip_events) and the two flood-protection tiers.
// The tiers are timed with the real clock (500 ms / 5 s windows), so these
// tests wait in real time; quiet periods are resolved on the next sample tick.
#include "test_support.h"

#include "db.h"

#include <QThread>
#include <QtTest>

#include <set>

using namespace TestSupport;

namespace {

int eventRows(int tripId, const char* name) {
	return queryValue(QStringLiteral("SELECT COUNT(*) FROM trip_events WHERE trip=%1 AND event='%2'").arg(tripId).arg(name)).toInt();
}

int allEventRows() {
	return queryValue("SELECT COUNT(*) FROM trip_events").toInt();
}

// Waits out a quiet period, then sends a sample tick so the recorder resolves it.
void quiet(FlightDriver& sim, int ms) {
	QThread::msleep(ms);
	sim.tick();
}

// Settles every pending write so "no row" assertions aren't racing the writer.
void drainWrites(FlightDriver& sim) {
	quiet(sim, 600);
	QThread::msleep(200);
}

}

class TstEvents : public QObject {
	Q_OBJECT

private slots:
	void initTestCase() { isolateFiles(); }
	void init() { removeDatabase(); }

	void singleEventIsRecordedAfterQuietPeriod() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		QSignalSpy committed(&sim.bridge(), &RecorderBridge::eventCommitted);
		sim.simEvent(EVENT_GEAR_UP);
		QCOMPARE(committed.count(), 0); // held back until it proves not to be a burst
		quiet(sim, 600);
		QCOMPARE(committed.count(), 1);
		QCOMPARE(committed.at(0).at(0).toInt(), tripId);
		QCOMPARE(committed.at(0).at(2).toString(), QStringLiteral("Event: GEAR_UP"));
		QVERIFY(waitFor([tripId] { return eventRows(tripId, "GEAR_UP") == 1; }));
		const QVariantMap row = queryRows("SELECT * FROM trip_events").value(0);
		// Stamped with the latest sample's time when the event arrived.
		QCOMPARE(row["time_zulu"].toString(), QStringLiteral("2026-01-02T10:00:00.500+00:00_5"));
		QCOMPARE(row["event_seq"].toLongLong(), (qlonglong)committed.at(0).at(1).toULongLong());
	}

	void everyMappedEventIsRecordedUnderItsSimName() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		std::set<QString> expected;
		for (const FakeSim::MappedEvent& e : FakeSim::state().mappedEvents) {
			sim.simEvent(e.eventId);
			expected.insert(QString::fromStdString(e.simEventName));
		}
		quiet(sim, 600);
		QVERIFY(waitFor([&expected] { return allEventRows() == (int)expected.size(); }));
		std::set<QString> recorded;
		for (const QVariantMap& row : queryRows(QStringLiteral("SELECT event FROM trip_events WHERE trip=%1").arg(tripId)))
			recorded.insert(row["event"].toString());
		QCOMPARE(recorded, expected);
	}

	void eventWithoutTripIsNotRecorded() {
		FlightDriver sim;
		QSignalSpy committed(&sim.bridge(), &RecorderBridge::eventCommitted);
		sim.simEvent(EVENT_GEAR_UP);
		drainWrites(sim);
		QCOMPARE(committed.count(), 0);
		QCOMPARE(allEventRows(), 0);
	}

	void flapBurstsAreRecordedImmediately() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		for (int i = 0; i < 6; ++i)
			sim.simEvent(EVENT_FLAPS_INCR);
		QVERIFY(waitFor([tripId] { return eventRows(tripId, "FLAPS_INCR") == 6; }));
		drainWrites(sim);
		QCOMPARE(eventRows(tripId, "FLAPS_INCR"), 6);
	}

	void twoQuickRepeatsAreBothRecorded() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.simEvent(EVENT_GEAR_TOGGLE);
		sim.simEvent(EVENT_GEAR_TOGGLE);
		quiet(sim, 600);
		QVERIFY(waitFor([tripId] { return eventRows(tripId, "GEAR_TOGGLE") == 2; }));
	}

	void fastBurstIsSuppressedThenRecoversAfterQuiet() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		for (int i = 0; i < 5; ++i)
			sim.simEvent(EVENT_AP_MASTER);
		drainWrites(sim);
		QCOMPARE(eventRows(tripId, "AP_MASTER"), 0);
		// The burst is over: the next single occurrence is recorded normally.
		sim.simEvent(EVENT_AP_MASTER);
		quiet(sim, 600);
		QVERIFY(waitFor([tripId] { return eventRows(tripId, "AP_MASTER") == 1; }));
	}

	void slowRepeatsAreRetractedThenSuppressedThenRecover() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		QSignalSpy retracted(&sim.bridge(), &RecorderBridge::eventsRetracted);
		// Three occurrences 0.6 s apart: each passes the fast-burst tier, but
		// three within 5 s trip the slow-flood tier.
		for (int i = 0; i < 3; ++i) {
			sim.simEvent(EVENT_AP_HDG_HOLD);
			quiet(sim, 600);
		}
		QCOMPARE(retracted.count(), 1);
		QCOMPARE(retracted.at(0).at(0).value<QList<quint64>>().size(), 3);
		QVERIFY(waitFor([tripId] { return eventRows(tripId, "AP_HDG_HOLD") == 0; }));
		// Still inside the 5 s window: suppressed.
		sim.simEvent(EVENT_AP_HDG_HOLD);
		drainWrites(sim);
		QCOMPARE(eventRows(tripId, "AP_HDG_HOLD"), 0);
		// After 5 s of quiet it is recorded again.
		quiet(sim, 5200);
		sim.simEvent(EVENT_AP_HDG_HOLD);
		quiet(sim, 600);
		QVERIFY(waitFor([tripId] { return eventRows(tripId, "AP_HDG_HOLD") == 1; }));
	}

	void eventResolvedAfterTripEndsBelongsToThatTrip() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.simEvent(EVENT_PARKING_BRAKES);
		sim.endTrip(); // before the event's 500 ms quiet period elapsed
		quiet(sim, 600);
		QVERIFY(waitFor([tripId] { return eventRows(tripId, "PARKING_BRAKES") == 1; }));
	}

	void pendingEventIsWrittenOnShutdown() {
		int tripId = 0;
		{
			FlightDriver sim;
			tripId = sim.startTrip();
			sim.simEvent(EVENT_SPOILERS_ARM_ON);
		}
		QCOMPARE(eventRows(tripId, "SPOILERS_ARM_ON"), 1);
	}

	void eventForDeletedTripIsDropped() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.simEvent(EVENT_GEAR_DOWN);
		sqlite3* db = connect_db_readwrite();
		QVERIFY(db);
		sqlite3_exec(db, QStringLiteral("DELETE FROM trips WHERE id=%1").arg(tripId).toUtf8().constData(), nullptr, nullptr, nullptr);
		sqlite3_close(db);
		drainWrites(sim);
		QCOMPARE(allEventRows(), 0);
	}

	void crashIsLoggedNotRecorded() {
		FlightDriver sim;
		sim.startTrip();
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		sim.simEvent(EVENT_CRASHED);
		QVERIFY(log.contains(QVariantList{ QStringLiteral("Plane crashed!") }));
		drainWrites(sim);
		QCOMPARE(allEventRows(), 0);
	}

	void unknownEventIdIsLogged() {
		FlightDriver sim;
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		sim.simEvent(999);
		QVERIFY(log.contains(QVariantList{ QStringLiteral("Unknown event ID: 999") }));
	}
};

QTEST_MAIN(TstEvents)
#include "tst_events.moc"
