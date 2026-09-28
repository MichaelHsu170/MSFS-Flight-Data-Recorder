// Connection, SimConnect registration, trip start/stop and sample recording,
// driven through a real RecorderBridge wired to the fake SimConnect.
#include "test_support.h"

#include "app_settings.h"

#include <QSettings>
#include <QtTest>

#include <set>

using namespace TestSupport;

class TstRecording : public QObject {
	Q_OBJECT

private:
	static int tripCount() { return queryValue("SELECT COUNT(*) FROM trips").toInt(); }
	static int sampleCount(int tripId) {
		return queryValue(QStringLiteral("SELECT COUNT(*) FROM trip_data WHERE trip=%1").arg(tripId)).toInt();
	}

private slots:
	void initTestCase() { isolateFiles(); }
	void init() {
		removeDatabase();
		removeSettings();
	}

	// --- Connection and SimConnect registration ---

	void connectRegistersSystemEventsAndDataRequest() {
		FlightDriver sim;
		const FakeSim::State& fake = FakeSim::state();
		QCOMPARE(fake.openCalls, 1);
		QCOMPARE(fake.systemEvents, (std::vector<std::string>{ "Sim", "Pause", "Crashed" }));
		QCOMPARE(fake.dataRequests, 1);
	}

	void dataDefinitionMatchesSampleLayout() {
		// The bytes SimConnect sends are laid out by the registered data
		// definitions; MyDispatchProc copies them straight into
		// FLIGHT_DATA_RECORD. Their total size must equal the copied size.
		FlightDriver sim;
		size_t wireBytes = 0;
		for (const FakeSim::DataDefinition& def : FakeSim::state().dataDefinitions) {
			QCOMPARE(def.defineId, (DWORD)DEFINITION_FLIGHT);
			switch (def.datumType) {
			case SIMCONNECT_DATATYPE_FLOAT64: wireBytes += 8; break;
			case SIMCONNECT_DATATYPE_STRING8: wireBytes += 8; break;
			case SIMCONNECT_DATATYPE_STRING32: wireBytes += 32; break;
			case SIMCONNECT_DATATYPE_STRING64: wireBytes += 64; break;
			case SIMCONNECT_DATATYPE_STRING256: wireBytes += 256; break;
			default: QFAIL(qPrintable(QStringLiteral("unexpected datatype for %1").arg(QString::fromStdString(def.datumName))));
			}
		}
		QCOMPARE(wireBytes, sizeof(FLIGHT_DATA_RECORD) - sizeof(double));
	}

	void everyMappedEventJoinsTheNotificationGroup() {
		FlightDriver sim;
		const FakeSim::State& fake = FakeSim::state();
		std::set<DWORD> mapped;
		std::set<std::string> names;
		for (const FakeSim::MappedEvent& e : fake.mappedEvents) {
			mapped.insert(e.eventId);
			names.insert(e.simEventName);
		}
		QCOMPARE(mapped.size(), fake.mappedEvents.size());
		QCOMPARE(names.size(), fake.mappedEvents.size());
		QCOMPARE(std::set<DWORD>(fake.notificationGroupEvents.begin(), fake.notificationGroupEvents.end()), mapped);
		QCOMPARE(fake.notificationGroupEvents.size(), fake.mappedEvents.size());
	}

	void openPacketReportsConnected() {
		FakeSim::reset();
		RecorderBridge bridge;
		QSignalSpy connected(&bridge, &RecorderBridge::connectionChanged);
		FakeSim::queue(recvPacket(SIMCONNECT_RECV_ID_OPEN, sizeof(SIMCONNECT_RECV)));
		QMetaObject::invokeMethod(&bridge, "pollDispatch", Qt::DirectConnection);
		QCOMPARE(connected.count(), 1);
		QCOMPARE(connected.at(0).at(0).toBool(), true);
	}

	void failedOpenIsRetried() {
		FakeSim::reset();
		FakeSim::state().openFails = true;
		RecorderBridge bridge;
		QCOMPARE(FakeSim::state().openCalls, 1);
		QVERIFY(FakeSim::state().dataDefinitions.empty());
		FakeSim::state().openFails = false;
		// connectTimer_ retries every 2 s.
		QVERIFY(waitFor([] { return !FakeSim::state().dataDefinitions.empty(); }, 5000));
		QCOMPARE(FakeSim::state().openCalls, 2);
	}

	void quitPacketDisconnectsAndReconnects() {
		FlightDriver sim;
		QSignalSpy connected(&sim.bridge(), &RecorderBridge::connectionChanged);
		sim.send(recvPacket(SIMCONNECT_RECV_ID_QUIT, sizeof(SIMCONNECT_RECV)));
		QCOMPARE(connected.count(), 1);
		QCOMPARE(connected.at(0).at(0).toBool(), false);
		sim.pump(); // sees quit -> shutdown()
		QCOMPARE(FakeSim::state().closeCalls, 1);
		QVERIFY(waitFor([] { return FakeSim::state().openCalls == 2; }, 5000));
	}

	void failedDispatchIsTreatedAsDisconnect() {
		FlightDriver sim;
		QSignalSpy connected(&sim.bridge(), &RecorderBridge::connectionChanged);
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		FakeSim::state().dispatchFails = true;
		sim.pump();
		QCOMPARE(connected.count(), 1);
		QCOMPARE(connected.at(0).at(0).toBool(), false);
		QCOMPARE(log.count(), 1);
		QCOMPARE(log.at(0).at(0).toString(), QStringLiteral("Disconnected from Microsoft Flight Simulator"));
		sim.pump();
		QCOMPARE(FakeSim::state().closeCalls, 1);
	}

	// --- When a trip starts ---

	void noTripWhileEnginesOff() {
		FlightDriver sim;
		sim.ticks(5);
		QCOMPARE(tripCount(), 0);
		QVERIFY(!sim.bridge().isRecording());
	}

	void noTripWhenSimNotRunning() {
		FlightDriver sim;
		sim.simEvent(EVENT_SIM, 0);
		sim.setEngines(true);
		sim.ticks(3);
		QCOMPARE(tripCount(), 0);
	}

	void noTripWhilePaused() {
		FlightDriver sim;
		sim.simEvent(EVENT_PAUSE, 1);
		sim.setEngines(true);
		sim.ticks(3);
		QCOMPARE(tripCount(), 0);
		sim.simEvent(EVENT_PAUSE, 0);
		sim.tick();
		QCOMPARE(tripCount(), 1);
	}

	void noTripWhenSurfaceUnknown() {
		// surface_type 255 = not in a loaded flight (e.g. main menu).
		FlightDriver sim;
		sim.record.surface_type = 255;
		sim.setEngines(true);
		sim.ticks(3);
		QCOMPARE(tripCount(), 0);
	}

	void noTripWhenAirborneWithEnginesRunning() {
		FlightDriver sim;
		sim.setOnGround(false);
		sim.setEngines(true);
		sim.ticks(3);
		QCOMPARE(tripCount(), 0);
	}

	void eitherEngineStartsATrip() {
		FlightDriver sim;
		sim.record.eng_combustion_2 = 1;
		sim.tick();
		QCOMPARE(tripCount(), 1);
	}

	void disablingRecordingPreventsTripAndPersists() {
		FlightDriver sim;
		QSignalSpy changed(&sim.bridge(), &RecorderBridge::recordingEnabledChanged);
		sim.bridge().setRecordingEnabled(false);
		QCOMPARE(changed.count(), 1);
		QVERIFY(!sim.bridge().isRecordingEnabled());
		QCOMPARE(AppSettings::instance().recordingEnabled(), false);
		sim.setEngines(true);
		sim.ticks(3);
		QCOMPARE(tripCount(), 0);
		sim.bridge().setRecordingEnabled(true);
		sim.tick();
		QCOMPARE(tripCount(), 1);
	}

	void recordingToggleIgnoredWhileRecording() {
		FlightDriver sim;
		sim.startTrip();
		QSignalSpy changed(&sim.bridge(), &RecorderBridge::recordingEnabledChanged);
		sim.bridge().setRecordingEnabled(false);
		QCOMPARE(changed.count(), 0);
		QVERIFY(sim.bridge().isRecordingEnabled());
	}

	void recordingEnabledReadFromSettingsAtStartup() {
		AppSettings::instance().setRecordingEnabled(false);
		FlightDriver sim;
		QVERIFY(!sim.bridge().isRecordingEnabled());
		sim.setEngines(true);
		sim.tick();
		QCOMPARE(tripCount(), 0);
	}

	void tripStartWritesTripRowAndSignals() {
		FlightDriver sim;
		QSignalSpy started(&sim.bridge(), &RecorderBridge::recordingStateChanged);
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		const int tripId = sim.startTrip();
		QVERIFY(tripId > 0);
		QVERIFY(sim.bridge().isRecording());
		QCOMPARE(sim.bridge().currentTripId(), tripId);
		QCOMPARE(started.count(), 1);
		QCOMPARE(started.at(0).at(0).toInt(), tripId);
		QVERIFY(log.contains(QVariantList{ QStringLiteral("Recording started") }));

		const QVariantMap trip = queryRows(QStringLiteral("SELECT * FROM trips WHERE id=%1").arg(tripId)).value(0);
		QCOMPARE(trip["title"].toString(), QStringLiteral("Test Aircraft"));
		QCOMPARE(trip["atc_airline"].toString(), QStringLiteral("TESTAIR"));
		QCOMPARE(trip["atc_flight_number"].toString(), QStringLiteral("123"));
		QCOMPARE(trip["atc_id"].toString(), QStringLiteral("N123TA"));
		QCOMPARE(trip["atc_model"].toString(), QStringLiteral("A320"));
		QCOMPARE(trip["atc_type"].toString(), QStringLiteral("AIRBUS"));
		QCOMPARE(trip["departure_latitude"].toDouble(), 43.0);
		QCOMPARE(trip["departure_longitude"].toDouble(), 1.0);
		QCOMPARE(trip["departure_zulu_time"].toString(), QStringLiteral("2026-01-02T10:00:00.500+00:00_5"));
		QCOMPARE(trip["departure_local_time"].toString(), QStringLiteral("2026-01-02T10:00:00.500+00:00_5"));
		QVERIFY(trip["destination_zulu_time"].isNull());
		QVERIFY(trip["group_id"].isNull());
	}

	// --- Samples ---

	void everyTickAtTheIntervalIsRecorded() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.ticks(9, 0.5);
		sim.endTrip();
		// startTrip()'s tick and the 9 after it; endTrip()'s tick stops the
		// trip before sampling (see tripEndStopsBeforeSampling).
		QCOMPARE(sampleCount(tripId), 10);
	}

	void ticksFasterThanTheIntervalAreSkipped() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.ticks(10, 0.25);
		sim.endTrip();
		QCOMPARE(sampleCount(tripId), 1 + 5);
	}

	void sampleIntervalReadFromSettings() {
		QSettings settings(AppSettings::filePath(), QSettings::IniFormat);
		settings.setValue(QStringLiteral("recording/sample_interval_ms"), 1000);
		settings.sync();
		FlightDriver sim;
		QCOMPARE(sim.status().sample_interval_ms, 1000);
		const int tripId = sim.startTrip();
		sim.ticks(10, 0.5);
		sim.endTrip();
		QCOMPARE(sampleCount(tripId), 1 + 5);
	}

	void samplingContinuesAcrossMidnight() {
		FlightDriver sim;
		sim.record.time_zulu.time_day = 86399.0;
		sim.record.time_local.time_day = 86399.0;
		const int tripId = sim.startTrip();
		sim.ticks(4, 0.5);
		sim.endTrip();
		QCOMPARE(sampleCount(tripId), 5);
		QCOMPARE(queryValue(QStringLiteral("SELECT zulu_time FROM trip_data WHERE trip=%1 ORDER BY rowid DESC LIMIT 1").arg(tripId)).toString(),
			QStringLiteral("2026-01-03T00:00:01.500+00:00_5"));
	}

	void noSamplesWhilePaused() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.simEvent(EVENT_PAUSE, 1);
		sim.ticks(4);
		sim.simEvent(EVENT_PAUSE, 0);
		sim.tick();
		sim.endTrip();
		QCOMPARE(sampleCount(tripId), 2);
	}

	void pitchAndBankAreStoredInAviationConvention() {
		// SimConnect reports nose-down/left-wing-down as positive.
		FlightDriver sim;
		sim.record.plane_pitch_degrees = -5;
		sim.record.plane_bank_degrees = 10;
		sim.record.plane_touchdown_pitch_degrees = -3;
		sim.record.plane_touchdown_bank_degrees = 2;
		const int tripId = sim.startTrip();
		sim.endTrip();
		const QVariantMap row = queryRows(QStringLiteral("SELECT * FROM trip_data WHERE trip=%1 LIMIT 1").arg(tripId)).value(0);
		QCOMPARE(row["plane_pitch_degrees"].toDouble(), 5.0);
		QCOMPARE(row["plane_bank_degrees"].toDouble(), -10.0);
		QCOMPARE(row["plane_touchdown_pitch_degrees"].toDouble(), 3.0);
		QCOMPARE(row["plane_touchdown_bank_degrees"].toDouble(), -2.0);
	}

	void liveSampleSignalsCarryTheSample() {
		FlightDriver sim;
		QSignalSpy points(&sim.bridge(), &RecorderBridge::liveDataPoint);
		QSignalSpy updated(&sim.bridge(), &RecorderBridge::sampleUpdated);
		sim.record.plane_altitude = 1234.7;
		sim.record.airspeed_indicated = 140.9;
		sim.record.ground_velocity = 150;
		sim.record.vertical_speed = -700;
		sim.record.turb_eng_n1_1 = 85.5;
		sim.record.gear_is_on_ground_1 = 1;
		sim.startTrip();
		QCOMPARE(points.count(), 1);
		QCOMPARE(updated.count(), 1);
		const TripSamplePoint p = points.at(0).at(0).value<TripSamplePoint>();
		QCOMPARE(p.latitude, 43.0);
		QCOMPARE(p.longitude, 1.0);
		QCOMPARE(p.altitude, 1234);
		QCOMPARE(p.airspeed, 140);
		QCOMPARE(p.groundSpeed, 150);
		QCOMPARE(p.verticalSpeed, -700);
		QCOMPARE(p.n1_1, 85.5);
		QCOMPARE(p.gearOnGround[0], false);
		QCOMPARE(p.gearOnGround[1], true);
		QCOMPARE(p.zuluTime, QStringLiteral("2026-01-02T10:00:00.500+00:00_5"));
		const FLIGHT_DATA& data = sim.bridge().currentData();
		QCOMPARE(data.altitude, 1234);
		QCOMPARE(data.speed, 140);
		QCOMPARE(data.heading, 90);
		QCOMPARE(data.vertical_speed, -700);
	}

	void currentDataUpdatesEvenWithoutATrip() {
		FlightDriver sim;
		sim.record.plane_altitude = 500;
		sim.tick();
		QCOMPARE(sim.bridge().currentData().altitude, 500);
	}

	// --- When a trip ends ---

	void engineShutdownEndsTrip() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		QSignalSpy ended(&sim.bridge(), &RecorderBridge::tripEnded);
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		sim.ticks(2);
		sim.endTrip();
		QVERIFY(!sim.bridge().isRecording());
		QCOMPARE(sim.bridge().currentTripId(), -1);
		QVERIFY(waitFor([&ended] { return ended.count() == 1; }));
		QCOMPARE(ended.at(0).at(0).toInt(), tripId);
		QVERIFY(waitFor([&log] { return log.contains(QVariantList{ QStringLiteral("Recording stopped") }); }));
		const QVariantMap trip = queryRows(QStringLiteral("SELECT * FROM trips WHERE id=%1").arg(tripId)).value(0);
		// Arrival time is the last recorded sample's time (the tick before shutdown).
		QCOMPARE(trip["destination_zulu_time"].toString(), QStringLiteral("2026-01-02T10:00:01.500+00:00_5"));
		QCOMPARE(trip["destination_local_time"].toString(), QStringLiteral("2026-01-02T10:00:01.500+00:00_5"));
	}

	void tripEndStopsBeforeSampling() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.endTrip();
		QCOMPARE(sampleCount(tripId), 1);
	}

	void leavingTheFlightEndsTrip() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.simEvent(EVENT_SIM, 0);
		QVERIFY(!sim.bridge().isRecording());
		QVERIFY(waitFor([&sim, tripId] { return !sim.bridge().isTripFlushing(tripId); }));
		QVERIFY(!queryValue(QStringLiteral("SELECT destination_zulu_time FROM trips WHERE id=%1").arg(tripId)).isNull());
	}

	void closingTheAppEndsTheTrip() {
		int tripId = 0;
		{
			FlightDriver sim;
			tripId = sim.startTrip();
			sim.tick();
		}
		QVERIFY(!queryValue(QStringLiteral("SELECT destination_zulu_time FROM trips WHERE id=%1").arg(tripId)).isNull());
		QCOMPARE(sampleCount(tripId), 2);
	}

	void simulatorQuitEndsTheTrip() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.send(recvPacket(SIMCONNECT_RECV_ID_QUIT, sizeof(SIMCONNECT_RECV)));
		sim.pump();
		QVERIFY(!sim.bridge().isRecording());
		QVERIFY(waitFor([&sim, tripId] { return !sim.bridge().isTripFlushing(tripId); }));
		QVERIFY(!queryValue(QStringLiteral("SELECT destination_zulu_time FROM trips WHERE id=%1").arg(tripId)).isNull());
	}

	void secondTripGetsItsOwnRows() {
		FlightDriver sim;
		const int first = sim.startTrip();
		sim.ticks(2);
		sim.endTrip();
		sim.ticks(2);
		const int second = sim.startTrip();
		sim.ticks(3);
		sim.endTrip();
		QVERIFY(second != first);
		QCOMPARE(tripCount(), 2);
		QCOMPARE(sampleCount(first), 3);
		QCOMPARE(sampleCount(second), 4);
	}

	void unknownSimObjectRequestIsIgnored() {
		FlightDriver sim;
		std::vector<char> packet = samplePacket(sim.record);
		reinterpret_cast<SIMCONNECT_RECV_SIMOBJECT_DATA*>(packet.data())->dwRequestID = 99;
		sim.setEngines(true);
		sim.send(packet);
		QCOMPARE(tripCount(), 0);
	}
};

QTEST_MAIN(TstRecording)
#include "tst_recording.moc"
