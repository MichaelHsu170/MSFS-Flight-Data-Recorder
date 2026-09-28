// Database layer: schema creation and upgrade, a full write/read round trip
// of every trip_data field, the Trip History queries, event rows and trip
// deletion.
#include "test_support.h"

#include "db.h"
#include "db_history.h"
#include "trip_data_fields.h"

#include <QtTest>

#include <set>

using namespace TestSupport;

namespace {

void exec(sqlite3* db, const char* sql) {
	char* err = nullptr;
	if (sqlite3_exec(db, sql, nullptr, nullptr, &err) != SQLITE_OK) {
		const QString message = QString::fromUtf8(err ? err : "?");
		sqlite3_free(err);
		QFAIL(qPrintable(message + " in: " + sql));
	}
}

std::set<QString> names(const char* sql) {
	std::set<QString> out;
	for (const QVariantMap& row : queryRows(sql))
		out.insert(row.first().toString());
	return out;
}

// A migrated, empty database with a writable connection the test closes.
sqlite3* freshDatabase() {
	migrate_db();
	return connect_db_readwrite();
}

TripSamplePoint point(const char* zulu, double lat) {
	TripSamplePoint p;
	p.zuluTime = QString::fromLatin1(zulu);
	p.latitude = lat;
	return p;
}

}

class TstDatabase : public QObject {
	Q_OBJECT

private slots:
	void initTestCase() { isolateFiles(); }
	void init() { removeDatabase(); }

	// --- Schema ---

	void connectionsFailWithoutADatabase() {
		QVERIFY(connect_db_readonly() == nullptr);
		QVERIFY(connect_db_readwrite() == nullptr);
	}

	void migrateCreatesAllTablesAndIndexes() {
		migrate_db();
		QCOMPARE(names("SELECT name FROM sqlite_master WHERE type='table' AND name NOT LIKE 'sqlite_%'"),
			(std::set<QString>{ "trips", "trip_data", "trip_events", "trip_liftoffs", "trip_touchdowns", "trip_groups" }));
		QCOMPARE(names("SELECT name FROM sqlite_master WHERE type='index' AND name LIKE 'idx_%'"),
			(std::set<QString>{ "idx_trip_data_trip", "idx_trip_events_trip", "idx_trip_events_event_seq",
				"idx_trip_liftoffs_trip", "idx_trip_touchdowns_trip", "idx_trips_group", "idx_trip_groups_name" }));
	}

	void migrateIsRepeatable() {
		migrate_db();
		migrate_db();
		QCOMPARE(queryValue("SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name='trips'").toInt(), 1);
	}

	void migrateAddsColumnsMissingFromOlderDatabases() {
		char path[MAX_PATH];
		resolve_db_path(path, sizeof(path));
		sqlite3* db = nullptr;
		QCOMPARE(sqlite3_open(path, &db), SQLITE_OK);
		exec(db, "CREATE TABLE trips (id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL UNIQUE, title VARCHAR(256) NOT NULL);");
		exec(db, "INSERT INTO trips (title) VALUES ('Old Trip');");
		sqlite3_close(db);

		migrate_db();
		const QVariantMap row = queryRows("SELECT * FROM trips").value(0);
		QCOMPARE(row["title"].toString(), QStringLiteral("Old Trip"));
		QVERIFY(row.contains("group_id"));
		QVERIFY(row.contains("departure_name"));
		QVERIFY(row["group_id"].isNull());
	}

	void groupNamesAreUniqueIgnoringAsciiCase() {
		sqlite3* db = freshDatabase();
		exec(db, "INSERT INTO trip_groups (name) VALUES ('Training');");
		QVERIFY(sqlite3_exec(db, "INSERT INTO trip_groups (name) VALUES ('TRAINING');", nullptr, nullptr, nullptr) != SQLITE_OK);
		sqlite3_close(db);
	}

	// --- trip_data round trip ---

	void everyTripDataFieldSurvivesWriteAndReadBack() {
		// Every numeric field gets a distinct non-integral value and the
		// bools a fixed pattern; the sample is recorded, then read back through
		// queryTripData(). The live signal's decoding (RecorderBridge) and the
		// stored/reloaded decoding (db.cpp + db_history.cpp) must agree field
		// for field -- they are separate code paths over the same field list.
		FlightDriver sim;
		double value = 1.25;
#define SET_NUM(dbColumn, memberExpr) sim.record.memberExpr = (value += 1.0);
		TRIP_DATA_NUM_FIELDS(SET_NUM)
#undef SET_NUM
		int boolIndex = 0;
#define SET_BOOL(name, group, bit) sim.record.name = (boolIndex++ % 3 == 0) ? 1 : 0;
		TRIP_DATA_BOOL_FIELDS(SET_BOOL)
#undef SET_BOOL
		sim.record.sim_on_ground = 1;
		sim.record.eng_combustion_1 = 1;
		const FLIGHT_DATA_RECORD sent = sim.record;

		QSignalSpy live(&sim.bridge(), &RecorderBridge::liveDataPoint);
		sim.tick();
		const int trip = sim.status().id_trip;
		QVERIFY(trip > 0);
		sim.endTrip();
		QCOMPARE(live.count(), 1);
		const TripSamplePoint livePoint = live.at(0).at(0).value<TripSamplePoint>();

		sqlite3* db = connect_db_readonly();
		const TripDataset stored = queryTripData(db, trip);
		sqlite3_close(db);
		QCOMPARE(stored.points.size(), size_t(1));
		const TripSamplePoint& p = stored.points[0];

		// Expected values: what was sent, with pitch/bank sign-flipped to
		// aviation convention by the recorder.
		FLIGHT_DATA_RECORD expected = sent;
		expected.plane_pitch_degrees = -sent.plane_pitch_degrees;
		expected.plane_bank_degrees = -sent.plane_bank_degrees;
		expected.plane_touchdown_pitch_degrees = -sent.plane_touchdown_pitch_degrees;
		expected.plane_touchdown_bank_degrees = -sent.plane_touchdown_bank_degrees;
		int i = 0;
#define CHECK_NUM(dbColumn, memberExpr) \
		QVERIFY2(i < (int)p.rawNums.size() && p.rawNums[i] == expected.memberExpr, #dbColumn " (stored)"); \
		QVERIFY2(i < (int)livePoint.rawNums.size() && livePoint.rawNums[i] == expected.memberExpr, #dbColumn " (live)"); \
		++i;
		TRIP_DATA_NUM_FIELDS(CHECK_NUM)
#undef CHECK_NUM
		QCOMPARE((int)p.rawNums.size(), i);

		uint32_t groups[4] = {};
#define PACK(name, group, bit) if (expected.name != 0) groups[group] |= (1u << (bit));
		TRIP_DATA_BOOL_FIELDS(PACK)
#undef PACK
		QCOMPARE(p.boolGroup1, groups[1]);
		QCOMPARE(p.boolGroup2, groups[2]);
		QCOMPARE(p.boolGroup3, groups[3]);
		QCOMPARE(livePoint.boolGroup1, groups[1]);
		QCOMPARE(livePoint.boolGroup2, groups[2]);
		QCOMPARE(livePoint.boolGroup3, groups[3]);

		// The named convenience fields decode identically on both paths.
		QCOMPARE(p.latitude, livePoint.latitude);
		QCOMPARE(p.longitude, livePoint.longitude);
		QCOMPARE(p.altitude, livePoint.altitude);
		QCOMPARE(p.airspeed, livePoint.airspeed);
		QCOMPARE(p.groundSpeed, livePoint.groundSpeed);
		QCOMPARE(p.verticalSpeed, livePoint.verticalSpeed);
		QCOMPARE(p.n1_1, livePoint.n1_1);
		QCOMPARE(p.n1_2, livePoint.n1_2);
		QCOMPARE(p.n2_1, livePoint.n2_1);
		QCOMPARE(p.n2_2, livePoint.n2_2);
		QCOMPARE(p.gearHandlePosition, livePoint.gearHandlePosition);
		for (int g = 0; g < 3; ++g) {
			QCOMPARE(p.gearPosition[g], livePoint.gearPosition[g]);
			QCOMPARE(p.gearOnGround[g], livePoint.gearOnGround[g]);
		}
		QCOMPARE(p.brakeIndicator, livePoint.brakeIndicator);
		QCOMPARE(p.flapsHandleIndex, livePoint.flapsHandleIndex);
		QCOMPARE(p.spoilersHandlePosition, livePoint.spoilersHandlePosition);
		QCOMPARE(p.fuelTotalQuantityWeight, livePoint.fuelTotalQuantityWeight);
		QCOMPARE(p.pitchDegrees, livePoint.pitchDegrees);
		QCOMPARE(p.bankDegrees, livePoint.bankDegrees);
		QCOMPARE(p.zuluTime, livePoint.zuluTime);
		QCOMPARE(p.localTime, livePoint.localTime);
	}

	// --- Trip History queries ---

	void tripListIsNewestFirstWithStatusAndGroup() {
		sqlite3* db = freshDatabase();
		exec(db, "INSERT INTO trip_groups (id, name) VALUES (5, 'Training');");
		exec(db, "INSERT INTO trips (id,title,atc_airline,atc_flight_number,atc_id,atc_model,atc_type,departure_latitude,departure_longitude,"
			"departure_zulu_time,departure_local_time,destination_zulu_time,departure_icao,departure_name,departure_region,departure_rwy,"
			"destination_icao,destination_name,destination_region,destination_rwy,destination_latitude,destination_longitude,group_id) VALUES "
			"(1,'Done','AIR','1','ID','M','T',1.5,2.5,'2026-01-01T10:00:00.000+00:00_4','x','2026-01-01T11:00:00.000+00:00_4',"
			"'AAAA','Alpha','AA','09','BBBB','Bravo','BB','27',3.5,4.5,5),"
			"(2,'Open','AIR','2','ID','M','T',0,0,'2026-01-02T10:00:00.000+00:00_5','x',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL),"
			"(3,'Live','AIR','3','ID','M','T',0,0,'2026-01-03T10:00:00.000+00:00_6','x',NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL);");
		const std::vector<TripSummary> trips = queryAllTrips(db, 3);
		sqlite3_close(db);
		QCOMPARE(trips.size(), size_t(3));
		QCOMPARE(trips[0].id, 3);
		QVERIFY(trips[0].status == TripStatus::Live);
		QVERIFY(trips[1].status == TripStatus::Open);
		QVERIFY(trips[2].status == TripStatus::Completed);
		const TripSummary& done = trips[2];
		QCOMPARE(done.title, QStringLiteral("Done"));
		QCOMPARE(done.atcAirline, QStringLiteral("AIR"));
		QCOMPARE(done.departureIcao, QStringLiteral("AAAA"));
		QCOMPARE(done.departureName, QStringLiteral("Alpha"));
		QCOMPARE(done.departureRegion, QStringLiteral("AA"));
		QCOMPARE(done.departureRwy, QStringLiteral("09"));
		QCOMPARE(done.destinationIcao, QStringLiteral("BBBB"));
		QCOMPARE(done.destinationName, QStringLiteral("Bravo"));
		QCOMPARE(done.destinationRegion, QStringLiteral("BB"));
		QCOMPARE(done.destinationRwy, QStringLiteral("27"));
		QCOMPARE(done.departureLat, 1.5);
		QCOMPARE(done.departureLng, 2.5);
		QCOMPARE(done.destinationLat, 3.5);
		QCOMPARE(done.destinationLng, 4.5);
		QCOMPARE(done.groupId, 5);
		QCOMPARE(done.groupName, QStringLiteral("Training"));
		QCOMPARE(trips[1].groupId, 0);
		QCOMPARE(trips[1].groupName, QString());
	}

	void liftoffAndTouchdownRowsReadBack() {
		sqlite3* db = freshDatabase();
		exec(db, "INSERT INTO trip_liftoffs (trip,airspeed_indicated,vertical_speed,plane_pitch_degrees,plane_bank_degrees,heading_indicator,"
			"plane_latitude,plane_longitude,icao,airport_name,runway,runway_heading,distance_length,distance_width,distance_length_percent,"
			"distance_width_percent,wind_direction,wind_velocity,time_zulu,time_local,analysis_report) VALUES "
			"(7,150,500,8.5,-1.5,92,43.1,1.1,'TEST','Test Field','09',91,1800.5,-3.25,0.6,-0.14,270,12,'z1','l1','report'),"
			"(7,140,400,7,0,90,43.2,1.2,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,NULL,'z2','l2',NULL);");
		exec(db, "INSERT INTO trip_touchdowns (trip,airspeed_indicated,vertical_speed,g_force,plane_pitch_degrees,plane_bank_degrees,heading_indicator,"
			"plane_latitude,plane_longitude,icao,airport_name,runway,runway_heading,distance_length,distance_width,distance_length_percent,"
			"distance_width_percent,wind_direction,wind_velocity,time_zulu,time_local) VALUES "
			"(7,130,-180,1.3,3,0.5,271,43.3,1.3,'TEST','Test Field','27',270,-50,2,-0.02,0.09,260,8,'z3','l3');");
		const std::vector<LiftoffPoint> los = queryLiftoffs(db, 7);
		const std::vector<TouchdownPoint> tds = queryTouchdowns(db, 7);
		sqlite3_close(db);
		QCOMPARE(los.size(), size_t(2));
		QCOMPARE(los[0].icao, QStringLiteral("TEST"));
		QCOMPARE(los[0].airportName, QStringLiteral("Test Field"));
		QCOMPARE(los[0].runway, QStringLiteral("09"));
		QCOMPARE(los[0].runwayHeading, 91);
		QCOMPARE(los[0].airspeed, 150);
		QCOMPARE(los[0].verticalSpeed, 500);
		QCOMPARE(los[0].pitchDegrees, 8.5);
		QCOMPARE(los[0].bankDegrees, -1.5);
		QCOMPARE(los[0].headingDegrees, 92);
		QCOMPARE(los[0].distanceLength, 1800.5);
		QCOMPARE(los[0].distanceWidth, -3.25);
		QCOMPARE(los[0].distanceLengthPercent, 0.6);
		QCOMPARE(los[0].distanceWidthPercent, -0.14);
		QCOMPARE(los[0].windDirection, 270);
		QCOMPARE(los[0].windVelocity, 12);
		QCOMPARE(los[0].zuluTime, QStringLiteral("z1"));
		QCOMPARE(los[0].localTime, QStringLiteral("l1"));
		QCOMPARE(los[0].analysisReport, QStringLiteral("report"));
		QVERIFY(los[0].rowId > 0);
		// NULL runway heading reads as -1 ("unknown"); NULL distances as 0.
		QCOMPARE(los[1].runwayHeading, -1);
		QCOMPARE(los[1].distanceLength, 0.0);
		QCOMPARE(los[1].runway, QString());
		QCOMPARE(tds.size(), size_t(1));
		QCOMPARE(tds[0].gForce, 1.3);
		QCOMPARE(tds[0].verticalSpeed, -180);
		QCOMPARE(tds[0].runway, QStringLiteral("27"));
		QCOMPARE(tds[0].runwayHeading, 270);
		QCOMPARE(tds[0].distanceLength, -50.0);
		QCOMPARE(tds[0].analysisReport, QString());
	}

	void eventListSkipsBrakesAndKeepsOrder() {
		sqlite3* db = freshDatabase();
		exec(db, "INSERT INTO trip_events (trip,event,time_zulu,time_local,event_seq) VALUES "
			"(1,'GEAR_UP','z2','l',1),(1,'BRAKES','z3','l',2),(1,'FLAPS_UP','z1','l',3),(2,'GEAR_DOWN','z1','l',4);");
		const std::vector<TripEvent> events = queryEvents(db, 1);
		sqlite3_close(db);
		QCOMPARE(events.size(), size_t(2));
		QCOMPARE(events[0].event, QStringLiteral("GEAR_UP"));
		QCOMPARE(events[0].zuluTime, QStringLiteral("z2"));
		QCOMPARE(events[1].event, QStringLiteral("FLAPS_UP"));
	}

	void eventPositionsComeFromTheFirstSampleAtOrAfterTheEvent() {
		TripDataset dataset;
		dataset.points = { point("2026-01-01T10:00:00.000", 1), point("2026-01-01T10:00:01.000", 2), point("2026-01-01T10:00:02.000", 3) };
		TripEvent between, exact, after, before;
		between.zuluTime = "2026-01-01T10:00:00.200"; // closer to sample 0, but sample 1 is used
		exact.zuluTime = "2026-01-01T10:00:02.000";
		after.zuluTime = "2026-01-01T10:00:09.000";
		before.zuluTime = "2026-01-01T09:00:00.000";
		dataset.events = { between, exact, after, before };
		resolveEventPositions(dataset);
		QCOMPARE(dataset.events[0].sampleIndex, 1);
		QCOMPARE(dataset.events[0].latitude, 2.0);
		QCOMPARE(dataset.events[1].sampleIndex, 2);
		QCOMPARE(dataset.events[2].sampleIndex, 2);
		QCOMPARE(dataset.events[3].sampleIndex, 0);
	}

	void eventPositionsNeedSamples() {
		TripDataset dataset;
		TripEvent e;
		e.zuluTime = "2026-01-01T10:00:00.000";
		dataset.events = { e };
		resolveEventPositions(dataset);
		QCOMPARE(dataset.events[0].sampleIndex, -1);
	}

	void deletingATripRemovesOnlyItsRows() {
		sqlite3* db = freshDatabase();
		for (int trip : { 1, 2 }) {
			const QByteArray t = QByteArray::number(trip);
			exec(db, "INSERT INTO trips (id,title,atc_airline,atc_flight_number,atc_id,atc_model,atc_type,departure_latitude,"
				"departure_longitude,departure_zulu_time,departure_local_time) VALUES (" + t + ",'T','A','1','I','M','T',0,0,'z','l');");
			exec(db, "INSERT INTO trip_events (trip,event,time_zulu,time_local) VALUES (" + t + ",'GEAR_UP','z','l');");
			exec(db, "INSERT INTO trip_liftoffs (trip,airspeed_indicated,vertical_speed,plane_pitch_degrees,plane_bank_degrees,"
				"heading_indicator,plane_latitude,plane_longitude,time_zulu,time_local) VALUES (" + t + ",0,0,0,0,0,0,0,'z','l');");
			exec(db, "INSERT INTO trip_touchdowns (trip,airspeed_indicated,vertical_speed,g_force,plane_pitch_degrees,plane_bank_degrees,"
				"heading_indicator,plane_latitude,plane_longitude,time_zulu,time_local) VALUES (" + t + ",0,0,0,0,0,0,0,0,'z','l');");
		}
		QVERIFY(deleteTripData(db, 1));
		sqlite3_close(db);
		for (const char* table : { "trip_events", "trip_liftoffs", "trip_touchdowns" }) {
			QCOMPARE(queryValue(QStringLiteral("SELECT COUNT(*) FROM %1 WHERE trip=1").arg(table)).toInt(), 0);
			QCOMPARE(queryValue(QStringLiteral("SELECT COUNT(*) FROM %1 WHERE trip=2").arg(table)).toInt(), 1);
		}
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trips").toInt(), 1);
		QCOMPARE(queryValue("SELECT id FROM trips").toInt(), 2);
	}

	void deletingRemovesSamplesToo() {
		int tripId = 0;
		{
			FlightDriver sim;
			tripId = sim.startTrip();
			sim.ticks(3);
			sim.endTrip();
		}
		QVERIFY(queryValue(QStringLiteral("SELECT COUNT(*) FROM trip_data WHERE trip=%1").arg(tripId)).toInt() > 0);
		sqlite3* db = connect_db_readwrite();
		QVERIFY(deleteTripData(db, tripId));
		sqlite3_close(db);
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trip_data").toInt(), 0);
	}
};

QTEST_MAIN(TstDatabase)
#include "tst_database.moc"
