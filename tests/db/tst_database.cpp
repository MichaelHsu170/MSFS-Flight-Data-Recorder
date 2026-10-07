// Database layer: schema creation and upgrade, a full write/read round trip
// of every trip_data field, the recorder's write API (trips, liftoff and
// touchdown rows), the Trip History queries, event rows, trip deletion, the
// UI's connections and AI analysis reports.
#include "test_support.h"

#include "db.h"
#include "db_history.h"
#include "trip_data_fields.h"
#include "logger.h"

#include <QTemporaryDir>
#include <QtTest>

#include <algorithm>
#include <cstring>
#include <functional>
#include <memory>
#include <set>

using namespace TestSupport;

namespace {

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

// A STATUS whose write connection (status->sql) is a fresh database, for
// the recorder write API.
class Writer {
public:
	Writer() : status_(std::make_unique<STATUS>()) { status_->sql = freshDatabase(); }
	~Writer() {
		sqlite3_close(status_->sql);
		status_->sql = nullptr;
	}
	STATUS* status() { return status_.get(); }

private:
	std::unique_ptr<STATUS> status_;
};

AIRPORT airport(const char* icao, const char* region, const char* name) {
	AIRPORT a;
	copy_cstr(a.icao, icao);
	copy_cstr(a.region, region);
	copy_cstr(a.name, name);
	return a;
}

FLIGHT_DATA contactData() {
	const FLIGHT_DATA_RECORD r = makeRecord();
	FLIGHT_DATA d;
	d.speed = 142;
	d.vertical_speed = -310;
	d.g_force = 1.75;
	d.pitch = 4.5;
	d.bank = -1.25;
	d.heading = 164;
	d.coordinate.latitude = 47.4;
	d.coordinate.longitude = -122.3;
	d.wind_direction = 200;
	d.wind_velocity = 12;
	d.time_zulu = r.time_zulu;
	d.time_local = r.time_local;
	return d;
}

// Every column that stores a local time (DATETIME::format_date_time()).
const std::pair<const char*, const char*> kLocalTimeColumns[] = {
	{ "trips", "departure_local_time" }, { "trips", "destination_local_time" },
	{ "trip_data", "local_time" }, { "trip_events", "time_local" },
	{ "trip_liftoffs", "time_local" }, { "trip_touchdowns", "time_local" },
};

// Creates flight_data.db as an older build left it (user_version 0), with
// only the local-time columns, each holding in rowid order: UTC+2 and
// UTC-5:30 with the old reversed sign, UTC, and a value not in that format.
void createOldLocalTimes() {
	sqlite3* db = openDatabaseFile();
	QVERIFY(db);
	exec(db, "CREATE TABLE trips (id INTEGER PRIMARY KEY, departure_local_time, destination_local_time);"
		"CREATE TABLE trip_data (local_time);"
		"CREATE TABLE trip_events (time_local);"
		"CREATE TABLE trip_liftoffs (time_local);"
		"CREATE TABLE trip_touchdowns (time_local);");
	for (const char* value : { "2026-07-03T06:33:15.303-02:00_5", "2026-07-03T08:00:00.000+05:30_1",
			"2026-07-03T08:00:00.000+00:00_1", "l" }) {
		const QByteArray v = QByteArray("'") + value + "'";
		exec(db, "INSERT INTO trips (departure_local_time, destination_local_time) VALUES (" + v + "," + v + ");");
		for (const char* table : { "trip_data (local_time)", "trip_events (time_local)", "trip_liftoffs (time_local)", "trip_touchdowns (time_local)" })
			exec(db, QByteArray("INSERT INTO ") + table + " VALUES (" + v + ");");
	}
	sqlite3_close(db);
}

// The column's values in rowid order, comma-separated.
QString localTimes(const char* table, const char* column) {
	return queryValue(QStringLiteral("SELECT group_concat(%2) FROM (SELECT %2 FROM %1 ORDER BY rowid)").arg(table, column)).toString();
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

	QTemporaryDir logDir_;
	QString logPath_;

private slots:
	// Logger::init() takes effect once per process, so it runs here.
	void initTestCase() {
		isolateFiles();
		logPath_ = logDir_.filePath(QStringLiteral("database.log"));
		Logger::init(Logger::Warning, logPath_);
	}
	void init() { removeDatabase(); }

	// --- Schema ---

	void connectionsFailWithoutADatabase() {
		QVERIFY(connect_db_readonly() == nullptr);
		QVERIFY(connect_db_readwrite() == nullptr);
	}

	void migrateFailsWhenTheDatabaseCantBeOpened() {
		const QString path = QString::fromStdString(db_file_path());
		QVERIFY(QDir().mkdir(path)); // a folder where the file should be
		const bool ok = migrate_db();
		QVERIFY(QDir().rmdir(path));
		QVERIFY(!ok);
	}

	void migrateCreatesAllTablesAndIndexes() {
		QVERIFY(migrate_db());
		QCOMPARE(names("SELECT name FROM sqlite_master WHERE type='table' AND name NOT LIKE 'sqlite_%'"),
			(std::set<QString>{ "trips", "trip_data", "trip_events", "trip_liftoffs", "trip_touchdowns", "trip_groups" }));
		QCOMPARE(names("SELECT name FROM sqlite_master WHERE type='index' AND name LIKE 'idx_%'"),
			(std::set<QString>{ "idx_trip_data_trip", "idx_trip_events_trip", "idx_trip_events_event_seq",
				"idx_trip_liftoffs_trip", "idx_trip_touchdowns_trip", "idx_trips_group", "idx_trip_groups_name" }));
	}

	// A missing index only slows queries, so it's logged and the database is
	// still used; the other indexes are still created.
	void anIndexThatCantBeCreatedIsLoggedNotFatal() {
		sqlite3* db = openDatabaseFile();
		QVERIFY(db);
		exec(db, "CREATE TABLE idx_trips_group (x);"); // takes the index's name
		sqlite3_close(db);
		QVERIFY(migrate_db());
		QVERIFY(warningLogged(logPath_, { QStringLiteral("index"), QStringLiteral("idx_trips_group") }));
		QCOMPARE(names("SELECT name FROM sqlite_master WHERE type='index' AND name LIKE 'idx_%'"),
			(std::set<QString>{ "idx_trip_data_trip", "idx_trip_events_trip", "idx_trip_events_event_seq",
				"idx_trip_liftoffs_trip", "idx_trip_touchdowns_trip", "idx_trip_groups_name" }));
	}

	// Every write names the tables, so one that can't be created fails the
	// migration; the others are still created.
	void aTableThatCantBeCreatedFailsTheMigration() {
		sqlite3* db = openDatabaseFile();
		QVERIFY(db);
		exec(db, "CREATE TABLE other (x);");
		exec(db, "CREATE INDEX trips ON other(x);"); // takes the table's name
		sqlite3_close(db);
		QVERIFY(!migrate_db());
		QVERIFY(lineLogged(logPath_, "FATAL", { QStringLiteral("create table trips") }));
		QCOMPARE(names("SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'trip_%'"),
			(std::set<QString>{ "trip_data", "trip_events", "trip_liftoffs", "trip_touchdowns", "trip_groups" }));
	}

	void migrateIsRepeatable() {
		migrate_db();
		migrate_db();
		QCOMPARE(queryValue("SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name='trips'").toInt(), 1);
	}

	void migrateAddsColumnsMissingFromOlderDatabases() {
		sqlite3* db = openDatabaseFile();
		QVERIFY(db);
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

	void migrateMovesLegacyN1N2IntoTheEngineColumns() {
		createLegacyTripData();
		migrate_db();
		// Little-endian float32: 85.5 = 42AB0000, 90.25 = 42B48000, 95 = 42BE0000, 96.5 = 42C10000.
		const QList<QVariantMap> rows = queryRows("SELECT hex(engine_speed) AS speed, hex(engine_load) AS load FROM trip_data ORDER BY rowid");
		QCOMPARE(rows.size(), 5);
		QCOMPARE(rows[0]["speed"].toString(), QStringLiteral("0000AB420080B442"));
		QCOMPARE(rows[0]["load"].toString(), QStringLiteral("0000BE420000C142"));
		QCOMPARE(rows[1]["speed"].toString(), QStringLiteral("0000AB42"));
		QCOMPARE(rows[1]["load"].toString(), QStringLiteral("0000BE42"));
		QCOMPARE(rows[2]["speed"].toString(), QStringLiteral("0000AB420080B442"));
		QCOMPARE(rows[3]["speed"].toString(), QString()); // hex(NULL) is ''
		QCOMPARE(rows[4]["speed"].toString(), QString());
		QCOMPARE(rows[4]["load"].toString(), QString());
		const QVariantMap columns = queryRows("SELECT * FROM trip_data").value(0);
		for (const char* old : { "turb_eng_n1_1", "turb_eng_n1_2", "turb_eng_n2_1", "turb_eng_n2_2" })
			QVERIFY2(!columns.contains(old), old);

		// The old columns are gone, so a rerun leaves the data alone.
		migrate_db();
		QCOMPARE(queryValue("SELECT hex(engine_speed) FROM trip_data ORDER BY rowid").toString(), QStringLiteral("0000AB420080B442"));
	}

	void theLegacyEngineRebuildKeepsRowsAndReportsProgress() {
		createLegacyTripData();
		exec("DELETE FROM trip_data WHERE rowid = 2;"); // a rowid gap

		std::vector<int> reported;
		migrate_db([&reported](int percent) { reported.push_back(percent); });
		// 4 rows, a hundredth of them (at least 1) per batch. Weights copy 65,
		// drop 10, commit 15, indexes 10: copying 0-65% (65*1/4, 65*2/4, ...
		// rounded down), then the drop (75%), the commit (90%) and the indexes
		// (100%).
		QCOMPARE(reported, (std::vector<int>{ 16, 32, 48, 65, 75, 90, 100 }));
		// Same rowids, and the column the definitions don't name is kept.
		QCOMPARE(queryValue("SELECT group_concat(rowid || ':' || retired_field || ':' || engine_type) FROM trip_data").toString(),
			QStringLiteral("1:10.0:1,3:30.0:1,4:40.0:1,5:50.0:0"));
		// NOT NULL stays where the old table had it, not where it allowed NULL.
		QCOMPARE(queryValue("SELECT \"notnull\" FROM pragma_table_info('trip_data') WHERE name='trip'").toInt(), 1);
		QCOMPARE(queryValue("SELECT \"notnull\" FROM pragma_table_info('trip_data') WHERE name='zulu_time'").toInt(), 0);
		QCOMPARE(queryValue("SELECT COUNT(*) FROM sqlite_master WHERE name='idx_trip_data_trip'").toInt(), 1);
		QCOMPARE(queryValue("SELECT COUNT(*) FROM sqlite_master WHERE name='trip_data_new'").toInt(), 0);

		// Nothing left to rebuild: no progress.
		reported.clear();
		migrate_db([&reported](int percent) { reported.push_back(percent); });
		QVERIFY(reported.empty());
	}

	void theRebuildsProgressOnlyRises() {
		createLegacyTripData();
		// 305 rows, 3 per batch: each batch adds about 0.64% of the copy's 65,
		// so the first rounds down to 0 and most repeat the one before.
		addLegacyJetRows(300);

		std::vector<int> reported;
		QVERIFY(migrate_db([&reported](int percent) { reported.push_back(percent); }));
		QVERIFY(!reported.empty());
		QCOMPARE(reported.front(), 1);
		QCOMPARE(reported.back(), 100);
		QVERIFY(std::adjacent_find(reported.begin(), reported.end(), std::greater_equal<int>()) == reported.end());
	}

	void theRebuildCopiesRowsAtTheSmallestAndLargestRowid() {
		createLegacyTripData();
		exec("INSERT INTO trip_data (rowid, trip, engine_type, number_of_engines, turb_eng_n1_1, turb_eng_n1_2, turb_eng_n2_1, turb_eng_n2_2)"
			" VALUES (-9223372036854775807 - 1, 2, 1, 2, 85.5, 90.25, 95, 96.5), (9223372036854775807, 2, 1, 2, 85.5, 90.25, 95, 96.5);");

		QVERIFY(migrate_db());
		QCOMPARE(queryValue("SELECT group_concat(rowid) FROM (SELECT rowid FROM trip_data ORDER BY rowid)").toString(),
			QStringLiteral("-9223372036854775808,1,2,3,4,5,9223372036854775807"));
	}

	void aFailedLegacyEngineMigrationChangesNothingAndIsRedoneNextTime() {
		createLegacyTripData();
		// Fails the last step, after every row was copied: renaming the new table
		// checks the schema, and this view reads a column the new table lacks.
		exec("CREATE VIEW blocks_rebuild AS SELECT turb_eng_n2_2 FROM trip_data;");

		QVERIFY(!migrate_db());
		QVERIFY(queryRows("SELECT * FROM trip_data").value(0).contains("turb_eng_n1_1"));
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trip_data WHERE engine_speed IS NOT NULL").toInt(), 0);
		QCOMPARE(queryValue("SELECT COUNT(*) FROM sqlite_master WHERE name='trip_data_new'").toInt(), 0);
		// The rest of the schema update still happened.
		QCOMPARE(queryValue("SELECT COUNT(*) FROM sqlite_master WHERE name='idx_trip_data_trip'").toInt(), 1);

		exec("DROP VIEW blocks_rebuild;");
		QVERIFY(migrate_db());
		QVERIFY(!queryRows("SELECT * FROM trip_data").value(0).contains("turb_eng_n1_1"));
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trip_data WHERE engine_speed IS NOT NULL").toInt(), 3);
	}

	// Whether trip_data still has the legacy columns can't be told, so the
	// migration fails, logged, rather than let recording start.
	void migrateFailsWhenTripDatasColumnsCantBeRead() {
		sqlite3* db = openDatabaseFile();
		QVERIFY(db);
		// CREATE TABLE IF NOT EXISTS leaves it alone, and its columns can't be
		// listed: the table it reads is gone.
		exec(db, "CREATE TABLE gone (x);");
		exec(db, "CREATE VIEW trip_data AS SELECT x FROM gone;");
		exec(db, "DROP TABLE gone;");
		sqlite3_close(db);
		QVERIFY(!migrate_db());
		QVERIFY(lineLogged(logPath_, "FATAL", { QStringLiteral("trip_data"), QStringLiteral("columns") }));
	}

	// Every write and query names the current columns, so one that can't be
	// added fails the migration; it's added by the next one.
	void migrateFailsWhenAColumnCantBeAdded() {
		sqlite3* db = openDatabaseFile();
		QVERIFY(db);
		// CREATE TABLE IF NOT EXISTS leaves the view alone, and a view can't
		// take a column: the ALTER fails to prepare.
		exec(db, "CREATE VIEW trip_groups AS SELECT 1 AS id, 'x' AS name;");
		sqlite3_close(db);
		QVERIFY(!migrate_db());
		QCOMPARE(queryValue("SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name='trips'").toInt(), 1);

		exec("DROP VIEW trip_groups;");
		QVERIFY(migrate_db());
		QVERIFY(queryRows("SELECT * FROM pragma_table_info('trip_groups') WHERE name='sort_order'").size() == 1);
	}

	void migrateFailsWhenAddingAColumnFails() {
		QVERIFY(migrate_db());
		exec("ALTER TABLE trip_groups DROP COLUMN sort_order;");
		// Another connection's write transaction: the ALTER prepares but its
		// step stays busy (after migrate_db()'s 5 s busy timeout).
		sqlite3* locker = openDatabaseFile();
		QVERIFY(locker);
		exec(locker, "BEGIN IMMEDIATE;");
		const bool ok = migrate_db();
		sqlite3_close(locker);
		QVERIFY(!ok);
		QVERIFY(queryRows("SELECT * FROM pragma_table_info('trip_groups') WHERE name='sort_order'").isEmpty());

		QVERIFY(migrate_db());
		QVERIFY(queryRows("SELECT * FROM pragma_table_info('trip_groups') WHERE name='sort_order'").size() == 1);
	}

	void aCancelledLegacyEngineRebuildIsRolledBackAndRedoneNextTime() {
		createLegacyTripData();
		// 5 rows, one per batch: asked after each of the 5 batches, then once
		// before committing (the 6th time).
		for (int cancelAt : { 1, 6 }) {
			int asked = 0;
			QVERIFY2(!migrate_db({}, [&asked, cancelAt] { return ++asked == cancelAt; }), qPrintable(QString::number(cancelAt)));
			QCOMPARE(asked, cancelAt);
			QVERIFY(queryRows("SELECT * FROM trip_data").value(0).contains("turb_eng_n1_1"));
			QCOMPARE(queryValue("SELECT COUNT(*) FROM trip_data").toInt(), 5);
			QCOMPARE(queryValue("SELECT COUNT(*) FROM sqlite_master WHERE name='trip_data_new'").toInt(), 0);
		}

		int asked = 0;
		QVERIFY(migrate_db({}, [&asked] { ++asked; return false; }));
		QCOMPARE(asked, 6);
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trip_data WHERE engine_speed IS NOT NULL").toInt(), 3);
	}

	// Older builds stored local times with the UTC offset's sign reversed
	// (UTC+2 as "-02:00"). The first migration flips it in every local-time
	// column; later ones leave the values alone.
	void migrateCorrectsTheSignOfOldLocalTimeOffsetsOnce() {
		createOldLocalTimes();
		QVERIFY(migrate_db());
		const QString fixed = QStringLiteral("2026-07-03T06:33:15.303+02:00_5,2026-07-03T08:00:00.000-05:30_1,"
			"2026-07-03T08:00:00.000+00:00_1,l");
		for (const auto& [table, column] : kLocalTimeColumns)
			QCOMPARE(localTimes(table, column), fixed);
		QCOMPARE(queryValue("PRAGMA user_version").toInt(), 1);

		QVERIFY(migrate_db());
		for (const auto& [table, column] : kLocalTimeColumns)
			QCOMPARE(localTimes(table, column), fixed);
	}

	void aNewDatabaseNeedsNoLocalTimeCorrection() {
		QVERIFY(migrate_db());
		QCOMPARE(queryValue("PRAGMA user_version").toInt(), 1);
	}

	// A failure rolls back every column (recording can't start with only
	// some corrected) and the next migration redoes it.
	void aFailedLocalTimeCorrectionChangesNothingAndIsRedoneNextTime() {
		createOldLocalTimes();
		exec("CREATE TRIGGER blocks_fix BEFORE UPDATE ON trip_events BEGIN SELECT RAISE(ABORT, 'blocked'); END;");
		const QString old = QStringLiteral("2026-07-03T06:33:15.303-02:00_5,2026-07-03T08:00:00.000+05:30_1,"
			"2026-07-03T08:00:00.000+00:00_1,l");

		QVERIFY(!migrate_db());
		QVERIFY(lineLogged(logPath_, "FATAL", { QStringLiteral("local times") }));
		for (const auto& [table, column] : kLocalTimeColumns)
			QCOMPARE(localTimes(table, column), old);
		QCOMPARE(queryValue("PRAGMA user_version").toInt(), 0);

		exec("DROP TRIGGER blocks_fix;");
		QVERIFY(migrate_db());
		QCOMPARE(localTimes("trip_events", "time_local"),
			QStringLiteral("2026-07-03T06:33:15.303+02:00_5,2026-07-03T08:00:00.000-05:30_1,2026-07-03T08:00:00.000+00:00_1,l"));
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
		// queryTripData(), raw and as its named fields.
		FlightDriver sim;
		double value = 1.25;
#define SET_NUM(dbColumn, memberExpr, sqlType) sim.record.memberExpr = (value += 1.0);
		TRIP_DATA_NUM_FIELDS(SET_NUM)
#undef SET_NUM
		int boolIndex = 0;
#define SET_BOOL(name, group, bit) sim.record.name = (boolIndex++ % 3 == 0) ? 1 : 0;
		TRIP_DATA_BOOL_FIELDS(SET_BOOL)
#undef SET_BOOL
		sim.record.sim_on_ground = 1;
		sim.record.eng_combustion_1 = 1;
		// A 3-engine turboprop: prop RPM and torque stored for engines 1..3,
		// not the other types' values or the 4th engine.
		sim.record.engine_type = 5;
		sim.record.number_of_engines = 3;
		for (int e = 0; e < MAX_ENGINES; ++e) {
			sim.record.prop_rpm[e] = 2100 + e;
			sim.record.turb_eng_max_torque_percent[e] = 25.5 + e;
			sim.record.general_eng_rpm[e] = 99;
			sim.record.turb_eng_n1[e] = 99;
		}
		const FLIGHT_DATA_RECORD sent = sim.record;

		sim.tick();
		const int trip = sim.status().id_trip;
		QVERIFY(trip > 0);
		sim.endTrip();

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
#define CHECK_NUM(dbColumn, memberExpr, sqlType) \
		QVERIFY2(i < (int)p.rawNums.size() && p.rawNums[i] == expected.memberExpr, #dbColumn); \
		++i;
		TRIP_DATA_NUM_FIELDS(CHECK_NUM)
#undef CHECK_NUM
		QCOMPARE((int)p.rawNums.size(), i);

		const std::array<uint32_t, 4> groups = tripBoolGroups(expected);
		for (int g = 1; g <= 3; ++g)
			QCOMPARE(p.boolGroups[g], groups[g]);

		// The named fields, as the map, charts and data table read them.
		QCOMPARE(p.latitude, expected.plane_coordinate.latitude);
		QCOMPARE(p.longitude, expected.plane_coordinate.longitude);
		QCOMPARE(p.altitude, (int)expected.plane_altitude);
		QCOMPARE(p.airspeed, (int)expected.airspeed_indicated);
		QCOMPARE(p.groundSpeed, (int)expected.ground_velocity);
		QCOMPARE(p.verticalSpeed, (int)expected.vertical_speed);
		QCOMPARE(p.engine.engineType, 5);
		QCOMPARE(p.engine.count, 3);
		QCOMPARE(p.engine.speed, (std::array<float, MAX_ENGINES>{ 2100, 2101, 2102, 0 }));
		QCOMPARE(p.engine.load, (std::array<float, MAX_ENGINES>{ 25.5f, 26.5f, 27.5f, 0 }));
		QCOMPARE(p.gearHandlePosition, expected.gear_handle_position);
		QCOMPARE(p.gearPosition[0], (int)expected.gear_position_0);
		QCOMPARE(p.gearPosition[1], (int)expected.gear_position_1);
		QCOMPARE(p.gearPosition[2], (int)expected.gear_position_2);
		QCOMPARE(p.gearOnGround[0], expected.gear_is_on_ground_0 != 0);
		QCOMPARE(p.gearOnGround[1], expected.gear_is_on_ground_1 != 0);
		QCOMPARE(p.gearOnGround[2], expected.gear_is_on_ground_2 != 0);
		QCOMPARE(p.brakeIndicator, (int)expected.brake_indicator);
		QCOMPARE(p.flapsHandleIndex, (double)expected.flaps_handle_index);
		QCOMPARE(p.spoilersHandlePosition, expected.spoilers_handle_position);
		QCOMPARE(p.fuelTotalQuantityWeight, (double)expected.fuel_total_quantity_weight);
		QCOMPARE(p.pitchDegrees, expected.plane_pitch_degrees);
		QCOMPARE(p.bankDegrees, expected.plane_bank_degrees);
		// makeRecord()'s 10:00:00 plus the one 0.5 s tick.
		QCOMPARE(p.zuluTime, QStringLiteral("2026-01-02T10:00:00.500+00:00_5"));
		QCOMPARE(p.localTime, QStringLiteral("2026-01-02T10:00:00.500+00:00_5"));
	}

	void aSampleWithNoEnginePowerStoresNull() {
		// Engine type 2 (none) records no power, whatever its engine count:
		// stored as NULL, not as an empty BLOB.
		FlightDriver sim;
		sim.record.sim_on_ground = 1;
		sim.record.eng_combustion_1 = 1;
		sim.record.engine_type = 2;
		sim.record.number_of_engines = 2;
		sim.tick();
		const int trip = sim.status().id_trip;
		QVERIFY(trip > 0);
		sim.endTrip();
		QCOMPARE(queryValue(QStringLiteral("SELECT typeof(engine_speed) || ',' || typeof(engine_load) FROM trip_data WHERE trip = %1").arg(trip)).toString(),
			QStringLiteral("null,null"));
	}

	void queryTripDataReturnsAnEmptyDatasetWhenTheTableIsMissing() {
		sqlite3* db = freshDatabase();
		exec(db, "DROP TABLE trip_data");
		const TripDataset dataset = queryTripData(db, 1);
		sqlite3_close(db);
		QVERIFY(dataset.points.empty());
		QCOMPARE(dataset.tripId, 1); // set before the query runs, kept on failure
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

	void tripListIsEmptyAndLoggedWhenItCantBeRead() {
		sqlite3* db = freshDatabase();
		addTrip(1);
		exec(db, "DROP TABLE trip_groups");
		const std::vector<TripSummary> trips = queryAllTrips(db, 0);
		sqlite3_close(db);
		QVERIFY(trips.empty());
		QVERIFY(warningLogged(logPath_, { QStringLiteral("queryAllTrips"), QStringLiteral("trip_groups") }));
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

	void liftoffsAndTouchdownsReturnEmptyWhenTheirTablesAreMissing() {
		sqlite3* db = freshDatabase();
		exec(db, "DROP TABLE trip_liftoffs");
		exec(db, "DROP TABLE trip_touchdowns");
		const std::vector<LiftoffPoint> los = queryLiftoffs(db, 1);
		const std::vector<TouchdownPoint> tds = queryTouchdowns(db, 1);
		sqlite3_close(db);
		QVERIFY(los.empty());
		QVERIFY(tds.empty());
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

	void queryEventsReturnsAnEmptyListWhenTheTableIsMissing() {
		sqlite3* db = freshDatabase();
		exec(db, "DROP TABLE trip_events");
		const std::vector<TripEvent> events = queryEvents(db, 1);
		sqlite3_close(db);
		QVERIFY(events.empty());
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

	void tripSamplesAreNamedAfterTheTrip() {
		sqlite3* db = freshDatabase();
		const TripDataset loaded = tripSamples(db, 4, QStringLiteral("Cessna"), QStringLiteral("z0"));
		sqlite3_close(db);
		const TripDataset none = tripSamples(nullptr, 5, QStringLiteral("Piper"), QStringLiteral("z1"));
		QCOMPARE(loaded.tripId, 4);
		QCOMPARE(loaded.aircraftTitle, QStringLiteral("Cessna"));
		QCOMPARE(loaded.departureZuluTime, QStringLiteral("z0"));
		QCOMPARE(none.tripId, 5);
		QCOMPARE(none.aircraftTitle, QStringLiteral("Piper"));
		QCOMPARE(none.departureZuluTime, QStringLiteral("z1"));
		QVERIFY(none.points.empty());
	}

	void completingADatasetAddsItsPartsAndPlacesEvents() {
		TripDataset dataset;
		dataset.points = { point("2026-01-01T10:00:00.000", 1), point("2026-01-01T10:00:01.000", 2) };
		LiftoffPoint liftoff;
		liftoff.icao = "AAAA";
		TouchdownPoint touchdown;
		touchdown.icao = "BBBB";
		TripEvent event;
		event.zuluTime = "2026-01-01T10:00:00.500";
		completeTripDataset(dataset, { liftoff }, { touchdown }, { event });
		QCOMPARE(dataset.liftoffPoints.size(), size_t(1));
		QCOMPARE(dataset.liftoffPoints[0].icao, QStringLiteral("AAAA"));
		QCOMPARE(dataset.touchdowns.size(), size_t(1));
		QCOMPARE(dataset.touchdowns[0].icao, QStringLiteral("BBBB"));
		QCOMPARE(dataset.events.size(), size_t(1));
		QCOMPARE(dataset.events[0].sampleIndex, 1);
		QCOMPARE(dataset.events[0].latitude, 2.0);
	}

	void deletingATripRemovesOnlyItsRows() {
		sqlite3* db = freshDatabase();
		for (int trip : { 1, 2 }) {
			const QByteArray t = QByteArray::number(trip);
			addTrip(trip);
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

	void deleteTripDataRollsBackWhenAChildTableIsMissing() {
		sqlite3* db = freshDatabase();
		addTrip(1);
		exec(db, "INSERT INTO trip_events (trip,event,time_zulu,time_local) VALUES (1,'GEAR_UP','z','l');");
		exec(db, "DROP TABLE trip_liftoffs");
		QVERIFY(!deleteTripData(db, 1));
		// trip_data and trip_events delete before the missing trip_liftoffs
		// table fails -- the whole transaction must roll back, not just stop.
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trips").toInt(), 1);
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trip_events WHERE trip=1").toInt(), 1);
		sqlite3_close(db);
	}

	// --- Recorder write API (db_insert_trip() etc.) ---

	void insertTripStoresDepartureFields() {
		Writer w;
		FLIGHT_DATA_RECORD r = makeRecord();
		copy_cstr(r.title, "Test Plane");
		copy_cstr(r.atc_airline, "Air Test");
		copy_cstr(r.atc_flight_number, "123");
		copy_cstr(r.atc_id, "N123");
		copy_cstr(r.atc_model, "B738");
		copy_cstr(r.atc_type, "Boeing");
		r.plane_coordinate.latitude = 47.25;
		r.plane_coordinate.longitude = -122.5;
		r.time_local.time_day = 39600; // 11:00 at UTC+1, so the two columns differ
		r.time_local.timezone_offset = -3600;
		const int id = db_insert_trip(w.status(), r);
		QVERIFY(id > 0);
		QCOMPARE(db_insert_trip(w.status(), r), id + 1);
		const QVariantMap t = tripRow(id);
		QCOMPARE(t["title"].toString(), QStringLiteral("Test Plane"));
		QCOMPARE(t["atc_airline"].toString(), QStringLiteral("Air Test"));
		QCOMPARE(t["atc_flight_number"].toString(), QStringLiteral("123"));
		QCOMPARE(t["atc_id"].toString(), QStringLiteral("N123"));
		QCOMPARE(t["atc_model"].toString(), QStringLiteral("B738"));
		QCOMPARE(t["atc_type"].toString(), QStringLiteral("Boeing"));
		QCOMPARE(t["departure_latitude"].toDouble(), 47.25);
		QCOMPARE(t["departure_longitude"].toDouble(), -122.5);
		QCOMPARE(t["departure_zulu_time"].toString(), QStringLiteral("2026-01-02T10:00:00.000+00:00_5"));
		QCOMPARE(t["departure_local_time"].toString(), QStringLiteral("2026-01-02T11:00:00.000+01:00_5"));
		QVERIFY(t["departure_icao"].isNull());
		QVERIFY(t["destination_zulu_time"].isNull());
	}

	void tripDestinationTimeAndPosition() {
		Writer w;
		FLIGHT_DATA_RECORD r = makeRecord();
		const int id = db_insert_trip(w.status(), r);
		r.time_zulu.time_day += 3600;
		r.time_local.time_day = 45000; // 12:30 at UTC+1
		r.time_local.timezone_offset = -3600;
		db_set_trip_destination_time(w.status(), id, r.time_zulu, r.time_local);
		COORDINATE position;
		position.latitude = 10.5;
		position.longitude = 20.25;
		db_set_trip_destination_position(w.status(), id, position);
		const QVariantMap t = tripRow(id);
		QCOMPARE(t["destination_zulu_time"].toString(), QStringLiteral("2026-01-02T11:00:00.000+00:00_5"));
		QCOMPARE(t["destination_local_time"].toString(), QStringLiteral("2026-01-02T12:30:00.000+01:00_5"));
		QCOMPARE(t["destination_latitude"].toDouble(), 10.5);
		QCOMPARE(t["destination_longitude"].toDouble(), 20.25);
	}

	void tripAirportWithAndWithoutRunway() {
		Writer w;
		const int id = db_insert_trip(w.status(), makeRecord());
		const AIRPORT dep = airport("KSEA", "K1", "Seattle-Tacoma");
		const AIRPORT dest = airport("KPDX", "K2", "Portland");
		db_set_trip_airport(w.status(), id, TRIP_END::DEPARTURE, dep, "16L");
		db_set_trip_airport(w.status(), id, TRIP_END::DESTINATION, dest, "28R");
		QVariantMap t = tripRow(id);
		QCOMPARE(t["departure_icao"].toString(), QStringLiteral("KSEA"));
		QCOMPARE(t["departure_region"].toString(), QStringLiteral("K1"));
		QCOMPARE(t["departure_name"].toString(), QStringLiteral("Seattle-Tacoma"));
		QCOMPARE(t["departure_rwy"].toString(), QStringLiteral("16L"));
		QCOMPARE(t["destination_icao"].toString(), QStringLiteral("KPDX"));
		QCOMPARE(t["destination_rwy"].toString(), QStringLiteral("28R"));
		// Airport found, no runway: the runway becomes NULL.
		db_set_trip_airport(w.status(), id, TRIP_END::DESTINATION, airport("KBFI", "K1", "Boeing Field"), nullptr);
		t = tripRow(id);
		QCOMPARE(t["destination_icao"].toString(), QStringLiteral("KBFI"));
		QCOMPARE(t["destination_name"].toString(), QStringLiteral("Boeing Field"));
		QVERIFY(t["destination_rwy"].isNull());
		QCOMPARE(t["departure_rwy"].toString(), QStringLiteral("16L"));
	}

	void clearingTheDestinationAirportClearsItsName() {
		Writer w;
		const int id = db_insert_trip(w.status(), makeRecord());
		db_set_trip_airport(w.status(), id, TRIP_END::DESTINATION, airport("KPDX", "K2", "Portland"), "28R");
		db_clear_trip_destination_airport(w.status(), id);
		const QVariantMap t = tripRow(id);
		QCOMPARE(t["id"].toInt(), id); // the trip itself stays
		QVERIFY(t["destination_icao"].isNull());
		QVERIFY(t["destination_rwy"].isNull());
		QVERIFY(t["destination_region"].isNull());
		QVERIFY(t["destination_name"].isNull());
	}

	void contactRowsStoreTheirFlightData() {
		Writer w;
		const int trip = db_insert_trip(w.status(), makeRecord());
		FLIGHT_DATA data = contactData();
		data.time_local.time_day = 39600; // 11:00 at UTC+1, so the two columns differ
		data.time_local.timezone_offset = -3600;
		const int lo = db_insert_contact(w.status(), CONTACT_TABLE::LIFTOFFS, trip, data);
		const int td = db_insert_contact(w.status(), CONTACT_TABLE::TOUCHDOWNS, trip, data);
		QVERIFY(lo > 0);
		QVERIFY(td > 0);
		for (const char* table : { "trip_liftoffs", "trip_touchdowns" }) {
			const QVariantMap row = queryRows(QStringLiteral("SELECT * FROM %1 WHERE trip=%2").arg(table).arg(trip)).value(0);
			QCOMPARE(row["airspeed_indicated"].toInt(), 142);
			QCOMPARE(row["vertical_speed"].toInt(), -310);
			QCOMPARE(row["plane_pitch_degrees"].toDouble(), 4.5);
			QCOMPARE(row["plane_bank_degrees"].toDouble(), -1.25);
			QCOMPARE(row["heading_indicator"].toInt(), 164);
			QCOMPARE(row["plane_latitude"].toDouble(), 47.4);
			QCOMPARE(row["plane_longitude"].toDouble(), -122.3);
			QCOMPARE(row["wind_direction"].toInt(), 200);
			QCOMPARE(row["wind_velocity"].toInt(), 12);
			QCOMPARE(row["time_zulu"].toString(), QStringLiteral("2026-01-02T10:00:00.000+00:00_5")); // makeRecord()
			QCOMPARE(row["time_local"].toString(), QStringLiteral("2026-01-02T11:00:00.000+01:00_5"));
			QVERIFY(row["icao"].isNull());
			QVERIFY(row["runway"].isNull());
		}
		QCOMPARE(queryValue(QStringLiteral("SELECT g_force FROM trip_touchdowns WHERE id=%1").arg(td)).toDouble(), 1.75);
	}

	void contactAirportWithRunway() {
		Writer w;
		const int trip = db_insert_trip(w.status(), makeRecord());
		const int lo = db_insert_contact(w.status(), CONTACT_TABLE::LIFTOFFS, trip, contactData());
		const int td = db_insert_contact(w.status(), CONTACT_TABLE::TOUCHDOWNS, trip, contactData());
		AIRPORT a = airport("KSEA", "K1", "Seattle-Tacoma");
		a.runway_act.heading = 164;
		a.runway_act.distances[0] = 1500.5;
		a.runway_act.distances[1] = -12.25;
		a.runway_act.distances_percent[0] = 0.25;
		a.runway_act.distances_percent[1] = -0.2;
		db_set_contact_airport(w.status(), CONTACT_TABLE::LIFTOFFS, lo, a, "16L");
		db_set_contact_airport(w.status(), CONTACT_TABLE::TOUCHDOWNS, td, a, "16L");
		for (const char* table : { "trip_liftoffs", "trip_touchdowns" }) {
			const QVariantMap row = queryRows(QStringLiteral("SELECT * FROM %1 WHERE trip=%2").arg(table).arg(trip)).value(0);
			QCOMPARE(row["icao"].toString(), QStringLiteral("KSEA"));
			QCOMPARE(row["airport_name"].toString(), QStringLiteral("Seattle-Tacoma"));
			QCOMPARE(row["runway"].toString(), QStringLiteral("16L"));
			QCOMPARE(row["runway_heading"].toInt(), 164);
			QCOMPARE(row["distance_length"].toDouble(), 1500.5);
			QCOMPARE(row["distance_width"].toDouble(), -12.25);
			QCOMPARE(row["distance_length_percent"].toDouble(), 0.25);
			QCOMPARE(row["distance_width_percent"].toDouble(), -0.2);
		}
	}

	void negativeThresholdDistanceIsClampedForLiftoffsOnly() {
		Writer w;
		const int trip = db_insert_trip(w.status(), makeRecord());
		const int lo = db_insert_contact(w.status(), CONTACT_TABLE::LIFTOFFS, trip, contactData());
		const int td = db_insert_contact(w.status(), CONTACT_TABLE::TOUCHDOWNS, trip, contactData());
		AIRPORT a = airport("KSEA", "K1", "Seattle-Tacoma");
		a.runway_act.distances[0] = -250;
		db_set_contact_airport(w.status(), CONTACT_TABLE::LIFTOFFS, lo, a, "16L");
		db_set_contact_airport(w.status(), CONTACT_TABLE::TOUCHDOWNS, td, a, "16L");
		QCOMPARE(queryValue(QStringLiteral("SELECT distance_length FROM trip_liftoffs WHERE id=%1").arg(lo)).toDouble(), -1.0);
		QCOMPARE(queryValue(QStringLiteral("SELECT distance_length FROM trip_touchdowns WHERE id=%1").arg(td)).toDouble(), -250.0);
	}

	void contactAirportWithoutRunwaySetsOnlyTheAirport() {
		Writer w;
		const int trip = db_insert_trip(w.status(), makeRecord());
		const int td = db_insert_contact(w.status(), CONTACT_TABLE::TOUCHDOWNS, trip, contactData());
		AIRPORT a = airport("KBFI", "K1", "Boeing Field");
		a.runway_act.heading = 130;
		a.runway_act.distances[0] = 900;
		db_set_contact_airport(w.status(), CONTACT_TABLE::TOUCHDOWNS, td, a, nullptr);
		const QVariantMap row = queryRows(QStringLiteral("SELECT * FROM trip_touchdowns WHERE id=%1").arg(td)).value(0);
		QCOMPARE(row["icao"].toString(), QStringLiteral("KBFI"));
		QCOMPARE(row["airport_name"].toString(), QStringLiteral("Boeing Field"));
		QVERIFY(row["runway"].isNull());
		QVERIFY(row["runway_heading"].isNull());
		QVERIFY(row["distance_length"].isNull());
	}

	void failedWriteThrowsAndRollsBack() {
		Writer w;
		const int trip = db_insert_trip(w.status(), makeRecord());
		exec(w.status()->sql, "DROP TABLE trip_touchdowns");
		bool threw = false;
		try {
			db_insert_contact(w.status(), CONTACT_TABLE::TOUCHDOWNS, trip, contactData());
		} catch (const db_exception& e) {
			threw = true;
			QVERIFY(QString::fromStdString(e.message).contains(QStringLiteral("INSERT INTO trip_touchdowns")));
		}
		QVERIFY(threw);
		// The connection is usable again: no transaction was left open.
		db_set_trip_destination_position(w.status(), trip, COORDINATE());
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trips").toInt(), 1);
	}

	// While another connection reads, a write gets as far as its COMMIT,
	// which that reader's lock refuses: the write is reported with SQLite's
	// message, not kept, and leaves no transaction open.
	void writeRefusedAtCommitThrowsAndRollsBack() {
		Writer w;
		sqlite3_busy_timeout(w.status()->sql, 0); // refused at once, not after 5 s
		sqlite3* reader = openDatabaseFile();
		QVERIFY(reader);
		exec(reader, "BEGIN;");
		sqlite3_stmt* read = nullptr;
		QCOMPARE(sqlite3_prepare_v2(reader, "SELECT name FROM sqlite_master;", -1, &read, nullptr), SQLITE_OK);
		QCOMPARE(sqlite3_step(read), SQLITE_ROW); // the read lock lasts until the reader's COMMIT
		bool threw = false;
		try {
			db_insert_trip(w.status(), makeRecord());
		} catch (const db_exception& e) {
			threw = true;
			QVERIFY2(QString::fromStdString(e.message).endsWith(QStringLiteral("failed with error database is locked")), e.message.c_str());
		}
		sqlite3_finalize(read);
		exec(reader, "COMMIT;");
		sqlite3_close(reader);
		QVERIFY(threw);
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trips").toInt(), 0);
		// The connection is usable again: no transaction was left open.
		db_insert_trip(w.status(), makeRecord());
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trips").toInt(), 1);
	}

	// db_write_worker runs on its own thread (started by connect_db(), unlike
	// Writer's synchronous status->sql above), so this needs a real
	// FlightDriver to reach its catch block: a sample pushed to
	// sample_write_queue that fails must be logged and dropped without taking
	// the worker thread down, or every later sample would silently stop being
	// written for the rest of the process.
	void writeWorkerLogsAndKeepsDrainingWhenASampleWriteFails() {
		FlightDriver sim;
		const int tripId = sim.startTrip();
		QSignalSpy log(&sim.bridge(), &RecorderBridge::logMessage);
		exec("DROP TABLE trip_data");
		sim.tick();
		QVERIFY(waitFor([&log, tripId] {
			return !lastLogWith(log, { QStringLiteral("db_write_worker"), QStringLiteral("trip %1").arg(tripId) }).isEmpty();
		}));
		// The worker thread survived the failure and is still draining the
		// queue: ending the trip still reaches the "Recording stopped" sentinel.
		sim.endTrip();
		QVERIFY(!sim.bridge().isRecording());
	}

	// --- UI connections and analysis reports ---

	void dbConnectionOpensClosesAndMoves() {
		QVERIFY(!DbConnection::readOnly());
		QVERIFY(!DbConnection::readWrite());
		QVERIFY(!openForReading(QStringLiteral("test")));
		migrate_db();
		QVERIFY(openForReading(QStringLiteral("test")));
		DbConnection a = DbConnection::readWrite();
		QVERIFY(a);
		sqlite3* raw = a.get();
		DbConnection b = std::move(a);
		QVERIFY(!a);
		QCOMPARE(b.get(), raw);
		QCOMPARE(sqlite3_exec(b.get(), "SELECT 1", nullptr, nullptr, nullptr), SQLITE_OK);
		// Read-only really is read-only.
		DbConnection ro = DbConnection::readOnly();
		QVERIFY(ro);
		QCOMPARE(sqlite3_exec(ro.get(), "DELETE FROM trips", nullptr, nullptr, nullptr), SQLITE_READONLY);
	}

	void analysisReportsAreSavedAndReplaced() {
		Writer w;
		const int trip = db_insert_trip(w.status(), makeRecord());
		const int lo = db_insert_contact(w.status(), CONTACT_TABLE::LIFTOFFS, trip, contactData());
		const int td = db_insert_contact(w.status(), CONTACT_TABLE::TOUCHDOWNS, trip, contactData());
		const QString text = QString::fromUtf8("Grade: A\nSmooth 3° flare, \"good\"");
		QVERIFY(saveAnalysisReport(w.status()->sql, CONTACT_TABLE::LIFTOFFS, lo, text));
		QVERIFY(saveAnalysisReport(w.status()->sql, CONTACT_TABLE::TOUCHDOWNS, td, "first"));
		QVERIFY(saveAnalysisReport(w.status()->sql, CONTACT_TABLE::TOUCHDOWNS, td, "second"));
		QCOMPARE(queryValue(QStringLiteral("SELECT analysis_report FROM trip_liftoffs WHERE id=%1").arg(lo)).toString(), text);
		QCOMPARE(queryValue(QStringLiteral("SELECT analysis_report FROM trip_touchdowns WHERE id=%1").arg(td)).toString(), QStringLiteral("second"));
	}

	void analysisReportForInvalidRowIsIgnored() {
		Writer w;
		QVERIFY(!saveAnalysisReport(w.status()->sql, CONTACT_TABLE::LIFTOFFS, 0, "x"));
		QVERIFY(!saveAnalysisReport(w.status()->sql, CONTACT_TABLE::TOUCHDOWNS, -3, "x"));
		// A row id that doesn't exist changes nothing.
		QVERIFY(saveAnalysisReport(w.status()->sql, CONTACT_TABLE::LIFTOFFS, 99, "x"));
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trip_liftoffs").toInt(), 0);
	}
};

QTEST_MAIN(TstDatabase)
#include "tst_database.moc"
