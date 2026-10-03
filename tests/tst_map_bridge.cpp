// MapBridge (map_bridge.cpp): the map page's calls into C++ -- cursor, range
// and overview-click forwarding, and saving AI analysis reports.
#include "test_support.h"

#include "db.h"
#include "map_bridge.h"

#include <QtTest>

using namespace TestSupport;

class TstMapBridge : public QObject {
	Q_OBJECT

private:
	static void exec(const char* sql) {
		sqlite3* db = connect_db_readwrite();
		QVERIFY(db);
		QCOMPARE(sqlite3_exec(db, sql, nullptr, nullptr, nullptr), SQLITE_OK);
		sqlite3_close(db);
	}

private slots:
	void initTestCase() { isolateFiles(); }
	void init() {
		removeDatabase();
		migrate_db();
	}

	void forwardsMapInteractions() {
		MapBridge bridge;
		QSignalSpy cursor(&bridge, &MapBridge::cursorIndexChanged);
		QSignalSpy range(&bridge, &MapBridge::visibleRangeChanged);
		QSignalSpy clicked(&bridge, &MapBridge::overviewTripClicked);
		bridge.markerMoved(12);
		bridge.rangeChanged(3, 40);
		bridge.overviewSegmentClicked(7);
		QCOMPARE(cursor.value(0).value(0).toInt(), 12);
		QCOMPARE(range.value(0).value(0).toInt(), 3);
		QCOMPARE(range.value(0).value(1).toInt(), 40);
		QCOMPARE(clicked.value(0).value(0).toInt(), 7);
	}

	void savesAnalysisReports() {
		exec("INSERT INTO trip_liftoffs (id,trip,airspeed_indicated,vertical_speed,plane_pitch_degrees,plane_bank_degrees,heading_indicator,"
			"plane_latitude,plane_longitude,time_zulu,time_local) VALUES (5,1,0,0,0,0,0,0,0,'z','l');");
		exec("INSERT INTO trip_touchdowns (id,trip,airspeed_indicated,vertical_speed,g_force,plane_pitch_degrees,plane_bank_degrees,heading_indicator,"
			"plane_latitude,plane_longitude,time_zulu,time_local) VALUES (6,1,0,0,0,0,0,0,0,0,'z','l');");
		MapBridge bridge;
		bridge.saveLiftoffAnalysisReport(5, QString::fromUtf8("Grade: A\nSmooth rotation — well done"));
		bridge.saveTouchdownAnalysisReport(6, "Grade: B");
		QCOMPARE(queryValue("SELECT analysis_report FROM trip_liftoffs WHERE id=5").toString(), QString::fromUtf8("Grade: A\nSmooth rotation — well done"));
		QCOMPARE(queryValue("SELECT analysis_report FROM trip_touchdowns WHERE id=6").toString(), QStringLiteral("Grade: B"));
	}

	void invalidRowIdsAreIgnored() {
		MapBridge bridge;
		bridge.saveLiftoffAnalysisReport(0, "x");
		bridge.saveTouchdownAnalysisReport(-1, "x");
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trip_liftoffs WHERE analysis_report IS NOT NULL").toInt(), 0);
	}

	void reportIsDroppedWithoutADatabase() {
		removeDatabase();
		MapBridge bridge;
		bridge.saveTouchdownAnalysisReport(6, "Grade: B");
		// The read-write connection doesn't create the file.
		sqlite3* db = connect_db_readwrite();
		QVERIFY(!db);
		sqlite3_close(db);
	}
};

QTEST_MAIN(TstMapBridge)
#include "tst_map_bridge.moc"
