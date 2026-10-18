// MapBridge (map_bridge.cpp): the map page's calls into C++ -- cursor, range
// and overview-click forwarding, saving AI analysis reports, and each
// contact's final trajectory for its analysis.
#include "test_support.h"

#include "app_settings.h"
#include "db.h"
#include "map_bridge.h"
#include "trip_data_fields.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QtTest>

#include <cmath>

using namespace TestSupport;

class TstMapBridge : public QObject {
	Q_OBJECT

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
		bridge.markerMoved(12, 4);
		bridge.rangeChanged(3, 40, 5);
		bridge.overviewSegmentClicked(7);
		QCOMPARE(cursor.value(0).value(0).toInt(), 12);
		QCOMPARE(cursor.value(0).value(1).toInt(), 4);
		QCOMPARE(range.value(0).value(0).toInt(), 3);
		QCOMPARE(range.value(0).value(1).toInt(), 40);
		QCOMPARE(range.value(0).value(2).toInt(), 5);
		QCOMPARE(clicked.value(0).value(0).toInt(), 7);
	}

	void savesAnalysisReports() {
		exec("INSERT INTO trip_liftoffs (id,trip,airspeed_indicated,vertical_speed,plane_pitch_degrees,plane_bank_degrees,heading_indicator,"
			"plane_latitude,plane_longitude,time_zulu,time_local) VALUES (5,1,0,0,0,0,0,0,0,'z','l');");
		exec("INSERT INTO trip_touchdowns (id,trip,airspeed_indicated,vertical_speed,g_force,plane_pitch_degrees,plane_bank_degrees,heading_indicator,"
			"plane_latitude,plane_longitude,time_zulu,time_local) VALUES (6,1,0,0,0,0,0,0,0,0,'z','l');");
		MapBridge bridge;
		QVERIFY(bridge.saveLiftoffAnalysisReport(5, QString::fromUtf8("Grade: A\nSmooth rotation to 8°, well done")));
		QVERIFY(bridge.saveTouchdownAnalysisReport(6, "Grade: B"));
		QCOMPARE(queryValue("SELECT analysis_report FROM trip_liftoffs WHERE id=5").toString(), QString::fromUtf8("Grade: A\nSmooth rotation to 8°, well done"));
		QCOMPARE(queryValue("SELECT analysis_report FROM trip_touchdowns WHERE id=6").toString(), QStringLiteral("Grade: B"));
		// A save that saveAnalysisReport() refuses (an invalid row id) is
		// reported too.
		QVERIFY(!bridge.saveLiftoffAnalysisReport(0, "x"));
	}

	// A contact's trajectory is built from the dataset MapWidget set, found
	// by its row id; null for a row not in it, without a dataset, or for a
	// contact without a trajectory (a trip without samples).
	void returnsEachContactsTrajectory() {
		auto sample = [](const char* time, double altitude) {
			TripSamplePoint p;
			p.zuluTime = QString::fromLatin1(time);
			p.rawNums.assign(TRIP_NUM_FIELD_COUNT, std::nan(""));
			p.rawNums[TRIP_NUM_plane_altitude] = altitude;
			p.rawNums[TRIP_NUM_plane_alt_above_ground] = altitude - 500;
			return p;
		};
		TripDataset dataset;
		dataset.points = { sample("2026-06-21T12:00:00.000+00:00_1", 500), sample("2026-06-21T12:00:01.000+00:00_1", 560),
			sample("2026-06-21T12:00:59.000+00:00_1", 520), sample("2026-06-21T12:01:00.000+00:00_1", 500) };
		LiftoffPoint liftoff;
		liftoff.rowId = 5;
		liftoff.zuluTime = QStringLiteral("2026-06-21T12:00:00.500+00:00_1");
		dataset.liftoffPoints = { liftoff };
		TouchdownPoint touchdown;
		touchdown.rowId = 6;
		touchdown.zuluTime = QStringLiteral("2026-06-21T12:01:00.000+00:00_1");
		dataset.touchdowns = { touchdown };

		MapBridge bridge;
		QVERIFY(bridge.touchdownProfile(6).isNull());  // no dataset yet
		bridge.setDataset(&dataset);

		// The liftoff's only gate is 35 ft, 0.5 s after it (60 ft).
		const QJsonObject climb = QJsonObject::fromVariantMap(bridge.liftoffProfile(5).toMap());
		const QJsonArray climbGates = climb["gates"].toArray();
		QCOMPARE(climbGates.size(), 1);
		QCOMPARE(climbGates.at(0)["gate"].toString(), QStringLiteral("35 ft"));
		QCOMPARE(climbGates.at(0)["heightFt"].toDouble(), 60.0);
		QCOMPARE(climbGates.at(0)["seconds"].toDouble(), 0.5);
		// The touchdown's series, 15 s before to 10 s after it: the samples
		// 1 s before and at it.
		const QJsonArray series = QJsonObject::fromVariantMap(bridge.touchdownProfile(6).toMap())["series"].toArray();
		QCOMPARE(series.size(), 2);
		QCOMPARE(series.at(0)["seconds"].toDouble(), -1.0);
		QCOMPARE(series.at(0)["heightFt"].toDouble(), 20.0);
		QCOMPARE(series.at(1)["seconds"].toDouble(), 0.0);

		// Each table's own rows only.
		QVERIFY(bridge.touchdownProfile(5).isNull());
		QVERIFY(bridge.liftoffProfile(6).isNull());
		QVERIFY(bridge.touchdownProfile(7).isNull());
		TripDataset noSamples = dataset;
		noSamples.points.clear();
		bridge.setDataset(&noSamples);
		QVERIFY(bridge.touchdownProfile(6).isNull());
		bridge.setDataset(nullptr);
		QVERIFY(bridge.liftoffProfile(5).isNull());
	}

	// The popup's model choice goes to settings.ini, both values.
	void savesTheAiModelChoice() {
		MapBridge bridge;
		bridge.saveAiModel(QStringLiteral("other"), QStringLiteral("gemini-3-pro"));
		QCOMPARE(AppSettings::instance().aiModel(), QStringLiteral("other"));
		QCOMPARE(AppSettings::instance().aiCustomModel(), QStringLiteral("gemini-3-pro"));
		bridge.saveAiModel(QStringLiteral("gemma-4-26b-a4b-it"), QString());
		QCOMPARE(AppSettings::instance().aiModel(), QStringLiteral("gemma-4-26b-a4b-it"));
		QCOMPARE(AppSettings::instance().aiCustomModel(), QString());
	}

	// So does its evaluation choice.
	void savesTheAiProgressEvaluationChoice() {
		MapBridge bridge;
		bridge.saveAiProgressEvaluation(false);
		QCOMPARE(AppSettings::instance().aiProgressEvaluation(), false);
		bridge.saveAiProgressEvaluation(true);
		QCOMPARE(AppSettings::instance().aiProgressEvaluation(), true);
	}

	// Reported as not saved, so the page can say so.
	void reportIsDroppedWithoutADatabase() {
		removeDatabase();
		MapBridge bridge;
		QVERIFY(!bridge.saveTouchdownAnalysisReport(6, "Grade: B"));
		QVERIFY(!bridge.saveLiftoffAnalysisReport(5, "Grade: A"));
		// The read-write connection doesn't create the file.
		sqlite3* db = connect_db_readwrite();
		QVERIFY(!db);
		sqlite3_close(db);
	}
};

QTEST_MAIN(TstMapBridge)
#include "tst_map_bridge.moc"
