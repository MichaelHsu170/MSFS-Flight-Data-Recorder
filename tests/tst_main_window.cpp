// Main window (main_window.cpp): opening before the database is migrated --
// the notice holding Trip History's place ("Checking the database", then the
// percentage done once the trip_data rebuild reports it, up to 100% even
// for a quick rebuild, or the failure),
// Trip History and the simulator connection waiting for the migration (and
// never starting after a failed one), the window not being recreated when
// Trip History arrives, and closing it cancelling a rebuild still running
// (starting nothing when closed just as the migration finishes, and closing
// fine once there's none).
//
// Needs a custom main() and no QT_QPA_PLATFORM=offscreen, exactly like
// tst_map_widget.cpp: the window's TrajectoryView owns a MapWidget
// (QWebEngineView). See that file's header comment for why; for the same
// reason, no test relies on the map page loading.
#include "main_window.h"
#include "test_support.h"
#include "trip_history_panel.h"
#include "db.h"

#include <QApplication>
#include <QFile>
#include <QFutureWatcher>
#include <QLabel>
#include <QPointer>
#include <QRegularExpression>
#include <QtTest>

using namespace TestSupport;

namespace {

// A legacy database with enough jet rows that copying them takes long enough
// to see progress, held locked so the migration waits at its first read
// until unlock(). Unlock within its 5 s busy timeout (migrate_db()).
sqlite3* lockedLegacyDatabase() {
	QFile::remove(QString::fromStdString(db_file_path()));
	createLegacyTripData();
	addLegacyJetRows(200000);
	sqlite3* lock = nullptr;
	if (sqlite3_open(db_file_path().c_str(), &lock) != SQLITE_OK)
		qFatal("can't open the test database");
	exec(lock, "BEGIN EXCLUSIVE;");
	return lock;
}

void unlock(sqlite3* lock) {
	exec(lock, "COMMIT;");
	sqlite3_close(lock);
}

// The migration's watcher, a direct child of the window until it finishes.
QFutureWatcherBase* migrationWatcher(MainWindow& window) {
	return window.findChild<QFutureWatcherBase*>(QString(), Qt::FindDirectChildrenOnly);
}

}

class TstMainWindow : public QObject {
	Q_OBJECT

private slots:
	void initTestCase() {
		isolateFiles();
	}

	void aQuickMigrationReplacesTheCheckingNoticeWithTripHistory() {
		FakeSim::reset();
		RecorderBridge bridge;
		MainWindow window(bridge);
		window.show();
		QPointer<QLabel> notice = window.findChild<QLabel*>(QStringLiteral("migrationNotice"));
		QVERIFY(notice);
		QCOMPARE(notice->text(), QStringLiteral("Checking the database…"));

		QStringList texts;
		QVERIFY(waitFor([&] {
			if (notice && !texts.contains(notice->text()))
				texts << notice->text();
			return window.findChild<TripHistoryPanel*>() != nullptr;
		}, 10000));
		QVERIFY(!notice);
		QCOMPARE(texts, QStringList{ QStringLiteral("Checking the database…") }); // no upgrade, no percentage

		// Once the migration's watcher is gone, closing has nothing to cancel.
		QVERIFY(waitFor([&window] { return migrationWatcher(window) == nullptr; }, 10000));
		window.close();
		QVERIFY(!window.isVisible());
	}

	// Closed after the migration's worker returned but before its finished
	// signal is handled: too late to cancel, yet nothing starts.
	void closingAsTheMigrationFinishesStartsNothing() {
		FakeSim::reset();
		RecorderBridge bridge;
		MainWindow window(bridge);
		QFutureWatcherBase* migration = migrationWatcher(window);
		QVERIFY(migration);
		migration->waitForFinished();  // the finished signal is still queued
		window.close();
		QVERIFY(waitFor([&window] { return migrationWatcher(window) == nullptr; }, 10000));
		QVERIFY(!window.findChild<TripHistoryPanel*>());
		QCOMPARE(FakeSim::state().openCalls, 0);
	}

	void aSlowMigrationShowsItsProgressThenTheWindowFillsIn() {
		sqlite3* lock = lockedLegacyDatabase();
		FakeSim::reset();
		RecorderBridge bridge;
		MainWindow window(bridge);
		window.resize(1320, 900);
		window.show();
		const WId nativeWindow = window.winId();
		QPointer<QLabel> notice = window.findChild<QLabel*>(QStringLiteral("migrationNotice"));
		QVERIFY(notice);

		QTest::qWait(500); // the migration is blocked: nothing copied yet
		QCOMPARE(notice->text(), QStringLiteral("Checking the database…"));
		QVERIFY(!window.findChild<TripHistoryPanel*>());
		QCOMPARE(FakeSim::state().openCalls, 0);

		unlock(lock);
		const QRegularExpression percent(QStringLiteral("^Updating the database for this version… \\d{1,3}%$"));
		QVERIFY(waitFor([notice, &percent] { return notice && percent.match(notice->text()).hasMatch(); }, 10000));
		QVERIFY(waitFor([&window] { return window.findChild<TripHistoryPanel*>() != nullptr; }, 10000));
		QVERIFY(!notice);
		// Not destroyed and recreated, which looks like closing and reopening.
		QCOMPARE(window.winId(), nativeWindow);
		QCOMPARE(FakeSim::state().openCalls, 1);
		QVERIFY(!queryRows("SELECT * FROM trip_data LIMIT 1").value(0).contains("turb_eng_n1_1"));
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trip_data WHERE engine_speed IS NOT NULL").toInt(), 200003);
	}

	// A small rebuild's steps all finish within Qt's progress throttling
	// interval (40 ms), so only a signal reaching the maximum gets through.
	void aQuickRebuildStillReportsItsLast100Percent() {
		QFile::remove(QString::fromStdString(db_file_path()));
		createLegacyTripData();
		FakeSim::reset();
		RecorderBridge bridge;
		MainWindow window(bridge);
		QSignalSpy progress(migrationWatcher(window), &QFutureWatcherBase::progressValueChanged);
		QVERIFY(waitFor([&window] { return window.findChild<TripHistoryPanel*>() != nullptr; }, 10000));
		QVERIFY(!progress.isEmpty());
		QCOMPARE(progress.last().value(0).toInt(), 100);
	}

	void closingTheWindowMidRebuildCancelsIt() {
		sqlite3* lock = lockedLegacyDatabase();
		FakeSim::reset();
		RecorderBridge bridge;
		MainWindow window(bridge);
		window.show();
		// Closed while the migration waits on the lock, so it is cancelled
		// before the rebuild can finish however fast the copy runs: it stops
		// after the first batch.
		QTest::qWait(500);
		window.close();
		unlock(lock);
		QVERIFY(waitFor([&window] { return migrationWatcher(window) == nullptr; }, 10000));
		const QVariantMap row = queryRows("SELECT * FROM trip_data LIMIT 1").value(0);
		QVERIFY(row.contains("engine_speed"));  // the migration did run
		QVERIFY(row.contains("turb_eng_n1_1"));
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trip_data WHERE engine_speed IS NOT NULL").toInt(), 0);
		QVERIFY(!window.findChild<TripHistoryPanel*>());
		QCOMPARE(FakeSim::state().openCalls, 0);
	}

	void aFailedMigrationSaysSoAndLeavesTheSimulatorAlone() {
		QVERIFY(QFile::remove(QString::fromStdString(db_file_path())));
		createLegacyTripData();
		// Fails the rebuild's rename (see tst_database's failed-migration test).
		exec("CREATE VIEW blocks_rebuild AS SELECT turb_eng_n2_2 FROM trip_data;");

		FakeSim::reset();
		RecorderBridge bridge;
		MainWindow window(bridge);
		window.show();
		QPointer<QLabel> notice = window.findChild<QLabel*>(QStringLiteral("migrationNotice"));
		QVERIFY(notice);
		QVERIFY(waitFor([notice] { return notice && notice->text().startsWith(QStringLiteral("The database couldn't be updated")); }, 10000));
		QVERIFY(notice->wordWrap()); // too long for one line
		// RecorderBridge::start() tries to connect at once, from the same slot.
		QCOMPARE(FakeSim::state().openCalls, 0);
		QVERIFY(!window.findChild<TripHistoryPanel*>());
	}
};

int main(int argc, char* argv[]) {
	QApplication::setAttribute(Qt::AA_ShareOpenGLContexts, true);
	QApplication app(argc, argv);
	TstMainWindow tc;
	return QTest::qExec(&tc, argc, argv);
}

#include "tst_main_window.moc"
