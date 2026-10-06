// Main window (main_window.cpp): opening before the database is migrated --
// the notice holding Trip History's place ("Checking the database", then the
// percentage done once the trip_data rebuild reports it, up to 100% even
// for a quick rebuild, or the failure), Trip History and the simulator
// connection waiting for the migration (and never starting after a failed
// one), the window not being recreated when Trip History arrives, and
// closing it cancelling a rebuild still running (starting nothing when
// closed just as the migration finishes, and closing fine once there's none).
// Also the window's wiring: Live Status and the Data Table keeping one width
// (when the window grows, and when either side is dragged, saved on release),
// and Trip History and the trajectory view driving each other.
//
// Needs a custom main() and no QT_QPA_PLATFORM=offscreen, exactly like
// tst_map_widget.cpp: the window's TrajectoryView owns a MapWidget
// (QWebEngineView). See that file's header comment for why; for the same
// reason, no test relies on the map page loading.
#include "main_window.h"
#include "app_settings.h"
#include "charts_panel.h"
#include "data_table_panel.h"
#include "live_status_panel.h"
#include "test_support.h"
#include "trajectory_view.h"
#include "trip_history_panel.h"

#include <QApplication>
#include <QFutureWatcher>
#include <QLabel>
#include <QPointer>
#include <QQuickItem>
#include <QQuickWidget>
#include <QRegularExpression>
#include <QSplitter>
#include <QTableView>
#include <QTableWidget>
#include <QThreadPool>
#include <QtTest>

#include <memory>

using namespace TestSupport;

namespace {

struct Unlock {
	void operator()(sqlite3* lock) const {
		exec(lock, "COMMIT;");
		sqlite3_close(lock);
	}
};
// Released by reset(), or when it goes out of scope (a test that fails
// first); cleanup() then waits for the migration it held up, so a later test
// never finds the database locked or still being migrated.
using DatabaseLock = std::unique_ptr<sqlite3, Unlock>;

// A legacy database with enough jet rows that copying them takes long enough
// to see progress, held locked so the migration waits at its first read
// until the lock is released. Release it within the migration's 5 s busy
// timeout (migrate_db()).
DatabaseLock lockedLegacyDatabase() {
	removeDatabase();
	createLegacyTripData();
	addLegacyJetRows(200000);
	sqlite3* lock = openDatabaseFile();
	if (!lock)
		qFatal("can't open the test database");
	exec(lock, "BEGIN EXCLUSIVE;");
	return DatabaseLock(lock);
}

// The migration's watcher, a direct child of the window until it finishes.
QFutureWatcherBase* migrationWatcher(MainWindow& window) {
	return window.findChild<QFutureWatcherBase*>(QString(), Qt::FindDirectChildrenOnly);
}

// Waits for the migration to finish and Trip History to take the notice's
// place; nullptr if it never does.
TripHistoryPanel* tripHistoryOnceReady(MainWindow& window) {
	waitFor([&window] { return window.findChild<TripHistoryPanel*>() != nullptr; }, 10000);
	return window.findChild<TripHistoryPanel*>();
}

// The top row (Trip History + Live Status) and TrajectoryView's map/table
// row, each the splitter holding its right-hand panel.
QSplitter* topRow(MainWindow& window) {
	return qobject_cast<QSplitter*>(window.findChild<LiveStatusPanel*>()->parentWidget());
}
QSplitter* mapTableRow(MainWindow& window) {
	return qobject_cast<QSplitter*>(window.findChild<DataTablePanel*>()->parentWidget());
}

QString zuluShown(MainWindow& window) {
	QTableWidget* t = window.findChild<DataTablePanel*>()->findChild<QTableWidget*>();
	for (int r = 0; r < t->rowCount(); ++r)
		if (t->item(r, 0)->text() == QStringLiteral("Time (Zulu)"))
			return t->item(r, 1)->text();
	return QStringLiteral("<no Time (Zulu) row>");
}

}

class TstMainWindow : public QObject {
	Q_OBJECT

private slots:
	void initTestCase() {
		isolateFiles();
	}

	// The window doesn't wait for the migration's worker (QtConcurrent::run(),
	// on the global pool), so one a failed test left running would still have
	// the database open when the next test replaces it.
	void cleanup() {
		QThreadPool::globalInstance()->waitForDone();
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
		DatabaseLock lock = lockedLegacyDatabase();
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

		lock.reset();
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
		removeDatabase();
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
		DatabaseLock lock = lockedLegacyDatabase();
		FakeSim::reset();
		RecorderBridge bridge;
		MainWindow window(bridge);
		window.show();
		// Closed while the migration waits on the lock, so it is cancelled
		// before the rebuild can finish however fast the copy runs: it stops
		// after the first batch.
		QTest::qWait(500);
		window.close();
		lock.reset();
		QVERIFY(waitFor([&window] { return migrationWatcher(window) == nullptr; }, 10000));
		const QVariantMap row = queryRows("SELECT * FROM trip_data LIMIT 1").value(0);
		QVERIFY(row.contains("engine_speed"));  // the migration did run
		QVERIFY(row.contains("turb_eng_n1_1"));
		QCOMPARE(queryValue("SELECT COUNT(*) FROM trip_data WHERE engine_speed IS NOT NULL").toInt(), 0);
		QVERIFY(!window.findChild<TripHistoryPanel*>());
		QCOMPARE(FakeSim::state().openCalls, 0);
	}

	// The two right-hand panels line up: growing the window widens only the
	// left side of each row, so both keep the width the user chose.
	void growingTheWindowKeepsBothRightPanelsAtTheChosenWidth() {
		removeDatabase();
		// Below their 260 maximum, so either could grow, and above Live
		// Status's narrowest (239 px), so both can show it.
		AppSettings::instance().setRightPanelWidth(250);
		FakeSim::reset();
		RecorderBridge bridge;
		MainWindow window(bridge);
		window.resize(1000, 800);
		window.show();
		QVERIFY(tripHistoryOnceReady(window));
		QSplitter* top = topRow(window);
		QTRY_COMPARE(top->sizes().last(), 250);
		QTRY_COMPARE(mapTableRow(window)->sizes().last(), 250);

		window.resize(1600, 800);
		QTRY_VERIFY(top->width() > 1500);
		QCOMPARE(top->sizes().last(), 250);
		QCOMPARE(mapTableRow(window)->sizes().last(), 250);
	}

	void draggingEitherRightPanelResizesTheOtherAndSavesTheWidthOnRelease() {
		removeDatabase();
		AppSettings::instance().setRightPanelWidth(200);
		FakeSim::reset();
		RecorderBridge bridge;
		MainWindow window(bridge);
		window.resize(1000, 800);
		window.show();
		QVERIFY(tripHistoryOnceReady(window));
		QSplitter* top = topRow(window);
		QSplitter* mapTable = mapTableRow(window);

		// Live Status's handle (splitterMoved is what a drag emits; see
		// tst_trajectory_view for why it's emitted directly).
		top->setSizes({ top->width() - 220, 220 });
		const int dragged = top->sizes().last();
		emit top->splitterMoved(dragged, 1);
		QCOMPARE(mapTable->sizes().last(), dragged);
		QCOMPARE(AppSettings::instance().rightPanelWidth(), 200); // not saved mid-drag
		sendLeftButton(top->handle(1), QEvent::MouseButtonRelease, QPoint(0, 0));
		QCOMPARE(AppSettings::instance().rightPanelWidth(), dragged);

		// The Data Table's handle.
		mapTable->setSizes({ mapTable->width() - 240, 240 });
		const int draggedBelow = mapTable->sizes().last();
		QVERIFY(draggedBelow != dragged);
		emit mapTable->splitterMoved(draggedBelow, 1);
		QCOMPARE(top->sizes().last(), draggedBelow);
	}

	// A trip clicked on the map is loaded by Trip History and shown by the
	// trajectory view; its rendering finishing unlocks Trip History; Reset
	// Zoom and Deselect from Trip History reach the view.
	void tripHistoryAndTheTrajectoryViewDriveEachOther() {
		removeDatabase();
		FlightDriver sim;
		const int tripId = sim.startTrip();
		sim.ticks(3);
		sim.endTrip();

		MainWindow window(sim.bridge());
		window.resize(1000, 800);
		window.show();
		TripHistoryPanel* tripHistory = tripHistoryOnceReady(window);
		QVERIFY(tripHistory);
		TrajectoryView* view = window.findChild<TrajectoryView*>();
		ChartsPanel* charts = window.findChild<ChartsPanel*>();
		QSignalSpy ready(tripHistory, &TripHistoryPanel::tripDatasetReady);
		QSignalSpy chartsLoaded(charts, &ChartsPanel::seriesLoaded);

		emit view->overviewTripClicked(tripId);
		QVERIFY(waitFor([&ready] { return ready.count() == 1; }));
		const auto dataset = ready.at(0).at(0).value<std::shared_ptr<TripDataset>>();
		QCOMPARE(dataset->tripId, tripId);
		QVERIFY(!dataset->points.empty());
		// The data table's cursor starts on the trip's last sample.
		QCOMPARE(zuluShown(window), dataset->points.back().zuluTime);

		emit view->renderingFinished();
		QVERIFY(tripHistory->findChild<QTableView*>()->isEnabled());

		QVERIFY(waitFor([&chartsLoaded] { return chartsLoaded.count() >= 1; }));
		QQuickItem* chartsRoot = charts->findChild<QQuickWidget*>()->rootObject();
		charts->setVisibleRange(0, 1);
		QCOMPARE(chartsRoot->property("isFullRangeVisible").toBool(), false);
		emit tripHistory->zoomResetRequested();
		QCOMPARE(chartsRoot->property("isFullRangeVisible").toBool(), true);

		emit tripHistory->tripDeselected({});
		QCOMPARE(zuluShown(window), QString());
	}

	void aFailedMigrationSaysSoAndLeavesTheSimulatorAlone() {
		removeDatabase();
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
