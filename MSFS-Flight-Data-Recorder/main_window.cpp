#include "main_window.h"
#include "live_status_panel.h"
#include "trip_history_panel.h"
#include "trajectory_view.h"
#include "recorder_bridge.h"
#include "app_settings.h"
#include "splitter_utils.h"
#include "logger.h"
#include "db.h"

#include <QFutureWatcher>
#include <QLabel>
#include <QPromise>
#include <QSplitter>
#include <QVBoxLayout>
#include <QWidget>
#include <QtConcurrent/QtConcurrentRun>

MainWindow::MainWindow(RecorderBridge& bridge, QWidget* parent)
	: QMainWindow(parent)
{
	setWindowTitle("MSFS Flight Data Recorder");

	// Holds Trip History's place until the migration below finishes.
	auto* notice = new QLabel(QStringLiteral("Checking the database…"), this);
	notice->setObjectName(QStringLiteral("migrationNotice"));
	notice->setAlignment(Qt::AlignCenter);
	liveStatusPanel_ = new LiveStatusPanel(bridge, this);
	trajectoryView_ = new TrajectoryView(this);

	liveStatusPanel_->setMaximumWidth(kRightPanelWidth);

	topSplitter_ = new QSplitter(Qt::Horizontal, this);
	topSplitter_->addWidget(notice);
	topSplitter_->addWidget(liveStatusPanel_);
	topSplitter_->setStretchFactor(0, 4);
	topSplitter_->setStretchFactor(1, 1);
	topSplitter_->setSizes({ 1000, AppSettings::instance().rightPanelWidth() });
	// Default handle width (and each panel's own default QVBoxLayout margins)
	// compounded into a wide gap between the trip table and Live Status --
	// thin the handle down since a 1px divider is plenty to show the split.
	topSplitter_->setHandleWidth(1);
	connect(topSplitter_, &QSplitter::splitterMoved, this, [this](int, int) {
		trajectoryView_->setRightPanelWidth(topSplitter_->sizes().last());
	});
	// Persist to settings.ini only once the drag ends -- splitterMoved above
	// fires continuously while dragging (once per pixel), which would
	// otherwise rewrite the whole ini file that often.
	connectSplitterHandleReleased(topSplitter_, 1, [this]() {
		AppSettings::instance().setRightPanelWidth(topSplitter_->sizes().last());
	});
	connect(trajectoryView_, &TrajectoryView::rightPanelWidthChanged, this, [this](int w) {
		setSecondSectionSize(topSplitter_, w);
	});

	// Without explicit sizes, QSplitter divides initial space by each child's
	// sizeHint() -- the trip table's sizeHint can dwarf trajectoryView_'s
	// QQuickWidget-hosted charts, starving them down to a sliver. Give the
	// trip history row a fixed-ish share and let the trajectory view (map +
	// table + charts) claim the rest.
	auto* mainSplitter = new QSplitter(Qt::Vertical, this);
	mainSplitter->addWidget(topSplitter_);
	mainSplitter->addWidget(trajectoryView_);
	mainSplitter->setStretchFactor(0, 0);
	mainSplitter->setStretchFactor(1, 1);
	mainSplitter->setSizes({ 220, 700 });
	mainSplitter->setCollapsible(1, false);
	setCentralWidget(mainSplitter);

	// Upgrading a large database after an app update can take a while (moving
	// legacy columns rebuilds trip_data), so the window opens first and says
	// what it waits for. migrate_db() reports progress only for that upgrade;
	// a normal start's quick schema check just has the notice replaced by
	// addTripHistory(). If it fails, the notice says so and stays: neither
	// Trip History nor recording can work on that database, and starting the
	// bridge would only have connect_db() redo the migration on this thread.
	auto* migration = new QFutureWatcher<bool>(this);
	migration_ = migration;
	connect(migration, &QFutureWatcher<bool>::progressValueChanged, notice, [notice](int percent) {
		// setFuture() replays the started future's progress, 0, which isn't
		// an upgrade reporting any.
		if (percent > 0)
			notice->setText(QStringLiteral("Updating the database for this version… %1%").arg(percent));
	});
	connect(migration, &QFutureWatcher<bool>::finished, this, [this, migration, notice, &bridge]() {
		migration->deleteLater();
		if (migration->isCanceled())
			// Closed (closeEvent()): mid-rebuild there's no result, and a future
			// that had already finished is marked cancelled too, so closing just
			// as it finished starts nothing either.
			return;
		if (!migration->result()) {
			notice->setText(QStringLiteral("The database couldn't be updated for this version, so trips can't be "
				"shown or recorded. Restart the app to try again; msfs_fdr_debug.log has the details."));
			notice->setWordWrap(true);
			return;
		}
		Logger::log(Logger::Trace, "MainWin", QStringLiteral("Database migration checked/applied"));
		addTripHistory(bridge);
		bridge.start();
	});
	migration->setFuture(QtConcurrent::run([](QPromise<bool>& promise) {
		// Qt throttles progress signals, except one reaching the maximum, so
		// without this range a quick last step's 100% could be dropped.
		promise.setProgressRange(0, 100);
		promise.addResult(migrate_db([&promise](int percent) { promise.setProgressValue(percent); },
			[&promise] { return promise.isCanceled(); }));
	}));
}

// The migration runs on Qt's global thread pool, which the app waits for on
// exit: left running, a long rebuild would keep the process (and its
// single-instance lock) alive with no window until it finished. Cancelling
// rolls it back once the batch being copied (or the old table's drop)
// finishes, to be redone on the next start; once it is committing or
// recreating the indexes it can't be stopped, and the app still waits for it.
void MainWindow::closeEvent(QCloseEvent* event) {
	if (migration_)
		migration_->cancel();
	QMainWindow::closeEvent(event);
}

void MainWindow::addTripHistory(RecorderBridge& bridge) {
	tripHistoryPanel_ = new TripHistoryPanel(bridge, this);
	delete topSplitter_->replaceWidget(0, tripHistoryPanel_);
	topSplitter_->setStretchFactor(0, 4);

	connect(tripHistoryPanel_, &TripHistoryPanel::tripDatasetReady,  trajectoryView_, &TrajectoryView::setDataset);
	connect(tripHistoryPanel_, &TripHistoryPanel::tripDeselected,   trajectoryView_, &TrajectoryView::clearAndShowOverview);
	connect(tripHistoryPanel_, &TripHistoryPanel::zoomResetRequested, trajectoryView_, &TrajectoryView::resetZoom);
	connect(trajectoryView_, &TrajectoryView::renderingFinished, tripHistoryPanel_, &TripHistoryPanel::setLoadingFinished);
	connect(trajectoryView_, &TrajectoryView::overviewTripClicked, tripHistoryPanel_, &TripHistoryPanel::selectTripById);

	// Show departure→destination arcs for all trips on the initial map load.
	// If the WebEngine page isn't ready yet, MapWidget stores the trips and
	// re-sends them when onLoadFinished fires via refreshProvider.
	Logger::log(Logger::Trace, "MainWin", QStringLiteral("Requesting initial trip overview on map"));
	tripHistoryPanel_->showInitialOverview();
}
