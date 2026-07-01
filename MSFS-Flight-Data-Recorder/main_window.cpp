#include "main_window.h"
#include "live_status_panel.h"
#include "trip_history_panel.h"
#include "trajectory_view.h"
#include "recorder_bridge.h"
#include "app_settings.h"

#include <QSplitter>
#include <QVBoxLayout>
#include <QWidget>

MainWindow::MainWindow(RecorderBridge& bridge, QWidget* parent)
	: QMainWindow(parent)
{
	setWindowTitle("MSFS Flight Data Recorder");

	tripHistoryPanel_ = new TripHistoryPanel(bridge, this);
	liveStatusPanel_ = new LiveStatusPanel(bridge, this);
	trajectoryView_ = new TrajectoryView(this);

	auto* topSplitter = new QSplitter(Qt::Horizontal, this);
	topSplitter->addWidget(tripHistoryPanel_);
	topSplitter->addWidget(liveStatusPanel_);
	topSplitter->setStretchFactor(0, 4);
	topSplitter->setStretchFactor(1, 1);
	topSplitter->setSizes({ 1000, 260 });
	// Default handle width (and each panel's own default QVBoxLayout margins)
	// compounded into a wide gap between the trip table and Live Status --
	// thin the handle down since a 1px divider is plenty to show the split.
	topSplitter->setHandleWidth(1);
	QByteArray savedTop = AppSettings::instance().topSplitterState();
	if (!savedTop.isEmpty())
		topSplitter->restoreState(savedTop);
	connect(topSplitter, &QSplitter::splitterMoved, this, [topSplitter]() {
		AppSettings::instance().setTopSplitterState(topSplitter->saveState());
	});

	// Without explicit sizes, QSplitter divides initial space by each child's
	// sizeHint() -- the trip table's sizeHint can dwarf trajectoryView_'s
	// QQuickWidget-hosted charts, starving them down to a sliver. Give the
	// trip history row a fixed-ish share and let the trajectory view (map +
	// table + charts) claim the rest.
	auto* mainSplitter = new QSplitter(Qt::Vertical, this);
	mainSplitter->addWidget(topSplitter);
	mainSplitter->addWidget(trajectoryView_);
	mainSplitter->setStretchFactor(0, 0);
	mainSplitter->setStretchFactor(1, 1);
	mainSplitter->setSizes({ 220, 700 });
	mainSplitter->setCollapsible(1, false);
	setCentralWidget(mainSplitter);

	connect(tripHistoryPanel_, &TripHistoryPanel::tripDatasetReady, trajectoryView_, &TrajectoryView::setDataset);
	connect(trajectoryView_, &TrajectoryView::renderingFinished, tripHistoryPanel_, &TripHistoryPanel::setLoadingFinished);
}
