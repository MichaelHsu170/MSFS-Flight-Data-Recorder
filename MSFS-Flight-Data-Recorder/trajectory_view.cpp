#include "trajectory_view.h"
#include "charts_panel.h"
#include "map_widget.h"
#include "data_table_panel.h"
#include "app_settings.h"
#include "splitter_utils.h"

#include "logger.h"
#include <QSplitter>
#include <QVBoxLayout>
#include <QElapsedTimer>
#include <QThreadPool>

namespace {

// Destroys dataset off the main thread instead of letting it go out of scope
// inline: freeing 50k+ TripSamplePoints (each with a std::vector<double>
// rawNums) inline can visibly stall the UI on Windows.
void destroyOffMainThread(std::shared_ptr<TripDataset>&& dataset) {
	if (dataset)
		QThreadPool::globalInstance()->start([d = std::move(dataset)]() mutable {});
}

}

TrajectoryView::TrajectoryView(QWidget* parent) : QWidget(parent) {
	mapWidget_ = new MapWidget(this);
	dataTablePanel_ = new DataTablePanel(this);
	chartsPanel_ = new ChartsPanel(this);

	// Data table only needs to show one field/value pair's worth of text at a
	// time, so it should stay a narrow, fixed-ish slice of the width -- a
	// stretch factor of 0 (all extra resize space goes to the map) plus a
	// hard maximum width keeps it from ballooning as the window grows.
	dataTablePanel_->setMaximumWidth(kRightPanelWidth);
	mapTableSplitter_ = new QSplitter(Qt::Horizontal, this);
	mapTableSplitter_->addWidget(mapWidget_);
	mapTableSplitter_->addWidget(dataTablePanel_);
	mapTableSplitter_->setStretchFactor(0, 1);
	mapTableSplitter_->setStretchFactor(1, 0);
	mapTableSplitter_->setSizes({ 900, AppSettings::instance().rightPanelWidth() });
	mapTableSplitter_->setCollapsible(0, false);
	mapTableSplitter_->setCollapsible(1, false);
	connect(mapTableSplitter_, &QSplitter::splitterMoved, this, [this]() {
		emit rightPanelWidthChanged(mapTableSplitter_->sizes().last());
	});
	// Persist to settings.ini only once the drag ends -- splitterMoved above
	// fires continuously while dragging (once per pixel), which would
	// otherwise rewrite the whole ini file that often.
	connectSplitterHandleReleased(mapTableSplitter_, 1, [this]() {
		AppSettings::instance().setRightPanelWidth(mapTableSplitter_->sizes().last());
	});

	// Both sections need a floor so neither can steal all the space from the
	// other when the window is small -- without these, QSplitter clips from
	// the last child first and the charts panel can reach 0 height.
	mapTableSplitter_->setMinimumHeight(120);
	chartsPanel_->setMinimumHeight(180);

	auto* mainSplitter = new QSplitter(Qt::Vertical, this);
	mainSplitter->addWidget(mapTableSplitter_);
	mainSplitter->addWidget(chartsPanel_);
	mainSplitter->setStretchFactor(0, 1);
	mainSplitter->setStretchFactor(1, 1);
	mainSplitter->setSizes({ 400, AppSettings::instance().chartsPanelHeight() });
	mainSplitter->setCollapsible(0, false);
	mainSplitter->setCollapsible(1, false);
	// Persist to settings.ini only once the drag ends, not on every
	// intermediate splitterMoved (once per pixel of movement).
	connectSplitterHandleReleased(mainSplitter, 1, [mainSplitter]() {
		AppSettings::instance().setChartsPanelHeight(mainSplitter->sizes().last());
	});

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(mainSplitter);

	connect(mapWidget_, &MapWidget::cursorIndexChanged, chartsPanel_, &ChartsPanel::setCursorIndex);
	connect(mapWidget_, &MapWidget::cursorIndexChanged, dataTablePanel_, &DataTablePanel::setCursorIndex);
	connect(mapWidget_, &MapWidget::visibleRangeChanged, chartsPanel_, &ChartsPanel::setVisibleRange);

	connect(chartsPanel_, &ChartsPanel::seriesLoaded,   this, &TrajectoryView::onSubviewLoaded);
	connect(mapWidget_,   &MapWidget::trajectoryLoaded, this, &TrajectoryView::onSubviewLoaded);
	connect(mapWidget_, &MapWidget::overviewTripClicked, this, &TrajectoryView::overviewTripClicked);
}

void TrajectoryView::setDataset(std::shared_ptr<TripDataset> dataset) {
	if (!dataset) {
		Logger::log(Logger::Warning, "TrajView", QStringLiteral("setDataset() called with a null dataset; ignoring"));
		return;
	}
	// Move the old dataset out before assigning the new one; it's destroyed
	// below, off the main thread, only after every panel has switched over.
	auto oldDataset = std::move(dataset_);
	dataset_ = std::move(dataset);
	pendingRenders_ = 2;  // chartsPanel_ worker + mapWidget_ trajectory worker
	genTimer_.start();
	Logger::logf(Logger::Profile, "TrajView", "--- rendering start: %zu pts ---", dataset_->points.size());
	chartsPanel_->setDataset(*dataset_);
	mapWidget_->setDataset(*dataset_);
	dataTablePanel_->setDataset(dataset_.get());
	// Scheduled only now, after dataTablePanel_->setDataset() above: it holds a
	// raw, non-owning pointer to dataset_, so starting this background deletion
	// any earlier would leave that pointer referencing memory whose destruction
	// may already be running concurrently on another thread.
	destroyOffMainThread(std::move(oldDataset));
}

void TrajectoryView::onSubviewLoaded() {
	// Guard against calls with no render actually pending -- e.g.
	// clearAndShowOverview() sets pendingRenders_ = 0 and then calls
	// chartsPanel_->setDataset(kEmpty), which (empty dataset) takes the
	// synchronous early-return path and emits seriesLoaded() immediately.
	// Without this guard that decrements an already-zeroed pendingRenders_ to
	// -1 and spuriously emits renderingFinished() on every overview reset,
	// which TripHistoryPanel would mistake for "the real load just finished"
	// and use to clear its loading_ guard mid-load.
	if (pendingRenders_ <= 0) {
		Logger::log(Logger::Trace, "TrajView", QStringLiteral("onSubviewLoaded: ignoring, no render pending (e.g. overview reset already completed synchronously)"));
		return;
	}
	if (--pendingRenders_ <= 0) {
		pendingRenders_ = 0;
		if (genTimer_.isValid()) {
			Logger::logf(Logger::Profile, "TrajView", "both subviews loaded: %lld ms total rendering time", genTimer_.nsecsElapsed() / 1000000);
			genTimer_.invalidate();
		}
		emit renderingFinished();
	}
}

void TrajectoryView::clearAndShowOverview(const std::vector<TripSummary>& trips) {
	QElapsedTimer t; t.start();
	pendingRenders_ = 0;
	// Move the dataset out before anything else so it can be destroyed off the
	// main thread below, once the panels have switched away from it.
	auto oldDataset = std::move(dataset_);
	static const TripDataset kEmpty;
	chartsPanel_->setDataset(kEmpty);
	Logger::logf(Logger::Profile, "TrajView", "clearAndShowOverview: chartsPanel done %lld ms", t.nsecsElapsed() / 1000000);
	dataTablePanel_->setDataset(nullptr);
	Logger::logf(Logger::Profile, "TrajView", "clearAndShowOverview: dataTable done %lld ms", t.nsecsElapsed() / 1000000);
	mapWidget_->showOverview(trips);
	Logger::logf(Logger::Profile, "TrajView", "clearAndShowOverview: complete %lld ms total", t.nsecsElapsed() / 1000000);
	destroyOffMainThread(std::move(oldDataset));
}

void TrajectoryView::resetZoom() {
	mapWidget_->resetZoom();
	chartsPanel_->setVisibleRange(-1, -1);
}

void TrajectoryView::setRightPanelWidth(int w) {
	auto sizes = mapTableSplitter_->sizes();
	if (sizes.size() < 2) return;
	mapTableSplitter_->setSizes({ sizes[0] + sizes[1] - w, w });
}
