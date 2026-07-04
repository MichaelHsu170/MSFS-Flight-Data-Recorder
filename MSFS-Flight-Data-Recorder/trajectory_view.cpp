#include "trajectory_view.h"
#include "charts_panel.h"
#include "map_widget.h"
#include "data_table_panel.h"
#include "app_settings.h"

#include "logger.h"
#include <QSplitter>
#include <QVBoxLayout>

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
		int w = mapTableSplitter_->sizes().last();
		AppSettings::instance().setRightPanelWidth(w);
		emit rightPanelWidthChanged(w);
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
	connect(mainSplitter, &QSplitter::splitterMoved, this, [mainSplitter]() {
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
}

void TrajectoryView::setDataset(std::shared_ptr<TripDataset> dataset) {
	dataset_ = std::move(dataset);
	currentTripId_ = dataset_ ? dataset_->tripId : -1;
	pendingLivePoints_.clear();
	pendingRenders_ = 2;  // chartsPanel_ worker + mapWidget_ trajectory worker
	genTimer_.start();
	Logger::logf(Logger::Profile, "TrajView", "--- rendering start: %zu pts ---", dataset_ ? dataset_->points.size() : 0u);
	chartsPanel_->setDataset(*dataset_);
	mapWidget_->setDataset(*dataset_);
	dataTablePanel_->setDataset(dataset_.get());
}

void TrajectoryView::onSubviewLoaded() {
	if (--pendingRenders_ <= 0) {
		pendingRenders_ = 0;
		if (genTimer_.isValid()) {
			Logger::logf(Logger::Profile, "TrajView", "both subviews loaded: %lld ms total rendering time", genTimer_.nsecsElapsed() / 1000000);
			genTimer_.invalidate();
		}
		emit renderingFinished();
	}
}

void TrajectoryView::appendLivePoint(const TripSamplePoint& point) {
	if (liveFollow_) {
		if (dataset_) dataset_->points.push_back(point);
		chartsPanel_->appendLivePoint(point);
		mapWidget_->appendLivePoint(point);
		dataTablePanel_->appendLivePoint(point);
	} else {
		pendingLivePoints_.push_back(point);
	}
}

void TrajectoryView::setRightPanelWidth(int w) {
	auto sizes = mapTableSplitter_->sizes();
	if (sizes.size() < 2) return;
	mapTableSplitter_->setSizes({ sizes[0] + sizes[1] - w, w });
}

void TrajectoryView::setLiveFollow(bool follow) {
	liveFollow_ = follow;

	if (follow && !pendingLivePoints_.empty()) {
		for (const TripSamplePoint& point : pendingLivePoints_) {
			if (dataset_) dataset_->points.push_back(point);
			chartsPanel_->appendLivePoint(point);
			mapWidget_->appendLivePoint(point);
			dataTablePanel_->appendLivePoint(point);
		}
		pendingLivePoints_.clear();
	}
}
