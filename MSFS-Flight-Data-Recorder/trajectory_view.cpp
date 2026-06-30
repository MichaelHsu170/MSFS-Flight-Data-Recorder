#include "trajectory_view.h"
#include "charts_panel.h"
#include "map_widget.h"
#include "data_table_panel.h"

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
	dataTablePanel_->setMaximumWidth(260);
	auto* topSplitter = new QSplitter(Qt::Horizontal, this);
	topSplitter->addWidget(mapWidget_);
	topSplitter->addWidget(dataTablePanel_);
	topSplitter->setStretchFactor(0, 1);
	topSplitter->setStretchFactor(1, 0);
	topSplitter->setSizes({ 900, 220 });
	topSplitter->setCollapsible(0, false);
	topSplitter->setCollapsible(1, false);

	// QSplitter sizes new children by sizeHint() unless told otherwise; the
	// QQuickWidget hosting the charts has a tiny default sizeHint compared to
	// the map/table row, so without explicit setSizes()+a minimum height the
	// charts panel collapses to near-nothing and looks like it isn't
	// rendering at all.
	chartsPanel_->setMinimumHeight(220);

	auto* mainSplitter = new QSplitter(Qt::Vertical, this);
	mainSplitter->addWidget(topSplitter);
	mainSplitter->addWidget(chartsPanel_);
	mainSplitter->setStretchFactor(0, 1);
	mainSplitter->setStretchFactor(1, 1);
	mainSplitter->setSizes({ 400, 400 });
	mainSplitter->setCollapsible(0, false);
	mainSplitter->setCollapsible(1, false);

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(mainSplitter);

	connect(mapWidget_, &MapWidget::cursorIndexChanged, chartsPanel_, &ChartsPanel::setCursorIndex);
	connect(mapWidget_, &MapWidget::cursorIndexChanged, dataTablePanel_, &DataTablePanel::setCursorIndex);
	connect(mapWidget_, &MapWidget::visibleRangeChanged, chartsPanel_, &ChartsPanel::setVisibleRange);
}

void TrajectoryView::setDataset(const TripDataset& dataset) {
	currentTripId_ = dataset.tripId;
	pendingLivePoints_.clear();
	chartsPanel_->setDataset(dataset);
	mapWidget_->setDataset(dataset);
	dataTablePanel_->setDataset(dataset);
}

void TrajectoryView::appendLivePoint(const TripSamplePoint& point) {
	if (liveFollow_) {
		chartsPanel_->appendLivePoint(point);
		mapWidget_->appendLivePoint(point);
		dataTablePanel_->appendLivePoint(point);
	} else {
		pendingLivePoints_.push_back(point);
	}
}

void TrajectoryView::setLiveFollow(bool follow) {
	liveFollow_ = follow;

	if (follow && !pendingLivePoints_.empty()) {
		for (const TripSamplePoint& point : pendingLivePoints_) {
			chartsPanel_->appendLivePoint(point);
			mapWidget_->appendLivePoint(point);
			dataTablePanel_->appendLivePoint(point);
		}
		pendingLivePoints_.clear();
	}
}
