#pragma once

#include <QWidget>
#include <vector>

#include "trip_dataset.h"

class ChartsPanel;
class MapWidget;
class DataTablePanel;
class QSplitter;

// Composite "view a trip's trajectory" feature: privately owns MapWidget,
// DataTablePanel, and ChartsPanel (map + data table side by side, charts
// spanning full width below), wiring the cursor-index sync between them
// (dragging the map marker highlights the matching point on every chart and
// the data table) internally so MainWindow never needs to know they exist.
class TrajectoryView : public QWidget {
	Q_OBJECT
public:
	explicit TrajectoryView(QWidget* parent = nullptr);

	// tripId of the dataset currently shown, or -1 if none. MainWindow uses
	// this to decide whether a RecorderBridge::liveDataPoint belongs here.
	int currentTripId() const { return currentTripId_; }

signals:
	// Emitted when BOTH the chart series and the map trajectory have been
	// rendered — i.e. both async workers finished. TripHistoryPanel listens to
	// this to re-enable the table and hide the loading bar.
	void renderingFinished();

public slots:
	void setDataset(const TripDataset& dataset);

	// Live mode: append one point to the map/charts. While unpinned (see
	// setLiveFollow), points are buffered instead of pushed to the UI, so
	// "Jump to Live" can catch up in one go without losing samples.
	void appendLivePoint(const TripSamplePoint& point);
	void setLiveFollow(bool follow);

private slots:
	void onSubviewLoaded();

private:
	ChartsPanel* chartsPanel_;
	MapWidget* mapWidget_;
	DataTablePanel* dataTablePanel_;
	QSplitter* mapTableSplitter_;
	// Owns the trip dataset so sub-panels can hold non-owning pointers into it
	// rather than each making their own copy.
	TripDataset dataset_;
	int currentTripId_ = -1;
	bool liveFollow_ = true;
	std::vector<TripSamplePoint> pendingLivePoints_;
	// Counts how many async sub-renders (charts worker + map trajectory) are
	// still outstanding. renderingFinished() is emitted when this hits zero.
	int pendingRenders_ = 0;
};
