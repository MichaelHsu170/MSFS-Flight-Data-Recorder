#pragma once

#include <QWidget>
#include <vector>

#include "trip_dataset.h"

class ChartsPanel;
class MapWidget;
class DataTablePanel;

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

public slots:
	void setDataset(const TripDataset& dataset);

	// Live mode: append one point to the map/charts. While unpinned (see
	// setLiveFollow), points are buffered instead of pushed to the UI, so
	// "Jump to Live" can catch up in one go without losing samples.
	void appendLivePoint(const TripSamplePoint& point);
	void setLiveFollow(bool follow);

private:
	ChartsPanel* chartsPanel_;
	MapWidget* mapWidget_;
	DataTablePanel* dataTablePanel_;
	int currentTripId_ = -1;
	bool liveFollow_ = true;
	std::vector<TripSamplePoint> pendingLivePoints_;
};
