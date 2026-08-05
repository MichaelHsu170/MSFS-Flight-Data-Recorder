#pragma once

#include <QWidget>

#include <utility>
#include <vector>

#include "trip_dataset.h"

class QWebEngineView;
class QWebChannel;
class MapBridge;
class QToolButton;
class QTimer;

// Trajectory map: QWebEngineView loading bundled resources/map.html, which
// draws the trip's polyline + a draggable cursor marker via Leaflet/OSM.
// Bridged to the page's JS via QWebChannel/MapBridge. A small floating icon
// button in the top-right corner toggles cockpit-event markers.
class MapWidget : public QWidget {
	Q_OBJECT
public:
	explicit MapWidget(QWidget* parent = nullptr);

	void setDataset(const TripDataset& dataset);
	// Live mode: append one point without re-sending the whole trajectory.
	void appendLivePoint(const TripSamplePoint& point);
	// Clears the trajectory and draws departure→destination line segments for
	// every trip (the default overview shown when no trip is selected).
	void showOverview(const std::vector<TripSummary>& trips);
	// Refits the map to the current trajectory bounds — same view as right
	// after a trip was loaded.
	void resetZoom();
	// Re-inits the Leaflet map and re-pushes the current trajectory.
	// Called internally from onLoadFinished.
	void refreshProvider();
	// Shows/hides the cockpit-event markers pushed by setDataset() -- the
	// takeoff/touchdown markers are always shown regardless of this setting.
	// Wired internally to the floating events-toggle icon button.
	void setEventsVisible(bool visible);

signals:
	// Forwarded from MapBridge when the user drags the map marker.
	void cursorIndexChanged(int index);
	// Forwarded from MapBridge once the map's zoom/pan viewport settles.
	// startIndex/endIndex are sample indices into the current dataset's
	// points, or (-1, -1) when the full trajectory is back in view.
	void visibleRangeChanged(int startIndex, int endIndex);
	// Emitted once the trajectory polyline has been pushed to the Leaflet page.
	// TrajectoryView uses this to know the map is visually complete.
	void trajectoryLoaded();

private slots:
	void onLoadFinished(bool ok);
	void flushLivePoints();

private:
	void pushTrajectory();
	void pushTakeoffs();
	void pushTouchdownsAndEvents();
	void runJs(const QString& script);
	void resizeEvent(QResizeEvent* event) override;

	QWebEngineView* view_;
	QWebChannel* channel_;
	MapBridge* bridge_;
	QToolButton* eventsToggle_;
	QTimer* liveUpdateTimer_ = nullptr;
	// Lat/lng pairs only -- MapWidget never needs the full TripSamplePoint
	// (rawNums, boolGroups, etc.), so we copy only coordinates into trajCoords_
	// rather than storing the whole TripDataset.
	std::vector<std::pair<double, double>> trajCoords_;
	std::vector<TakeoffPoint> takeoffs_;
	std::vector<TouchdownPoint> touchdowns_;
	std::vector<TripEvent> events_;
	// Buffered lat/lng pairs waiting for the next liveUpdateTimer_ flush.
	std::vector<std::pair<double, double>> pendingLiveCoords_;
	// Stored by showOverview so refreshProvider can re-send them when the
	// WebEngine page finishes loading (the page may not be ready yet on the
	// first showOverview call at startup).
	std::vector<TripSummary> overviewTrips_;
	bool inOverviewMode_ = true;  // start in overview mode; cleared by setDataset
	QString aircraftTitle_;
	bool pageReady_ = false;
	// Incremented by setDataset and showOverview to invalidate any in-flight
	// pushTrajectory / pushTouchdownsAndEvents workers so their finished lambdas
	// don't call setTrajectory() / setTouchdowns() after showOverview() was issued.
	int datasetVersion_ = 0;
	// Set when setDataset() emits trajectoryLoaded() early because the page
	// wasn't ready yet (see setDataset()). Consumed by pushTrajectory()'s
	// completion lambda so the real, deferred push (fired later once the page
	// loads and refreshProvider() catches up) doesn't emit trajectoryLoaded()
	// a second time for the same dataset.
	bool suppressNextTrajectoryLoaded_ = false;
	// Last visibility requested via setEventsVisible(), applied to the page's JS
	// state whenever it's (re)built -- both immediately if the page is already
	// ready, and by refreshProvider() once it becomes ready or is reloaded,
	// since the JS-side toggle would otherwise silently reset to its default.
	bool eventsVisible_ = true;
	// Index of the last marker position reported by a drag/click on the map
	// (via MapBridge::cursorIndexChanged), or -1 if the cursor has never been
	// pinned for the current dataset. setTrajectory() on the JS side always
	// snaps the marker back to the first point, so without reapplying this
	// after every rebuild of the JS-side map state (refreshProvider(), e.g. a
	// provider switch or the WebEngine page reloading), a pinned cursor would
	// silently jump back to the trip start. Reset to -1 by setDataset() and
	// showOverview() -- a genuinely new dataset has no pinned cursor of its
	// own to restore.
	int lastCursorIndex_ = -1;
};
