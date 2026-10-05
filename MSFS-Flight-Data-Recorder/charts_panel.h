#pragma once

#include <QWidget>
#include <QDateTime>
#include <QVariantMap>
#include <array>
#include <climits>
#include <optional>
#include <utility>
#include <vector>

#include "chart_data.h"
#include "trip_dataset.h"

class QQuickWidget;
class QLineSeries;
class QDateTimeAxis;
class QValueAxis;

// Stacked timeline charts (engine power -- N1/N2, RPM/manifold pressure etc.
// by engine type, see setEngine() -- vertical speed, speed, altitude, gear,
// brake/flaps/spoiler, fuel weight, pitch, bank) sharing one X axis of real Zulu
// timestamps (matching the database's zulu_time column), each with titled axes.
//
// Qt Graphs' 2D chart surface (GraphsView) has no public C++/QWidget header
// in this Qt version -- only QML (QML_NAMED_ELEMENT) -- so the chart layout
// lives in resources/charts_panel.qml, hosted here via QQuickWidget. The
// series/axis objects it declares (QLineSeries, QValueAxis) are plain public
// QObjects though, so setDataset() drives them straight from C++ by
// objectName, with no QML scripting involved.
class ChartsPanel : public QWidget {
	Q_OBJECT
public:
	explicit ChartsPanel(QWidget* parent = nullptr);

	// Also clears the cursor (see setCursorIndex), which belonged to the
	// previous dataset. An empty dataset empties every line, and each chart
	// hides its axes and legend and shows "No trip selected" -- or "No data
	// recorded" when the dataset has a tripId (a trip with no point);
	// seriesLoaded is then emitted before this returns. Loading a trip after
	// that shows "Loading…" until its lines are in.
	void setDataset(const TripDataset& dataset);
	// Draws the cursor line at the loaded trip's sample index (-1 or out of
	// range hides it). While a trip loads, the last index is applied once
	// its lines are in.
	void setCursorIndex(int index);

	// Zooms the shared X axis to [startIndex, endIndex] (translated to actual
	// timestamps), or resets to the full trip span if startIndex < 0. Driven
	// by MapWidget::visibleRangeChanged so the charts track the map's current
	// viewport. Does nothing with no trip loaded; while a trip loads, the
	// last range is applied once its lines are in. Only a range it applies,
	// or the full range a load shows, counts for skipping a repeat of it.
	void setVisibleRange(int startIndex, int endIndex);

	// Called from QML via the "chartsBridge" context property. Finds the sample
	// nearest to timeMs (epoch ms, same scale as the chart X axis) and returns
	// all series values at that index as a JS-ready map. Returns an empty map
	// when no dataset is loaded. While a trip loads, it reads the previous
	// trip, whose lines are still shown.
	Q_INVOKABLE QVariantMap valueAt(double timeMs) const;

signals:
	// Emitted once a setDataset() call's lines are in: after the worker thread
	// has built all series and they're pushed to QML, or right away for an
	// empty dataset or when charts_panel.qml failed to load (no QML root).
	// Not for a load that a later setDataset() call superseded. TrajectoryView
	// uses this to know when charts are visible.
	void seriesLoaded();

private:
	// Resolves every QLineSeries (by CHART_SERIES' objectNames) and the axes
	// once from the QML object tree and caches them. Idempotent -- safe to
	// call multiple times; no-op after the first successful resolution.
	void buildSeriesCache();
	// Pushes lo/hi directly to every QDateTimeAxis in the QML tree (driver + all
	// SyncedXAxis instances) so QML date-binding conversion never touches the values.
	void setAllXAxisRange(const QDateTime& lo, const QDateTime& hi);
	// Sizes the Y axes to extents (see niceAxisMax()/niceSignedAxisRange(),
	// or engine_'s fixed axis max). extents is always of at least one sample:
	// an empty trip never gets here.
	void setYAxes(const ChartExtents& extents);
	// Labels the engine power chart for engine (chartEngineSpec()): its
	// series, axis titles and, for no recorded power, the no-data message.
	void setEngine(const EnginePower& engine);
	// Loads samples lo..hi of full_, thinned to at most kDisplayPoints plus
	// sample hi, into every series.
	void loadFullSlice(int lo, int hi);

	QQuickWidget* view_;
	// Parallel to the loaded dataset's points -- epoch milliseconds (see
	// chartTimeMs()), used to translate a map-driven sample-index range
	// (setVisibleRange) into an X axis time range and for valueAt().
	std::vector<double> pointTimesMs_;

	// Deduplication: skip setVisibleRange when zoomend+moveend both fire with
	// identical bounds (Leaflet fires both on every zoom interaction).
	int lastRangeStart_ = INT_MIN;
	int lastRangeEnd_   = INT_MIN;

	// Cached QML object pointers -- resolved lazily, once, instead of a
	// findChild tree traversal per series on every load and zoom.
	struct SeriesCache {
		// Indexed by ChartSeriesId.
		std::array<QLineSeries*, CHART_SERIES_COUNT> series{};
		// Driver axis -- the range the QML cursor line and hover readout map
		// against; setAllXAxisRange() sets it along with every per-chart axis.
		QDateTimeAxis* xAxis = nullptr;
		QValueAxis* engSpeedYAxis = nullptr;
		QValueAxis* engLoadYAxis  = nullptr;
		QValueAxis* vsYAxis    = nullptr;
		QValueAxis* speedYAxis = nullptr;
		QValueAxis* altYAxis   = nullptr;
		QValueAxis* fuelYAxis  = nullptr;
		QValueAxis* pitchYAxis = nullptr;
		QValueAxis* bankYAxis  = nullptr;
		bool valid = false;
	} cache_;

	// Extents of the whole loaded trip. setVisibleRange restores the Y axes to
	// these on zoom-out.
	ChartExtents fullExtents_;
	// What the engine power chart is labeled by (setEngine()).
	EnginePower engine_;

	// Full-resolution points of every series, parallel to pointTimesMs_: both
	// are replaced together once a load's lines are in, and cleared together
	// by an empty dataset. setVisibleRange slices this to give Qt Graphs only
	// the points it needs to render, avoiding ~938K-point iteration per frame.
	static constexpr int kDisplayPoints = 2500;
	ChartSeriesLists full_;

	// Incremented at the start of each setDataset call. The background worker's
	// finished lambda captures this value and bails out if it no longer matches,
	// preventing a stale (pre-Deselect) worker from reloading its series after
	// the charts have already been cleared.
	int datasetVersion_ = 0;
	// True from setDataset() of a trip until its lines are in. The old trip's
	// lines stay shown meanwhile, so setVisibleRange() and setCursorIndex()
	// leave them alone: the range or index is the new trip's, and the scales
	// were reset for it. They keep the last one in pendingRange_ and
	// pendingCursorIndex_ instead, applied once the lines are in.
	bool loading_ = false;
	std::optional<std::pair<int, int>> pendingRange_;
	int pendingCursorIndex_ = -1;

};
