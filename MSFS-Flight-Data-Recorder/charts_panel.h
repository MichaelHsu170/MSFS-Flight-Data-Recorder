#pragma once

#include <QWidget>
#include <QDateTime>
#include <QVariantMap>
#include <array>
#include <vector>

#include "chart_data.h"
#include "trip_dataset.h"

class QQuickWidget;
class QLineSeries;
class QDateTimeAxis;
class QValueAxis;

// Stacked timeline charts (N1/N2, vertical speed, speed, altitude, gear,
// brake/flaps/spoiler, fuel weight) sharing one X axis of real Zulu
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
	// previous dataset.
	void setDataset(const TripDataset& dataset);
	void setCursorIndex(int index);

	// Live mode: append one point without rebuilding every series from
	// scratch. Uses cached series pointers so there are no findChild
	// traversals per sample. pointCount_ tracks the next sample index
	// (reset by setDataset). Returns false if the point was dropped
	// (malformed zuluTime, or the QML series cache isn't ready yet) instead
	// of appended -- callers that keep other, index-synced views (MapWidget,
	// DataTablePanel) in lockstep with this one must skip the point there too
	// on a false return, or their point counts desync from this panel's.
	bool appendLivePoint(const TripSamplePoint& point);
	// Zooms the shared X axis to [startIndex, endIndex] (translated to actual
	// timestamps), or resets to the full trip span if startIndex < 0. Driven
	// by MapWidget::visibleRangeChanged so the charts track the map's current
	// viewport.
	void setVisibleRange(int startIndex, int endIndex);

	// Called from QML via the "chartsBridge" context property. Finds the sample
	// nearest to timeMs (epoch ms, same scale as the chart X axis) and returns
	// all series values at that index as a JS-ready map. Returns an empty map
	// when no dataset is loaded.
	Q_INVOKABLE QVariantMap valueAt(double timeMs) const;

signals:
	// Emitted once the worker thread has finished building all series and pushed
	// them to QML. TrajectoryView uses this to know when charts are visible.
	void seriesLoaded();

private:
	// Resolves every QLineSeries (by CHART_SERIES' objectNames) and the axes
	// once from the QML object tree and caches them. Idempotent -- safe to
	// call multiple times; no-op after the first successful resolution.
	void buildSeriesCache();
	// Pushes lo/hi directly to every QDateTimeAxis in the QML tree (driver + all
	// SyncedXAxis instances) so QML date-binding conversion never touches the values.
	void setAllXAxisRange(const QDateTime& lo, const QDateTime& hi);
	// Sizes the Y axes to extents (see niceAxisMax()/niceSignedAxisRange());
	// the signed axes only once extents is valid.
	void setYAxes(const ChartExtents& extents);
	// Loads samples lo..hi of full_, thinned to at most kDisplayPoints, into
	// every series.
	void loadFullSlice(int lo, int hi);

	QQuickWidget* view_;
	int pointCount_ = 0;
	// Parallel to the loaded dataset's points -- epoch milliseconds (see
	// chartTimeMs()), used to translate a map-driven sample-index range
	// (setVisibleRange) into an X axis time range and for valueAt().
	std::vector<double> pointTimesMs_;

	// Deduplication: skip setVisibleRange when zoomend+moveend both fire with
	// identical bounds (Leaflet fires both on every zoom interaction).
	int lastRangeStart_ = INT_MIN;
	int lastRangeEnd_   = INT_MIN;

	// Cached QML object pointers -- resolved lazily so the per-sample live
	// path avoids a findChild tree traversal per series per sample.
	struct SeriesCache {
		// Indexed by ChartSeriesId.
		std::array<QLineSeries*, CHART_SERIES_COUNT> series{};
		// Driver axis -- C++ calls setMin/setMax here; per-chart axes bind to it.
		QDateTimeAxis* xAxis = nullptr;
		QValueAxis* vsYAxis    = nullptr;
		QValueAxis* speedYAxis = nullptr;
		QValueAxis* altYAxis   = nullptr;
		QValueAxis* fuelYAxis  = nullptr;
		QValueAxis* pitchYAxis = nullptr;
		QValueAxis* bankYAxis  = nullptr;
		bool valid = false;
	} cache_;

	// Extents of the whole loaded trip, grown by each live point (reset by
	// setDataset). setVisibleRange restores the Y axes to these on zoom-out.
	ChartExtents fullExtents_;

	// Full-resolution points of every series. Populated by the setDataset
	// apply callback; setVisibleRange slices this to give Qt Graphs only the
	// points it needs to render, avoiding ~938K-point iteration per frame.
	// Invalidated (fullReady_ = false) at the start of each setDataset call so
	// a stale slice is never used while a new background compute is in
	// flight, and by live points, which aren't added to it.
	static constexpr int kDisplayPoints = 2500;
	ChartSeriesLists full_;
	bool fullReady_ = false;

	// Incremented at the start of each setDataset call. The background worker's
	// finished lambda captures this value and bails out if it no longer matches,
	// preventing a stale (pre-Deselect) worker from reloading its series after
	// the charts have already been cleared.
	int datasetVersion_ = 0;

};
