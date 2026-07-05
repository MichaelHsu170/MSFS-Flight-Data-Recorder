#pragma once

#include <QWidget>
#include <QDateTime>
#include <QVariantMap>
#include <vector>

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

	void setDataset(const TripDataset& dataset);
	void setCursorIndex(int index);

	// Live mode: append one point without rebuilding every series from
	// scratch. Uses cached series pointers so there are no findChild
	// traversals per sample. pointCount_ tracks the next sample index
	// (reset by setDataset).
	void appendLivePoint(const TripSamplePoint& point);
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
	// Resolves all 19 QLineSeries* + the shared QDateTimeAxis* once from the
	// QML object tree and caches them. Idempotent -- safe to call multiple
	// times; no-op after the first successful resolution.
	void buildSeriesCache();

	QQuickWidget* view_;
	int pointCount_ = 0;
	// Parallel to the loaded dataset's points -- epoch milliseconds, used
	// both to fill series and to translate a map-driven sample-index range
	// (setVisibleRange) into an X axis time range.
	std::vector<double> pointTimesMs_;

	// Deduplication: skip setVisibleRange when zoomend+moveend both fire with
	// identical bounds (Leaflet fires both on every zoom interaction).
	int lastRangeStart_ = INT_MIN;
	int lastRangeEnd_   = INT_MIN;

	// Cached QML object pointers -- resolved lazily on the first
	// appendLivePoint call so the per-sample hot path avoids 20 findChild
	// tree traversals per incoming sample.
	struct SeriesCache {
		QLineSeries* n1_1 = nullptr;
		QLineSeries* n1_2 = nullptr;
		QLineSeries* n2_1 = nullptr;
		QLineSeries* n2_2 = nullptr;
		QLineSeries* verticalSpeed = nullptr;
		QLineSeries* airspeed = nullptr;
		QLineSeries* groundSpeed = nullptr;
		QLineSeries* altitude = nullptr;
		QLineSeries* gearHandle = nullptr;
		QLineSeries* gearPos0 = nullptr;
		QLineSeries* gearPos1 = nullptr;
		QLineSeries* gearPos2 = nullptr;
		QLineSeries* gearOnGround0 = nullptr;
		QLineSeries* gearOnGround1 = nullptr;
		QLineSeries* gearOnGround2 = nullptr;
		QLineSeries* brake = nullptr;
		QLineSeries* flaps = nullptr;
		QLineSeries* spoilers = nullptr;
		QLineSeries* fuelWeight = nullptr;
		// Driver axis -- C++ calls setMin/setMax here; per-chart axes bind to it.
		QDateTimeAxis* xAxis = nullptr;
		QValueAxis* speedYAxis = nullptr;
		QValueAxis* altYAxis   = nullptr;
		QValueAxis* fuelYAxis  = nullptr;
		bool valid = false;
	} cache_;

	// Running Y-axis maximums for the live append path -- updated whenever an
	// incoming point exceeds the current axis max (reset by setDataset).
	// Also used by setVisibleRange to restore full-range maxima on zoom-out.
	double liveSpeedMax_ = 0.0;
	double liveAltMax_   = 0.0;
	double liveFuelMax_  = 0.0;

	// Cached local-UTC offset used by epochMillisFor to avoid per-call timezone
	// DST lookups. Refreshed at setDataset() time and initialized in the ctor.
	qint64 localOffsetMs_ = 0;

	// Full-resolution QPointF arrays for all 19 series. Populated by the
	// setDataset apply callback; setVisibleRange slices this to give Qt Graphs
	// only the points it needs to render, avoiding ~938K-point iteration per
	// frame. Invalidated (ready=false) at the start of each setDataset call so
	// a stale slice is never used while a new background compute is in flight.
	static constexpr int kDisplayPoints = 2500;
	struct FullSeriesData {
		QList<QPointF> n1_1, n1_2, n2_1, n2_2;
		QList<QPointF> verticalSpeed, airspeed, groundSpeed, altitude;
		QList<QPointF> gearHandle, gearPos0, gearPos1, gearPos2;
		QList<QPointF> gearOnGround0, gearOnGround1, gearOnGround2;
		QList<QPointF> brake, flaps, spoilers, fuelWeight;
		bool ready = false;
	} full_;
	int displayStride_ = 1;

	// Incremented at the start of each setDataset call. The background worker's
	// finished lambda captures this value and bails out if it no longer matches,
	// preventing a stale (pre-Deselect) worker from reloading its series after
	// the charts have already been cleared.
	int datasetVersion_ = 0;

};
