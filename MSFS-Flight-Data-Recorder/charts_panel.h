#pragma once

#include <QWidget>
#include <QDateTime>
#include <vector>

#include "trip_dataset.h"

class QQuickWidget;
class QLineSeries;
class QDateTimeAxis;

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

signals:
	// Emitted once the worker thread has finished building all series and pushed
	// them to QML. TrajectoryView uses this to know when charts are visible.
	void seriesLoaded();

private:
	// Milliseconds since the Unix epoch, parsed from a DATETIME::
	// format_date_time() string ("yyyy-MM-ddTHH:mm:ss.zzz+HH:MM_<dow>") and
	// interpreted as UTC, matching the database's zulu_time column.
	double epochMillisFor(const QString& zuluTime);

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
		bool valid = false;
	} cache_;
};
