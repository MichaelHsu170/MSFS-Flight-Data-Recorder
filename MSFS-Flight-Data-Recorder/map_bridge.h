#pragma once

#include <QObject>
#include <QString>

// QObject exposed to the embedded map page's JavaScript via QWebChannel
// (registered as "mapBridge" in map.html). JS calls markerMoved() when the
// user drags the trajectory marker; MapWidget re-emits that as
// cursorIndexChanged() for TrajectoryView to relay to ChartsPanel. JS calls
// rangeChanged() after the map's viewport settles (zoom/pan), re-emitted as
// visibleRangeChanged() so ChartsPanel can zoom its X axis to match.
// JS calls saveAnalysisReport() after a successful AI landing analysis to
// persist the report text into the trip_touchdowns.analysis_report column.
class MapBridge : public QObject {
	Q_OBJECT
public:
	explicit MapBridge(QObject* parent = nullptr);

public slots:
	void markerMoved(int index);
	void rangeChanged(int startIndex, int endIndex);
	void saveAnalysisReport(int rowId, const QString& report);

signals:
	void cursorIndexChanged(int index);
	void visibleRangeChanged(int startIndex, int endIndex);
};
