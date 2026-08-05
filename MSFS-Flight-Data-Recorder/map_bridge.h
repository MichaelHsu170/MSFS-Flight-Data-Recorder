#pragma once

#include <QObject>
#include <QString>

// QObject exposed to the embedded map page's JavaScript via QWebChannel
// (registered as "mapBridge" in map.html). JS calls markerMoved() when the
// user drags the trajectory marker; MapWidget re-emits that as
// cursorIndexChanged() for TrajectoryView to relay to ChartsPanel. JS calls
// rangeChanged() after the map's viewport settles (zoom/pan), re-emitted as
// visibleRangeChanged() so ChartsPanel can zoom its X axis to match.
// JS calls saveTakeoffAnalysisReport() after a successful AI takeoff analysis
// to persist the report text into the trip_takeoffs.analysis_report column.
// saveTouchdownAnalysisReport() is the same idea for a landing analysis, into
// trip_touchdowns.analysis_report.
class MapBridge : public QObject {
	Q_OBJECT
public:
	explicit MapBridge(QObject* parent = nullptr);

public slots:
	void markerMoved(int index);
	void rangeChanged(int startIndex, int endIndex);
	void saveTakeoffAnalysisReport(int rowId, const QString& report);
	void saveTouchdownAnalysisReport(int rowId, const QString& report);

signals:
	void cursorIndexChanged(int index);
	void visibleRangeChanged(int startIndex, int endIndex);
};
