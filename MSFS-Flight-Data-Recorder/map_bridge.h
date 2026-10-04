#pragma once

#include <QObject>
#include <QString>

#include "db_history.h"

// QObject exposed to the embedded map page's JavaScript via QWebChannel
// (registered as "mapBridge" in map.html). JS calls markerMoved() when the
// user drags the trajectory marker; MapWidget re-emits that as
// cursorIndexChanged() for TrajectoryView to relay to ChartsPanel. JS calls
// rangeChanged() after the map's viewport settles (zoom/pan), with the
// version of the trajectory it was measured on (mapSetTrajectoryJs()),
// re-emitted as visibleRangeChanged() for MapWidget to pass on to
// ChartsPanel -- unless that trajectory has since been replaced.
// JS calls saveLiftoffAnalysisReport() after a successful AI liftoff
// analysis to persist the report text into the trip_liftoffs.analysis_report
// column. saveTouchdownAnalysisReport() is the same idea for a landing
// analysis, into trip_touchdowns.analysis_report.
// JS calls overviewSegmentClicked() when the user clicks a trip's
// departure-destination line on the overview map, re-emitted as
// overviewTripClicked() so MapWidget/TrajectoryView can forward it up to
// TripHistoryPanel to select and load that trip, the same as clicking its
// row in the table.
class MapBridge : public QObject {
	Q_OBJECT
public:
	explicit MapBridge(QObject* parent = nullptr);

public slots:
	void markerMoved(int index);
	void rangeChanged(int startIndex, int endIndex, int version);
	void saveLiftoffAnalysisReport(int rowId, const QString& report);
	void saveTouchdownAnalysisReport(int rowId, const QString& report);
	void overviewSegmentClicked(int tripId);

signals:
	void cursorIndexChanged(int index);
	void visibleRangeChanged(int startIndex, int endIndex, int version);
	void overviewTripClicked(int tripId);

private:
	void saveReport(CONTACT_TABLE table, int rowId, const QString& report);
};
