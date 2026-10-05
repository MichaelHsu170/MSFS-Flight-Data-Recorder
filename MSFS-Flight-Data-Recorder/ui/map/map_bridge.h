#pragma once

#include <QObject>
#include <QString>

#include "db_history.h"

// QObject exposed to the embedded map page's JavaScript via QWebChannel
// (registered as "mapBridge" in map.html). JS calls markerMoved() when the
// user drags the trajectory marker or clicks the line, and rangeChanged()
// after the map's viewport settles (zoom/pan), each with the version of the
// trajectory it was measured on (mapSetTrajectoryJs()). They are re-emitted
// as cursorIndexChanged() and visibleRangeChanged() for MapWidget to pass on
// (to ChartsPanel and DataTablePanel, via TrajectoryView) -- unless that
// trajectory has since been replaced.
// JS calls saveLiftoffAnalysisReport() after a successful AI liftoff
// analysis to persist the report text into the trip_liftoffs.analysis_report
// column. saveTouchdownAnalysisReport() is the same idea for a landing
// analysis, into trip_touchdowns.analysis_report. Both return whether the
// report was saved (false if the database couldn't be opened or written), so
// the page can tell the user (showSaveResult() in map.html).
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
	void markerMoved(int index, int version);
	void rangeChanged(int startIndex, int endIndex, int version);
	bool saveLiftoffAnalysisReport(int rowId, const QString& report);
	bool saveTouchdownAnalysisReport(int rowId, const QString& report);
	void overviewSegmentClicked(int tripId);

signals:
	void cursorIndexChanged(int index, int version);
	void visibleRangeChanged(int startIndex, int endIndex, int version);
	void overviewTripClicked(int tripId);

private:
	bool saveReport(CONTACT_TABLE table, int rowId, const QString& report);
};
