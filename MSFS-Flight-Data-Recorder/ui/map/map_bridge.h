#pragma once

#include <QObject>
#include <QString>
#include <QVariant>

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
// JS calls liftoffProfile()/touchdownProfile() when an AI analysis starts or
// its prompt is copied (copyPrompt() in map.html), for that contact's final
// trajectory (contactProfile(), mapProfileJson()), built then rather than
// when the trip loads, as a QVariantMap (QWebChannel hands JS a QJsonValue
// result as an empty array). Each returns an invalid QVariant (null in JS)
// when the contact isn't in the dataset MapWidget set (setDataset()) or has
// no trajectory, and the prompt covers the contact alone.
// JS calls copyText() to put text (an analysis prompt, copyPrompt() in
// map.html) on the system clipboard.
// JS calls saveAiModel() when the AI analysis popup's model drop-down or its
// Other model name changes, to keep both in settings.ini for the next time
// (AppSettings::aiModel()/aiCustomModel()); saveAiProgressEvaluation() when
// its progress-evaluation checkbox changes (AppSettings::aiProgressEvaluation()).
// JS calls overviewSegmentClicked() when the user clicks a trip's
// departure-destination line on the overview map, re-emitted as
// overviewTripClicked() so MapWidget/TrajectoryView can forward it up to
// TripHistoryPanel to select and load that trip, the same as clicking its
// row in the table.
class MapBridge : public QObject {
	Q_OBJECT
public:
	explicit MapBridge(QObject* parent = nullptr);
	// The trip the profile slots look contacts up in; nullptr for none (the
	// overview). Not owned: MapWidget's dataset_, with the same lifetime.
	void setDataset(const TripDataset* dataset) { dataset_ = dataset; }

public slots:
	void markerMoved(int index, int version);
	void rangeChanged(int startIndex, int endIndex, int version);
	bool saveLiftoffAnalysisReport(int rowId, const QString& report);
	bool saveTouchdownAnalysisReport(int rowId, const QString& report);
	void overviewSegmentClicked(int tripId);
	QVariant liftoffProfile(int rowId) const;
	QVariant touchdownProfile(int rowId) const;
	void copyText(const QString& text);
	void saveAiModel(const QString& model, const QString& customModel);
	void saveAiProgressEvaluation(bool progress);

signals:
	void cursorIndexChanged(int index, int version);
	void visibleRangeChanged(int startIndex, int endIndex, int version);
	void overviewTripClicked(int tripId);

private:
	bool saveReport(CONTACT_TABLE table, int rowId, const QString& report);

	const TripDataset* dataset_ = nullptr;
};
