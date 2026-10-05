#pragma once

#include <QStringList>
#include <QWidget>

#include "trip_dataset.h"

class QTableWidget;

// "Value at a point in time" readout sitting beside the map: one row per
// field of TripSamplePoint (every trip_data column -- see rawNums/boolGroups
// in trip_dataset.h -- plus one row per engine speed/load), showing the
// sample at the map's cursor (dragged or clicked) once one has been set,
// otherwise the trip's last point; fields of an engine past the aircraft's
// engine count are left blank (tripFieldEngine()). Row labels
// are fixed at construction time (same field list/order every point
// produces, see trip_data_fields.h), so the table is built once and only the
// value column is refreshed per point. A filter icon
// embedded in the "Field" header cell (Excel-style) opens a dialog of
// checkboxes to choose which rows are visible (the long field list can
// otherwise take a lot of scrolling); the chosen set is persisted via
// AppSettings so it survives an app restart.
class DataTablePanel : public QWidget {
	Q_OBJECT
public:
	explicit DataTablePanel(QWidget* parent = nullptr);

public slots:
	void setDataset(const TripDataset* dataset);
	void setCursorIndex(int index);

private slots:
	void openFieldsDialog();

private:
	void showPoint(const TripSamplePoint& point);
	void showEmpty();
	// Shows text in row's value cell, and as its tooltip so a value clipped
	// by the narrow column can still be read in full.
	void setValue(int row, const QString& text);
	void applyHiddenFields();

	QTableWidget* table_;
	QStringList rowLabels_;
	const TripDataset* dataset_ = nullptr;
};
