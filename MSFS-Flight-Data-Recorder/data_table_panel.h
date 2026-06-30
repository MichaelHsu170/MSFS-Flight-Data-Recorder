#pragma once

#include <QStringList>
#include <QWidget>

#include "trip_dataset.h"

class QTableWidget;

// "Value at a point in time" readout sitting beside the map: one row per
// field of TripSamplePoint (every trip_data column -- see allFields in
// trip_dataset.h), showing the sample at whatever index is currently
// designated -- the dragged map/chart cursor if one has been set, otherwise
// the most recent point (live or historical). Row labels are fixed at
// construction time (same field list/order every point produces, see
// trip_data_fields.h), so the table is built once and only the value column
// is refreshed per point. A filter icon embedded in the "Field" header cell
// (Excel-style) opens a dialog of checkboxes to choose which rows are
// visible (the long field list can otherwise take a lot of scrolling); the
// chosen set is persisted via AppSettings so it survives an app restart.
class DataTablePanel : public QWidget {
	Q_OBJECT
public:
	explicit DataTablePanel(QWidget* parent = nullptr);

public slots:
	void setDataset(const TripDataset& dataset);
	void setCursorIndex(int index);
	void appendLivePoint(const TripSamplePoint& point);

private slots:
	void openFieldsDialog();

private:
	void showPoint(const TripSamplePoint& point);
	void showEmpty();
	void applyHiddenFields();

	QTableWidget* table_;
	QStringList rowLabels_;
	TripDataset dataset_;
	// -1 means "no explicit selection" -- track the latest point instead.
	int cursorIndex_ = -1;
};
