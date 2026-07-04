#include "data_table_panel.h"
#include "app_settings.h"
#include "trip_data_fields.h"

#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QGridLayout>
#include <QHeaderView>
#include <QScrollArea>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QVector>

namespace {

// "Time (Zulu)"/"Time (Local)" get their own dedicated rows ahead of the
// generic field list (formatted as plain timestamps, not numbers/booleans);
// every other trip_data column follows in TRIP_DATA_NUM_FIELDS then
// TRIP_DATA_BOOL_FIELDS order, matching the rawNums/boolGroup layout in
// TripSamplePoint so showPoint's macro expansion maps row N+2 → rawNums[N].
QStringList buildFieldRowLabels() {
	QStringList labels = { QStringLiteral("Time (Zulu)"), QStringLiteral("Time (Local)") };

#define TRIP_NUM_FIELD(dbColumn, memberExpr) labels.append(tripFieldLabel(#dbColumn));
	TRIP_DATA_NUM_FIELDS(TRIP_NUM_FIELD)
#undef TRIP_NUM_FIELD

#define TRIP_BOOL_FIELD(name, group, bit) labels.append(tripFieldLabel(#name));
	TRIP_DATA_BOOL_FIELDS(TRIP_BOOL_FIELD)
#undef TRIP_BOOL_FIELD

	return labels;
}

}

DataTablePanel::DataTablePanel(QWidget* parent) : QWidget(parent) {
	rowLabels_ = buildFieldRowLabels();

	table_ = new QTableWidget(rowLabels_.size(), 2, this);
	// The filter lives in the header cell itself (a dropdown-style glyph,
	// clicking anywhere on the "Field" header opens the same checkbox dialog
	// the old standalone "Fields…" button did) rather than as a separate
	// button row above the table, mirroring how Excel puts column filters in
	// the header instead of a toolbar.
	table_->setHorizontalHeaderLabels({ QStringLiteral("Field ▾"), QStringLiteral("Value") });
	table_->horizontalHeader()->setCursor(Qt::PointingHandCursor);
	table_->horizontalHeader()->setToolTip(QStringLiteral("Click to choose visible fields"));
	connect(table_->horizontalHeader(), &QHeaderView::sectionClicked, this, [this](int section) {
		if (section == 0)
			openFieldsDialog();
	});
	table_->verticalHeader()->setVisible(false);
	table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
	table_->setSelectionMode(QAbstractItemView::NoSelection);
	// ResizeToContents on the Field column let long labels ("Eng Exhaust Gas
	// Temperature 1") claim the whole panel width, squeezing Value down to
	// nothing -- give Field a fixed width sized for a couple of wrapped words
	// instead, and let Value stretch into whatever's left. Field gets the
	// larger share since most values here are short numbers, while several
	// field labels need two wrapped lines.
	table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Interactive);
	table_->setColumnWidth(0, 140);
	table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
	table_->horizontalHeader()->setStretchLastSection(true);
	table_->setStyleSheet(QStringLiteral("QTableWidget { font-size: 9pt; }"));
	// Long values (e.g. full ISO timestamps) get clipped at the fixed 20px row
	// height with no wrap -- wrap them instead and let each row grow to fit,
	// with the full value always available via tooltip regardless.
	table_->setWordWrap(true);
	table_->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

	for (int row = 0; row < rowLabels_.size(); ++row) {
		auto* label = new QTableWidgetItem(rowLabels_[row]);
		label->setFlags(label->flags() & ~Qt::ItemIsEditable);
		table_->setItem(row, 0, label);
		auto* value = new QTableWidgetItem();
		value->setFlags(value->flags() & ~Qt::ItemIsEditable);
		table_->setItem(row, 1, value);
	}

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(table_);

	applyHiddenFields();
	showEmpty();
}

void DataTablePanel::setDataset(const TripDataset* dataset) {
	dataset_ = dataset;
	cursorIndex_ = -1;
	if (dataset_ && !dataset_->points.empty())
		showPoint(dataset_->points.back());
	else
		showEmpty();
}

void DataTablePanel::setCursorIndex(int index) {
	cursorIndex_ = index;
	if (dataset_ && index >= 0 && index < (int)dataset_->points.size())
		showPoint(dataset_->points[index]);
}

void DataTablePanel::appendLivePoint(const TripSamplePoint& point) {
	// The point has already been appended to the owning TripDataset by
	// TrajectoryView. Just update the display if no cursor is pinned.
	if (cursorIndex_ == -1)
		showPoint(point);
}

void DataTablePanel::openFieldsDialog() {
	QStringList hidden = AppSettings::instance().dataTableHiddenFields();

	QDialog dialog(this);
	dialog.setWindowTitle(QStringLiteral("Visible Fields"));
	auto* dialogLayout = new QVBoxLayout(&dialog);

	// The full field list is too long for one column without the dialog
	// growing taller than the screen -- wrap into a fixed number of columns
	// instead, inside a scroll area so it still works if more fields are
	// added later.
	auto* gridHost = new QWidget(&dialog);
	auto* grid = new QGridLayout(gridHost);
	const int columns = 3;

	QVector<QCheckBox*> boxes;
	boxes.reserve(rowLabels_.size());
	for (int i = 0; i < rowLabels_.size(); ++i) {
		auto* box = new QCheckBox(rowLabels_[i], gridHost);
		box->setChecked(!hidden.contains(rowLabels_[i]));
		grid->addWidget(box, i / columns, i % columns);
		boxes.append(box);
	}

	auto* scrollArea = new QScrollArea(&dialog);
	scrollArea->setWidget(gridHost);
	scrollArea->setWidgetResizable(true);
	scrollArea->setMinimumSize(560, 420);
	dialogLayout->addWidget(scrollArea);

	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
	connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
	connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
	dialogLayout->addWidget(buttons);

	if (dialog.exec() != QDialog::Accepted)
		return;

	QStringList newHidden;
	for (int row = 0; row < rowLabels_.size(); ++row) {
		if (!boxes[row]->isChecked())
			newHidden.append(rowLabels_[row]);
	}
	AppSettings::instance().setDataTableHiddenFields(newHidden);
	applyHiddenFields();
}

void DataTablePanel::applyHiddenFields() {
	QStringList hidden = AppSettings::instance().dataTableHiddenFields();
	for (int row = 0; row < rowLabels_.size(); ++row)
		table_->setRowHidden(row, hidden.contains(rowLabels_[row]));
}

void DataTablePanel::showPoint(const TripSamplePoint& point) {
	// ResizeToContents triggers a word-wrap text layout pass per setText call.
	// 52+ rows × one layout pass each = 8 seconds. Switch to Fixed while
	// updating so Qt defers all size calculations, then do one batch pass.
	QHeaderView* vh = table_->verticalHeader();
	vh->setSectionResizeMode(QHeaderView::Fixed);

	table_->item(0, 1)->setText(point.zuluTime);
	table_->item(0, 1)->setToolTip(point.zuluTime);
	table_->item(1, 1)->setText(point.localTime);
	table_->item(1, 1)->setToolTip(point.localTime);
	{
		int ni = 0, row = 2;
#define TRIP_NUM_DISP(dbColumn, memberExpr) \
		if (row < table_->rowCount()) { \
			QString v = ni < (int)point.rawNums.size() \
				? QString::number(point.rawNums[ni], 'g', 6) : QString(); \
			table_->item(row, 1)->setText(v); \
			table_->item(row, 1)->setToolTip(v); \
			++row; \
		} \
		++ni;
		TRIP_DATA_NUM_FIELDS(TRIP_NUM_DISP)
#undef TRIP_NUM_DISP
#define TRIP_BOOL_DISP(name, group, bit) \
		if (row < table_->rowCount()) { \
			uint32_t bg = (group) == 1 ? point.boolGroup1 \
						: (group) == 2 ? point.boolGroup2 : point.boolGroup3; \
			QString v = (bg >> (bit)) & 1u ? QStringLiteral("Yes") : QStringLiteral("No"); \
			table_->item(row, 1)->setText(v); \
			table_->item(row, 1)->setToolTip(v); \
			++row; \
		}
		TRIP_DATA_BOOL_FIELDS(TRIP_BOOL_DISP)
#undef TRIP_BOOL_DISP
	}

	vh->setSectionResizeMode(QHeaderView::ResizeToContents);
	table_->resizeRowsToContents();
}

void DataTablePanel::showEmpty() {
	for (int row = 0; row < rowLabels_.size(); ++row) {
		table_->item(row, 1)->setText(QString());
		table_->item(row, 1)->setToolTip(QString());
	}
}
