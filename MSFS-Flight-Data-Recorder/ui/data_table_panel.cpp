#include "data_table_panel.h"
#include "app_settings.h"
#include "trip_data_fields.h"
#include "logger.h"
#include "types.h"

#include <QAction>
#include <QCheckBox>
#include <QClipboard>
#include <QDialog>
#include <QDialogButtonBox>
#include <QGridLayout>
#include <QGuiApplication>
#include <QHeaderView>
#include <QMenu>
#include <QScrollArea>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QVector>

#include <algorithm>
#include <cmath>

namespace {

// The two columns shown together in the "GPS Position" row, not in rows of
// their own.
bool isGpsPositionColumn(QLatin1String column) {
	return column == QLatin1String("gps_position_lat") || column == QLatin1String("gps_position_lon");
}

// "Time (Zulu)"/"Time (Local)" get their own dedicated rows ahead of the
// generic field list (formatted as plain timestamps, not numbers/booleans),
// then the single "GPS Position" row that combines gps_position_lat and
// gps_position_lon. Every other trip_data column follows from row 3 in
// TRIP_DATA_NUM_FIELDS then TRIP_DATA_BOOL_FIELDS order, matching the
// rawNums/boolGroup layout in TripSamplePoint, so showPoint() fills the rows
// by walking the same macros. Last come the per-engine rows: for each
// TRIP_ENGINE_FIELDS value in order, one row per engine 1..engines, labeled
// like the "<value>_<engine>" trip_data columns that held engines 1-2 before
// trip_engine_data ("Eng Oil Pressure 2"), so a field hidden then stays
// hidden.
QStringList buildFieldRowLabels(int engines) {
	QStringList labels = { QStringLiteral("Time (Zulu)"), QStringLiteral("Time (Local)"), QStringLiteral("GPS Position") };

#define TRIP_NUM_FIELD(dbColumn, memberExpr, sqlType) \
	if (!isGpsPositionColumn(QLatin1String(#dbColumn))) \
		labels.append(tripFieldLabel(#dbColumn));
	TRIP_DATA_NUM_FIELDS(TRIP_NUM_FIELD)
#undef TRIP_NUM_FIELD

#define TRIP_BOOL_FIELD(name, group, bit) labels.append(tripFieldLabel(#name));
	TRIP_DATA_BOOL_FIELDS(TRIP_BOOL_FIELD)
#undef TRIP_BOOL_FIELD

#define TRIP_ENGINE_FIELD(name, sqlType, onOff) \
	for (int engine = 1; engine <= engines; ++engine) \
		labels.append(tripFieldLabel((QByteArray(#name "_") + QByteArray::number(engine)).constData()));
	TRIP_ENGINE_FIELDS(TRIP_ENGINE_FIELD)
#undef TRIP_ENGINE_FIELD

	return labels;
}

// The most engines any of dataset's points has values for (0 without one).
int datasetEngineCount(const TripDataset* dataset) {
	int engines = 0;
	if (dataset)
		for (const TripSamplePoint& point : dataset->points)
			engines = std::max(engines, point.engineCount());
	return engines;
}

// The index of column in TRIP_DATA_NUM_FIELDS' order (== TripSamplePoint::
// rawNums index), from the X-macro itself so it stays right when fields are
// added or reordered; -1 if it isn't one.
int numFieldIndex(QLatin1String column) {
	int idx = 0;
#define TRIP_NUM_INDEX(dbColumn, memberExpr, sqlType) \
	if (QLatin1String(#dbColumn) == column) return idx; \
	++idx;
	TRIP_DATA_NUM_FIELDS(TRIP_NUM_INDEX)
#undef TRIP_NUM_INDEX
	return -1;
}

// point's rawNums[index] (numFieldIndex()), or NaN if it has none.
double rawNum(const TripSamplePoint& point, int index) {
	return index >= 0 && index < (int)point.rawNums.size() ? point.rawNums[index] : std::nan("");
}

// Decimal degrees -> "lat lng" in DMS (COORDINATE::coordinate_decimal_to_dms()).
QString formatDMS(double lat, double lng) {
	COORDINATE c;
	c.latitude = lat;
	c.longitude = lng;
	return QString::fromStdString(c.coordinate_decimal_to_dms(COORDINATE::LATITUDE) + " "
		+ c.coordinate_decimal_to_dms(COORDINATE::LONGITUDE));
}

}

DataTablePanel::DataTablePanel(QWidget* parent) : QWidget(parent) {
	table_ = new QTableWidget(0, 2, this);
	// The filter lives in the header cell itself (a dropdown-style glyph;
	// clicking anywhere on the "Field" header opens the checkbox dialog),
	// mirroring how Excel puts column filters in the header instead of a
	// toolbar.
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
	// instead, and let Value stretch into whatever's left. At the default
	// widths Field gets the larger share, since most values here are short
	// numbers while several field labels need two wrapped lines.
	table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Interactive);
	table_->setColumnWidth(0, AppSettings::instance().dataTableFieldColumnWidth());
	table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
	table_->horizontalHeader()->setStretchLastSection(true);
	// Value (column 1) is Stretch-mode, so it also fires sectionResized when
	// the panel width changes -- only persist column 0's width.
	connect(table_->horizontalHeader(), &QHeaderView::sectionResized, this, [this](int column, int, int newSize) {
		if (column == 0)
			AppSettings::instance().setDataTableFieldColumnWidth(newSize);
	});
	table_->setStyleSheet(QStringLiteral("QTableWidget { font-size: 9pt; }"));
	// Value cells (column 1) hold read-only display text with no built-in
	// way to select/copy it (NoEditTriggers, NoSelection) -- a right-click
	// Copy entry is the only way to get a value (e.g. the GPS Position row
	// below) out to the clipboard.
	table_->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(table_, &QTableWidget::customContextMenuRequested, this, [this](const QPoint& pos) {
		QTableWidgetItem* item = table_->itemAt(pos);
		if (!item || item->column() != 1 || item->text().isEmpty())
			return;
		QMenu menu(table_);
		QAction* copyAction = menu.addAction(QStringLiteral("Copy"));
		connect(copyAction, &QAction::triggered, this, [item]() {
			QGuiApplication::clipboard()->setText(item->text());
		});
		menu.exec(table_->viewport()->mapToGlobal(pos));
	});
	// Long values (e.g. full ISO timestamps) would be clipped at a fixed row
	// height with no wrap -- wrap them instead and let each row grow to fit,
	// with the full value always available via tooltip regardless.
	table_->setWordWrap(true);
	table_->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(table_);

	setEngineRows(0);
	showEmpty();
}

void DataTablePanel::setEngineRows(int engines) {
	if (engines == engineRows_ && !rowLabels_.isEmpty())
		return;
	engineRows_ = engines;
	rowLabels_ = buildFieldRowLabels(engines);
	const int oldRows = table_->rowCount();
	table_->setRowCount(rowLabels_.size());
	for (int row = oldRows; row < rowLabels_.size(); ++row) {
		auto* label = new QTableWidgetItem();
		label->setFlags(label->flags() & ~Qt::ItemIsEditable);
		table_->setItem(row, 0, label);
		auto* value = new QTableWidgetItem();
		value->setFlags(value->flags() & ~Qt::ItemIsEditable);
		table_->setItem(row, 1, value);
	}
	for (int row = 0; row < rowLabels_.size(); ++row)
		table_->item(row, 0)->setText(rowLabels_[row]);
	applyHiddenFields();
}

void DataTablePanel::setDataset(const TripDataset* dataset) {
	dataset_ = dataset;
	setEngineRows(datasetEngineCount(dataset));
	if (dataset_ && !dataset_->points.empty()) {
		Logger::logf(Logger::Trace, "DataTbl", "Dataset selected: %zu point(s); showing last sample", dataset_->points.size());
		showPoint(dataset_->points.back());
	} else {
		Logger::log(Logger::Trace, "DataTbl", QStringLiteral("Dataset cleared or empty; showing blank table"));
		showEmpty();
	}
}

void DataTablePanel::setCursorIndex(int index) {
	if (dataset_ && index >= 0 && index < (int)dataset_->points.size())
		showPoint(dataset_->points[index]);
	else if (dataset_ && !dataset_->points.empty())
		// No such point (e.g. -1): the trip's last point, as with no cursor,
		// instead of leaving whatever was last shown stuck on screen.
		showPoint(dataset_->points.back());
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

	if (dialog.exec() != QDialog::Accepted) {
		Logger::log(Logger::Trace, "DataTbl", QStringLiteral("Visible Fields dialog cancelled; field visibility unchanged"));
		return;
	}

	// Hidden fields this trip has no row for (an engine it lacks) stay hidden.
	QStringList newHidden;
	for (const QString& field : hidden)
		if (!rowLabels_.contains(field))
			newHidden.append(field);
	for (int row = 0; row < rowLabels_.size(); ++row) {
		if (!boxes[row]->isChecked())
			newHidden.append(rowLabels_[row]);
	}
	Logger::logf(Logger::Trace, "DataTbl", "Visible Fields dialog accepted: %d field(s) now hidden", (int)newHidden.size());
	AppSettings::instance().setDataTableHiddenFields(newHidden);
	applyHiddenFields();
}

void DataTablePanel::applyHiddenFields() {
	QStringList hidden = AppSettings::instance().dataTableHiddenFields();
	for (int row = 0; row < rowLabels_.size(); ++row)
		table_->setRowHidden(row, hidden.contains(rowLabels_[row]));
}

void DataTablePanel::showPoint(const TripSamplePoint& point) {
	// ResizeToContents triggers a word-wrap text layout pass per setText call,
	// one for each of the 200+ rows. Switch to Fixed while updating so Qt
	// defers all size calculations, then do one batch pass.
	QHeaderView* vh = table_->verticalHeader();
	vh->setSectionResizeMode(QHeaderView::Fixed);

	setValue(0, point.zuluTime);
	setValue(1, point.localTime);
	{
		static const int gpsLatIdx = numFieldIndex(QLatin1String("gps_position_lat"));
		static const int gpsLonIdx = numFieldIndex(QLatin1String("gps_position_lon"));
		const double lat = rawNum(point, gpsLatIdx), lon = rawNum(point, gpsLonIdx);
		setValue(2, std::isnan(lat) || std::isnan(lon) ? QString() : formatDMS(lat, lon));

		// The rows were built from the same macros (buildFieldRowLabels()), so
		// row stays within the table.
		int ni = 0, row = 3;
#define TRIP_NUM_DISP(dbColumn, memberExpr, sqlType) \
		if (!isGpsPositionColumn(QLatin1String(#dbColumn))) { \
			const double v = rawNum(point, ni); \
			setValue(row++, !std::isnan(v) ? QString::number(v, 'g', 6) : QString()); \
		} \
		++ni;
		TRIP_DATA_NUM_FIELDS(TRIP_NUM_DISP)
#undef TRIP_NUM_DISP
#define TRIP_BOOL_DISP(name, group, bit) \
		setValue(row++, TripBoolBit{ group, bit }.isSet(point.boolGroups) ? QStringLiteral("Yes") : QStringLiteral("No"));
		TRIP_DATA_BOOL_FIELDS(TRIP_BOOL_DISP)
#undef TRIP_BOOL_DISP
		// An engine value not recorded (NaN: an engine past this sample's
		// rows, or a value an older build didn't record) is left blank.
#define TRIP_ENGINE_DISP(name, sqlType, onOff) \
		for (int engine = 1; engine <= engineRows_; ++engine) { \
			const double v = point.engineValue(engine, TRIP_ENGINE_##name); \
			setValue(row++, std::isnan(v) ? QString() \
				: onOff ? (v != 0 ? QStringLiteral("Yes") : QStringLiteral("No")) \
				: QString::number(v, 'g', 6)); \
		}
		TRIP_ENGINE_FIELDS(TRIP_ENGINE_DISP)
#undef TRIP_ENGINE_DISP
	}

	vh->setSectionResizeMode(QHeaderView::ResizeToContents);
	table_->resizeRowsToContents();
}

void DataTablePanel::showEmpty() {
	QHeaderView* vh = table_->verticalHeader();
	vh->setSectionResizeMode(QHeaderView::Fixed);
	for (int row = 0; row < rowLabels_.size(); ++row)
		setValue(row, QString());
	vh->setSectionResizeMode(QHeaderView::ResizeToContents);
	table_->resizeRowsToContents();
}

void DataTablePanel::setValue(int row, const QString& text) {
	QTableWidgetItem* item = table_->item(row, 1);
	item->setText(text);
	item->setToolTip(text);
}
