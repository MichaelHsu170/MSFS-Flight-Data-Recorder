#include "trip_history_panel.h"
#include "recorder_bridge.h"
#include "db_history.h"
#include "db_groups.h"
#include "manage_groups_dialog.h"
#include "db.h"
#include "app_settings.h"

#include <memory>

#include <QTableView>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QMenu>
#include <QProxyStyle>
#include <QStyleOption>
#include <QPainter>
#include <QMessageBox>
#include <QMap>
#include <QMetaEnum>
#include <QSignalBlocker>
#include <QtConcurrent/QtConcurrent>
#include <QBrush>
#include <QColor>
#include "logger.h"
#include <algorithm>

#include "sqlite3.h"
#include <QEvent>
#include <QMouseEvent>
#include <QStyledItemDelegate>
#include <QPainter>

namespace {

// Strips the per-cell hover state before Qt's style paints, so the row-level
// BackgroundRole color shows uniformly across all cells in the hovered row.
class HoverStripDelegate : public QStyledItemDelegate {
public:
    using QStyledItemDelegate::QStyledItemDelegate;
    void paint(QPainter* p, const QStyleOptionViewItem& opt, const QModelIndex& idx) const override {
        QStyleOptionViewItem o = opt;
        o.state &= ~QStyle::State_MouseOver;
        QStyledItemDelegate::paint(p, o, idx);
    }
};

}

TripHistoryModel::TripHistoryModel(QObject* parent) : QAbstractTableModel(parent) {}

void TripHistoryModel::setTrips(std::vector<TripSummary> trips) {
	beginResetModel();
	allTrips_ = std::move(trips);
	hoveredRow_ = -1;
	applyFilter();
	endResetModel();
}

void TripHistoryModel::setGroupFilter(int groupId) {
	if (groupId == groupFilter_)
		return;
	beginResetModel();
	groupFilter_ = groupId;
	hoveredRow_ = -1;
	applyFilter();
	endResetModel();
}

void TripHistoryModel::applyFilter() {
	if (groupFilter_ == -1) {
		trips_ = allTrips_;
		return;
	}
	trips_.clear();
	trips_.reserve(allTrips_.size());
	for (const TripSummary& trip : allTrips_) {
		if (trip.groupId == groupFilter_)
			trips_.push_back(trip);
	}
}

void TripHistoryModel::setHoveredRow(int row) {
	if (row == hoveredRow_)
		return;
	int old = hoveredRow_;
	hoveredRow_ = row;
	auto notify = [&](int r) {
		if (r >= 0 && r < (int)trips_.size())
			emit dataChanged(index(r, 0), index(r, ColumnCount - 1), {Qt::BackgroundRole});
	};
	notify(old);
	notify(hoveredRow_);
}

const TripSummary* TripHistoryModel::tripAt(int row) const {
	if (row < 0 || row >= (int)trips_.size())
		return nullptr;
	return &trips_[row];
}

int TripHistoryModel::rowCount(const QModelIndex& parent) const {
	return parent.isValid() ? 0 : (int)trips_.size();
}

int TripHistoryModel::columnCount(const QModelIndex& parent) const {
	return parent.isValid() ? 0 : ColumnCount;
}

QVariant TripHistoryModel::data(const QModelIndex& index, int role) const {
	if (!index.isValid() || index.row() >= (int)trips_.size())
		return QVariant();
	const TripSummary& trip = trips_[index.row()];

	if (role == Qt::ToolTipRole)
		return data(index, Qt::DisplayRole);

	if (role == Qt::DisplayRole) {
		switch (index.column()) {
		case TitleColumn: return trip.title;
		case FlightColumn: return trip.atcAirline + " " + trip.atcFlightNumber;
		case GroupColumn: return trip.groupId == 0 ? QStringLiteral("-") : trip.groupName;
		case DepartureRegionColumn: return trip.departureRegion;
		case DepartureColumn: return trip.departureName.isEmpty() ? trip.departureIcao : trip.departureIcao + " [" + trip.departureName + "]";
		case DepartureRwyColumn: return trip.departureRwy;
		case DestinationRegionColumn: return trip.destinationRegion;
		case DestinationColumn: return trip.destinationName.isEmpty() ? trip.destinationIcao : trip.destinationIcao + " [" + trip.destinationName + "]";
		case DestinationRwyColumn: return trip.destinationRwy;
		case DepartureTimeColumn: return trip.departureZuluTime;
		case DestinationTimeColumn: return trip.destinationZuluTime.isEmpty() ? QStringLiteral("—") : trip.destinationZuluTime;
		}
	} else if (role == Qt::BackgroundRole) {
		switch (trip.status) {
		case TripStatus::Live: return QBrush(QColor(200, 255, 200));
		case TripStatus::Open: return QBrush(QColor(255, 235, 180));
		case TripStatus::Completed:
			if (index.row() == hoveredRow_)
				return QBrush(QColor(220, 230, 245));
			break;
		}
	}
	return QVariant();
}

Qt::ItemFlags TripHistoryModel::flags(const QModelIndex& index) const {
	if (!index.isValid() || index.row() >= (int)trips_.size())
		return Qt::NoItemFlags;
	if (trips_[index.row()].status == TripStatus::Live)
		return Qt::ItemIsEnabled;
	return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

QVariant TripHistoryModel::headerData(int section, Qt::Orientation orientation, int role) const {
	if (orientation != Qt::Horizontal || role != Qt::DisplayRole)
		return QVariant();
	switch (section) {
	case TitleColumn: return QStringLiteral("Aircraft");
	case FlightColumn: return QStringLiteral("Flight");
	case GroupColumn: return QStringLiteral("Group");
	case DepartureRegionColumn: return QStringLiteral("Region");
	case DepartureColumn: return QStringLiteral("From");
	case DepartureRwyColumn: return QStringLiteral("Dep Rwy");
	case DestinationRegionColumn: return QStringLiteral("Region");
	case DestinationColumn: return QStringLiteral("To");
	case DestinationRwyColumn: return QStringLiteral("Dest Rwy");
	case DepartureTimeColumn: return QStringLiteral("Departed (Z)");
	case DestinationTimeColumn: return QStringLiteral("Arrived (Z)");
	}
	return QVariant();
}

TripHistoryPanel::TripHistoryPanel(RecorderBridge& bridge, QWidget* parent)
	: QWidget(parent)
	, bridge_(bridge)
{
	model_ = new TripHistoryModel(this);
	table_ = new QTableView(this);
	table_->setModel(model_);
	table_->setSelectionBehavior(QAbstractItemView::SelectRows);
	table_->setSelectionMode(QAbstractItemView::SingleSelection);
	table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
	// Airport/runway codes are always short (4-5 letters) -- give those
	// columns a tight fixed width instead of splitting space evenly with
	// everything else (ResizeToContents was tried here first, but Qt sized
	// it wildly wider than the actual cell text for reasons that didn't
	// trace back to the data), and let the two timestamp columns (by far the
	// widest values) share whatever's left.
	auto* header = table_->horizontalHeader();
	header->setStretchLastSection(false);
	header->setSectionResizeMode(TripHistoryModel::GroupColumn, QHeaderView::Interactive);
	table_->setColumnWidth(TripHistoryModel::GroupColumn, 64);
	header->setSectionResizeMode(TripHistoryModel::DepartureRegionColumn, QHeaderView::Interactive);
	header->setSectionResizeMode(TripHistoryModel::DepartureColumn, QHeaderView::Stretch);
	header->setSectionResizeMode(TripHistoryModel::DepartureRwyColumn, QHeaderView::Interactive);
	header->setSectionResizeMode(TripHistoryModel::DestinationRegionColumn, QHeaderView::Interactive);
	header->setSectionResizeMode(TripHistoryModel::DestinationColumn, QHeaderView::Stretch);
	header->setSectionResizeMode(TripHistoryModel::DestinationRwyColumn, QHeaderView::Interactive);
	table_->setColumnWidth(TripHistoryModel::DepartureRegionColumn, 55);
	table_->setColumnWidth(TripHistoryModel::DepartureRwyColumn, 60);
	table_->setColumnWidth(TripHistoryModel::DestinationRegionColumn, 55);
	table_->setColumnWidth(TripHistoryModel::DestinationRwyColumn, 60);
	header->setSectionResizeMode(TripHistoryModel::DepartureTimeColumn, QHeaderView::Interactive);
	header->setSectionResizeMode(TripHistoryModel::DestinationTimeColumn, QHeaderView::Interactive);
	table_->setColumnWidth(TripHistoryModel::DepartureTimeColumn, 175);
	table_->setColumnWidth(TripHistoryModel::DestinationTimeColumn, 175);

	// Restore any widths the user dragged in a previous session, overriding
	// the coded defaults set above. Must happen before the sectionResized
	// connect below so this initial resize doesn't immediately re-save itself.
	// Keyed by enum member name (via QMetaEnum) rather than ordinal position,
	// so a saved width from an older settings.ini can't get silently
	// misassigned to the wrong column if TripHistoryModel::Column ever gains,
	// loses, or reorders a member -- unrecognized/missing keys are just
	// skipped, leaving that column at its coded default.
	{
		const QMetaEnum columnEnum = QMetaEnum::fromType<TripHistoryModel::Column>();
		const QMap<QString, int> savedWidths = AppSettings::instance().tripHistoryColumnWidths();
		for (auto it = savedWidths.constBegin(); it != savedWidths.constEnd(); ++it) {
			bool ok = false;
			const int col = columnEnum.keyToValue(it.key().toUtf8().constData(), &ok);
			if (ok && col >= 0 && col < TripHistoryModel::ColumnCount && it.value() > 0)
				table_->setColumnWidth(col, it.value());
		}
	}
	// Stretch-mode columns (From/To) resize themselves whenever the panel's
	// width changes, which also fires sectionResized -- filter to only save
	// on Interactive columns so those don't trigger a write on every splitter
	// drag or window resize.
	connect(header, &QHeaderView::sectionResized, this, [this, header](int column, int, int) {
		if (header->sectionResizeMode(column) != QHeaderView::Interactive)
			return;
		const QMetaEnum columnEnum = QMetaEnum::fromType<TripHistoryModel::Column>();
		QMap<QString, int> widths;
		for (int col = 0; col < TripHistoryModel::ColumnCount; ++col) {
			if (header->sectionResizeMode(col) != QHeaderView::Interactive)
				continue;
			widths.insert(QString::fromUtf8(columnEnum.valueToKey(col)), table_->columnWidth(col));
		}
		AppSettings::instance().setTripHistoryColumnWidths(widths);
	});

	table_->setItemDelegate(new HoverStripDelegate(this));
	table_->verticalHeader()->setVisible(false);
	// Default selection highlight clashes with the status BackgroundRole
	// colors (Live/Open rows) and looks washed out -- a solid, high-contrast
	// highlight reads better and still lets unselected rows show their
	// status tint.
	table_->setStyleSheet(
		"QTableView::item:selected { background-color: #2f6fd6; color: white; }"
		"QTableView { outline: none; font-size: 9pt; }"
	);
	table_->verticalHeader()->setDefaultSectionSize(20);

	// Indeterminate (range 0,0) so Qt animates a generic "busy" bar -- shown
	// only while a trip's dataset is loading in the background. A bright,
	// fixed-color chunk is used because the default style's progress chunk
	// can be nearly invisible at this thin a height on some platform themes.
	loadingBar_ = new QProgressBar(this);
	loadingBar_->setRange(0, 0);
	loadingBar_->setFixedHeight(6);
	loadingBar_->setTextVisible(false);
	loadingBar_->setStyleSheet(
		"QProgressBar { background-color: #e0e0e0; border: none; }"
		"QProgressBar::chunk { background-color: #2f6fd6; }"
	);
	loadingBar_->setVisible(false);

	groupFilterCombo_ = new QComboBox(this);
	manageGroupsButton_ = new QPushButton(QStringLiteral("Manage Groups…"), this);
	auto* filterRow = new QHBoxLayout();
	filterRow->addWidget(new QLabel(QStringLiteral("Group:"), this));
	filterRow->addWidget(groupFilterCombo_);
	filterRow->addStretch();
	filterRow->addWidget(manageGroupsButton_);

	auto* layout = new QVBoxLayout(this);
	// Default QVBoxLayout margins add visible padding on every side -- on the
	// right edge that compounds with the splitter handle and Live Status's
	// own left margin into a wide gap between the two panels.
	layout->setContentsMargins(4, 4, 0, 4);
	layout->addLayout(filterRow);
	layout->addWidget(table_);
	layout->addWidget(loadingBar_);

	pointsWatcher_ = new QFutureWatcher<std::shared_ptr<TripDataset>>(this);
	touchdownsWatcher_ = new QFutureWatcher<std::vector<TouchdownPoint>>(this);
	eventsWatcher_ = new QFutureWatcher<std::vector<TripEvent>>(this);
	connect(pointsWatcher_, &QFutureWatcher<std::shared_ptr<TripDataset>>::finished, this, &TripHistoryPanel::tryFinishLoad);
	connect(touchdownsWatcher_, &QFutureWatcher<std::vector<TouchdownPoint>>::finished, this, &TripHistoryPanel::tryFinishLoad);
	connect(eventsWatcher_, &QFutureWatcher<std::vector<TripEvent>>::finished, this, &TripHistoryPanel::tryFinishLoad);

	// A single click selects a trip and loads its data -- the previous
	// double-click ("activated") requirement felt sluggish for browsing.
	connect(table_, &QTableView::clicked, this, &TripHistoryPanel::onRowActivated);
	connect(&bridge_, &RecorderBridge::recordingStateChanged, this, [this](int) {
		refreshTrips();
		// If no trip is selected the overview map is visible. Re-emit tripDeselected
		// so a newly-started/ended live trip shows up on it immediately -- same
		// reasoning as the tripEnded handler below. Skip while a trip is actively
		// loading: selectedTripId_ still holds the pre-load value (possibly -1)
		// until tryFinishLoad() completes, so without the loading_ guard this would
		// briefly flash the overview map over the trip the user just clicked.
		if (!loading_ && selectedTripId_ == -1)
			emit tripDeselected(model_->trips());
	});
	connect(&bridge_, &RecorderBridge::tripEnded, this, [this](int) {
		refreshTrips();
		// If no trip is selected the overview map is visible. Re-emit tripDeselected
		// so it picks up the new departure→destination segment immediately.
		if (!loading_ && selectedTripId_ == -1)
			emit tripDeselected(model_->trips());
	});
	connect(&bridge_, &RecorderBridge::tripUpdated, this, [this](int) {
		refreshTrips();
		// A live trip's departure/destination can resolve mid-flight well after
		// recordingStateChanged fired -- without this, the overview map would
		// keep showing the trip with no segment until it ends (tripEnded).
		if (!loading_ && selectedTripId_ == -1)
			emit tripDeselected(model_->trips());
	});

	table_->viewport()->setMouseTracking(true);
	table_->viewport()->installEventFilter(this);

	table_->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(table_, &QTableView::customContextMenuRequested, this, &TripHistoryPanel::onTableContextMenu);

	connect(manageGroupsButton_, &QPushButton::clicked, this, &TripHistoryPanel::openManageGroupsDialog);
	connect(groupFilterCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
		const int groupId = groupFilterCombo_->itemData(index).toInt();
		Logger::logf(Logger::Trace, "DB", "group filter changed to groupId=%d; deselecting current trip", groupId);
		model_->setGroupFilter(groupId);
		// Switching the filter shows a different set of trips in the table --
		// a previously selected trip may not even be in it anymore, so drop
		// the selection and show the overview map for the newly filtered set
		// instead, keeping the map aligned with the table.
		selectedTripId_ = -1;
		table_->clearSelection();
		emit tripDeselected(model_->trips());
	});

	reloadGroupFilterCombo();
	refreshTrips();
}

TripHistoryPanel::~TripHistoryPanel() {
	if (historySql_ != nullptr)
		sqlite3_close(historySql_);
}

sqlite3* TripHistoryPanel::ensureHistoryConnection() {
	if (historySql_ == nullptr) {
		historySql_ = connect_db_readonly();
		if (historySql_ == nullptr)
			Logger::log(Logger::Warning, "DB", QStringLiteral("Trip history: failed to open a read-only database connection"));
	}
	return historySql_;
}

void TripHistoryPanel::reloadGroupFilterCombo() {
	int previousData = groupFilterCombo_->count() > 0 ? groupFilterCombo_->currentData().toInt() : -1;

	sqlite3* sql = ensureHistoryConnection();
	std::vector<TripGroup> groups = sql ? queryAllGroups(sql) : std::vector<TripGroup>();

	QSignalBlocker blocker(groupFilterCombo_);
	groupFilterCombo_->clear();
	groupFilterCombo_->addItem(QStringLiteral("All Trips"), -1);
	groupFilterCombo_->addItem(QStringLiteral("Ungrouped"), 0);
	for (const TripGroup& group : groups)
		groupFilterCombo_->addItem(group.name, group.id);

	int idx = groupFilterCombo_->findData(previousData);
	groupFilterCombo_->setCurrentIndex(idx >= 0 ? idx : 0);
	int newData = groupFilterCombo_->currentData().toInt();
	model_->setGroupFilter(newData);
	// The combo's currentIndexChanged signal is suppressed above, so if the
	// filter actually changed (e.g. the previously-selected group was
	// deleted, falling back to "All Trips"), this needs to do the same
	// deselect-and-realign-the-map the combo's own change handler does. Only
	// do this when the filter really changed, since groupsChanged fires on
	// every add/rename/delete in the Manage Groups dialog and most of those
	// don't affect the current filter.
	if (newData != previousData) {
		Logger::logf(Logger::Trace, "DB", "reloadGroupFilterCombo: active filter changed externally (%d -> %d, e.g. selected group was deleted); deselecting current trip", previousData, newData);
		selectedTripId_ = -1;
		table_->clearSelection();
		emit tripDeselected(model_->trips());
	}
}

void TripHistoryPanel::setTripGroupFromUi(int tripId, int groupId) {
	sqlite3* sql = connect_db_readwrite();
	if (!sql) {
		QMessageBox::critical(this, QStringLiteral("Error"),
			QStringLiteral("Could not open the database for writing."));
		return;
	}
	bool ok = setTripGroup(sql, tripId, groupId);
	sqlite3_close(sql);
	if (!ok) {
		QMessageBox::critical(this, QStringLiteral("Error"),
			QStringLiteral("Failed to update the trip's group."));
		return;
	}
	// Changing the selected trip's group can move it out of the active group
	// filter (or just no longer matches what the user picked it for) -- drop
	// the selection rather than leave it selected under a changed group, same
	// as deleting a trip does.
	const bool wasSelected = (tripId == selectedTripId_);
	if (wasSelected)
		selectedTripId_ = -1;
	refreshTrips();
	if (wasSelected)
		table_->clearSelection();
	// Under an active group filter, this reassignment may have added or
	// removed the trip from the filtered set. If no trip is selected the
	// overview map is showing that set, so nudge it to match.
	if (selectedTripId_ == -1)
		emit tripDeselected(model_->trips());
}

void TripHistoryPanel::openManageGroupsDialog() {
	ManageGroupsDialog dialog(this);
	// Refresh live as each add/rename/delete commits, rather than only after
	// the dialog closes -- a deleted group can't be undone, so the table
	// behind the dialog shouldn't keep showing a trip's now-deleted group
	// until the user closes the dialog.
	connect(&dialog, &ManageGroupsDialog::groupsChanged, this, [this]() {
		reloadGroupFilterCombo();
		refreshTrips();
	});
	dialog.exec();
	reloadGroupFilterCombo();
	refreshTrips();
}

void TripHistoryPanel::refreshTrips() {
	sqlite3* sql = ensureHistoryConnection();
	if (sql == nullptr)
		return;
	model_->setTrips(queryAllTrips(sql, bridge_.currentTripId()));

	// setTrips() resets the model, clearing any selection. While a trip is
	// mid-load, selectedTripId_ still holds the *previous* selection (it only
	// becomes pendingTripId_ once tryFinishLoad() completes) -- re-selecting it
	// here would desync the highlighted row from the trip actually being loaded
	// if a bridge signal (tripUpdated/tripEnded/recordingStateChanged) fires
	// mid-load and calls refreshTrips(). Highlight whichever trip is relevant
	// right now.
	int highlightTripId = loading_ ? pendingTripId_ : selectedTripId_;
	if (highlightTripId != -1) {
		const auto& trips = model_->trips();
		for (int i = 0; i < (int)trips.size(); ++i) {
			if (trips[i].id == highlightTripId) {
				table_->selectRow(i);
				break;
			}
		}
	}
}

void TripHistoryPanel::showInitialOverview() {
	emit tripDeselected(model_->trips());
}

void TripHistoryPanel::onRowActivated(const QModelIndex& index) {
	// Block re-entrant loads: the table itself is disabled below for the same
	// reason (belt and suspenders against a queued click slipping through).
	if (loading_) {
		Logger::log(Logger::Trace, "DB", QStringLiteral("onRowActivated: ignoring click, a load is already in progress"));
		return;
	}

	const TripSummary* trip = model_->tripAt(index.row());
	if (trip == nullptr || trip->status == TripStatus::Live) {
		Logger::log(Logger::Trace, "DB", QStringLiteral("onRowActivated: ignoring click on invalid row or Live trip (not yet loadable)"));
		return;
	}

	Logger::logf(Logger::Trace, "DB", "onRowActivated: trip %d selected; starting load", trip->id);
	loading_ = true;
	pendingTripId_ = trip->id;
	table_->setEnabled(false);
	// Also block the filter/group controls: tryFinishLoad() applies
	// pendingTripId_ unconditionally once the load completes, so allowing
	// the filter to change (or a group to be renamed/deleted) mid-load
	// would let a completing load silently override the selection the
	// user just moved away from.
	groupFilterCombo_->setEnabled(false);
	manageGroupsButton_->setEnabled(false);
	loadingBar_->setVisible(true);
	loadTimer_.start();
	Logger::logf(Logger::Profile, "DB", "--- start: trip %d ---", trip->id);

	int tripId = trip->id;
	QString aircraftTitle = trip->title;
	// The three queries (trip_data is by far the largest) each get their own
	// connect_db_readonly() connection and are launched directly from the GUI
	// thread (not nested inside a wrapping QtConcurrent::run that blocks on
	// their .result()) so they genuinely run in parallel on the thread pool
	// without risking starving it: a task that occupies a pool thread and
	// then blocks waiting for further pool threads to free up can deadlock
	// if the pool's max thread count is too small to run all of them at
	// once. Joining happens in tryFinishLoad() once all three watchers report
	// finished.
	pointsWatcher_->setFuture(QtConcurrent::run([tripId, aircraftTitle]() {
		QElapsedTimer t; t.start();
		sqlite3* sql = connect_db_readonly();
		if (!sql)
			Logger::logf(Logger::Warning, "DB", "queryTripData(trip %d): failed to open read-only connection", tripId);
		auto dataset = std::make_shared<TripDataset>(sql ? queryTripData(sql, tripId) : TripDataset());
		if (sql)
			sqlite3_close(sql);
		dataset->tripId = tripId;
		dataset->aircraftTitle = aircraftTitle;
		Logger::logf(Logger::Profile, "DB", "queryTripData: %lld ms  (%zu pts)", t.nsecsElapsed() / 1000000, dataset->points.size());
		return dataset;
	}));
	touchdownsWatcher_->setFuture(QtConcurrent::run([tripId]() {
		QElapsedTimer t; t.start();
		sqlite3* sql = connect_db_readonly();
		if (!sql)
			Logger::logf(Logger::Warning, "DB", "queryTouchdowns(trip %d): failed to open read-only connection", tripId);
		std::vector<TouchdownPoint> touchdowns = sql ? queryTouchdowns(sql, tripId) : std::vector<TouchdownPoint>();
		if (sql)
			sqlite3_close(sql);
		Logger::logf(Logger::Profile, "DB", "queryTouchdowns: %lld ms  (%zu touchdowns)", t.nsecsElapsed() / 1000000, touchdowns.size());
		return touchdowns;
	}));
	eventsWatcher_->setFuture(QtConcurrent::run([tripId]() {
		QElapsedTimer t; t.start();
		sqlite3* sql = connect_db_readonly();
		if (!sql)
			Logger::logf(Logger::Warning, "DB", "queryEvents(trip %d): failed to open read-only connection", tripId);
		std::vector<TripEvent> events = sql ? queryEvents(sql, tripId) : std::vector<TripEvent>();
		if (sql)
			sqlite3_close(sql);
		Logger::logf(Logger::Profile, "DB", "queryEvents: %lld ms  (%zu events)", t.nsecsElapsed() / 1000000, events.size());
		return events;
	}));
}

void TripHistoryPanel::tryFinishLoad() {
	if (!pointsWatcher_->isFinished() || !touchdownsWatcher_->isFinished() || !eventsWatcher_->isFinished())
		return;

	auto dataset = pointsWatcher_->result();
	dataset->touchdowns = touchdownsWatcher_->result();
	dataset->events = eventsWatcher_->result();
	Logger::logf(Logger::Profile, "DB", "all joined: %lld ms wall time from click", loadTimer_.nsecsElapsed() / 1000000);

	// trip_events only stores a timestamp, not a position -- resolve each
	// event to the nearest sample by zuluTime (lexicographically comparable
	// since every row shares the same "%04d-%02d-%02dT..." format and
	// timezone) so it can be placed on the map.
	for (TripEvent& event : dataset->events) {
		auto it = std::lower_bound(dataset->points.begin(), dataset->points.end(), event.zuluTime,
			[](const TripSamplePoint& point, const QString& time) { return point.zuluTime < time; });
		if (it == dataset->points.end() && !dataset->points.empty())
			--it;
		if (it != dataset->points.end()) {
			event.latitude = it->latitude;
			event.longitude = it->longitude;
			event.sampleIndex = (int)(it - dataset->points.begin());
		}
	}

	selectedTripId_ = pendingTripId_;
	// Keep loading_ = true and table disabled until TrajectoryView signals that
	// both the chart series and map trajectory workers have finished rendering.
	// setLoadingFinished() (connected via MainWindow) clears that state.
	emit tripDatasetReady(dataset);
}

void TripHistoryPanel::setLoadingFinished() {
	if (!loading_)
		return;
	loading_ = false;
	table_->setEnabled(true);
	groupFilterCombo_->setEnabled(true);
	manageGroupsButton_->setEnabled(true);
	loadingBar_->setVisible(false);
}

bool TripHistoryPanel::eventFilter(QObject* obj, QEvent* event) {
	if (obj == table_->viewport()) {
		if (event->type() == QEvent::MouseMove) {
			QPoint pos = static_cast<QMouseEvent*>(event)->pos();
			model_->setHoveredRow(table_->indexAt(pos).row());
		} else if (event->type() == QEvent::Leave) {
			model_->setHoveredRow(-1);
		} else if (event->type() == QEvent::MouseButtonPress) {
			QPoint pos = static_cast<QMouseEvent*>(event)->pos();
			const TripSummary* trip = model_->tripAt(table_->indexAt(pos).row());
			if (trip && (trip->status == TripStatus::Live || trip->id == selectedTripId_))
				return true; // consume: prevent selection model from processing this click
		}
	}
	return QWidget::eventFilter(obj, event);
}

void TripHistoryPanel::onTableContextMenu(const QPoint& pos) {
	if (loading_) {
		Logger::log(Logger::Trace, "DB", QStringLiteral("onTableContextMenu: suppressed, a load is already in progress"));
		return;
	}

	// Colours the "Delete Trip" item red by intercepting its CE_MenuItem paint
	// call and substituting a red Text/ButtonText palette role. Uses QProxyStyle
	// so the item keeps its normal geometry, selection highlight, and alignment.
	struct RedDeleteStyle : QProxyStyle {
		using QProxyStyle::QProxyStyle;
		void drawControl(ControlElement ce, const QStyleOption* opt,
		                 QPainter* p, const QWidget* w) const override {
			if (ce == CE_MenuItem) {
				if (const auto* mi = qstyleoption_cast<const QStyleOptionMenuItem*>(opt)) {
					if (mi->text.section('\t', 0, 0) == QLatin1String("Delete Trip")) {
						QStyleOptionMenuItem m = *mi;
						const QColor red(0xd0, 0x30, 0x30);
						m.palette.setColor(QPalette::Text, red);
						m.palette.setColor(QPalette::ButtonText, red);
						QProxyStyle::drawControl(ce, &m, p, w);
						return;
					}
				}
			}
			QProxyStyle::drawControl(ce, opt, p, w);
		}
	};

	// Declare before menu: local variables are destroyed in reverse order, so
	// menu is destroyed first and never holds a dangling style pointer.
	// Pass the style name string (not a pointer) so QProxyStyle creates its own
	// internal base style — passing a pointer would transfer ownership and delete
	// the app-wide QStyle when redStyle goes out of scope.
	RedDeleteStyle redStyle(this->style()->name());
	QMenu menu(this);
	menu.setStyle(&redStyle);

	// Identify the right-clicked row up front — used by both action groups below.
	QModelIndex index = table_->indexAt(pos);
	const TripSummary* rightClickedTrip = index.isValid() ? model_->tripAt(index.row()) : nullptr;

	// Deselect / Reset Zoom — only when right-clicking the currently selected row.
	QAction* deselectAction = nullptr;
	QAction* resetZoomAction = nullptr;
	if (selectedTripId_ != -1 && rightClickedTrip && rightClickedTrip->id == selectedTripId_) {
		deselectAction  = menu.addAction(QStringLiteral("Deselect"));
		resetZoomAction = menu.addAction(QStringLiteral("Reset Zoom"));
	}

	// Row-specific "Delete Trip" at the bottom — only for the right-clicked, non-Live row.
	QAction* deleteAction = nullptr;
	int deleteId = -1;
	QString deleteName;
	QString deleteFrom;
	QString deleteTo;
	if (rightClickedTrip && rightClickedTrip->status != TripStatus::Live) {
		if (!menu.isEmpty())
			menu.addSeparator();
		deleteAction = menu.addAction(QStringLiteral("Delete Trip"));
		deleteId = rightClickedTrip->id;
		deleteName = rightClickedTrip->title.isEmpty()
			? QStringLiteral("Trip #%1").arg(rightClickedTrip->id)
			: rightClickedTrip->title;
		auto airportLabel = [](const QString& icao, const QString& name) {
			return icao.isEmpty() ? QStringLiteral("—")
			     : name.isEmpty() ? icao
			     : QStringLiteral("%1 (%2)").arg(name, icao);
		};
		deleteFrom = airportLabel(rightClickedTrip->departureIcao,   rightClickedTrip->departureName);
		deleteTo   = airportLabel(rightClickedTrip->destinationIcao, rightClickedTrip->destinationName);
	}

	// "Set Group" submenu, listing every group as a checkable action (checked
	// = the right-clicked trip's current group) plus "Ungrouped" and a
	// shortcut into the management dialog.
	QAction* ungroupedAction = nullptr;
	QAction* manageGroupsAction = nullptr;
	QMap<QAction*, int> groupActionIds;
	// Captured up front (not read from rightClickedTrip after menu.exec()) because
	// menu.exec() runs a nested event loop; a tripUpdated/recordingStateChanged
	// signal delivered while the menu is open can reallocate the model's backing
	// vector and dangle rightClickedTrip.
	const int rightClickedTripId = rightClickedTrip ? rightClickedTrip->id : -1;
	if (rightClickedTrip) {
		if (!menu.isEmpty())
			menu.addSeparator();
		QMenu* groupMenu = menu.addMenu(QStringLiteral("Set Group"));
		ungroupedAction = groupMenu->addAction(QStringLiteral("Ungrouped"));
		ungroupedAction->setCheckable(true);
		ungroupedAction->setChecked(rightClickedTrip->groupId == 0);

		sqlite3* groupsSql = ensureHistoryConnection();
		std::vector<TripGroup> groups = groupsSql ? queryAllGroups(groupsSql) : std::vector<TripGroup>();
		if (!groups.empty())
			groupMenu->addSeparator();
		for (const TripGroup& group : groups) {
			QAction* action = groupMenu->addAction(group.name);
			action->setCheckable(true);
			action->setChecked(rightClickedTrip->groupId == group.id);
			groupActionIds.insert(action, group.id);
		}
		groupMenu->addSeparator();
		manageGroupsAction = groupMenu->addAction(QStringLiteral("Manage Groups…"));
	}

	if (menu.isEmpty())
		return;

	QAction* chosen = menu.exec(table_->viewport()->mapToGlobal(pos));
	if (!chosen)
		return;

	if (chosen == ungroupedAction) {
		setTripGroupFromUi(rightClickedTripId, 0);
	} else if (chosen == manageGroupsAction) {
		openManageGroupsDialog();
	} else if (groupActionIds.contains(chosen)) {
		setTripGroupFromUi(rightClickedTripId, groupActionIds.value(chosen));
	} else if (chosen == deleteAction) {
		// Re-check right before deleting (not just via the TripStatus::Live
		// filter that built the menu): a trip that just stopped recording can
		// still have samples draining onto the DB-write thread even though it
		// no longer looks Live (id_trip is reset synchronously, before that
		// flush completes -- see flushing_trip_id in types.h). Deleting now
		// would race the worker's still-pending trip_data inserts and orphan
		// rows with no parent trip.
		if (deleteId == bridge_.flushingTripId()) {
			Logger::logf(Logger::Trace, "DB", "delete trip %d blocked: trip data is still flushing", deleteId);
			QMessageBox::information(this, QStringLiteral("Trip Still Saving"),
				QStringLiteral("This trip's data is still being saved. Please wait a moment and try again."));
			return;
		}
		QMessageBox confirm(this);
		confirm.setWindowTitle(QStringLiteral("Delete Trip"));
		confirm.setText(QStringLiteral("Delete the trip?"));
		confirm.setInformativeText(
			QStringLiteral("Aircraft: %1\nFrom: %2\nTo: %3\n\nAll flight data for this trip will be permanently deleted.")
				.arg(deleteName, deleteFrom, deleteTo));
		confirm.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
		confirm.setDefaultButton(QMessageBox::Cancel);
		confirm.setIcon(QMessageBox::Warning);
		if (confirm.exec() != QMessageBox::Yes)
			return;
		// confirm.exec() runs a nested event loop -- re-check once more right
		// before the actual delete, since the flush could still be draining
		// (or could have started, if this trip only just stopped recording)
		// while the dialog was open.
		if (deleteId == bridge_.flushingTripId()) {
			Logger::logf(Logger::Trace, "DB", "delete trip %d blocked (re-check after dialog): trip data is still flushing", deleteId);
			QMessageBox::information(this, QStringLiteral("Trip Still Saving"),
				QStringLiteral("This trip's data is still being saved. Please wait a moment and try again."));
			return;
		}
		sqlite3* sql = connect_db_readwrite();
		if (!sql) {
			QMessageBox::critical(this, QStringLiteral("Error"),
				QStringLiteral("Could not open the database for writing."));
			return;
		}
		bool ok = deleteTripData(sql, deleteId);
		sqlite3_close(sql);
		if (!ok) {
			QMessageBox::critical(this, QStringLiteral("Error"),
				QStringLiteral("Failed to delete trip data."));
			return;
		}
		const bool wasSelected = (deleteId == selectedTripId_);
		if (wasSelected)
			selectedTripId_ = -1;
		refreshTrips();
		if (wasSelected)
			table_->clearSelection();
		// If no trip is selected the overview map is showing every trip,
		// including whichever one was just deleted -- nudge it to drop that
		// trip's arc whether or not it was the selected one.
		if (selectedTripId_ == -1)
			emit tripDeselected(model_->trips());
	} else if (chosen == deselectAction) {
		table_->clearSelection();
		selectedTripId_ = -1;
		emit tripDeselected(model_->trips());
	} else if (chosen == resetZoomAction) {
		emit zoomResetRequested();
	}
}
