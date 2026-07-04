#include "trip_history_panel.h"
#include "recorder_bridge.h"
#include "db_history.h"
#include "db.h"

#include <memory>

#include <QTableView>
#include <QVBoxLayout>
#include <QHeaderView>
#include <QProgressBar>
#include <QMenu>
#include <QMessageBox>
#include <QtConcurrent/QtConcurrent>
#include <QBrush>
#include <QColor>
#include <QDebug>
#include <algorithm>

#include "sqlite3.h"

TripHistoryModel::TripHistoryModel(QObject* parent) : QAbstractTableModel(parent) {}

void TripHistoryModel::setTrips(std::vector<TripSummary> trips) {
	beginResetModel();
	trips_ = std::move(trips);
	endResetModel();
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
		case TripStatus::Completed: break;
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

	auto* layout = new QVBoxLayout(this);
	// Default QVBoxLayout margins add visible padding on every side -- on the
	// right edge that compounds with the splitter handle and Live Status's
	// own left margin into a wide gap between the two panels.
	layout->setContentsMargins(4, 4, 0, 4);
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
	connect(&bridge_, &RecorderBridge::recordingStateChanged, this, &TripHistoryPanel::refreshTrips);
	connect(&bridge_, &RecorderBridge::tripEnded, this, &TripHistoryPanel::refreshTrips);
	connect(&bridge_, &RecorderBridge::tripUpdated, this, &TripHistoryPanel::refreshTrips);

	table_->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(table_, &QTableView::customContextMenuRequested, this, &TripHistoryPanel::onTableContextMenu);

	refreshTrips();
}

TripHistoryPanel::~TripHistoryPanel() {
	if (historySql_ != nullptr)
		sqlite3_close(historySql_);
}

sqlite3* TripHistoryPanel::ensureHistoryConnection() {
	if (historySql_ == nullptr)
		historySql_ = connect_db_readonly();
	return historySql_;
}

void TripHistoryPanel::refreshTrips() {
	sqlite3* sql = ensureHistoryConnection();
	if (sql == nullptr)
		return;
	model_->setTrips(queryAllTrips(sql, bridge_.currentTripId()));
}

void TripHistoryPanel::onRowActivated(const QModelIndex& index) {
	// Block re-entrant loads: the table itself is disabled below for the same
	// reason (belt and suspenders against a queued click slipping through).
	if (loading_)
		return;

	const TripSummary* trip = model_->tripAt(index.row());
	if (trip == nullptr || trip->status == TripStatus::Live)
		return;

	loading_ = true;
	pendingTripId_ = trip->id;
	table_->setEnabled(false);
	loadingBar_->setVisible(true);
	loadTimer_.start();
	qDebug("[Gen/DB  ] --- start: trip %d ---", trip->id);

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
		auto dataset = std::make_shared<TripDataset>(sql ? queryTripData(sql, tripId) : TripDataset());
		if (sql)
			sqlite3_close(sql);
		dataset->tripId = tripId;
		dataset->aircraftTitle = aircraftTitle;
		qDebug("[Gen/DB  ] queryTripData: %lld ms  (%zu pts)", t.nsecsElapsed() / 1000000, dataset->points.size());
		return dataset;
	}));
	touchdownsWatcher_->setFuture(QtConcurrent::run([tripId]() {
		QElapsedTimer t; t.start();
		sqlite3* sql = connect_db_readonly();
		std::vector<TouchdownPoint> touchdowns = sql ? queryTouchdowns(sql, tripId) : std::vector<TouchdownPoint>();
		if (sql)
			sqlite3_close(sql);
		qDebug("[Gen/DB  ] queryTouchdowns: %lld ms  (%zu touchdowns)", t.nsecsElapsed() / 1000000, touchdowns.size());
		return touchdowns;
	}));
	eventsWatcher_->setFuture(QtConcurrent::run([tripId]() {
		QElapsedTimer t; t.start();
		sqlite3* sql = connect_db_readonly();
		std::vector<TripEvent> events = sql ? queryEvents(sql, tripId) : std::vector<TripEvent>();
		if (sql)
			sqlite3_close(sql);
		qDebug("[Gen/DB  ] queryEvents: %lld ms  (%zu events)", t.nsecsElapsed() / 1000000, events.size());
		return events;
	}));
}

void TripHistoryPanel::tryFinishLoad() {
	if (!pointsWatcher_->isFinished() || !touchdownsWatcher_->isFinished() || !eventsWatcher_->isFinished())
		return;

	auto dataset = pointsWatcher_->result();
	dataset->touchdowns = touchdownsWatcher_->result();
	dataset->events = eventsWatcher_->result();
	qDebug("[Gen/DB  ] all joined: %lld ms wall time from click", loadTimer_.nsecsElapsed() / 1000000);

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
	loadingBar_->setVisible(false);
}

void TripHistoryPanel::onTableContextMenu(const QPoint& pos) {
	if (loading_)
		return;
	QModelIndex index = table_->indexAt(pos);
	if (!index.isValid())
		return;
	const TripSummary* trip = model_->tripAt(index.row());
	if (!trip || trip->status == TripStatus::Live)
		return;

	QMenu menu(this);
	QAction* deleteAction = menu.addAction(QStringLiteral("Delete Trip"));
	if (menu.exec(table_->viewport()->mapToGlobal(pos)) != deleteAction)
		return;

	int tripId = trip->id;
	QString displayName = trip->title.isEmpty()
		? QStringLiteral("Trip #%1").arg(tripId)
		: trip->title;

	QMessageBox confirm(this);
	confirm.setWindowTitle(QStringLiteral("Delete Trip"));
	confirm.setText(QStringLiteral("Delete \"%1\"?").arg(displayName));
	confirm.setInformativeText(QStringLiteral("All flight data for this trip will be permanently deleted."));
	confirm.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
	confirm.setDefaultButton(QMessageBox::Cancel);
	confirm.setIcon(QMessageBox::Warning);
	if (confirm.exec() != QMessageBox::Yes)
		return;

	sqlite3* sql = connect_db_readwrite();
	if (!sql) {
		QMessageBox::critical(this, QStringLiteral("Error"),
			QStringLiteral("Could not open the database for writing."));
		return;
	}
	bool ok = deleteTripData(sql, tripId);
	sqlite3_close(sql);

	if (!ok) {
		QMessageBox::critical(this, QStringLiteral("Error"),
			QStringLiteral("Failed to delete trip data."));
		return;
	}
	refreshTrips();
}
