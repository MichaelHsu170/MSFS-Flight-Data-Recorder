#pragma once

#include <QWidget>
#include <QAbstractTableModel>
#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QMap>
#include <memory>
#include <utility>

#include "trip_dataset.h"

struct sqlite3;
class RecorderBridge;
class QTableView;
class QProgressBar;
class QComboBox;
class QPushButton;
class QLabel;

// Read-only table model backing TripHistoryPanel's QTableView. One row per
// (filtered) trip; status (Completed/Open/Live) is conveyed entirely through
// the row's BackgroundRole tint (see TripHistoryModel::data) rather than its
// own column.
class TripHistoryModel : public QAbstractTableModel {
	Q_OBJECT
public:
	enum Column {
		TitleColumn,
		FlightColumn,
		GroupColumn,
		DepartureRegionColumn,
		DepartureColumn,
		DepartureRwyColumn,
		DestinationRegionColumn,
		DestinationColumn,
		DestinationRwyColumn,
		DepartureTimeColumn,
		DestinationTimeColumn,
		DurationColumn,
		ColumnCount,
	};
	// Lets AppSettings key persisted column widths by enum member name
	// (via QMetaEnum) instead of ordinal position, so a future column being
	// inserted, removed, or reordered can't silently misassign saved widths
	// onto the wrong column -- see trip_history_panel.cpp's use of
	// QMetaEnum::fromType<Column>().
	Q_ENUM(Column)

	explicit TripHistoryModel(QObject* parent = nullptr);

	void setTrips(std::vector<TripSummary> trips);
	// -1 = show all trips, 0 = ungrouped only, >0 = only trips in that group id.
	void setGroupFilter(int groupId);
	void setHoveredRow(int row);
	const TripSummary* tripAt(int row) const;
	// The currently filtered/displayed set (not necessarily all trips).
	const std::vector<TripSummary>& trips() const { return trips_; }
	// Sum of the Duration column over the currently filtered/displayed set,
	// formatted the same way as a single row's cell ("-" if none of the
	// filtered trips have a completed duration to sum).
	QString totalDurationText() const;

	int rowCount(const QModelIndex& parent = QModelIndex()) const override;
	int columnCount(const QModelIndex& parent = QModelIndex()) const override;
	QVariant data(const QModelIndex& index, int role) const override;
	QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
	Qt::ItemFlags flags(const QModelIndex& index) const override;

private:
	void applyFilter();

	std::vector<TripSummary> allTrips_;
	std::vector<TripSummary> trips_;
	int groupFilter_ = -1;
	int hoveredRow_ = -1;
};

// Trip History feature: lists every trip from the database (refreshing its
// status tint whenever the Recording feature starts/ends a trip) and, on
// selection, loads that trip's full sample data in the background for the
// Trajectory View. Row selection is blocked (and a progress indicator shown)
// while a load is in flight, so the user can't pile up overlapping loads.
class TripHistoryPanel : public QWidget {
	Q_OBJECT
public:
	explicit TripHistoryPanel(RecorderBridge& bridge, QWidget* parent = nullptr);
	~TripHistoryPanel() override;

signals:
	void tripDatasetReady(std::shared_ptr<TripDataset> dataset);
	// Emitted when the user deselects the current trip; carries all trip
	// summaries so the map can render the departure→destination overview.
	void tripDeselected(std::vector<TripSummary> trips);
	void zoomResetRequested();

public slots:
	// Called by MainWindow when TrajectoryView signals that both chart series
	// and map trajectory workers have finished rendering. Clears loading state.
	void setLoadingFinished();
	// Emits tripDeselected with the current trip list so the map shows the
	// departure→destination overview immediately at startup.
	void showInitialOverview();

protected:
	bool eventFilter(QObject* obj, QEvent* event) override;

private slots:
	void refreshTrips();
	void onRowActivated(const QModelIndex& index);
	void tryFinishLoad();
	void onTableContextMenu(const QPoint& pos);
	void openManageGroupsDialog();

private:
	// Lazily (re)opens historySql_ if the database file didn't exist yet the
	// last time this was called (e.g. app launched before any flight was ever
	// recorded). Returns nullptr if it still can't be opened.
	sqlite3* ensureHistoryConnection();
	// Rebuilds the group filter combo's items from trip_groups, preserving
	// the current selection where possible (e.g. across a refresh after the
	// Manage Groups dialog closes). Also refreshes groupRank_ from the same
	// query, since the two must stay in sync -- see groupRank_. If the active
	// filter selection itself changed as a result (e.g. its group was
	// deleted), resets selectedTripId_ to -1 and clears the table selection,
	// but deliberately does NOT emit tripDeselected itself -- model_ still
	// holds pre-mutation data at this point (this only re-filters it, see
	// TripHistoryModel::applyFilter), so it's the caller's job to emit, after
	// calling refreshTrips(), so the map gets fresh data.
	void reloadGroupFilterCombo();
	// Persists a trip's group assignment and refreshes the table. groupId=0
	// ungroups the trip.
	void setTripGroupFromUi(int tripId, int groupId);

	RecorderBridge& bridge_;
	QTableView* table_;
	TripHistoryModel* model_;
	QComboBox* groupFilterCombo_;
	QLabel* totalDurationLabel_;
	QPushButton* manageGroupsButton_;
	QProgressBar* loadingBar_;
	// One watcher per parallel query, all launched directly from the GUI
	// thread and joined in tryFinishLoad() rather than via a wrapping
	// QtConcurrent::run that blocks on their .result() -- that
	// nested-blocking pattern can starve QThreadPool's limited thread count
	// and deadlock (1 outer task occupying a pool thread while waiting on 3
	// more pool threads that may never become free). Liftoff points and
	// touchdowns are merged into one watcher/one connection below: both are
	// trivial single-table "WHERE trip = ?" lookups (see queryLiftoffs/
	// queryTouchdowns in db_history.cpp), so splitting them across two
	// connections just for parallelism isn't worth a second DB connection's
	// overhead the way the much larger trip_data query is.
	QFutureWatcher<std::shared_ptr<TripDataset>>* pointsWatcher_;
	QFutureWatcher<std::pair<std::vector<LiftoffPoint>, std::vector<TouchdownPoint>>>* liftoffTouchdownsWatcher_;
	QFutureWatcher<std::vector<TripEvent>>* eventsWatcher_;
	QElapsedTimer loadTimer_;
	int pendingTripId_ = -1;
	int selectedTripId_ = -1;
	bool loading_ = false;
	// tryFinishLoad() is connected to all three watchers' finished signals; a
	// future's completion is visible via isFinished() before its queued signal
	// is delivered, so if several watchers finish within the same event loop
	// tick, each queued slot invocation would otherwise see all three as
	// finished and re-run the join (and re-emit tripDatasetReady) once each.
	// Set when the first invocation actually joins the results; reset when a
	// new load starts.
	bool loadJoined_ = false;
	// Independent from RecorderBridge's STATUS::sql: that connection is opened
	// with SQLITE_OPEN_NOMUTEX (single-thread-only) so it cannot be shared with
	// this panel's main-thread refreshes or its QtConcurrent background loads.
	sqlite3* historySql_ = nullptr;
	// group id -> 1-based position in the Manage Groups list order (the same
	// order reloadGroupFilterCombo() lists groups in). Rebuilt every time that
	// function runs; refreshTrips() stamps each TripSummary's groupRank from
	// this map so the map view can key a group's color/shape off its list
	// position instead of its arbitrary creation id, and so that position
	// stays correct even when the map is showing a filtered subset of groups.
	QMap<int, int> groupRank_;
};
