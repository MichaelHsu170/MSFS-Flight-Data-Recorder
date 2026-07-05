#pragma once

#include <QWidget>
#include <QAbstractTableModel>
#include <QElapsedTimer>
#include <QFutureWatcher>
#include <memory>

#include "trip_dataset.h"

struct sqlite3;
class RecorderBridge;
class QTableView;
class QProgressBar;

// Read-only table model backing TripHistoryPanel's QTableView. One row per
// trip; status (Completed/Open/Live) is conveyed entirely through the row's
// BackgroundRole tint (see TripHistoryModel::data) rather than its own column.
class TripHistoryModel : public QAbstractTableModel {
	Q_OBJECT
public:
	enum Column {
		TitleColumn,
		FlightColumn,
		DepartureRegionColumn,
		DepartureColumn,
		DepartureRwyColumn,
		DestinationRegionColumn,
		DestinationColumn,
		DestinationRwyColumn,
		DepartureTimeColumn,
		DestinationTimeColumn,
		ColumnCount,
	};

	explicit TripHistoryModel(QObject* parent = nullptr);

	void setTrips(std::vector<TripSummary> trips);
	const TripSummary* tripAt(int row) const;
	const std::vector<TripSummary>& trips() const { return trips_; }

	int rowCount(const QModelIndex& parent = QModelIndex()) const override;
	int columnCount(const QModelIndex& parent = QModelIndex()) const override;
	QVariant data(const QModelIndex& index, int role) const override;
	QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
	Qt::ItemFlags flags(const QModelIndex& index) const override;

private:
	std::vector<TripSummary> trips_;
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

private slots:
	void refreshTrips();
	void onRowActivated(const QModelIndex& index);
	void tryFinishLoad();
	void onTableContextMenu(const QPoint& pos);

private:
	// Lazily (re)opens historySql_ if the database file didn't exist yet the
	// last time this was called (e.g. app launched before any flight was ever
	// recorded). Returns nullptr if it still can't be opened.
	sqlite3* ensureHistoryConnection();

	RecorderBridge& bridge_;
	QTableView* table_;
	TripHistoryModel* model_;
	QProgressBar* loadingBar_;
	// One watcher per parallel query, all launched directly from the GUI
	// thread and joined in tryFinishLoad() rather than via a wrapping
	// QtConcurrent::run that blocks on their .result() -- that
	// nested-blocking pattern can starve QThreadPool's limited thread count
	// and deadlock (1 outer task occupying a pool thread while waiting on 3
	// more pool threads that may never become free).
	QFutureWatcher<std::shared_ptr<TripDataset>>* pointsWatcher_;
	QFutureWatcher<std::vector<TouchdownPoint>>* touchdownsWatcher_;
	QFutureWatcher<std::vector<TripEvent>>* eventsWatcher_;
	QElapsedTimer loadTimer_;
	int pendingTripId_ = -1;
	int selectedTripId_ = -1;
	bool loading_ = false;
	// Independent from RecorderBridge's STATUS::sql: that connection is opened
	// with SQLITE_OPEN_NOMUTEX (single-thread-only) so it cannot be shared with
	// this panel's main-thread refreshes or its QtConcurrent background loads.
	sqlite3* historySql_ = nullptr;
};
