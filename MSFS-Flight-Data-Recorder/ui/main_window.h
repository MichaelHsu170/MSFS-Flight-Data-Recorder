#pragma once

#include <QFutureWatcher>
#include <QMainWindow>
#include <QPointer>

class RecorderBridge;
class TrajectoryView;
class QSplitter;

// Thin shell: owns one instance of each feature module and wires only the
// signals that genuinely cross feature boundaries. Top row is Trip History
// (wide) next to the compact Live Status panel; Trajectory View (map + data
// table + charts) fills the rest.
//
// Opens before the database is ready: migrate_db() runs on a worker thread,
// and Trip History is built and the bridge started (RecorderBridge::start())
// only once it finishes, since both use the database. Until then a label
// holds Trip History's place: "Checking the database", or the percentage
// done once a long upgrade reports progress; if the migration fails, it says
// so and stays, and the bridge isn't started. Closing the window cancels a
// migration still running (see closeEvent()). Everything else is built before
// the window is shown: adding the map's and charts' GPU-rendered widgets to an
// already visible window makes Qt destroy and recreate it, which looks like
// the window closing and reopening.
class MainWindow : public QMainWindow {
	Q_OBJECT
public:
	explicit MainWindow(RecorderBridge& bridge, QWidget* parent = nullptr);

protected:
	void closeEvent(QCloseEvent* event) override;

private:
	void addTripHistory(RecorderBridge& bridge);

	// The migration's watcher; null once deleted (deleteLater() in its
	// finished handler). Cancelling it before that handler runs -- even after
	// the migration itself returned -- makes the handler start nothing.
	QPointer<QFutureWatcher<bool>> migration_;
	TrajectoryView* trajectoryView_ = nullptr;
	QSplitter* topSplitter_ = nullptr;
};
