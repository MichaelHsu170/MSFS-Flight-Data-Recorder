#pragma once

#include <QMainWindow>

class RecorderBridge;
class LiveStatusPanel;
class TripHistoryPanel;
class TrajectoryView;

// Thin shell: owns one instance of each feature module and wires only the
// signals that genuinely cross feature boundaries. Top row is Trip History
// (wide) next to the compact Live Status panel; Trajectory View (map + data
// table + charts) fills the rest.
class MainWindow : public QMainWindow {
	Q_OBJECT
public:
	explicit MainWindow(RecorderBridge& bridge, QWidget* parent = nullptr);

private:
	LiveStatusPanel* liveStatusPanel_;
	TripHistoryPanel* tripHistoryPanel_;
	TrajectoryView* trajectoryView_;
};
