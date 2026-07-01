#pragma once

#include <QString>
#include <QWidget>

class QLabel;
class QListWidget;
class RecorderBridge;

// Mirrors what the console build used to printf(): connection state, recording
// start/stop, takeoff/touchdown/crash messages, and a live flight-data snapshot.
// Connection/recording are shown as small painted dots (black = true, grey =
// false -- painted rather than drawn from a Unicode glyph, since glyphs like
// the power/record symbols get rendered by Windows' color emoji font and
// ignore QLabel's text color entirely) with the descriptive text in a
// tooltip. The full message history is an always-visible scrolling list
// (newest entry on top) inline in this panel's own layout rather than behind
// a separate History button/dialog. Also hosts the "Live Follow" toggle for
// the Trajectory View feature, forwarded out via liveFollowToggled(bool)
// since this panel has no direct dependency on TrajectoryView.
class LiveStatusPanel : public QWidget {
	Q_OBJECT
public:
	explicit LiveStatusPanel(RecorderBridge& bridge, QWidget* parent = nullptr);

private slots:
	void onLogMessage(const QString& text);
	void onConnectionChanged(bool connected);
	void onRecordingStateChanged(int tripId);
	void onTripEnded(int tripId);
	void onSampleUpdated();

private:
	static void setIndicator(QLabel* icon, bool active, const QString& tooltip);

	RecorderBridge& bridge_;
	QLabel* connectionIcon_;
	QLabel* recordingIcon_;
	QLabel* snapshotLabel_;
	QListWidget* historyList_;
};
