#pragma once

#include <QHash>
#include <QList>
#include <QString>
#include <QWidget>

class QLabel;
class QListWidget;
class QListWidgetItem;
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
	// One event occurrence committed to trip_events -- adds its line like
	// onLogMessage, but also remembers seq -> item so a later retraction (see
	// onEventsRetracted) can find and remove it. Skipped if tripId no longer
	// matches recordingTripId_: a tier-1/tier-2-delayed occurrence can commit
	// (correctly, against the trip it actually happened in) after that trip
	// has already stopped recording or been superseded by a new one, and
	// showing it live at that point would misrepresent it as current -- same
	// staleness race onTripEnded() already guards against. See RecorderBridge::
	// eventCommitted / gui_notify_event_committed (gui_notify.h).
	void onEventCommitted(int tripId, quint64 seq, const QString& text);
	// A tier-2-confirmed flood's already-shown occurrences, or a single
	// occurrence whose DB write later failed -- removes each still-present
	// line by seq. Seqs with no matching entry (already pruned by the
	// history cap below, or never shown -- e.g. a no-active-trip occurrence)
	// are silently skipped; that's expected, not an error. See
	// RecorderBridge::eventsRetracted / gui_notify_events_retracted.
	void onEventsRetracted(const QList<quint64>& seqs);

private:
	static void setIndicator(QLabel* icon, bool active, const QString& tooltip);
	// Shared by onLogMessage/onEventCommitted: appends one timestamped line,
	// enforces the history cap (pruning from the front), and keeps
	// eventItems_ consistent with whatever got pruned. Returns the new item
	// (never null -- kMaxHistoryItems is always > 0, so the just-added item is
	// never itself the one pruned).
	QListWidgetItem* appendHistoryItem(const QString& text);

	RecorderBridge& bridge_;
	QLabel* versionLabel_;
	QLabel* connectionIcon_;
	QLabel* recordingIcon_;
	QLabel* snapshotLabel_;
	QListWidget* historyList_;
	// Trip currently shown as "Recording" -- lets a tripEnded() for a stale
	// (already-superseded) trip arriving late from the DB writer's queue be
	// ignored instead of clearing the indicator for the trip that replaced it.
	int recordingTripId_ = -1;
	// Tracks history-list items added via onEventCommitted by their
	// event_seq, so onEventsRetracted can remove the right one by identity
	// rather than by row index (which shifts as appendHistoryItem's cap
	// prunes from the front). Entries are removed here whenever the
	// corresponding item is pruned by the cap or retracted, so this never
	// holds a dangling pointer.
	QHash<quint64, QListWidgetItem*> eventItems_;
};
