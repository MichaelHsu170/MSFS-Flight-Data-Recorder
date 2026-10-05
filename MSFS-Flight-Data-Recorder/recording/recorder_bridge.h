#pragma once

#include <QObject>
#include <QFuture>
#include <QList>
#include <QString>

#include "types.h"
#include "simconnect_defs.h"

class QTimer;

// Owns the STATUS struct and drives SimConnect_CallDispatch() on the Qt main
// thread from a timer, retrying SimConnect_Open() until the simulator is
// available. gui_notify_*() free functions (declared in gui_notify.h,
// implemented in recorder_bridge.cpp) reach back into this object via
// status->gui_context to turn recorder.cpp's, flight_phase.cpp's,
// airport_lookup.cpp's and db.cpp's state-transition points into Qt signals.
// Idle until start().
class RecorderBridge : public QObject {
	Q_OBJECT
public:
	explicit RecorderBridge(QObject* parent = nullptr);
	~RecorderBridge() override;

	// Starts connecting to the simulator, retrying every 2 s until it's
	// available. Not done on construction: connecting opens the recorder's
	// database connection (connect_db()), which must wait for migrate_db() --
	// see MainWindow.
	void start();

	const FLIGHT_DATA& currentData() const { return status_.data; }
	bool isRecording() const { return status_.recording; }
	// User-facing gate on automatic recording start; see STATUS::recording_enabled.
	bool isRecordingEnabled() const { return status_.recording_enabled; }
	// No-op if a trip is currently recording (see isRecording()) -- callers
	// (LiveStatusPanel's click handler) are expected to check that first, but
	// this guards the underlying flag either way since toggling it mid-trip
	// couldn't affect that trip regardless.
	void setRecordingEnabled(bool enabled);
	int currentTripId() const { return status_.id_trip; }
	// True if tripId stopped recording but its tail samples may still be
	// draining onto the DB-write thread. A trip can look non-Live
	// (currentTripId() no longer matches it) while this is still true for
	// it -- callers that need to know a trip's data is safe to delete should
	// check both. See STATUS::flushing_trip_ids in types.h.
	bool isTripFlushing(int tripId) const {
		std::lock_guard<std::mutex> lock(status_.flushing_trip_ids_mutex);
		return status_.flushing_trip_ids.count(tripId) != 0;
	}

	STATUS* status() { return &status_; }

signals:
	void logMessage(const QString& text);
	void connectionChanged(bool connected);
	void recordingStateChanged(int tripId);
	void tripEnded(int tripId);
	void recordingEnabledChanged(bool enabled);
	void tripUpdated(int tripId);
	// A sample was just queued for trip_data (see gui_notify_sample);
	// currentData() has its values.
	void sampleUpdated();
	// One event occurrence committed to trip_events, carrying its event_seq
	// and the trip it was committed under (which can already be stale by the
	// time this fires) -- see gui_notify_event_committed() in gui_notify.h.
	void eventCommitted(int tripId, quint64 seq, const QString& text);
	// A tier-2-confirmed flood's already-shown occurrences, or a single
	// occurrence whose DB write later failed, retracted from the DB (or never
	// written at all) and due to be pulled back out of the Live Status list --
	// see gui_notify_events_retracted() in gui_notify.h.
	void eventsRetracted(QList<quint64> seqs);

private slots:
	void pollDispatch();
	void tryConnect();

private:
	void shutdown();

	STATUS status_;
	QTimer* dispatchTimer_;
	QTimer* connectTimer_;
	bool connected_ = false;
	// Logs the SimConnect_Open failure once per disconnected episode instead of
	// every 2s retry, so the log shows why it isn't connecting without spamming.
	bool connectFailureLogged_ = false;
	// Holds the in-progress future when shutdown() offloads wait_for_db_writers()
	// and the sqlite3 close to a worker thread, so the GUI thread stays live while
	// the DB writers drain.
	QFuture<void> stopFuture_;
};
