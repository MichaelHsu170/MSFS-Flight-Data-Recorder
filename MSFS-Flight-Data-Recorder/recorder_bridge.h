#pragma once

#include <QObject>
#include <QFuture>
#include <QList>
#include <QString>

#include "types.h"
#include "simconnect_defs.h"
#include "trip_dataset.h"

class QTimer;

// Owns the STATUS struct and drives SimConnect_CallDispatch() on the Qt main thread
// (replacing the console main()'s busy while-loop with a timer), retrying
// SimConnect_Open() until the simulator is available. gui_notify_*() free functions
// (implemented below, in this .cpp) reach back into this object via status->gui_context
// to turn recorder.cpp's existing printf/state-transition points into Qt signals.
class RecorderBridge : public QObject {
	Q_OBJECT
public:
	explicit RecorderBridge(QObject* parent = nullptr);
	~RecorderBridge() override;

	const FLIGHT_DATA& currentData() const { return status_.data; }
	bool isConnected() const { return connected_; }
	bool isRecording() const { return status_.recording; }
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
	void tripUpdated(int tripId);
	void sampleUpdated();
	// Same sample just queued for trip_data (see gui_notify_sample), decoded
	// into the shared TripSamplePoint shape so live and historical data look
	// identical to TrajectoryView/ChartsPanel/MapWidget.
	void liveDataPoint(const TripSamplePoint& point);
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
	// Holds the in-progress future when stop_recording is offloaded to a
	// worker thread so the GUI thread stays live while the DB flush completes.
	QFuture<void> stopFuture_;
};
