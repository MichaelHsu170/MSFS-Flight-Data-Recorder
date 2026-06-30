#pragma once

#include <QObject>
#include <QFuture>
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

	STATUS* status() { return &status_; }

signals:
	void logMessage(const QString& text);
	void connectionChanged(bool connected);
	void recordingStateChanged(int tripId);
	void tripEnded(int tripId);
	void sampleUpdated();
	// Same sample just queued for trip_data (see gui_notify_sample), decoded
	// into the shared TripSamplePoint shape so live and historical data look
	// identical to TrajectoryView/ChartsPanel/MapWidget.
	void liveDataPoint(const TripSamplePoint& point);

private slots:
	void pollDispatch();
	void tryConnect();

private:
	void shutdown();

	STATUS status_;
	QTimer* dispatchTimer_;
	QTimer* connectTimer_;
	bool connected_ = false;
	// Holds the in-progress future when stop_recording is offloaded to a
	// worker thread so the GUI thread stays live while the DB flush completes.
	QFuture<void> stopFuture_;
};
