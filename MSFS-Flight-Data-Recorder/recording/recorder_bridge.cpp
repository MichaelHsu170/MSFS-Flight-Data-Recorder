#include "recorder_bridge.h"
#include "recorder.h"
#include "sim_link.h"
#include "airport_lookup.h"
#include "flight_phase.h"
#include "db.h"
#include "gui_notify.h"
#include "logger.h"
#include "app_settings.h"

#include <QTimer>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>

#include <cstdarg>
#include <cstdio>

RecorderBridge::RecorderBridge(QObject* parent)
	: QObject(parent)
	, dispatchTimer_(new QTimer(this))
	, connectTimer_(new QTimer(this))
{
	status_.gui_context = this;
	status_.recording_enabled = AppSettings::instance().recordingEnabled();

	connect(dispatchTimer_, &QTimer::timeout, this, &RecorderBridge::pollDispatch);
	connect(connectTimer_, &QTimer::timeout, this, &RecorderBridge::tryConnect);
}

void RecorderBridge::start() {
	connectTimer_->start(2000);
	tryConnect();
}

RecorderBridge::~RecorderBridge() {
	// If shutdown()'s async DB-writer drain is still running (MSFS quit
	// mid-session), wait for it before the STATUS struct gets destroyed.
	if (stopFuture_.isRunning())
		stopFuture_.waitForFinished();
	// Synchronous cleanup for anything shutdown() didn't already handle
	// (normal app-close path where MSFS is still running).
	dispatchTimer_->stop();
	connectTimer_->stop();
	if (connected_) {
		connected_ = false;
		if (status_.recording)
			stop_recording(&status_);
		SimConnect_Close(status_.hSimConnect);
		// Must happen before closing sql: the persistent db_writer_thread may
		// still be draining queued samples, so sql would otherwise be
		// closed/nulled while that thread is still writing to it.
		wait_for_db_writers(&status_);
		if (status_.sql) {
			sqlite3_close_v2(status_.sql);
			status_.sql = nullptr;
		}
	}
}

void RecorderBridge::setRecordingEnabled(bool enabled) {
	if (status_.recording)
		return;
	if (status_.recording_enabled == enabled)
		return;
	status_.recording_enabled = enabled;
	AppSettings::instance().setRecordingEnabled(enabled);
	Logger::logf(Logger::Info, "Recorder", "Recording %s by user", enabled ? "enabled" : "disabled");
	emit recordingEnabledChanged(enabled);
}

void RecorderBridge::tryConnect() {
	if (connected_)
		return;

	HRESULT hr = SimConnect_Open(&status_.hSimConnect, "Flight Data Recorder", NULL, 0, 0, SIMCONNECT_OPEN_CONFIGINDEX_LOCAL);
	if (FAILED(hr)) {
		if (!connectFailureLogged_) {
			Logger::logf(Logger::Trace, "Recorder", "Waiting for Microsoft Flight Simulator to start (hr=0x%08lX); retrying every 2s", hr);
			connectFailureLogged_ = true;
		}
		return;
	}
	connectFailureLogged_ = false;

	connectTimer_->stop();

	SimConnect_SubscribeToSystemEvent(status_.hSimConnect, EVENT_SIM, "Sim");
	SimConnect_SubscribeToSystemEvent(status_.hSimConnect, EVENT_PAUSE, "Pause");
	SimConnect_SubscribeToSystemEvent(status_.hSimConnect, EVENT_CRASHED, "Crashed");
	add_client_events(status_.hSimConnect);
	add_flight_definition(status_.hSimConnect);

	connect_db(&status_);
	status_.sample_interval_ms = AppSettings::instance().sampleIntervalMs();

	// A fresh SimConnect connection starts with no data definitions registered
	// and no requests outstanding, so any facility-lookup state carried over
	// from a previous connection (e.g. MSFS quit/crashed with a lookup still
	// in flight, leaving lookup.pending stuck true forever with no
	// response ever able to arrive to clear it) must be reset here. Safe to
	// clear departure/destination unconditionally: shutdown() always stops
	// any active recording before connected_ can go false and tryConnect()
	// can fire again, so no trip can be recording at this point.
	reset_airport_lookup(&status_);
	status_.flight.departure_lookup_needed = false;
	status_.flight.departure_lookup_initiated = false;
	status_.departure.clear();
	status_.destination.clear();

	Logger::logf(Logger::Trace, "Recorder", "SimConnect connected: sample_interval_ms=%d",
		status_.sample_interval_ms);

	connected_ = true;
	dispatchTimer_->start(15);
}

void RecorderBridge::pollDispatch() {
	if (status_.quit) {
		shutdown();
		return;
	}
	// MSFS doesn't always send SIMCONNECT_RECV_ID_QUIT on exit (depends on how
	// it's closed) -- a failing dispatch call means the connection is dead
	// either way, so treat it the same as an explicit quit.
	HRESULT hr = SimConnect_CallDispatch(status_.hSimConnect, MyDispatchProc, &status_);
	if (FAILED(hr)) {
		Logger::logf(Logger::Warning, "Recorder", "SimConnect_CallDispatch failed (hr=0x%08lX); treating as disconnect", hr);
		sim_disconnected(&status_);
	}
}

void RecorderBridge::shutdown() {
	if (!connected_)
		return;

	Logger::logf(Logger::Trace, "Recorder", "shutdown: recording=%d", status_.recording ? 1 : 0);

	connected_ = false;
	connectFailureLogged_ = false;
	dispatchTimer_->stop();
	SimConnect_Close(status_.hSimConnect);
	status_.quit = FALSE;

	if (status_.recording)
		stop_recording(&status_);

	// The persistent db_writer_thread may still be draining queued samples
	// through status_.sql -- either the end-of-trip barrier stop_recording()
	// just pushed above, or samples from earlier in the trip that hadn't been
	// flushed yet (recording can already be false here, e.g. the plane landed
	// a while before MSFS quit). wait_for_db_writers() stops and joins that
	// thread, which can block for a bit, so do it on a worker thread to keep
	// the UI responsive; reconnect once the connection is closed.
	// Connect before setFuture(): if the worker finishes fast (little/nothing
	// queued to drain), setFuture()-then-connect() leaves a window where the
	// finished signal fires before this handler is wired up, and the missed
	// signal would leave connectTimer_ never restarted.
	auto* watcher = new QFutureWatcher<void>(this);
	connect(watcher, &QFutureWatcher<void>::finished, this, [this, watcher]() {
		watcher->deleteLater();
		Logger::log(Logger::Trace, "Recorder", QStringLiteral("shutdown: db writers drained, sql closed; scheduling reconnect"));
		connectTimer_->start(2000);
	});
	stopFuture_ = QtConcurrent::run([this]() {
		wait_for_db_writers(&status_);
		sqlite3_close_v2(status_.sql);
		status_.sql = nullptr;
	});
	watcher->setFuture(stopFuture_);
}

// The RecorderBridge that owns status, or null when there's none to notify.
static RecorderBridge* bridgeOf(struct STATUS* status) {
	return status ? static_cast<RecorderBridge*>(status->gui_context) : nullptr;
}

void gui_notify_log(struct STATUS* status, GuiLogLevel level, const char* text) {
	Logger::log(static_cast<Logger::Level>(level), "Recorder", QString::fromUtf8(text));
	// Only forward Warning and Info to the LiveStatusPanel UI feed.
	// Trace lines (e.g. the airport/runway lookup detail) stay log-only.
	if (level > GUI_LOG_INFO)
		return;
	if (RecorderBridge* bridge = bridgeOf(status))
		emit bridge->logMessage(QString::fromUtf8(text));
}

void gui_log_printf(struct STATUS* status, GuiLogLevel level, const char* fmt, ...) {
	va_list args;
	va_start(args, fmt);
	const std::string text = Logger::vformat(fmt, args);
	va_end(args);
	gui_notify_log(status, level, text.c_str());
}

void gui_notify_connection_changed(struct STATUS* status, bool connected) {
	Logger::logf(Logger::Trace, "Recorder", "connection changed: connected=%d", connected ? 1 : 0);
	if (RecorderBridge* bridge = bridgeOf(status))
		emit bridge->connectionChanged(connected);
}

void gui_notify_recording_changed(struct STATUS* status, bool recording, int tripId) {
	Logger::logf(Logger::Trace, "Recorder", "recording changed: recording=%d, trip=%d", recording ? 1 : 0, tripId);
	RecorderBridge* bridge = bridgeOf(status);
	if (!bridge)
		return;
	if (recording)
		emit bridge->recordingStateChanged(tripId);
	else
		emit bridge->tripEnded(tripId);
}

void gui_notify_trip_updated(struct STATUS* status) {
	RecorderBridge* bridge = bridgeOf(status);
	if (!bridge)
		return;
	Logger::logf(Logger::Trace, "Recorder", "trip updated: trip=%d", status->id_trip);
	emit bridge->tripUpdated(status->id_trip);
}

void gui_notify_sample(struct STATUS* status) {
	if (RecorderBridge* bridge = bridgeOf(status))
		emit bridge->sampleUpdated();
}

void gui_notify_event_committed(struct STATUS* status, int tripId, unsigned long long seq, const char* name) {
	char buf[300];
	snprintf(buf, sizeof(buf), "Event: %s", name);
	// Logged directly, not via gui_notify_log: that would also emit it as a
	// logMessage, and the eventCommitted below already adds it to the
	// LiveStatusPanel feed.
	Logger::log(Logger::Info, "Recorder", QString::fromUtf8(buf));
	RecorderBridge* bridge = bridgeOf(status);
	if (!bridge)
		return;
	// Deliberately its own signal rather than logMessage: LiveStatusPanel
	// needs to record seq -> QListWidgetItem atomically with adding the line,
	// which a generic text-only logMessage plus a separately-ordered seq
	// notification can't guarantee as cleanly.
	emit bridge->eventCommitted(tripId, static_cast<quint64>(seq), QString::fromUtf8(buf));
}

void gui_notify_events_retracted(struct STATUS* status, const unsigned long long* seqs, size_t count) {
	RecorderBridge* bridge = bridgeOf(status);
	if (!bridge)
		return;
	QList<quint64> list;
	list.reserve((int)count);
	for (size_t i = 0; i < count; i++)
		list.append(static_cast<quint64>(seqs[i]));
	emit bridge->eventsRetracted(list);
}

