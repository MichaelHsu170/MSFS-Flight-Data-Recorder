#include "recorder_bridge.h"
#include "recorder.h"
#include "db.h"
#include "gui_notify.h"
#include "logger.h"
#include "app_settings.h"
#include "trip_data_fields.h"

#include <QTimer>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>

#include <cstdarg>
#include <cstdio>

namespace {

TripSamplePoint toSamplePoint(FLIGHT_DATA_RECORD sample) {
	TripSamplePoint p;
	p.latitude = sample.plane_coordinate.latitude;
	p.longitude = sample.plane_coordinate.longitude;
	p.altitude = (int)sample.plane_altitude;
	p.airspeed = (int)sample.airspeed_indicated;
	p.groundSpeed = (int)sample.ground_velocity;
	p.verticalSpeed = (int)sample.vertical_speed;
	p.n1_1 = sample.turb_eng_n1_1;
	p.n1_2 = sample.turb_eng_n1_2;
	p.n2_1 = sample.turb_eng_n2_1;
	p.n2_2 = sample.turb_eng_n2_2;
	p.gearHandlePosition = sample.gear_handle_position;
	p.gearPosition[0] = (int)sample.gear_position_0;
	p.gearPosition[1] = (int)sample.gear_position_1;
	p.gearPosition[2] = (int)sample.gear_position_2;
	p.gearOnGround[0] = sample.gear_is_on_ground_0 != 0;
	p.gearOnGround[1] = sample.gear_is_on_ground_1 != 0;
	p.gearOnGround[2] = sample.gear_is_on_ground_2 != 0;
	p.brakeIndicator = (int)sample.brake_indicator;
	p.flapsHandleIndex = sample.flaps_handle_index;
	p.spoilersHandlePosition = sample.spoilers_handle_position;
	p.fuelTotalQuantityWeight = sample.fuel_total_quantity_weight;
	p.zuluTime = QString::fromStdString(sample.time_zulu.format_date_time());
	p.localTime = QString::fromStdString(sample.time_local.format_date_time());

#define TRIP_NUM_PUSH(dbColumn, memberExpr) \
	p.rawNums.push_back((double)(sample.memberExpr));
	TRIP_DATA_NUM_FIELDS(TRIP_NUM_PUSH)
#undef TRIP_NUM_PUSH

	{
		uint32_t bgs[4] = {};
#define TRIP_BOOL_PACK(name, group, bit) \
		bgs[group] |= sample.name != 0 ? (1u << (bit)) : 0u;
		TRIP_DATA_BOOL_FIELDS(TRIP_BOOL_PACK)
#undef TRIP_BOOL_PACK
		p.boolGroup1 = bgs[1];
		p.boolGroup2 = bgs[2];
		p.boolGroup3 = bgs[3];
	}

	return p;
}

}

RecorderBridge::RecorderBridge(QObject* parent)
	: QObject(parent)
	, dispatchTimer_(new QTimer(this))
	, connectTimer_(new QTimer(this))
{
	status_.gui_context = this;

	connect(dispatchTimer_, &QTimer::timeout, this, &RecorderBridge::pollDispatch);
	connect(connectTimer_, &QTimer::timeout, this, &RecorderBridge::tryConnect);

	connectTimer_->start(2000);
	tryConnect();
}

RecorderBridge::~RecorderBridge() {
	// If an async stop_recording task is still running (MSFS quit mid-session),
	// wait for it before the STATUS struct gets destroyed.
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
		if (status_.sql) {
			sqlite3_close_v2(status_.sql);
			status_.sql = nullptr;
		}
	}
}

void RecorderBridge::tryConnect() {
	if (connected_)
		return;

	HRESULT hr = SimConnect_Open(&status_.hSimConnect, "Flight Data Recorder", NULL, 0, 0, SIMCONNECT_OPEN_CONFIGINDEX_LOCAL);
	if (FAILED(hr))
		return;

	connectTimer_->stop();

	SimConnect_SubscribeToSystemEvent(status_.hSimConnect, EVENT_SIM, "Sim");
	SimConnect_SubscribeToSystemEvent(status_.hSimConnect, EVENT_PAUSE, "Pause");
	SimConnect_SubscribeToSystemEvent(status_.hSimConnect, EVENT_CRASHED, "Crashed");
	add_client_events(status_.hSimConnect);
	add_flight_definition(status_.hSimConnect);

	connect_db(&status_);
	status_.sample_interval_ms = AppSettings::instance().sampleIntervalMs();

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
		emit logMessage(QStringLiteral("Disconnected from Microsoft Flight Simulator"));
		emit connectionChanged(false);
		status_.quit = TRUE;
	}
}

void RecorderBridge::shutdown() {
	if (!connected_)
		return;

	connected_ = false;
	dispatchTimer_->stop();
	SimConnect_Close(status_.hSimConnect);
	status_.quit = FALSE;

	if (status_.recording) {
		// stop_recording joins the db_consume worker thread, which can block
		// for several seconds flushing the final batch of samples to SQLite.
		// Run it on a worker thread so the UI stays responsive; reconnect and
		// close the write connection in the watcher callback once it's done.
		stopFuture_ = QtConcurrent::run([this]() {
			stop_recording(&status_);
			sqlite3_close_v2(status_.sql);
			status_.sql = nullptr;
		});
		auto* watcher = new QFutureWatcher<void>(this);
		watcher->setFuture(stopFuture_);
		connect(watcher, &QFutureWatcher<void>::finished, this, [this, watcher]() {
			watcher->deleteLater();
			connectTimer_->start(2000);
		});
	} else {
		sqlite3_close_v2(status_.sql);
		status_.sql = nullptr;
		connectTimer_->start(2000);
	}
}

void gui_notify_log(struct STATUS* status, GuiLogLevel level, const char* text) {
	Logger::log(static_cast<Logger::Level>(level), "Recorder", QString::fromUtf8(text));
	// Only forward Info-and-below to the LiveStatusPanel UI feed.
	// Profile messages (e.g. SIMCONNECT_RECV_SIMOBJECT_DATA spam) stay log-only.
	if (level > GUI_LOG_INFO)
		return;
	if (!status || !status->gui_context)
		return;
	auto* bridge = static_cast<RecorderBridge*>(status->gui_context);
	emit bridge->logMessage(QString::fromUtf8(text));
}

void gui_log_printf(struct STATUS* status, GuiLogLevel level, const char* fmt, ...) {
	char buf[512];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	gui_notify_log(status, level, buf);
}

void gui_notify_connection_changed(struct STATUS* status, bool connected) {
	if (!status || !status->gui_context)
		return;
	auto* bridge = static_cast<RecorderBridge*>(status->gui_context);
	emit bridge->connectionChanged(connected);
}

void gui_notify_recording_changed(struct STATUS* status, bool recording, int tripId) {
	if (!status || !status->gui_context)
		return;
	auto* bridge = static_cast<RecorderBridge*>(status->gui_context);
	if (recording)
		emit bridge->recordingStateChanged(tripId);
	else
		emit bridge->tripEnded(tripId);
}

void gui_notify_trip_updated(struct STATUS* status) {
	if (!status || !status->gui_context)
		return;
	auto* bridge = static_cast<RecorderBridge*>(status->gui_context);
	emit bridge->tripUpdated(status->id_trip);
}

void gui_notify_sample(struct STATUS* status, const struct FLIGHT_DATA_RECORD* sample) {
	if (!status || !status->gui_context)
		return;
	auto* bridge = static_cast<RecorderBridge*>(status->gui_context);
	emit bridge->sampleUpdated();
	if (sample != nullptr)
		emit bridge->liveDataPoint(toSamplePoint(*sample));
}

