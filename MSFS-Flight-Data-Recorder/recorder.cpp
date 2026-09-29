#include "recorder.h"
#include "db.h"
#include "gui_notify.h"
#include "logger.h"
#include "airport_lookup.h"
#include "flight_phase.h"
#include "sim_link.h"
#include <chrono>
#include <thread>

// Rate-limits the "Event ignored (no active trip)" TRACE line in
// commit_event() below -- independent of EventFloodFilter's tiers, and it
// never affects whether an occurrence commits: entries just age out. Exists
// for the flap events the filter lets straight through (FLAPS_INCR/
// FLAPS_DECR): without it, a flap lever held with no trip active would print
// one line per occurrence, unbounded. Same 5 s as the filter's slow-flood
// window, for consistency only.
static const std::chrono::milliseconds EVENT_NO_TRIP_LOG_COOLDOWN(5000);

// Where an event occurrence that has passed (or bypassed) EventFloodFilter's
// tiers either gets recorded or discarded, based purely on whether a trip
// exists to attach it to. Has no memory of its own governing *whether* an
// occurrence commits -- flood-shaped repetition is the filter's job; the only
// state kept here (STATUS::no_trip_log_throttle) rate-limits its own TRACE
// line, see EVENT_NO_TRIP_LOG_COOLDOWN above. trip_id was captured when the
// occurrence happened (a new trip may be live by the time a held-back one
// commits). seq is the filter's id for it, stored in the DB row (if one is
// written) so a slow flood can be retracted precisely later -- see
// db_delete_events() in db.cpp. Returns true if the occurrence was written to
// trip_events and shown in the Live Status list, false if it was dropped.
static bool commit_event(struct STATUS* status, const std::string& name, int trip_id, const std::string& time_zulu, const std::string& time_local, unsigned long long seq) {
	if (trip_id <= 0) {
		auto now = std::chrono::steady_clock::now();
		auto& last_logged = status->no_trip_log_throttle[name];
		if (now - last_logged >= EVENT_NO_TRIP_LOG_COOLDOWN) {
			gui_log_printf(status, GUI_LOG_TRACE, "Event ignored (no active trip): %s", name.c_str());
			last_logged = now;
		}
		return false;
	}
	gui_notify_event_committed(status, trip_id, seq, name.c_str());
	status->event_write_queue.push(trip_id, name, time_zulu, time_local, seq);
	return true;
}

// Wires the flood filter's output to this trip-aware commit and to the DB/UI
// retraction. A retraction's Delete goes onto the same single-threaded write
// queue as the inserts that created those rows, so it can never run before
// they land -- see EventWriteQueue in types.h.
static EventFloodFilter::Output event_output(struct STATUS* status) {
	EventFloodFilter::Output out;
	out.commit = [status](const std::string& name, int trip_id, const std::string& time_zulu,
		const std::string& time_local, unsigned long long seq) {
		commit_event(status, name, trip_id, time_zulu, time_local, seq);
	};
	out.retract = [status](const std::vector<unsigned long long>& seqs) {
		status->event_write_queue.push_delete(seqs);
		gui_notify_events_retracted(status, seqs.data(), seqs.size());
	};
	return out;
}

// Signals the DB-write worker to drain and exit, then joins it. Callers must
// call this before closing/nulling status->sql.
void wait_for_db_writers(struct STATUS* status) {
	// Must run before event_write_queue.stop() below: this is the last chance
	// for any occurrence still held back by the flood filter to reach the
	// queue at all -- once the worker thread is stopped and joined, and
	// status->sql is closed by the caller right after this function returns,
	// nothing will ever flush them again for this session. Each still carries
	// the trip_id it was captured with, so it lands in the trip it happened in.
	status->event_filter.flush_all(event_output(status));
	status->sample_write_queue.stop();
	if (status->db_writer_thread.joinable())
		status->db_writer_thread.join();
	status->event_write_queue.stop();
	if (status->event_writer_thread.joinable())
		status->event_writer_thread.join();
}

void CALLBACK MyDispatchProc(SIMCONNECT_RECV* pData, DWORD cbData, void* pContext) {
	struct STATUS* status = (struct STATUS*)pContext;
	try {
	switch (pData->dwID) {
	case SIMCONNECT_RECV_ID_OPEN:
		gui_log_printf(status, GUI_LOG_INFO, "Connected to Microsoft Flight Simulator");
		gui_notify_connection_changed(status, true);
		break;
	case SIMCONNECT_RECV_ID_QUIT:
		gui_log_printf(status, GUI_LOG_INFO, "Disconnected from Microsoft Flight Simulator");
		gui_notify_connection_changed(status, false);
		status->quit = TRUE;
		break;
	case SIMCONNECT_RECV_ID_EVENT_EX1:
	case SIMCONNECT_RECV_ID_EVENT:
	{
		SIMCONNECT_RECV_EVENT* evt = (SIMCONNECT_RECV_EVENT*)pData;
		switch (evt->uEventID) {
		case EVENT_SIM:
			status->sim_running = (bool)evt->dwData;
			if (!status->sim_running && status->in_sim) {
				status->in_sim = FALSE;
				if (status->recording)
					stop_recording(status);
			}
			break;
		case EVENT_PAUSE:
			status->paused = (bool)evt->dwData;
			break;
		case EVENT_CRASHED:
			gui_log_printf(status, GUI_LOG_WARNING, "Plane crashed!");
			break;
		default:
			if (evt->uEventID > EVENT_CRASHED && evt->uEventID < EVENT_ID_COUNT) {
				// A cockpit event (COCKPIT_EVENTS). Flood detection runs
				// regardless of trip state; whether a trip exists to attach
				// this occurrence to is decided per-occurrence by
				// commit_event(), not here.
				status->event_filter.record(event_name(evt->uEventID), status->id_trip,
					status->data.time_zulu.format_date_time(), status->data.time_local.format_date_time(),
					event_output(status));
			} else {
				gui_log_printf(status, GUI_LOG_WARNING, "Unknown event ID: %ld", evt->uEventID);
			}
			break;
		}
	}
	break;
	case SIMCONNECT_RECV_ID_SIMOBJECT_DATA:
	{
		SIMCONNECT_RECV_SIMOBJECT_DATA* pObjData = (SIMCONNECT_RECV_SIMOBJECT_DATA*)pData;
		switch (pObjData->dwRequestID) {
		case REQUEST_FLIGHT:
		{
			// Piggybacks on this periodic tick to resolve flood-filter entries
			// whose quiet period has elapsed, so no dedicated timer is needed.
			status->event_filter.flush_stale(event_output(status));
			struct FLIGHT_DATA_RECORD tmp;
			decode_flight_sample(pObjData, tmp);
			flight_on_sample(status, tmp);
		}
		break;
		default:
			gui_log_printf(status, GUI_LOG_WARNING, "SIMCONNECT_RECV_SIMOBJECT_DATA: %d", pObjData->dwRequestID);
			break;
		}
	}
	break;
	case SIMCONNECT_RECV_ID_AIRPORT_LIST:
		lookup_on_airport_list(status, (SIMCONNECT_RECV_AIRPORT_LIST*)pData);
		break;
	case SIMCONNECT_RECV_ID_FACILITY_DATA:
		lookup_on_facility_data(status, (SIMCONNECT_RECV_FACILITY_DATA*)pData);
		break;
	case SIMCONNECT_RECV_ID_FACILITY_DATA_END:
		lookup_on_facility_data_end(status);
		break;
	case SIMCONNECT_RECV_ID_EXCEPTION: {
		// SimConnect reports failed AddToDataDefinition/MapClientEventToSimEvent/
		// RequestDataOnSimObject calls asynchronously here rather than through
		// their own (synchronous, "queued OK") return values, so this is the
		// only place a bad simvar/event name from an SDK or aircraft-SDK change
		// would ever surface -- silently dropping it would desync the data
		// definition's field ordering with zero trace in the log.
		SIMCONNECT_RECV_EXCEPTION* except = (SIMCONNECT_RECV_EXCEPTION*)pData;
		gui_log_printf(status, GUI_LOG_WARNING,
			"SimConnect exception SIMCONNECT_EXCEPTION_%s (%lu) (SendID=%lu, Index=%lu)",
			simconnect_exception_name(except->dwException), except->dwException,
			except->dwSendID, except->dwIndex);
		// If this exception corresponds to the currently outstanding facility-lookup
		// request (matched by SendID -- see lookup.send_id in types.h), the
		// lookup will never receive its normal terminal response (the AIRPORT_LIST
		// no-match branch or FACILITY_DATA_END), so without this lookup.pending
		// would stay stuck true forever, silently disabling all future departure/
		// destination airport-runway resolution for the rest of the app session.
		lookup_on_exception(status, except->dwSendID);
		break;
	}
	default:
		gui_log_printf(status, GUI_LOG_WARNING, "SIMCONNECT_RECV: %d", pData->dwID);
		break;
	}
	} catch (const db_exception& e) {
		gui_log_printf(status, GUI_LOG_WARNING, "Database error in dispatch: %s", e.message.c_str());
	} catch (...) {
		// Catch-all, not just db_exception -- this callback also does raw SimConnect
		// data handling, malloc/free, and geodesic math, any of which could throw
		// something else. Left uncaught, it would escape a Qt timer slot and likely
		// terminate the app instead of just logging and continuing.
		gui_log_printf(status, GUI_LOG_WARNING, "Unknown error in dispatch");
	}
}
