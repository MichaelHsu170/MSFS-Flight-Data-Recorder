#include "flight_phase.h"
#include "airport_lookup.h"
#include "db.h"
#include "engine_power.h"
#include "gui_notify.h"
#include "simconnect_defs.h"

#include <cstdlib>
#include <cstring>
#include <string>

// What a liftoff/touchdown row records (see db_insert_contact()), from the
// sample it happened in.
static void fill_contact_flight_data(FLIGHT_DATA& data, const FLIGHT_DATA_RECORD& sample) {
	data.heading = (int)sample.plane_heading_degrees_magnetic;
	data.pitch = sample.plane_pitch_degrees;
	data.bank = sample.plane_bank_degrees;
	data.speed = (int)sample.airspeed_indicated;
	data.vertical_speed = (int)sample.vertical_speed;
	data.g_force = sample.g_force;
	data.wind_direction = (int)sample.ambient_wind_direction;
	data.wind_velocity = (int)sample.ambient_wind_velocity;
	data.coordinate.latitude = sample.plane_coordinate.latitude;
	data.coordinate.longitude = sample.plane_coordinate.longitude;
	data.time_zulu = sample.time_zulu;
	data.time_local = sample.time_local;
}

// The earliest liftoff/touchdown in list whose airport lookup hasn't
// resolved yet (runway_act.distances[0] still -1), or NULL. Lookups resolve
// in order, so this is the record an in-flight lookup of that kind is for.
template <typename Record>
static Record* first_unresolved(Record* list) {
	while (list != NULL && list->airport.runway_act.distances[0] != -1)
		list = list->next;
	return list;
}

// Appends a new, zeroed liftoff/touchdown record to the list head..tail and
// gives it the next lookup sequence number (see CONTACT_RECORD::seq). db_id is
// -1 until its row is inserted: malloc+memset never runs the member
// initializer, and 0 would make a later "UPDATE ... WHERE id = 0" silently
// match nothing. NULL if allocation fails.
template <typename Record>
static Record* append_contact_record(struct STATUS* status, Record*& head, Record*& tail) {
	Record* record = (Record*)malloc(sizeof(Record));
	if (record == NULL)
		return NULL;
	memset(record, 0, sizeof(Record));
	record->airport.clear();
	record->db_id = -1;
	record->seq = status->flight.next_facility_lookup_seq++;
	if (head == NULL)
		head = record;
	else
		tail->next = record;
	tail = record;
	return record;
}

namespace {

// Log/warning text for recording a liftoff marker or a touchdown (see
// record_contact()).
struct CONTACT_TEXT {
	const char* alloc_failed;
	const char* inserted;      // db_id
	const char* insert_failed; // trip id, error
};
const CONTACT_TEXT LIFTOFF_TEXT = {
	"Liftoff: malloc failed for liftoff record; this liftoff marker will not be recorded",
	"Liftoff marker trip_liftoffs row inserted: db_id=%d",
	"Liftoff (trip %d, subsequent): trip_liftoffs insert failed, this liftoff marker will not be recorded: %s",
};
const CONTACT_TEXT TOUCHDOWN_TEXT = {
	"Landing: malloc failed for touchdown record; this touchdown will not be recorded",
	"Touchdown trip_touchdowns row inserted: db_id=%d",
	"Landing (trip %d): trip_touchdowns insert failed, this touchdown will not be recorded: %s",
};

}

// Records a liftoff marker or touchdown from sample: appends it to the list
// head..tail and inserts its row right away, so the row survives a crash
// before stop_recording. Airport/runway fields stay NULL until its lookup
// resolves. The record, or NULL if allocation failed; inserted says whether
// its row was written. A failed insert leaves db_id -1, which the lookup
// completion (on_lookup_resolved()) treats as "row was never inserted;
// dropping this resolution" -- the marker just loses persistence instead of
// corrupting downstream state or leaving lookup.pending stuck.
template <typename Record>
static Record* record_contact(struct STATUS* status, Record*& head, Record*& tail, CONTACT_TABLE table,
	const FLIGHT_DATA_RECORD& sample, const CONTACT_TEXT& text, bool& inserted) {
	inserted = false;
	Record* record = append_contact_record(status, head, tail);
	if (record == NULL) {
		gui_log_printf(status, GUI_LOG_WARNING, "%s", text.alloc_failed);
		return NULL;
	}
	fill_contact_flight_data(record->flight_data, sample);
	try {
		record->db_id = db_insert_contact(status, table, status->id_trip, record->flight_data);
		gui_log_printf(status, GUI_LOG_TRACE, text.inserted, record->db_id);
		inserted = true;
	} catch (const db_exception& e) {
		gui_log_printf(status, GUI_LOG_WARNING, text.insert_failed, status->id_trip, e.message.c_str());
	}
	return record;
}

// Frees the liftoff/touchdown list head..tail, leaving it empty.
template <typename Record>
static void free_contact_list(Record*& head, Record*& tail) {
	while (head != NULL) {
		Record* cur = head;
		head = head->next;
		cur->airport.clear();
		free(cur);
	}
	tail = NULL;
}

void stop_recording(struct STATUS* status) {
	gui_log_printf(status, GUI_LOG_TRACE, "stop_recording: trip=%d, last_sample=%s",
		status->id_trip, status->flight.last_sample != NULL ? "present" : "none");
	status->recording = FALSE;
	// Destination lat/lon was written at each touchdown; only the arrival time
	// (engine shutdown) is set here -- consistent with departure time being engine start.
	// last_sample is NULL if the trip ended before a single sample was ever
	// recorded (e.g. engine start immediately followed by engine cutoff, within
	// one sample interval) -- there's no flight data to source a destination
	// time from, so leave those columns unset instead of dereferencing NULL.
	if (status->flight.last_sample != NULL) {
		// Caught, not left to propagate: everything below this point (freeing
		// touchdown_data, resetting id_trip, pushing the end-of-trip barrier)
		// must still run even if this UPDATE fails, or the next trip to start
		// inherits this one's dangling touchdown list/id_trip. Two of this
		// function's four callers (RecorderBridge destructor/shutdown()) have
		// no try/catch of their own, so an uncaught db_exception here would
		// otherwise crash the app on quit instead of just losing one UPDATE.
		try {
			db_set_trip_destination_time(status, status->id_trip, status->flight.last_sample->time_zulu, status->flight.last_sample->time_local);
		} catch (const db_exception& e) {
			gui_log_printf(status, GUI_LOG_WARNING, "stop_recording: failed to write destination time (trip %d): %s",
				status->id_trip, e.message.c_str());
		}
	}
	// trip_touchdowns/trip_liftoffs rows were already inserted when each
	// happened; just free the lists. The trip's single departure record
	// (status->departure/departure_db_id) is untouched -- it isn't part of
	// the liftoff list.
	free_contact_list(status->flight.touchdown_data, status->flight.touchdown_data_end);
	free_contact_list(status->flight.liftoff_data, status->flight.liftoff_data_end);
	// A lookup this trip skipped (because another one was still in flight) and
	// meant to retry later is now moot -- the trip that needed it is gone.
	// Note this deliberately leaves lookup.pending/lookup.trip_id
	// untouched: if this trip's own lookup is still in flight, it must stay
	// pending so a new trip's liftoff doesn't race it, and the staleness checks
	// in airport_lookup.cpp recognize and drop that response once it does arrive.
	status->flight.departure_lookup_needed = FALSE;
	int ended_trip_id = status->id_trip;
	// Marks this trip as still-draining until db_write_worker processes the
	// barrier pushed below, so the UI can keep treating it as undeletable even
	// though id_trip (reset next) will already say no trip is live -- see
	// flushing_trip_ids in types.h.
	{
		std::lock_guard<std::mutex> lock(status->flushing_trip_ids_mutex);
		status->flushing_trip_ids.insert(ended_trip_id);
	}
	// Reset id_trip synchronously (not from the worker thread) so a new trip
	// starting right after this one can never have its events committed under
	// the ended trip's id (commit_event() in recorder.cpp drops an event whose
	// captured id_trip is <= 0).
	status->id_trip = -1;
	// Push an end-of-trip barrier instead of flushing here: the worker thread
	// still has this trip's earlier samples queued ahead of this entry, and
	// processes everything strictly in order, so "Recording stopped" and the
	// GUI notification only fire once every sample has actually been written.
	// A new trip's samples pushed after this point queue up safely behind it
	// -- there's nothing to race, since it's the same one worker thread and
	// the same queue for every trip.
	status->sample_write_queue.push(NULL, ended_trip_id);
}

// Starts the next waiting lookup unless one is already in flight (only one
// may be -- see lookup.pending in types.h). Called right after a liftoff
// marker or touchdown is recorded, and by end_lookup() (airport_lookup.cpp)
// once the in-flight lookup ends, so a record that happened while another
// lookup was in flight is picked up then. A deferred departure takes
// priority since it always happens first within a trip; liftoff markers and
// touchdowns are then matched in the same FIFO order
// on_lookup_resolved() uses to attach a resolved lookup to a touchdown row,
// which requires strict in-order resolution -- skipping straight to a later
// touchdown here would attribute its resolved airport/runway to an earlier,
// still-unresolved one instead.
void request_next_touchdown_facility_lookup(struct STATUS* status) {
	if (status->lookup.pending)
		return;
	if (status->flight.departure_lookup_needed) {
		gui_log_printf(status, GUI_LOG_TRACE, "Facility lookup slot free: picking up deferred departure lookup (trip %d)", status->id_trip);
		status->flight.departure_lookup_needed = FALSE;
		start_facility_lookup(status, LOOKUP_TARGET::DEPARTURE, status->flight.departure_data.coordinate, status->flight.departure_data.heading);
		return;
	}
	struct TOUCHDOWN_DATA* next_td = first_unresolved(status->flight.touchdown_data);
	struct LIFTOFF_DATA* next_lo = first_unresolved(status->flight.liftoff_data);
	if (next_td == NULL && next_lo == NULL)
		return;
	// Both lists are each individually in chronological order (appended at
	// their own tail), but interleaved with each other -- e.g. a touch-and-go
	// produces liftoff, touchdown, liftoff in that order. seq (shared across
	// both lists, see types.h) picks whichever of the two earliest-unresolved
	// candidates actually happened first, so on_lookup_resolved()'s strict
	// in-order-resolution assumption still holds across the combined stream.
	bool pick_liftoff = next_td == NULL || (next_lo != NULL && next_lo->seq < next_td->seq);
	gui_log_printf(status, GUI_LOG_TRACE, "Facility lookup slot free: picking up queued %s lookup (trip %d)",
		pick_liftoff ? "liftoff" : "touchdown", status->id_trip);
	const FLIGHT_DATA& next = pick_liftoff ? next_lo->flight_data : next_td->flight_data;
	start_facility_lookup(status, pick_liftoff ? LOOKUP_TARGET::LIFTOFF : LOOKUP_TARGET::TOUCHDOWN, next.coordinate, next.heading,
		pick_liftoff ? nullptr : &next_td->loc_dh);
}

namespace {

// Log/warning text for each kind of record a lookup resolves, indexed by
// LOOKUP_TARGET.
struct RESOLUTION_TEXT {
	const char* runway;       // name, icao, runway, time
	const char* airport;      // name, icao, lat, lon, time
	const char* no_airport;   // lat, lon, time
	const char* runway_lost;  // name, icao, runway -- the record's row was never inserted
	const char* airport_lost; // name, icao
};
const RESOLUTION_TEXT RESOLUTION_TEXTS[] = {
	{ // DEPARTURE
		"Liftoff from %s (%s) runway %s at %s",
		"Liftoff from %s (%s) [%s, %s] at %s",
		"Liftoff from %s, %s at %s",
		"Liftoff from %s (%s) runway %s: trip_liftoffs row was never inserted; dropping this resolution",
		"Liftoff from %s (%s): trip_liftoffs row was never inserted; dropping this resolution",
	},
	{ // LIFTOFF (touch-and-go marker)
		"Liftoff (subsequent) from %s (%s) runway %s at %s",
		"Liftoff (subsequent) from %s (%s) [%s, %s] at %s",
		"Liftoff (subsequent) from %s, %s at %s",
		"Liftoff (subsequent) from %s (%s) runway %s: trip_liftoffs row was never inserted; dropping this resolution",
		"Liftoff (subsequent) from %s (%s): trip_liftoffs row was never inserted; dropping this resolution",
	},
	{ // TOUCHDOWN
		"Touchdown at %s (%s) runway %s at %s",
		"Touchdown at %s (%s) [%s, %s] at %s",
		"Touchdown at %s, %s at %s",
		"Touchdown at %s (%s) runway %s: trip_touchdowns row was never inserted; dropping this resolution",
		"Touchdown at %s (%s): trip_touchdowns row was never inserted; dropping this resolution",
	},
};

}

// Applies a finished facility lookup's result (see airport_lookup.h) to the
// record it was for -- the trip's departure, or the earliest liftoff marker/
// touchdown still waiting -- logging it with the time the record happened,
// storing the airport/runway on the record's row (and, for the departure
// and a touchdown, on the trip as its departure/destination) and notifying
// the UI after the writes. A record whose row was never inserted is logged
// and skipped. slot is the lookup's AIRPORT (facility_lookup_target()).
void on_lookup_resolved(struct STATUS* status, AIRPORT* slot, LOOKUP_OUTCOME outcome) {
	const LOOKUP_TARGET target = status->lookup.target;
	const RESOLUTION_TEXT& text = RESOLUTION_TEXTS[(int)target];
	const bool departure = target == LOOKUP_TARGET::DEPARTURE;
	const CONTACT_TABLE table = target == LOOKUP_TARGET::TOUCHDOWN ? CONTACT_TABLE::TOUCHDOWNS : CONTACT_TABLE::LIFTOFFS;

	// The record: the departure (whose airport is the slot itself) or the
	// earliest unresolved liftoff marker/touchdown -- lookups resolve in order.
	AIRPORT* record_airport = nullptr;
	int record_db_id = -1;
	const DATETIME* record_time = nullptr;
	if (departure) {
		record_airport = &status->departure;
		record_db_id = status->flight.departure_db_id;
		record_time = &status->flight.departure_data.time_local;
	} else {
		CONTACT_RECORD* record = target == LOOKUP_TARGET::LIFTOFF
			? static_cast<CONTACT_RECORD*>(first_unresolved(status->flight.liftoff_data))
			: first_unresolved(status->flight.touchdown_data);
		if (record != NULL) {
			record_airport = &record->airport;
			record_db_id = record->db_id;
			record_time = &record->flight_data.time_local;
		}
	}
	const std::string time = record_time != nullptr ? record_time->format_date_time() : std::string("unknown time");
	// Marks the record resolved without a runway: the departure by its
	// runway_act.index, a list record by distances[0] (see first_unresolved()).
	auto mark_no_runway = [&]() {
		if (departure)
			record_airport->runway_act.index = -2;
		else if (record_airport != nullptr)
			record_airport->runway_act.distances[0] = -2;
	};
	const std::string lat = status->lookup.coordinate.coordinate_decimal_to_dms(COORDINATE::LATITUDE);
	const std::string lon = status->lookup.coordinate.coordinate_decimal_to_dms(COORDINATE::LONGITUDE);

	switch (outcome) {
	case LOOKUP_OUTCOME::RUNWAY:
	case LOOKUP_OUTCOME::AIRPORT: {
		const bool with_runway = outcome == LOOKUP_OUTCOME::RUNWAY;
		const std::string runway_code = with_runway ? slot->runway_code_generator() : std::string();
		const char* runway = with_runway ? runway_code.c_str() : nullptr;
		if (with_runway)
			gui_log_printf(status, GUI_LOG_INFO, text.runway, slot->name, slot->icao, runway, time.c_str());
		else
			gui_log_printf(status, GUI_LOG_INFO, text.airport, slot->name, slot->icao, lat.c_str(), lon.c_str(), time.c_str());
		// Resolve the record in memory before any write: a write that throws
		// must not leave it unresolved, or ending this lookup would request
		// the same record again, for as long as the database error lasts.
		if (record_airport != nullptr) {
			if (!departure) {
				// The list record keeps its own copy: the slot is reused by the
				// next lookup.
				if (with_runway) {
					record_airport->copy(slot);
				} else {
					memcpy(record_airport->icao, slot->icao, sizeof(slot->icao));
					memcpy(record_airport->name, slot->name, sizeof(slot->name));
				}
			}
			if (!with_runway)
				mark_no_runway();
		}
		if (departure)
			db_set_trip_airport(status, status->id_trip, TRIP_END::DEPARTURE, status->departure, runway);
		else if (target == LOOKUP_TARGET::TOUCHDOWN)
			db_set_trip_airport(status, status->id_trip, TRIP_END::DESTINATION, status->destination, runway);
		if (record_airport != nullptr) {
			if (record_db_id < 0) {
				// The immediate INSERT when it happened never got a valid rowid
				// (e.g. it hit SQLITE_BUSY and threw) -- "WHERE id=?" would just
				// match nothing, so skip it and say why instead.
				if (with_runway)
					gui_log_printf(status, GUI_LOG_WARNING, text.runway_lost, slot->name, slot->icao, runway);
				else
					gui_log_printf(status, GUI_LOG_WARNING, text.airport_lost, slot->name, slot->icao);
			} else {
				db_set_contact_airport(status, table, record_db_id, *record_airport, runway);
			}
			gui_notify_trip_updated(status);
		}
		// Cleared even with another record waiting: its lookup starts only
		// after this one ends, and writes the slot's ICAO and region before
		// reading them (airport_lookup.cpp).
		if (with_runway && !departure)
			slot->clear();
		break;
	}
	case LOOKUP_OUTCOME::NO_AIRPORT:
		// Coordinate-only: the record's row already has NULL airport fields
		// from its immediate INSERT.
		gui_log_printf(status, GUI_LOG_INFO, text.no_airport, lat.c_str(), lon.c_str(), time.c_str());
		mark_no_runway();  // before the write, as above
		if (target == LOOKUP_TARGET::TOUCHDOWN)
			db_clear_trip_destination_airport(status, status->id_trip);
		if (record_airport != nullptr)
			gui_notify_trip_updated(status);
		break;
	case LOOKUP_OUTCOME::FAILED:
		mark_no_runway();
		if (record_airport != nullptr)
			gui_notify_trip_updated(status);
		break;
	}
}

// One decoded sample from the simulator (every sim frame): updates the
// latest flight data, starts/stops the trip, detects liftoffs and
// touchdowns, and queues the sample for trip_data every sample_interval_ms.
void flight_on_sample(struct STATUS* status, const FLIGHT_DATA_RECORD& tmp) {
	status->data.altitude = (int)tmp.plane_altitude;
	status->data.heading = (int)tmp.plane_heading_degrees_magnetic;
	status->data.speed = (int)tmp.airspeed_indicated;
	status->data.vertical_speed = (int)tmp.vertical_speed;
	status->data.bank = tmp.plane_bank_degrees;
	status->data.pitch = tmp.plane_pitch_degrees;
	status->data.g_force = tmp.g_force;
	status->data.coordinate = tmp.plane_coordinate;
	status->data.time_zulu = tmp.time_zulu;
	status->data.time_local = tmp.time_local;
	// Only touchdown/destination runway-end matching uses this -- see
	// the bearing_tra computation in lookup_on_facility_data_end()
	// (airport_lookup.cpp) for why departure/liftoff never can (their
	// only candidate crossing of this band happens during climb-out,
	// after liftoff, not before).
	if (tmp.radio_height > 50 && tmp.radio_height < 100) {
		status->flight.loc_dh.latitude = tmp.plane_coordinate.latitude;
		status->flight.loc_dh.longitude = tmp.plane_coordinate.longitude;
	} else if (tmp.radio_height >= 100 && status->flight.loc_dh.latitude != 360) {
		// Invalidate once the aircraft climbs clear of the capture band above,
		// so a touchdown can never reuse a position frozen on a different,
		// earlier low pass (e.g. an earlier circuit/go-around in the same
		// trip). This runs every sim frame (SIMCONNECT_PERIOD_SIM_FRAME,
		// registered once at connect time via SimConnect_RequestDataOnSimObject),
		// so the latitude != 360 guard clears it exactly
		// once per climb-out rather than every frame for the rest of the time
		// spent above 100ft -- most of the flight. If this trip's next descent
		// happens to skip resampling inside the 50-100ft band (a sim-frame
		// hitch/stall), lookup_on_facility_data_end()'s approach.latitude
		// != 360 check then falls back to heading-based bearing instead of
		// silently reusing this now-cleared, stale position.
		status->flight.loc_dh.clear();
	}
	if (status->sim_running && !status->paused && tmp.surface_type != 255) {
		status->in_sim = TRUE;
		if ((bool)tmp.sim_on_ground) {
			if (anyEngineCombusting(tmp)) {
				if (!status->recording && status->recording_enabled) {
					status->recording = TRUE;
					gui_log_printf(status, GUI_LOG_INFO, "Recording started");

					// No queue state to reset here -- sample_write_queue is shared
					// across trips by design (see stop_recording()'s end-of-trip
					// barrier), so a previous trip's still-draining samples are
					// simply ahead of this trip's in line, not something this trip
					// needs to wait for or clear out.
					if (status->flight.last_sample != NULL) {
						free(status->flight.last_sample);
						status->flight.last_sample = NULL;
					}

					status->departure.clear();
					status->destination.clear();
					// Cleared too, like departure/destination: if a liftoff-marker
					// lookup targeting this object is still in flight when this trip
					// ends, the target reset below makes facility_lookup_target()
					// resolve that lookup's late response to &status->destination
					// instead, and nothing else would free this object's runways
					// buffer.
					status->lookup.liftoff_scratch.clear();
					// Flood-detection state (status->event_filter) is deliberately
					// NOT reset here -- it isn't trip-scoped. Each entry's own
					// quiet period resolves it regardless of trip boundaries,
					// and each held occurrence keeps the trip it happened in.
					status->flight.departure_lookup_initiated = FALSE;
					// Every call site that starts a lookup sets the target fresh (see
					// lookup.target in types.h), but a lookup still in flight when
					// this trip boundary is crossed reads it again when its stale
					// response arrives; see the liftoff_scratch.clear() above.
					status->lookup.target = LOOKUP_TARGET::TOUCHDOWN;
					status->flight.departure_db_id = -1;
					// A go-around or bounced landing from a previous trip can leave
					// this set -- between trips (engines off, on the ground) radio_height
					// stays well under 100ft, so neither of the two automatic resets above
					// (a fresh 50-100ft crossing, or climbing back above 100ft -- see the
					// loc_dh handling at the top of this function) reliably fires during that window.
					// Without clearing it here, if this trip's first flight stays below the
					// 50-100ft band (a low hop), its first touchdown's runway bearing would come from
					// the previous trip's stale position. (Departure and liftoff lookups
					// never use loc_dh.)
					status->flight.loc_dh.clear();
					status->flight.next_facility_lookup_seq = 0;
					status->flight.airborne = !(bool)tmp.sim_on_ground;

					// If this throws, status->id_trip is never assigned -- recording
					// must not stay TRUE in that case, or every sample from here on
					// gets tagged with whatever stale id_trip was left over (-1, or
					// an already-ended previous trip) instead of a real one. Reverting
					// to FALSE makes this same block retry on the next sample tick
					// (engine/on-ground state is unchanged) rather than silently
					// recording under the wrong trip for the rest of the flight.
					try {
						status->id_trip = db_insert_trip(status, tmp);
						gui_notify_recording_changed(status, true, status->id_trip);
					} catch (const db_exception& e) {
						status->recording = FALSE;
						gui_log_printf(status, GUI_LOG_WARNING, "Recording start failed (trip insert): %s", e.message.c_str());
					}
				}
			} else {
				if (status->recording)
					stop_recording(status);
			}
		}
	}
	if (status->recording && !status->paused) {
		// Liftoff
		// Guarded by departure_lookup_initiated (set right below), not by
		// departure.runway_act.index == -1 -- that field only flips once the
		// departure lookup actually *resolves*, so a touch-and-go (or several
		// full-stop taxi-back-and-liftoffs) occurring before a slow lookup
		// resolves would otherwise still see -1 and be mistaken for a fresh
		// departure. See departure_lookup_initiated in types.h.
		if (!(bool)tmp.sim_on_ground && !status->flight.airborne && !status->flight.departure_lookup_initiated) {
			status->flight.departure_lookup_initiated = TRUE;
			status->flight.next_facility_lookup_seq++;
			// Captured now (the actual liftoff moment) regardless of whether
			// the lookup fires immediately below or is deferred -- see
			// departure_data in types.h.
			fill_contact_flight_data(status->flight.departure_data, tmp);
			gui_log_printf(status, GUI_LOG_TRACE, "Liftoff detected (trip %d): lat=%.6f, lon=%.6f, heading=%d",
				status->id_trip, status->flight.departure_data.coordinate.latitude,
				status->flight.departure_data.coordinate.longitude, status->flight.departure_data.heading);
			// Insert immediately so the row survives a crash before stop_recording.
			// Airport/runway fields are NULL until the facility callback resolves --
			// same immediate-INSERT-then-async-UPDATE pattern as touchdowns below.
			// Wrapped in its own try/catch (mirroring the trip-creation insert
			// above) so a transient DB failure reverts departure_lookup_initiated
			// instead of leaving it stuck TRUE with this trip's departure never
			// resolved or retried -- see departure_lookup_initiated in types.h.
			try {
				status->flight.departure_db_id = db_insert_contact(status, CONTACT_TABLE::LIFTOFFS, status->id_trip, status->flight.departure_data);
				gui_log_printf(status, GUI_LOG_TRACE, "Liftoff trip_liftoffs row inserted: db_id=%d", status->flight.departure_db_id);
				if (!status->lookup.pending) {
					gui_log_printf(status, GUI_LOG_TRACE, "Requesting departure facility lookup (trip %d)", status->id_trip);
					start_facility_lookup(status, LOOKUP_TARGET::DEPARTURE, status->flight.departure_data.coordinate, status->flight.departure_data.heading);
				} else {
					// A previous trip's lookup is still draining (see
					// FLIGHT_PHASE::departure_lookup_needed in types.h) -- this
					// departure only fires once per trip, so if the request is
					// skipped now it must be retried later rather than lost.
					gui_log_printf(status, GUI_LOG_TRACE, "Deferring departure facility lookup (trip %d): another lookup in flight", status->id_trip);
					status->flight.departure_lookup_needed = TRUE;
				}
			} catch (const db_exception& e) {
				status->flight.departure_lookup_initiated = FALSE;
				gui_log_printf(status, GUI_LOG_WARNING, "Liftoff detected (trip %d): trip_liftoffs insert failed, will retry on next liftoff: %s",
					status->id_trip, e.message.c_str());
			}
		} else if (!(bool)tmp.sim_on_ground && !status->flight.airborne) {
			// Subsequent liftoff (touch-and-go, or a full stop followed by a
			// taxi-back and another departure -- this code can't and doesn't
			// try to tell the two apart): this trip's departure is already
			// locked in (departure_lookup_initiated above), so this is
			// recorded purely as a liftoff *marker* occurrence -- same
			// immediate-INSERT-then-async-UPDATE pattern as a touchdown, but
			// with no trips.* update (a trip has exactly one departure).
			gui_log_printf(status, GUI_LOG_TRACE, "Liftoff detected (trip %d, subsequent): lat=%.6f, lon=%.6f, heading=%d",
				status->id_trip, tmp.plane_coordinate.latitude, tmp.plane_coordinate.longitude,
				(int)tmp.plane_heading_degrees_magnetic);
			bool inserted = false;
			record_contact(status, status->flight.liftoff_data, status->flight.liftoff_data_end,
				CONTACT_TABLE::LIFTOFFS, tmp, LIFTOFF_TEXT, inserted);
			if (inserted) {
				gui_notify_trip_updated(status);
				// No-op if a previous lookup (departure, an earlier touchdown, or
				// an earlier liftoff marker) is still resolving -- this liftoff
				// marker's lookup will be picked up automatically once that one completes,
				// via request_next_touchdown_facility_lookup().
				request_next_touchdown_facility_lookup(status);
			}
		}
		// Landing
		if ((bool)tmp.sim_on_ground && status->flight.airborne) {
			gui_log_printf(status, GUI_LOG_TRACE, "Touchdown detected (trip %d): lat=%.6f, lon=%.6f, heading=%d",
				status->id_trip, tmp.plane_coordinate.latitude, tmp.plane_coordinate.longitude,
				(int)tmp.plane_heading_degrees_magnetic);
			bool touchdown_inserted = false;
			if (TOUCHDOWN_DATA* touchdown = record_contact(status, status->flight.touchdown_data, status->flight.touchdown_data_end,
					CONTACT_TABLE::TOUCHDOWNS, tmp, TOUCHDOWN_TEXT, touchdown_inserted)) {
				// Freeze this touchdown's own low-altitude position now -- see
				// TOUCHDOWN_DATA::loc_dh in types.h for why status->flight.loc_dh itself
				// can't be trusted once this touchdown's facility lookup is queued.
				touchdown->loc_dh = status->flight.loc_dh;
				// The destination UPDATE, notify and facility-lookup request below
				// are deliberately outside record_contact()'s insert try: once the
				// row is durably inserted they must still run even if one of them
				// individually fails, so a transient error there can't suppress the
				// facility lookup for an already-persisted row.
				if (touchdown_inserted) {
					// Update the destination position in trips to reflect this landing.
					// A failure here only means the trip's destination lat/lon stays
					// stale -- it must not stop the facility lookup below from being
					// requested for the touchdown row, which is already persisted.
					try {
						db_set_trip_destination_position(status, status->id_trip, touchdown->flight_data.coordinate);
					} catch (const db_exception& e) {
						gui_log_printf(status, GUI_LOG_WARNING, "Landing (trip %d): failed to update trip destination: %s",
							status->id_trip, e.message.c_str());
					}
					gui_notify_trip_updated(status);
					// No-op if a previous lookup (this trip's departure, or an earlier
					// touchdown from a bounce/go-around) is still resolving -- this
					// touchdown's lookup will be picked up automatically once that one
					// completes, via request_next_touchdown_facility_lookup().
					request_next_touchdown_facility_lookup(status);
				}
			}
		}
		status->flight.airborne = !(bool)tmp.sim_on_ground;

		double delta_s = status->sample_interval_ms / 1000.0;
		if (status->flight.last_sample != NULL)
			delta_s = tmp.time_zulu.time_day - status->flight.last_sample->time_zulu.time_day;
		if (delta_s < 0)
			delta_s += 86400;
		if (delta_s >= status->sample_interval_ms / 1000.0) {
			struct FLIGHT_DATA_RECORD* pS = (struct FLIGHT_DATA_RECORD*)malloc(sizeof(struct FLIGHT_DATA_RECORD));
			if (pS == NULL) {
				gui_log_printf(status, GUI_LOG_WARNING, "malloc failed for sample record; dropping this sample");
				return;
			}
			memcpy(pS, &tmp, sizeof(struct FLIGHT_DATA_RECORD));
			gui_notify_sample(status);
			// pS is handed off to the DB-write worker below, which owns it
			// from here and frees it once flushed. Keep our own copy so the
			// next delta_s calculation (and stop_recording()'s destination-
			// time update) don't touch memory the worker thread may be
			// using or have already freed.
			if (status->flight.last_sample != NULL)
				free(status->flight.last_sample);
			status->flight.last_sample = (struct FLIGHT_DATA_RECORD*)malloc(sizeof(struct FLIGHT_DATA_RECORD));
			if (status->flight.last_sample != NULL)
				memcpy(status->flight.last_sample, pS, sizeof(struct FLIGHT_DATA_RECORD));
			else
				gui_log_printf(status, GUI_LOG_WARNING, "malloc failed for last_sample cache; next delta_s will use the default interval");
			status->sample_write_queue.push(pS, status->id_trip);
		}
	}
}
