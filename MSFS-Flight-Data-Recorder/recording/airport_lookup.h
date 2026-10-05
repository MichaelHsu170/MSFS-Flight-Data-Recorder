#pragma once

#include "types.h"
#include "simconnect_defs.h"

// Finds the airport (and, if possible, the runway) a departure, later
// liftoff or touchdown happened at, one lookup at a time, through SimConnect:
//   1. start_facility_lookup() requests the airport list;
//   2. lookup_on_airport_list() keeps the nearest AIRPORT_LOOKUP::TOP_N
//      real airports and requests the nearest one's runways;
//   3. lookup_on_facility_data() collects its runways and thresholds;
//   4. lookup_on_facility_data_end() matches the runways (match_runways()),
//      walking on to the next-nearest candidate when none matches, then
//      falls back to a runway-margin hit, the nearest airport within 5 km,
//      or no airport at all.
// The result goes to on_lookup_resolved() (flight_phase.cpp). State lives in
// STATUS::lookup; a response for a trip that has since ended is dropped.

// How a lookup ended, reported to on_lookup_resolved().
enum class LOOKUP_OUTCOME {
	RUNWAY,     // slot->runway_act is the matched runway
	AIRPORT,    // slot's icao/region/name are set, no runway matched
	NO_AIRPORT, // no airport near enough
	FAILED,     // SimConnect rejected a request (lookup_on_exception())
};

// Starts a lookup for target at position (where it happened) and heading.
// approach, for a touchdown, is its final-approach position (see
// AIRPORT_LOOKUP::approach); nullptr otherwise. Only one lookup is in flight
// at a time: callers check STATUS::lookup.pending first.
void start_facility_lookup(struct STATUS* status, LOOKUP_TARGET target, const COORDINATE& position, int heading,
	const COORDINATE* approach = nullptr);

// The AIRPORT slot the in-flight lookup fills: STATUS::departure,
// STATUS::destination (touchdowns) or AIRPORT_LOOKUP::liftoff_scratch --
// chosen by AIRPORT_LOOKUP::target, fixed when the lookup started.
AIRPORT* facility_lookup_target(struct STATUS* status);

// Folds one AIRPORT_LIST chunk into top, kept nearest-first by distance from
// position: only 4-letter idents (real airports -- longer ones are
// heliports, vertiports and the like) nearer than top's farthest entry.
void add_nearest_airports(AIRPORT_LOOKUP::CANDIDATE (&top)[AIRPORT_LOOKUP::TOP_N], COORDINATE position,
	const SIMCONNECT_DATA_FACILITY_AIRPORT* airports, int count);

// SimConnect responses belonging to the lookup (see MyDispatchProc).
void lookup_on_airport_list(struct STATUS* status, SIMCONNECT_RECV_AIRPORT_LIST* list);
void lookup_on_facility_data(struct STATUS* status, SIMCONNECT_RECV_FACILITY_DATA* data);
void lookup_on_facility_data_end(struct STATUS* status);
// A SimConnect exception: ends the in-flight lookup as FAILED if send_id is
// its latest request, since no normal response will ever arrive for it.
void lookup_on_exception(struct STATUS* status, DWORD send_id);

// Forgets any lookup state from a previous SimConnect connection.
void reset_airport_lookup(struct STATUS* status);

// Implemented by the flight phase (flight_phase.cpp):
// Applies a finished lookup's result for slot (see facility_lookup_target()).
void on_lookup_resolved(struct STATUS* status, AIRPORT* slot, LOOKUP_OUTCOME outcome);
// Starts the next waiting lookup, if any, once none is in flight.
void request_next_touchdown_facility_lookup(struct STATUS* status);
