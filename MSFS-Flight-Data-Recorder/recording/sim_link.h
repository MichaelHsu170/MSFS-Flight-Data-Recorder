#pragma once

#include "types.h"
#include "simconnect_defs.h"

// What the recorder asks SimConnect for, and how its answers are read:
// the flight data definition, the cockpit events (COCKPIT_EVENTS in
// simconnect_defs.h), their names, SimConnect's exception names, and the
// decoding of a raw sample. Holds no state; MyDispatchProc (recorder.cpp)
// routes the answers.

// Registers every FLIGHT_DATA_RECORD field but the last
// (time_zulu.timezone_offset, which SimConnect has no ZULU variable for), from
// FLIGHT_DATA_FIELDS (simconnect_defs.h), and requests the data every sim
// frame.
void add_flight_definition(HANDLE hSimConnect);

// Maps every cockpit event to its sim event and adds it to GROUP_1, logging
// each request's SendID at trace level.
void add_client_events(HANDLE hSimConnect);

// The name an EVENT_ID is logged and stored (trip_events.event) under, or
// nullptr for an id that isn't one (EVENT_ID_COUNT or above).
const char* event_name(DWORD id);

// The SIMCONNECT_EXCEPTION_ name without its prefix, or "UNKNOWN".
const char* simconnect_exception_name(DWORD exception);

// The size in bytes of a complete REQUEST_FLIGHT packet.
DWORD flight_sample_packet_size(const SIMCONNECT_RECV_SIMOBJECT_DATA* data);

// Copies one REQUEST_FLIGHT packet of cbData bytes into sample, flipping pitch
// and bank to aviation sign convention (positive = nose up / right bank).
// Returns false and leaves sample untouched when the packet is shorter than
// flight_sample_packet_size(), e.g. because this simulator version rejected
// one of the registered variables.
bool decode_flight_sample(const SIMCONNECT_RECV_SIMOBJECT_DATA* data, DWORD cbData, FLIGHT_DATA_RECORD& sample);
