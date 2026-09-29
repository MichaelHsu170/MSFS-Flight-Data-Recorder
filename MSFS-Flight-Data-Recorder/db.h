#pragma once

#include "types.h"
#include "db_connection.h"
#include "db_history.h"

// Thrown by every db_* write below when SQLite fails; the write is rolled
// back. message says which statement failed and why (also logged).
struct db_exception {
	std::string message;
	db_exception(const std::string& msg) : message(msg) {}
};

// Trip recording writes (recorder.cpp). Each is one transaction on
// status->sql, serialized with the writer threads through
// STATUS::mutex_db_commit.

// Which end of a trip a trips.* airport update is for.
enum class TRIP_END { DEPARTURE, DESTINATION };
// (CONTACT_TABLE, which table a liftoff/touchdown row lives in, is in
// db_history.h.)

// New trips row for a trip starting at departure (title, ATC identity,
// position and times); returns its id.
int db_insert_trip(STATUS* status, const FLIGHT_DATA_RECORD& departure);
void db_set_trip_destination_time(STATUS* status, int trip_id, const DATETIME& time_zulu, const DATETIME& time_local);
void db_set_trip_destination_position(STATUS* status, int trip_id, const COORDINATE& position);
// The trip's departure or destination airport (ICAO, region, name) and
// runway; runway nullptr stores NULL (airport found, no runway matched).
void db_set_trip_airport(STATUS* status, int trip_id, TRIP_END end, const AIRPORT& airport, const char* runway);
// No airport found for the destination: ICAO, runway and region become NULL
// (the name is left as it was).
void db_clear_trip_destination_airport(STATUS* status, int trip_id);
// New trip_liftoffs/trip_touchdowns row for one liftoff or touchdown, with
// NULL airport/runway until db_set_contact_airport(); returns its id.
// Touchdowns also store data.g_force.
int db_insert_contact(STATUS* status, CONTACT_TABLE table, int trip_id, const FLIGHT_DATA& data);
// The airport a liftoff/touchdown row resolved to. With a runway, also its
// heading and airport.runway_act's threshold/centerline distances (a
// liftoff's negative threshold distance is stored as -1; a touchdown's is
// kept, since landing before the threshold is meaningful). runway nullptr:
// ICAO and name only, the runway columns stay NULL.
void db_set_contact_airport(STATUS* status, CONTACT_TABLE table, int row_id, const AIRPORT& airport, const char* runway);

// Writes one event row to trip_events for trip_id (captured by the caller at
// enqueue time, not read from status->id_trip here -- see
// EVENT_QUEUE_ITEM::trip_id). event_seq is EVENT_QUEUE_ITEM::seq, stored so a
// later db_delete_events() can retract this exact row -- see
// EventFloodFilter (event_filter.h). Called only from event_write_worker, on
// the event-write worker thread; serializes with the DB-write worker thread
// (draining STATUS::sample_write_queue) through STATUS::mutex_db_commit.
void db_insert_event(STATUS* status, int trip_id, const char* event, const char* time_zulu, const char* time_local, unsigned long long event_seq);

// Retracts previously-inserted trip_events rows by event_seq -- the DB side of
// slow-flood confirmation (see EventFloodFilter in event_filter.h and
// event_output() in recorder.cpp). Matches on event_seq rather than
// name/timestamp specifically so it can never delete an unrelated row that
// happens to share the same event name or timestamp string (time_zulu/
// time_local are read from a periodically-refreshed snapshot, not captured
// per-event, so two distinct occurrences can share an identical timestamp).
// No-op if seqs is empty. Called only from event_write_worker, same threading
// contract as db_insert_event above.
void db_delete_events(STATUS* status, const std::vector<unsigned long long>& seqs);

// Creates the schema and migrates any missing columns on an ephemeral R/W
// connection. Called at app startup so read-only queries always see the
// current schema, even when the simulator has never connected this session.
void migrate_db();

// Writes the flight_data.db path into fn_db: the current working directory in
// Debug builds, the executable's directory in Release builds.
void resolve_db_path(char* fn_db, size_t len);

// Opens (creating if needed) the recorder's own write connection,
// status->sql, and starts the two DB-writer threads. The UI's connections
// are in db_connection.h.
void connect_db(struct STATUS* status);
