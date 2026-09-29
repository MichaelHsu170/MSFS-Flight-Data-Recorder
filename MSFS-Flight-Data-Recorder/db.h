#pragma once

#include "types.h"

struct db_exception {
	std::string message;
	db_exception(const std::string& msg) : message(msg) {}
};

void db_error(const char* stmt_txt, int sql_ret, char** errmsg);

void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, int value);
void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, long long value);
void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, double value);
void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, char* value);
void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, const char* value);

void db_insert_update_table(
	sqlite3* sql,
	const char* stmt_txt,
	void* data,
	struct STATUS* status,
	void* aux,
	void (*func)(sqlite3_stmt*, const char*, void*, struct STATUS*, void*),
	int* out_rowid = nullptr
);

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

void connect_db(struct STATUS* status);
sqlite3* connect_db_readonly();
// Read-write connection for explicit GUI write operations (e.g. deleting a
// trip). Does NOT create the database (SQLITE_OPEN_READWRITE only — no CREATE),
// so it fails cleanly if no database exists yet. Caller must sqlite3_close().
sqlite3* connect_db_readwrite();
