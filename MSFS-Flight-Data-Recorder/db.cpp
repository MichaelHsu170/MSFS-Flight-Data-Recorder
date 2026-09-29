#include "db.h"
#include "logger_c.h"
#include "simconnect_defs.h"
#include "gui_notify.h"
#include "trip_data_fields.h"

#include <string>
#include <vector>

namespace {

// trip_data's columns: the trip key, the three bool_group_<n> packs (see
// TRIP_DATA_BOOL_FIELDS), every TRIP_DATA_NUM_FIELDS column, then the two
// timestamps.
std::string trip_data_fields() {
	std::string fields = "trip INTEGER NOT NULL,"
		"bool_group_1 INTEGER NOT NULL,"
		"bool_group_2 INTEGER NOT NULL,"
		"bool_group_3 INTEGER NOT NULL,";
#define TRIP_NUM_COLUMN(dbColumn, memberExpr, sqlType) fields += #dbColumn " " #sqlType " NOT NULL,";
	TRIP_DATA_NUM_FIELDS(TRIP_NUM_COLUMN)
#undef TRIP_NUM_COLUMN
	fields += "zulu_time VARCHAR(32) NOT NULL,"
		"local_time VARCHAR(32) NOT NULL";
	return fields;
}

// The INSERT for one trip_data row, columns in trip_data_fields() order --
// db_write_worker() binds them in the same order.
std::string trip_data_insert() {
	std::string columns = "trip,bool_group_1,bool_group_2,bool_group_3,";
	int count = 4;
#define TRIP_NUM_NAME(dbColumn, memberExpr, sqlType) columns += #dbColumn ","; ++count;
	TRIP_DATA_NUM_FIELDS(TRIP_NUM_NAME)
#undef TRIP_NUM_NAME
	columns += "zulu_time,local_time";
	count += 2;
	std::string placeholders = "?";
	for (int i = 1; i < count; ++i)
		placeholders += ",?";
	return "INSERT INTO trip_data (" + columns + ") VALUES (" + placeholders + ");";
}

struct TableDef {
	const char* name;
	std::string fields; // the column definitions inside CREATE TABLE's parentheses
};

// Every table, in creation order.
const std::vector<TableDef>& database_tables() {
	static const std::vector<TableDef> tables = {
		{ "trips",
			"id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL UNIQUE,"
			"title VARCHAR(256) NOT NULL,"
			"atc_airline VARCHAR(64) NOT NULL,"
			"atc_flight_number VARCHAR(8) NOT NULL,"
			"atc_id VARCHAR(32) NOT NULL,"
			"atc_model VARCHAR(32) NOT NULL,"
			"atc_type VARCHAR(64) NOT NULL,"
			"departure_latitude REAL NOT NULL,"
			"departure_longitude REAL NOT NULL,"
			"departure_icao VARCHAR(4),"
			"departure_name VARCHAR(64),"
			"departure_region VARCHAR(2),"
			"departure_rwy VARCHAR(3),"
			"departure_zulu_time VARCHAR(32) NOT NULL,"
			"departure_local_time VARCHAR(32) NOT NULL,"
			"destination_latitude REAL,"
			"destination_longitude REAL,"
			"destination_icao VARCHAR(4),"
			"destination_name VARCHAR(64),"
			"destination_region VARCHAR(2),"
			"destination_rwy VARCHAR(3),"
			"destination_zulu_time VARCHAR(32),"
			"destination_local_time VARCHAR(32),"
			"group_id INTEGER" },
		{ "trip_data", trip_data_fields() },
		{ "trip_events",
			"trip INTEGER NOT NULL,"
			"event VARCHAR(32) NOT NULL,"
			"time_zulu VARCHAR(32) NOT NULL,"
			"time_local VARCHAR(32) NOT NULL,"
			// Slow-flood retraction key -- see EventFloodFilter (event_filter.h)
			// and db_delete_events() below. Not NOT NULL/UNIQUE: rows written by
			// builds older than this column's introduction migrate in with NULL here
			// (migrate_table_columns() strips NOT NULL from the ALTER path anyway),
			// and NULL rows are simply never matched by a "WHERE event_seq IN (...)"
			// delete, which is the correct behavior for them (nothing to retract).
			"event_seq INTEGER" },
		{ "trip_liftoffs",
			"id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL UNIQUE,"
			"trip INTEGER NOT NULL,"
			"airspeed_indicated INTEGER NOT NULL,"
			"vertical_speed INTEGER NOT NULL,"
			"plane_pitch_degrees REAL NOT NULL,"
			"plane_bank_degrees REAL NOT NULL,"
			"heading_indicator INTEGER NOT NULL,"
			"plane_latitude REAL NOT NULL,"
			"plane_longitude REAL NOT NULL,"
			"icao VARCHAR(4),"
			"airport_name VARCHAR(64),"
			"runway VARCHAR(3),"
			"runway_heading INTEGER,"
			"distance_length REAL,"
			"distance_width REAL,"
			"distance_length_percent REAL,"
			"distance_width_percent REAL,"
			"wind_direction INTEGER,"
			"wind_velocity INTEGER,"
			"time_zulu VARCHAR(32) NOT NULL,"
			"time_local VARCHAR(32) NOT NULL,"
			"analysis_report TEXT" },
		{ "trip_touchdowns",
			"id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL UNIQUE,"
			"trip INTEGER NOT NULL,"
			"airspeed_indicated INTEGER NOT NULL,"
			"vertical_speed INTEGER NOT NULL,"
			"g_force REAL NOT NULL,"
			"plane_pitch_degrees REAL NOT NULL,"
			"plane_bank_degrees REAL NOT NULL,"
			"heading_indicator INTEGER NOT NULL,"
			"plane_latitude REAL NOT NULL,"
			"plane_longitude REAL NOT NULL,"
			"icao VARCHAR(4),"
			"airport_name VARCHAR(64),"
			"runway VARCHAR(3),"
			"runway_heading INTEGER,"
			"distance_length REAL,"
			"distance_width REAL,"
			"distance_length_percent REAL,"
			"distance_width_percent REAL,"
			"wind_direction INTEGER,"
			"wind_velocity INTEGER,"
			"time_zulu VARCHAR(32) NOT NULL,"
			"time_local VARCHAR(32) NOT NULL,"
			"analysis_report TEXT" },
		{ "trip_groups",
			"id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL UNIQUE,"
			"name VARCHAR(64) NOT NULL,"
			"sort_order INTEGER NOT NULL DEFAULT 0" },
	};
	return tables;
}

}


void db_error(const char* stmt_txt, int sql_ret, char** errmsg) {
	std::string msg;
	if (sql_ret != 0) {
		msg = std::string("db operation \"") + stmt_txt + "\" failed with error " + std::to_string(sql_ret);
		log_c(1, "DB", msg.c_str());
	}
	if (errmsg != NULL && *errmsg != NULL) {
		msg = std::string("db operation \"") + stmt_txt + "\" failed with error " + *errmsg;
		log_c(1, "DB", msg.c_str());
		sqlite3_free(*errmsg);
		*errmsg = NULL;
	}
	throw db_exception(msg);
}

void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, int value) {
	int sql_ret = sqlite3_bind_int(stmt, index, value);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, long long value) {
	int sql_ret = sqlite3_bind_int64(stmt, index, value);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, double value) {
	int sql_ret = sqlite3_bind_double(stmt, index, value);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, char* value) {
	int sql_ret = sqlite3_bind_text(stmt, index, value, (int)strlen(value), SQLITE_TRANSIENT);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, const char* value) {
	int sql_ret = sqlite3_bind_text(stmt, index, value, (int)strlen(value), SQLITE_TRANSIENT);
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

void db_insert_update_table(
	sqlite3* sql,
	const char* stmt_txt,
	void* data,
	struct STATUS* status,
	void* aux,
	void (*func)(sqlite3_stmt*, const char*, void*, struct STATUS*, void*),
	int* out_rowid
) {
	status->mutex_db_commit.lock();
	sqlite3_stmt* stmt = NULL;
	int sql_ret = 0;
	char* errmsg = NULL;
	try {
		sql_ret = sqlite3_exec(sql, "BEGIN TRANSACTION", NULL, NULL, &errmsg);
		if (errmsg != NULL)
			db_error(stmt_txt, 0, &errmsg);
		sql_ret = sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, NULL);
		if (sql_ret)
			db_error(stmt_txt, sql_ret, NULL);
		func(stmt, stmt_txt, data, status, aux);
		sql_ret = sqlite3_step(stmt);
		if (sql_ret != SQLITE_DONE)
			db_error(stmt_txt, sql_ret, NULL);
		sql_ret = sqlite3_reset(stmt);
		if (sql_ret)
			db_error(stmt_txt, sql_ret, NULL);
		sql_ret = sqlite3_exec(sql, "COMMIT TRANSACTION", NULL, NULL, &errmsg);
		if (errmsg != NULL)
			db_error(stmt_txt, 0, &errmsg);
		// Only capture the rowid once the transaction is durably committed --
		// reading it right after sqlite3_step() would give the caller a rowid
		// for a row that a later reset/commit failure could still roll back.
		if (out_rowid)
			*out_rowid = (int)sqlite3_last_insert_rowid(sql);
		// The transaction is durably committed at this point, so a finalize
		// failure here is only a statement-cleanup error, not a write failure.
		// It must not be reported as one via db_error/throw -- db_write_worker's
		// catch block logs any exception from this function as "dropped one
		// sample", which would misreport data that was in fact saved.
		sql_ret = sqlite3_finalize(stmt);
		stmt = NULL;
		if (sql_ret)
			log_cf(1, "DB", "db_insert_update_table: finalize failed after commit (data already saved): %s", sqlite3_errmsg(sql));
	}
	catch (...) {
		// Catch-all, not just db_exception -- func is a caller-supplied callback
		// that could throw something else entirely, and mutex_db_commit must be
		// released (and the transaction rolled back) either way, or every later
		// call deadlocks/finds a transaction still open.
		if (stmt != NULL)
			sqlite3_finalize(stmt);
		sqlite3_exec(sql, "ROLLBACK TRANSACTION", NULL, NULL, NULL);
		status->mutex_db_commit.unlock();
		throw;
	}
	status->mutex_db_commit.unlock();
}

void db_insert_event(STATUS* status, int trip_id, const char* event, const char* time_zulu, const char* time_local, unsigned long long event_seq) {
	struct EventInsertArgs { int trip_id; const char* event; const char* time_zulu; const char* time_local; long long event_seq; };
	EventInsertArgs args{ trip_id, event, time_zulu, time_local, (long long)event_seq };
	// The flood filter this is fed from (STATUS::event_filter, see
	// event_filter.h) can hold an occurrence back for several seconds after
	// the trip it belongs to already ended, so by the
	// time this runs the trip may have already been deleted from the UI. The
	// WHERE EXISTS guard, evaluated inside this same transaction, makes that a
	// silent no-op instead of an orphaned row: if the trip's gone by the time
	// this commits, 0 rows are affected; if it isn't gone yet, deleteTripData's
	// own "DELETE FROM trip_events WHERE trip = ?" sweeps this row up normally
	// either way, so there's no race to lose either way it lands.
	db_insert_update_table(status->sql,
		"INSERT INTO trip_events (trip,event,time_zulu,time_local,event_seq) "
		"SELECT ?,?,?,?,? WHERE EXISTS (SELECT 1 FROM trips WHERE id = ?);",
		(void*)&args, status, NULL,
		[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
			EventInsertArgs* a = (EventInsertArgs*)data;
			db_bind(stmt, stmt_txt, 1, a->trip_id);
			db_bind(stmt, stmt_txt, 2, a->event);
			db_bind(stmt, stmt_txt, 3, a->time_zulu);
			db_bind(stmt, stmt_txt, 4, a->time_local);
			db_bind(stmt, stmt_txt, 5, a->event_seq);
			db_bind(stmt, stmt_txt, 6, a->trip_id);
		}
	);
}

void db_delete_events(STATUS* status, const std::vector<unsigned long long>& seqs) {
	if (seqs.empty())
		return;
	std::string placeholders;
	for (size_t i = 0; i < seqs.size(); i++)
		placeholders += (i == 0) ? "?" : ",?";
	std::string stmt_txt = "DELETE FROM trip_events WHERE event_seq IN (" + placeholders + ");";
	db_insert_update_table(status->sql,
		stmt_txt.c_str(),
		(void*)&seqs, status, NULL,
		[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
			const std::vector<unsigned long long>* seqs = (const std::vector<unsigned long long>*)data;
			for (size_t i = 0; i < seqs->size(); i++)
				db_bind(stmt, stmt_txt, (int)(i + 1), (long long)(*seqs)[i]);
		}
	);
}

// Runs on the single persistent event-write worker thread (started in
// connect_db(), joined in wait_for_db_writers()) draining
// STATUS::event_write_queue. Moves db_insert_event's/db_delete_events'
// synchronous transaction (including its fsync on commit) off the SimConnect
// dispatch thread -- see EVENT_QUEUE_ITEM/EventWriteQueue in types.h. Insert
// and Delete items share this one queue/worker specifically so a tier-2
// retraction can never be dequeued and executed ahead of the inserts that
// created the rows it's retracting.
static void event_write_worker(STATUS* status) {
	log_cf(3, "DB", "event_write_worker: thread started");
	EVENT_QUEUE_ITEM item;
	while (status->event_write_queue.pop(item)) {
		try {
			if (item.kind == EVENT_QUEUE_ITEM::Kind::Insert)
				db_insert_event(status, item.trip_id, item.event.c_str(), item.time_zulu.c_str(), item.time_local.c_str(), item.seq);
			else
				db_delete_events(status, item.delete_seqs);
		}
		catch (const db_exception& e) {
			// item.trip_id is only ever set for Insert items (see
			// EVENT_QUEUE_ITEM::trip_id in types.h) and a Delete can retract more
			// than one row at once, so the two kinds need distinct messages here.
			if (item.kind == EVENT_QUEUE_ITEM::Kind::Insert)
				gui_log_printf(status, GUI_LOG_WARNING, "event_write_worker: dropped one event for trip %d: %s", item.trip_id, e.message.c_str());
			else
				gui_log_printf(status, GUI_LOG_WARNING, "event_write_worker: failed to retract %zu event(s): %s", item.delete_seqs.size(), e.message.c_str());
			// A dropped Insert was already shown in the Live Status list (see
			// commit_event() in recorder.cpp, which notifies the UI before this
			// write is even queued) -- pull it back out so the panel doesn't
			// keep showing an occurrence that never made it into trip_events.
			// Not needed for a dropped Delete: nothing was ever added to the UI
			// for a retraction, only removed, so there's nothing to undo.
			if (item.kind == EVENT_QUEUE_ITEM::Kind::Insert)
				gui_notify_events_retracted(status, &item.seq, 1);
		}
		catch (...) {
			if (item.kind == EVENT_QUEUE_ITEM::Kind::Insert)
				gui_log_printf(status, GUI_LOG_WARNING, "event_write_worker: dropped one event for trip %d: unknown exception", item.trip_id);
			else
				gui_log_printf(status, GUI_LOG_WARNING, "event_write_worker: failed to retract %zu event(s): unknown exception", item.delete_seqs.size());
			if (item.kind == EVENT_QUEUE_ITEM::Kind::Insert)
				gui_notify_events_retracted(status, &item.seq, 1);
		}
	}
	log_cf(3, "DB", "event_write_worker: queue stopped; thread exiting");
}

// Runs on the single persistent DB-write worker thread (started in
// connect_db(), joined in wait_for_db_writers()) draining
// STATUS::sample_write_queue. This is the only thread that ever flushes
// samples through STATUS::sql, so there is no possibility of two writer
// threads racing each other, or a new trip's samples being interleaved with
// (or lost during) a previous trip's flush.
static void db_write_worker(STATUS* status) {
	log_cf(3, "DB", "db_write_worker: thread started");
	const std::string insert_txt = trip_data_insert();
	SAMPLE_QUEUE_ITEM item;
	while (status->sample_write_queue.pop(item)) {
		if (item.data == NULL) {
			// End-of-trip barrier pushed by stop_recording() -- every sample
			// for this trip was pushed (and so flushed) ahead of it in the
			// queue, so it's now safe to announce the trip as finished.
			gui_log_printf(status, GUI_LOG_INFO, "Recording stopped");
			gui_notify_recording_changed(status, false, item.trip_id);
			// This trip's data is now fully flushed -- safe to delete.
			{
				std::lock_guard<std::mutex> lock(status->flushing_trip_ids_mutex);
				status->flushing_trip_ids.erase(item.trip_id);
			}
			continue;
		}
		struct FLIGHT_DATA_RECORD* pS = item.data;
		// db_insert_update_table already rolls back and rethrows on failure;
		// the try/catch is scoped to just this one call (not the whole loop)
		// so a single bad row is dropped and the worker keeps draining the
		// queue -- see the catch below for why that matters. No caller here
		// can catch an escaping exception either -- letting one propagate
		// calls std::terminate and kills the whole app mid-flight.
		try {
			db_insert_update_table(status->sql, insert_txt.c_str(), pS, status, (void*)&item.trip_id,
				[](sqlite3_stmt* stmt, const char* stmt_txt, void* data, struct STATUS* status, void* aux) {
					struct FLIGHT_DATA_RECORD* pS = (struct FLIGHT_DATA_RECORD*)data;
					const std::array<uint32_t, 4> bool_groups = tripBoolGroups(*pS);
					int index = 1;
					db_bind(stmt, stmt_txt, index++, *(int*)aux);
					for (int group = 1; group <= 3; group++)
						db_bind(stmt, stmt_txt, index++, (int)bool_groups[group]);
#define TRIP_NUM_BIND(dbColumn, memberExpr, sqlType) db_bind(stmt, stmt_txt, index++, pS->memberExpr);
					TRIP_DATA_NUM_FIELDS(TRIP_NUM_BIND)
#undef TRIP_NUM_BIND
					db_bind(stmt, stmt_txt, index++, pS->time_zulu.format_date_time().c_str());
					db_bind(stmt, stmt_txt, index++, pS->time_local.format_date_time().c_str());
				});
		}
		catch (const db_exception& e) {
			// Drop just this one sample and keep draining the queue, rather
			// than stalling every later sample (this trip's and any future
			// trip's) behind one bad row.
			gui_log_printf(status, GUI_LOG_WARNING, "db_write_worker: dropped one sample for trip %d: %s", item.trip_id, e.message.c_str());
		}
		catch (...) {
			// db_insert_update_table's own catch is now catch(...) too (a bound
			// callback could throw anything), so this must be as well -- otherwise
			// a non-db_exception would escape the worker thread and terminate
			// the app instead of just dropping one sample.
			gui_log_printf(status, GUI_LOG_WARNING, "db_write_worker: dropped one sample for trip %d: unknown exception", item.trip_id);
		}
		free(pS);
	}
	log_cf(3, "DB", "db_write_worker: queue stopped; thread exiting");
}

void resolve_db_path(char* fn_db, size_t len) {
#ifdef _DEBUG
	snprintf(fn_db, len, "%s.db", DATABASE_NAME);
#else
	char exe_path[MAX_PATH];
	GetModuleFileNameA(NULL, exe_path, MAX_PATH);
	char* last_slash = strrchr(exe_path, '\\');
	if (last_slash)
		*last_slash = '\0';
	snprintf(fn_db, len, "%s\\%s.db", exe_path, DATABASE_NAME);
#endif
}

// A second, independent connection for read-only history queries (Trip History
// feature). connect_db()'s connection is opened with SQLITE_OPEN_NOMUTEX, so it
// is only safe to use from the single thread that owns it (the dispatch loop /
// db_write_worker thread) — it must never be shared with a background query thread.
// This connection is opened without NOMUTEX, so SQLite's own per-connection
// mutex makes it safe to call from whichever single thread is using it at a time.
sqlite3* connect_db_readwrite() {
	char fn_db[MAX_PATH];
	resolve_db_path(fn_db, MAX_PATH);
	sqlite3* sql = NULL;
	if (sqlite3_open_v2(fn_db, &sql, SQLITE_OPEN_READWRITE, NULL) != SQLITE_OK) {
		if (sql != NULL)
			sqlite3_close(sql);
		return NULL;
	}
	sqlite3_busy_timeout(sql, 5000);
	return sql;
}

sqlite3* connect_db_readonly() {
	char fn_db[MAX_PATH];
	resolve_db_path(fn_db, MAX_PATH);

	sqlite3* sql = NULL;
	if (sqlite3_open_v2(fn_db, &sql, SQLITE_OPEN_READONLY, NULL) != SQLITE_OK) {
		if (sql != NULL)
			sqlite3_close(sql);
		return NULL;
	}
	sqlite3_busy_timeout(sql, 2000);
	return sql;
}

// Remove constraint keywords that SQLite disallows in ALTER TABLE ADD COLUMN:
// NOT NULL (requires a DEFAULT when rows exist), PRIMARY KEY, UNIQUE, AUTOINCREMENT.
// The added column defaults to NULL for any existing rows, which is fine — the
// app reads integer/real columns as 0 and text columns as empty when NULL.
static void strip_alter_column_constraints(const char* src, char* dst, int dst_size) {
	static const char* kws[] = { "NOT NULL", "PRIMARY KEY", "AUTOINCREMENT", "UNIQUE", nullptr };
	strncpy(dst, src, (size_t)(dst_size - 1));
	dst[dst_size - 1] = '\0';
	for (int i = 0; kws[i]; i++) {
		int klen = (int)strlen(kws[i]);
		char* p;
		while ((p = strstr(dst, kws[i])) != nullptr)
			memmove(p, p + klen, strlen(p + klen) + 1);
	}
}

// For each column in fields_def (comma-separated column definitions) that is
// absent from table_name, run ALTER TABLE ADD COLUMN. Called from
// create_schema() on a write connection, never from the readonly path.
static void migrate_table_columns(sqlite3* sql, const char* table_name, const char* fields_def) {
	// Collect existing column names via PRAGMA table_info.
	char pragma_buf[128];
	snprintf(pragma_buf, sizeof(pragma_buf), "PRAGMA table_info(%s);", table_name);
	sqlite3_stmt* stmt;
	if (sqlite3_prepare_v2(sql, pragma_buf, -1, &stmt, nullptr) != SQLITE_OK)
		return;

	const int MAX_COLS = 256;
	const int NAME_LEN = 64;
	char existing[MAX_COLS][NAME_LEN];
	int nexist = 0;
	while (sqlite3_step(stmt) == SQLITE_ROW && nexist < MAX_COLS) {
		const char* n = (const char*)sqlite3_column_text(stmt, 1); // column 1 = name
		if (n) {
			strncpy(existing[nexist], n, NAME_LEN - 1);
			existing[nexist][NAME_LEN - 1] = '\0';
			nexist++;
		}
	}
	sqlite3_finalize(stmt);

	// Walk fields_def, splitting by comma while respecting parentheses.
	int flen = (int)strlen(fields_def);
	char* buf = (char*)malloc((size_t)(flen + 1));
	if (!buf) return;
	memcpy(buf, fields_def, (size_t)(flen + 1));

	int depth = 0, seg_start = 0;
	for (int i = 0; i <= flen; i++) {
		char c = buf[i];
		if      (c == '(') { depth++; continue; }
		else if (c == ')') { depth--; continue; }
		else if ((c == ',' || c == '\0') && depth == 0) {
			buf[i] = '\0';
			char* seg = buf + seg_start;
			while (*seg == ' ' || *seg == '\t' || *seg == '\n') seg++;

			// Extract column name (first whitespace-delimited word).
			char col_name[NAME_LEN] = {};
			int k = 0;
			while (seg[k] && seg[k] != ' ' && seg[k] != '\t' && k < NAME_LEN - 1) {
				col_name[k] = seg[k];
				k++;
			}

			if (col_name[0] != '\0') {
				bool found = false;
				for (int m = 0; m < nexist && !found; m++)
					found = (strcmp(existing[m], col_name) == 0);

				if (!found) {
					char safe_def[512];
					strip_alter_column_constraints(seg, safe_def, (int)sizeof(safe_def));

					char alter_sql[640];
					snprintf(alter_sql, sizeof(alter_sql),
						"ALTER TABLE %s ADD COLUMN %s;", table_name, safe_def);

					sqlite3_stmt* alter_stmt;
					if (sqlite3_prepare_v2(sql, alter_sql, -1, &alter_stmt, nullptr) == SQLITE_OK) {
						int step_ret = sqlite3_step(alter_stmt);
						sqlite3_finalize(alter_stmt);
						if (step_ret == SQLITE_DONE)
							log_cf(2, "DB", "Schema migration: %s — added column %s", table_name, col_name);
						else
							log_cf(0, "DB", "Schema migration failed (%s): %s", sqlite3_errmsg(sql), alter_sql);
					} else {
						log_cf(1, "DB", "Schema migration failed (%s): %s", sqlite3_errmsg(sql), alter_sql);
					}
				}
			}

			seg_start = i + 1;
		}
	}
	free(buf);
}

// Part of create_schema(), so Trip History reads stay indexed even when the
// schema was created/updated by migrate_db() alone (SimConnect never
// connected this session -- see migrate_db()'s doc comment in db.h).
// trip_data/trip_events/trip_touchdowns are all queried with "WHERE trip = ?"
// (db_history.cpp) -- without an index that's a full table scan across every
// sample ever recorded, for every trip load. trips itself has no such index
// need: its only query is an unfiltered "ORDER BY id DESC" over the whole
// table, and id is already the PRIMARY KEY. IF NOT EXISTS makes this safe to
// run on every connect/migrate, same as create_schema()'s CREATE TABLEs; the
// one-time build cost for an existing large database is paid back on every
// load after. Failures are logged but never fatal -- a missing index only
// degrades query performance (full table scan), it doesn't stop the app from
// reading/writing trips.
static void create_db_indexes(sqlite3* sql) {
	static const char* index_stmts[] = {
		"CREATE INDEX IF NOT EXISTS idx_trip_data_trip ON trip_data(trip);",
		"CREATE INDEX IF NOT EXISTS idx_trip_events_trip ON trip_events(trip);",
		// Backs db_delete_events()'s "WHERE event_seq IN (...)" retraction --
		// without it, confirming a slow flood (see event_filter.h) would
		// force a full table scan of trip_events every time.
		"CREATE INDEX IF NOT EXISTS idx_trip_events_event_seq ON trip_events(event_seq);",
		"CREATE INDEX IF NOT EXISTS idx_trip_liftoffs_trip ON trip_liftoffs(trip);",
		"CREATE INDEX IF NOT EXISTS idx_trip_touchdowns_trip ON trip_touchdowns(trip);",
		"CREATE INDEX IF NOT EXISTS idx_trips_group ON trips(group_id);",
		// Enforced here (not just by db_groups.cpp's own pre-check) as a
		// backstop against two writers racing between the pre-check SELECT and
		// the INSERT/UPDATE -- migrate_db() runs this at startup, ahead of any
		// writer connection, so the constraint is always in place before
		// insertGroup()/renameGroup() can run. NOTE: SQLite's COLLATE NOCASE
		// only case-folds ASCII A-Z/a-z, so this backstop -- unlike
		// db_groups.cpp's groupNameExists(), which does full Unicode-aware
		// comparison -- cannot catch a race between names that differ only in
		// non-ASCII casing (e.g. "münchen" vs "MÜNCHEN"). In practice this is
		// harmless: group creation/rename only ever runs on the single Qt GUI
		// thread, so there is no concurrent writer for it to race in the first
		// place.
		"CREATE UNIQUE INDEX IF NOT EXISTS idx_trip_groups_name ON trip_groups(name COLLATE NOCASE);",
	};
	for (int i = 0; i < (int)(sizeof(index_stmts) / sizeof(char*)); i++) {
		sqlite3_stmt* stmt = nullptr;
		if (sqlite3_prepare_v2(sql, index_stmts[i], -1, &stmt, nullptr) == SQLITE_OK) {
			if (sqlite3_step(stmt) != SQLITE_DONE)
				log_cf(0, "DB", "Failed to create index \"%s\": %s", index_stmts[i], sqlite3_errmsg(sql));
		} else {
			log_cf(0, "DB", "Failed to prepare index \"%s\": %s", index_stmts[i], sqlite3_errmsg(sql));
		}
		sqlite3_finalize(stmt);
	}
}

// Creates any missing table, adds columns missing from tables made by an
// older build (see migrate_table_columns()) and creates the indexes. False if
// a table couldn't be created (already logged).
static bool create_schema(sqlite3* sql) {
	bool ok = true;
	for (const TableDef& table : database_tables()) {
		const std::string stmt_txt = std::string("CREATE TABLE IF NOT EXISTS ") + table.name + " (" + table.fields + ");";
		sqlite3_stmt* stmt = nullptr;
		if (sqlite3_prepare_v2(sql, stmt_txt.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
			log_cf(0, "DB", "Failed to prepare CREATE TABLE for %s: %s", table.name, sqlite3_errmsg(sql));
			ok = false;
		} else if (sqlite3_step(stmt) != SQLITE_DONE) {
			log_cf(0, "DB", "Failed to create table %s: %s", table.name, sqlite3_errmsg(sql));
			ok = false;
		}
		sqlite3_finalize(stmt);
	}
	for (const TableDef& table : database_tables())
		migrate_table_columns(sql, table.name, table.fields.c_str());
	create_db_indexes(sql);
	return ok;
}

void migrate_db() {
	char fn_db[MAX_PATH];
	resolve_db_path(fn_db, MAX_PATH);
	log_cf(3, "DB", "migrate_db: checking schema for %s", fn_db);
	sqlite3* sql = nullptr;
	if (sqlite3_open_v2(fn_db, &sql, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr) != SQLITE_OK) {
		log_cf(0, "DB", "migrate_db: cannot open database %s: %s", fn_db, sql ? sqlite3_errmsg(sql) : "unknown error");
		if (sql) sqlite3_close(sql);
		return;
	}
	sqlite3_busy_timeout(sql, 5000);
	create_schema(sql);
	log_cf(3, "DB", "migrate_db: schema check complete");
	sqlite3_close(sql);
}

void connect_db(struct STATUS* status) {
	char fn_db[MAX_PATH];
	resolve_db_path(fn_db, MAX_PATH);

	if (sqlite3_open_v2(fn_db, &status->sql, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX | SQLITE_OPEN_SHAREDCACHE, NULL) == SQLITE_OK)
		log_cf(2, "DB", "Opened database %s", fn_db);
	else {
		log_cf(0, "DB", "Cannot open database: %s", sqlite3_errmsg(status->sql));
		exit(1);
	}
	// Without this, a lock held by connect_db_readwrite() (group/delete-trip
	// operations from the Trip History UI) makes db_write_worker's writes fail
	// with SQLITE_BUSY immediately instead of waiting the few ms those short
	// operations actually take -- dropping recorded samples for no reason.
	sqlite3_busy_timeout(status->sql, 5000);

	if (!create_schema(status->sql))
		exit(2);

	// Start the single persistent DB-write worker for this connection. Reset
	// clears any stop() left over from a previous connection's shutdown, so
	// the freshly-started thread's pop() loop doesn't exit immediately.
	status->sample_write_queue.reset();
	log_cf(3, "DB", "connect_db: schema ready; starting db_write_worker");
	status->db_writer_thread = std::thread(db_write_worker, status);

	status->event_write_queue.reset();
	log_cf(3, "DB", "connect_db: schema ready; starting event_write_worker");
	status->event_writer_thread = std::thread(event_write_worker, status);
}
