#include "db.h"
#include "app_paths.h"
#include "logger_c.h"
#include "simconnect_defs.h"
#include "gui_notify.h"
#include "trip_data_fields.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

// trip_data's column definitions ("name TYPE [NOT NULL]"), in order: the
// trip key, the three bool_group_<n> packs (see TRIP_DATA_BOOL_FIELDS), every
// TRIP_DATA_NUM_FIELDS column, the engine power BLOBs (engine_power.h; NULL =
// not recorded), then the two timestamps.
const std::vector<std::string>& trip_data_columns() {
	static const std::vector<std::string> columns = [] {
		std::vector<std::string> c = { "trip INTEGER NOT NULL",
			"bool_group_1 INTEGER NOT NULL",
			"bool_group_2 INTEGER NOT NULL",
			"bool_group_3 INTEGER NOT NULL" };
#define TRIP_NUM_COLUMN(dbColumn, memberExpr, sqlType) c.push_back(#dbColumn " " #sqlType " NOT NULL");
		TRIP_DATA_NUM_FIELDS(TRIP_NUM_COLUMN)
#undef TRIP_NUM_COLUMN
		c.insert(c.end(), { "engine_speed BLOB", "engine_load BLOB",
			"zulu_time VARCHAR(32) NOT NULL", "local_time VARCHAR(32) NOT NULL" });
		return c;
	}();
	return columns;
}

// The column name a column definition starts with. Every definition here
// (database_tables(), trip_data_columns()) separates it with a space.
std::string column_name(const std::string& definition) {
	return definition.substr(0, definition.find(' '));
}

std::string trip_data_fields() {
	std::string fields;
	for (const std::string& definition : trip_data_columns())
		fields += (fields.empty() ? "" : ",") + definition;
	return fields;
}

// The INSERT for one trip_data row, columns in trip_data_columns() order --
// db_write_worker() binds them in the same order.
std::string trip_data_insert() {
	std::string columns, placeholders;
	for (const std::string& definition : trip_data_columns()) {
		columns += (columns.empty() ? "" : ",") + column_name(definition);
		placeholders += placeholders.empty() ? "?" : ",?";
	}
	return "INSERT INTO trip_data (" + columns + ") VALUES (" + placeholders + ");";
}

// trip_liftoffs' and trip_touchdowns' column definitions: the same columns,
// plus g_force (after vertical_speed) for touchdowns.
std::string contact_table_fields(CONTACT_TABLE table) {
	return std::string(
		"id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL UNIQUE,"
		"trip INTEGER NOT NULL,"
		"airspeed_indicated INTEGER NOT NULL,"
		"vertical_speed INTEGER NOT NULL,")
		+ (table == CONTACT_TABLE::TOUCHDOWNS ? "g_force REAL NOT NULL," : "")
		+ "plane_pitch_degrees REAL NOT NULL,"
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
		"analysis_report TEXT";
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
		{ contactTableName(CONTACT_TABLE::LIFTOFFS), contact_table_fields(CONTACT_TABLE::LIFTOFFS) },
		{ contactTableName(CONTACT_TABLE::TOUCHDOWNS), contact_table_fields(CONTACT_TABLE::TOUCHDOWNS) },
		{ "trip_groups",
			"id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL UNIQUE,"
			"name VARCHAR(64) NOT NULL,"
			"sort_order INTEGER NOT NULL DEFAULT 0" },
	};
	return tables;
}

}

// Logs and throws db_exception for a failed statement, naming the error
// message from sqlite3_exec (freed here) if given, else the SQLite error code.
static void db_error(const char* stmt_txt, int sql_ret, char** errmsg) {
	std::string error = std::to_string(sql_ret);
	if (errmsg != NULL && *errmsg != NULL) {
		error = *errmsg;
		sqlite3_free(*errmsg);
		*errmsg = NULL;
	}
	const std::string msg = std::string("db operation \"") + stmt_txt + "\" failed with error " + error;
	log_c(1, "DB", msg.c_str());
	throw db_exception(msg);
}

// Reports a failed sqlite3_bind_*() result (with db_error(); throws).
static void check_bind(const char* stmt_txt, int sql_ret) {
	if (sql_ret)
		db_error(stmt_txt, sql_ret, NULL);
}

static void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, int value) {
	check_bind(stmt_txt, sqlite3_bind_int(stmt, index, value));
}

static void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, long long value) {
	check_bind(stmt_txt, sqlite3_bind_int64(stmt, index, value));
}

static void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, double value) {
	check_bind(stmt_txt, sqlite3_bind_double(stmt, index, value));
}

static void db_bind(sqlite3_stmt* stmt, const char* stmt_txt, int index, const char* value) {
	check_bind(stmt_txt, sqlite3_bind_text(stmt, index, value, (int)strlen(value), SQLITE_TRANSIENT));
}

// value, or NULL if value is nullptr.
static void db_bind_text_or_null(sqlite3_stmt* stmt, const char* stmt_txt, int index, const char* value) {
	if (value != nullptr) {
		db_bind(stmt, stmt_txt, index, value);
		return;
	}
	check_bind(stmt_txt, sqlite3_bind_null(stmt, index));
}

// The first count values as a trip_data.engine_speed/engine_load BLOB (see
// packEngineValues()), or NULL if count is 0.
static void db_bind_engine_values(sqlite3_stmt* stmt, const char* stmt_txt, int index, const std::array<float, MAX_ENGINES>& values, int count) {
	const std::string_view blob = packEngineValues(values, count);
	check_bind(stmt_txt, blob.empty()
		? sqlite3_bind_null(stmt, index)
		: sqlite3_bind_blob(stmt, index, blob.data(), (int)blob.size(), SQLITE_TRANSIENT));
}

// Binds the statement's parameters (with db_bind(); may throw).
using DbBinder = std::function<void(sqlite3_stmt* stmt, const char* stmt_txt)>;

// Runs one INSERT/UPDATE/DELETE on status->sql as its own transaction, holding
// STATUS::mutex_db_commit. out_rowid, if given, receives the inserted row's
// id once committed. Throws db_exception (after rolling back) on failure.
static void db_insert_update_table(STATUS* status, const char* stmt_txt, const DbBinder& bind, int* out_rowid = nullptr) {
	sqlite3* sql = status->sql;
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
		bind(stmt, stmt_txt);
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
		// Catch-all, not just db_exception -- bind is caller-supplied and
		// could throw something else entirely, and mutex_db_commit must be
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

int db_insert_trip(STATUS* status, const FLIGHT_DATA_RECORD& departure) {
	int trip_id = -1;
	db_insert_update_table(status,
		"INSERT INTO trips ("
		"title,"
		"atc_airline,"
		"atc_flight_number,"
		"atc_id,"
		"atc_model,"
		"atc_type,"
		"departure_latitude,"
		"departure_longitude,"
		"departure_zulu_time,"
		"departure_local_time"
		") VALUES (?,?,?,?,?,?,?,?,?,?);",
		[&](sqlite3_stmt* stmt, const char* stmt_txt) {
			db_bind(stmt, stmt_txt, 1, departure.title);
			db_bind(stmt, stmt_txt, 2, departure.atc_airline);
			db_bind(stmt, stmt_txt, 3, departure.atc_flight_number);
			db_bind(stmt, stmt_txt, 4, departure.atc_id);
			db_bind(stmt, stmt_txt, 5, departure.atc_model);
			db_bind(stmt, stmt_txt, 6, departure.atc_type);
			db_bind(stmt, stmt_txt, 7, departure.plane_coordinate.latitude);
			db_bind(stmt, stmt_txt, 8, departure.plane_coordinate.longitude);
			db_bind(stmt, stmt_txt, 9, departure.time_zulu.format_date_time().c_str());
			db_bind(stmt, stmt_txt, 10, departure.time_local.format_date_time().c_str());
		},
		&trip_id);
	return trip_id;
}

void db_set_trip_destination_time(STATUS* status, int trip_id, const DATETIME& time_zulu, const DATETIME& time_local) {
	db_insert_update_table(status,
		"UPDATE trips SET destination_zulu_time=?,destination_local_time=? WHERE id=?;",
		[&](sqlite3_stmt* stmt, const char* stmt_txt) {
			db_bind(stmt, stmt_txt, 1, time_zulu.format_date_time().c_str());
			db_bind(stmt, stmt_txt, 2, time_local.format_date_time().c_str());
			db_bind(stmt, stmt_txt, 3, trip_id);
		});
}

void db_set_trip_destination_position(STATUS* status, int trip_id, const COORDINATE& position) {
	db_insert_update_table(status,
		"UPDATE trips SET destination_latitude=?,destination_longitude=? WHERE id=?;",
		[&](sqlite3_stmt* stmt, const char* stmt_txt) {
			db_bind(stmt, stmt_txt, 1, position.latitude);
			db_bind(stmt, stmt_txt, 2, position.longitude);
			db_bind(stmt, stmt_txt, 3, trip_id);
		});
}

void db_set_trip_airport(STATUS* status, int trip_id, TRIP_END end, const AIRPORT& airport, const char* runway) {
	const char* stmt_txt = end == TRIP_END::DEPARTURE
		? "UPDATE trips SET departure_icao=?,departure_rwy=?,departure_region=?,departure_name=? WHERE id=?;"
		: "UPDATE trips SET destination_icao=?,destination_rwy=?,destination_region=?,destination_name=? WHERE id=?;";
	db_insert_update_table(status, stmt_txt,
		[&](sqlite3_stmt* stmt, const char* stmt_txt) {
			db_bind(stmt, stmt_txt, 1, airport.icao);
			db_bind_text_or_null(stmt, stmt_txt, 2, runway);
			db_bind(stmt, stmt_txt, 3, airport.region);
			db_bind(stmt, stmt_txt, 4, airport.name);
			db_bind(stmt, stmt_txt, 5, trip_id);
		});
}

void db_clear_trip_destination_airport(STATUS* status, int trip_id) {
	db_insert_update_table(status,
		"UPDATE trips SET destination_icao=NULL,destination_rwy=NULL,destination_region=NULL,destination_name=NULL WHERE id=?;",
		[&](sqlite3_stmt* stmt, const char* stmt_txt) {
			db_bind(stmt, stmt_txt, 1, trip_id);
		});
}

int db_insert_contact(STATUS* status, CONTACT_TABLE table, int trip_id, const FLIGHT_DATA& data) {
	const bool touchdown = table == CONTACT_TABLE::TOUCHDOWNS;
	const std::string stmt_txt = std::string("INSERT INTO ") + contactTableName(table)
		+ " (trip,airspeed_indicated,vertical_speed," + (touchdown ? "g_force," : "")
		+ "plane_pitch_degrees,plane_bank_degrees,heading_indicator,plane_latitude,plane_longitude,"
		  "wind_direction,wind_velocity,time_zulu,time_local"
		  ") VALUES (?,?,?,?,?,?,?,?,?,?,?,?" + (touchdown ? ",?" : "") + ");";
	int row_id = -1;
	db_insert_update_table(status, stmt_txt.c_str(),
		[&](sqlite3_stmt* stmt, const char* stmt_txt) {
			int index = 1;
			db_bind(stmt, stmt_txt, index++, trip_id);
			db_bind(stmt, stmt_txt, index++, data.speed);
			db_bind(stmt, stmt_txt, index++, data.vertical_speed);
			if (touchdown)
				db_bind(stmt, stmt_txt, index++, data.g_force);
			db_bind(stmt, stmt_txt, index++, data.pitch);
			db_bind(stmt, stmt_txt, index++, data.bank);
			db_bind(stmt, stmt_txt, index++, data.heading);
			db_bind(stmt, stmt_txt, index++, data.coordinate.latitude);
			db_bind(stmt, stmt_txt, index++, data.coordinate.longitude);
			db_bind(stmt, stmt_txt, index++, data.wind_direction);
			db_bind(stmt, stmt_txt, index++, data.wind_velocity);
			db_bind(stmt, stmt_txt, index++, data.time_zulu.format_date_time().c_str());
			db_bind(stmt, stmt_txt, index++, data.time_local.format_date_time().c_str());
		},
		&row_id);
	return row_id;
}

void db_set_contact_airport(STATUS* status, CONTACT_TABLE table, int row_id, const AIRPORT& airport, const char* runway) {
	const bool touchdown = table == CONTACT_TABLE::TOUCHDOWNS;
	const std::string update = std::string("UPDATE ") + contactTableName(table) + " SET icao=?,airport_name=?";
	if (runway == nullptr) {
		db_insert_update_table(status, (update + " WHERE id=?;").c_str(),
			[&](sqlite3_stmt* stmt, const char* stmt_txt) {
				db_bind(stmt, stmt_txt, 1, airport.icao);
				db_bind(stmt, stmt_txt, 2, airport.name);
				db_bind(stmt, stmt_txt, 3, row_id);
			});
		return;
	}
	const RUNWAY_OPERATION& rwy = airport.runway_act;
	db_insert_update_table(status,
		(update + ",runway=?,runway_heading=?,"
			"distance_length=?,distance_width=?,distance_length_percent=?,distance_width_percent=?"
			" WHERE id=?;").c_str(),
		[&](sqlite3_stmt* stmt, const char* stmt_txt) {
			db_bind(stmt, stmt_txt, 1, airport.icao);
			db_bind(stmt, stmt_txt, 2, airport.name);
			db_bind(stmt, stmt_txt, 3, runway);
			db_bind(stmt, stmt_txt, 4, rwy.heading);
			// A runway is matched, so this is a real, resolved distance. A
			// touchdown before the marked/displaced threshold legitimately
			// reports negative and is stored as is; a negative liftoff distance
			// is stored as -1.
			db_bind(stmt, stmt_txt, 5, !touchdown && rwy.distances[0] < 0 ? -1.0 : rwy.distances[0]);
			db_bind(stmt, stmt_txt, 6, rwy.distances[1]);
			db_bind(stmt, stmt_txt, 7, rwy.distances_percent[0]);
			db_bind(stmt, stmt_txt, 8, rwy.distances_percent[1]);
			db_bind(stmt, stmt_txt, 9, row_id);
		});
}

void db_insert_event(STATUS* status, int trip_id, const char* event, const char* time_zulu, const char* time_local, unsigned long long event_seq) {
	// The flood filter this is fed from (STATUS::event_filter, see
	// event_filter.h) can hold an occurrence back for several seconds after
	// the trip it belongs to already ended, so by the
	// time this runs the trip may have already been deleted from the UI. The
	// WHERE EXISTS guard, evaluated inside this same transaction, makes that a
	// silent no-op instead of an orphaned row: if the trip's gone by the time
	// this commits, 0 rows are affected; if it isn't gone yet, deleteTripData's
	// own "DELETE FROM trip_events WHERE trip = ?" sweeps this row up normally
	// either way, so there's no race to lose either way it lands.
	db_insert_update_table(status,
		"INSERT INTO trip_events (trip,event,time_zulu,time_local,event_seq) "
		"SELECT ?,?,?,?,? WHERE EXISTS (SELECT 1 FROM trips WHERE id = ?);",
		[&](sqlite3_stmt* stmt, const char* stmt_txt) {
			db_bind(stmt, stmt_txt, 1, trip_id);
			db_bind(stmt, stmt_txt, 2, event);
			db_bind(stmt, stmt_txt, 3, time_zulu);
			db_bind(stmt, stmt_txt, 4, time_local);
			db_bind(stmt, stmt_txt, 5, (long long)event_seq);
			db_bind(stmt, stmt_txt, 6, trip_id);
		});
}

void db_delete_events(STATUS* status, const std::vector<unsigned long long>& seqs) {
	if (seqs.empty())
		return;
	std::string placeholders;
	for (size_t i = 0; i < seqs.size(); i++)
		placeholders += (i == 0) ? "?" : ",?";
	std::string stmt_txt = "DELETE FROM trip_events WHERE event_seq IN (" + placeholders + ");";
	db_insert_update_table(status, stmt_txt.c_str(),
		[&](sqlite3_stmt* stmt, const char* stmt_txt) {
			for (size_t i = 0; i < seqs.size(); i++)
				db_bind(stmt, stmt_txt, (int)(i + 1), (long long)seqs[i]);
		});
}

// The message of the exception being handled: a db_exception's own text, or
// "unknown exception" for anything else. Call only inside a catch block.
static std::string current_exception_message() {
	try {
		throw;
	}
	catch (const db_exception& e) {
		return e.message;
	}
	catch (...) {
		return "unknown exception";
	}
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
		catch (...) {
			const std::string message = current_exception_message();
			// item.trip_id is only ever set for Insert items (see
			// EVENT_QUEUE_ITEM::trip_id in types.h) and a Delete can retract more
			// than one row at once, so the two kinds need distinct messages here.
			if (item.kind == EVENT_QUEUE_ITEM::Kind::Insert) {
				gui_log_printf(status, GUI_LOG_WARNING, "event_write_worker: dropped one event for trip %d: %s", item.trip_id, message.c_str());
				// A dropped Insert was already shown in the Live Status list (see
				// commit_event() in recorder.cpp, which notifies the UI before this
				// write is even queued) -- pull it back out so the panel doesn't
				// keep showing an occurrence that never made it into trip_events.
				// Not needed for a dropped Delete: nothing was ever added to the UI
				// for a retraction, only removed, so there's nothing to undo.
				gui_notify_events_retracted(status, &item.seq, 1);
			} else {
				gui_log_printf(status, GUI_LOG_WARNING, "event_write_worker: failed to retract %zu event(s): %s", item.delete_seqs.size(), message.c_str());
			}
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
			db_insert_update_table(status, insert_txt.c_str(),
				[&](sqlite3_stmt* stmt, const char* stmt_txt) {
					const std::array<uint32_t, 4> bool_groups = tripBoolGroups(*pS);
					int index = 1;
					db_bind(stmt, stmt_txt, index++, item.trip_id);
					for (int group = 1; group <= 3; group++)
						db_bind(stmt, stmt_txt, index++, (int)bool_groups[group]);
#define TRIP_NUM_BIND(dbColumn, memberExpr, sqlType) db_bind(stmt, stmt_txt, index++, pS->memberExpr);
					TRIP_DATA_NUM_FIELDS(TRIP_NUM_BIND)
#undef TRIP_NUM_BIND
					const EnginePower power = enginePowerFromRecord(*pS);
					db_bind_engine_values(stmt, stmt_txt, index++, power.speed, power.count);
					db_bind_engine_values(stmt, stmt_txt, index++, power.load, power.count);
					db_bind(stmt, stmt_txt, index++, pS->time_zulu.format_date_time().c_str());
					db_bind(stmt, stmt_txt, index++, pS->time_local.format_date_time().c_str());
				});
		}
		catch (...) {
			// Drop just this one sample and keep draining the queue, rather
			// than stalling every later sample (this trip's and any future
			// trip's) behind one bad row. Any exception, not just db_exception:
			// a bound callback could throw anything, and one escaping this
			// thread would terminate the app.
			gui_log_printf(status, GUI_LOG_WARNING, "db_write_worker: dropped one sample for trip %d: %s", item.trip_id, current_exception_message().c_str());
		}
		free(pS);
	}
	log_cf(3, "DB", "db_write_worker: queue stopped; thread exiting");
}

std::string db_file_path() {
	return app_file_path(DATABASE_NAME ".db");
}

namespace {

// A UI connection (see connect_db_readwrite/readonly below), or NULL.
sqlite3* open_ui_connection(int flags, int busy_timeout_ms) {
	sqlite3* sql = NULL;
	if (sqlite3_open_v2(db_file_path().c_str(), &sql, flags, NULL) != SQLITE_OK) {
		if (sql != NULL)
			sqlite3_close(sql);
		return NULL;
	}
	sqlite3_busy_timeout(sql, busy_timeout_ms);
	return sql;
}

}

// See db_connection.h. connect_db()'s connection is opened with
// SQLITE_OPEN_NOMUTEX, so it must never be shared with a background query
// thread; these two are opened without it.
sqlite3* connect_db_readwrite() {
	return open_ui_connection(SQLITE_OPEN_READWRITE, 5000);
}

sqlite3* connect_db_readonly() {
	return open_ui_connection(SQLITE_OPEN_READONLY, 2000);
}

// Remove constraint keywords that SQLite disallows in ALTER TABLE ADD COLUMN:
// NOT NULL (requires a DEFAULT when rows exist), PRIMARY KEY, UNIQUE, AUTOINCREMENT.
// The added column defaults to NULL for any existing rows, which is fine — the
// app reads integer/real columns as 0 and text columns as empty when NULL.
template <size_t N>
static void strip_alter_column_constraints(const char* src, char (&dst)[N]) {
	static const char* kws[] = { "NOT NULL", "PRIMARY KEY", "AUTOINCREMENT", "UNIQUE", nullptr };
	copy_cstr(dst, src);
	for (int i = 0; kws[i]; i++) {
		int klen = (int)strlen(kws[i]);
		char* p;
		while ((p = strstr(dst, kws[i])) != nullptr)
			memmove(p, p + klen, strlen(p + klen) + 1);
	}
}

namespace {

// One column of an existing table, as PRAGMA table_info reports it.
struct TableColumn {
	std::string name;
	std::string type; // as declared, e.g. "VARCHAR(32)"; may be empty
	bool notNull;
};

}

// table_name's columns (PRAGMA table_info); empty (logged) if it can't be
// read. create_schema() made sure the table exists, so empty is a failure.
static std::vector<TableColumn> table_columns(sqlite3* sql, const char* table_name) {
	std::vector<TableColumn> columns;
	const std::string pragma = std::string("PRAGMA table_info(") + table_name + ");";
	sqlite3_stmt* stmt;
	if (sqlite3_prepare_v2(sql, pragma.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
		while (sqlite3_step(stmt) == SQLITE_ROW) {
			const char* n = (const char*)sqlite3_column_text(stmt, 1); // name
			const char* t = (const char*)sqlite3_column_text(stmt, 2); // type
			if (n)
				columns.push_back({ n, t ? t : "", sqlite3_column_int(stmt, 3) != 0 }); // 3 = notnull
		}
	}
	sqlite3_finalize(stmt);
	if (columns.empty())
		log_cf(0, "DB", "Schema migration failed to read %s's columns: %s", table_name, sqlite3_errmsg(sql));
	return columns;
}

// The column of columns named name, or columns.end().
static std::vector<TableColumn>::const_iterator find_column(const std::vector<TableColumn>& columns, const std::string& name) {
	return std::find_if(columns.begin(), columns.end(), [&name](const TableColumn& c) { return c.name == name; });
}

// Runs stmt_txt (no results). False on failure; the caller logs
// sqlite3_errmsg(sql).
static bool exec_sql(sqlite3* sql, const char* stmt_txt) {
	return sqlite3_exec(sql, stmt_txt, nullptr, nullptr, nullptr) == SQLITE_OK;
}

// For each column in fields_def (comma-separated column definitions) that is
// absent from table_name, run ALTER TABLE ADD COLUMN. False if the table's
// columns couldn't be read or one couldn't be added (logged): every write
// and query names them, so the database can't be used. Called from
// create_schema() on a write connection, never from the readonly path.
static bool migrate_table_columns(sqlite3* sql, const char* table_name, const char* fields_def) {
	const std::vector<TableColumn> existing = table_columns(sql, table_name);
	if (existing.empty())
		return false; // unreadable -- don't re-add every column

	// Walk fields_def, splitting by comma while respecting parentheses.
	int flen = (int)strlen(fields_def);
	char* buf = (char*)malloc((size_t)(flen + 1));
	if (!buf) return false;
	memcpy(buf, fields_def, (size_t)(flen + 1));

	bool ok = true;
	int depth = 0, seg_start = 0;
	for (int i = 0; i <= flen; i++) {
		char c = buf[i];
		if      (c == '(') { depth++; continue; }
		else if (c == ')') { depth--; continue; }
		else if ((c == ',' || c == '\0') && depth == 0) {
			buf[i] = '\0';
			char* seg = buf + seg_start;
			while (*seg == ' ' || *seg == '\t' || *seg == '\n') seg++;

			const std::string col_name = column_name(seg);
			if (!col_name.empty()) {
				if (find_column(existing, col_name) == existing.end()) {
					char safe_def[512];
					strip_alter_column_constraints(seg, safe_def);

					char alter_sql[640];
					snprintf(alter_sql, sizeof(alter_sql),
						"ALTER TABLE %s ADD COLUMN %s;", table_name, safe_def);

					if (exec_sql(sql, alter_sql)) {
						log_cf(2, "DB", "Schema migration: %s — added column %s", table_name, col_name.c_str());
					} else {
						log_cf(0, "DB", "Schema migration failed (%s): %s", sqlite3_errmsg(sql), alter_sql);
						ok = false;
					}
				}
			}

			seg_start = i + 1;
		}
	}
	free(buf);
	return ok;
}

// engine_pack(number_of_engines, value1, value2): a legacy trip_data row's
// engine values as an engine_speed/engine_load BLOB (packEngineValues()).
// Those rows recorded engines 1-2 only, so at most 2 are packed; NULL for no
// engines.
static void sql_engine_pack(sqlite3_context* ctx, int argc, sqlite3_value** argv) {
	std::array<float, MAX_ENGINES> values{};
	const int count = std::clamp(sqlite3_value_int(argv[0]), 0, argc - 1);
	for (int i = 0; i < count; ++i)
		values[i] = (float)sqlite3_value_double(argv[i + 1]);
	const std::string_view blob = packEngineValues(values, count);
	if (blob.empty())
		sqlite3_result_null(ctx);
	else
		sqlite3_result_blob(ctx, blob.data(), (int)blob.size(), SQLITE_TRANSIENT);
}

// Copies every trip_data row into trip_data_new with insert_select (an
// INSERT ... SELECT ... WHERE rowid >= ?1 ORDER BY rowid LIMIT ?2), about a
// hundredth of the rows per statement, calling on_batch(rows copied, total
// rows) after each one; on_batch returning false stops the copy. False on an
// SQLite error (left in sqlite3_errmsg()) or when on_batch stopped it.
static bool copy_rows_in_batches(sqlite3* sql, const std::string& insert_select,
		const std::function<bool(sqlite3_int64 copied, sqlite3_int64 total)>& on_batch) {
	sqlite3_stmt* count = nullptr;
	if (sqlite3_prepare_v2(sql, "SELECT COUNT(*) FROM trip_data", -1, &count, nullptr) != SQLITE_OK)
		return false;
	const bool counted = sqlite3_step(count) == SQLITE_ROW;
	const sqlite3_int64 total = counted ? sqlite3_column_int64(count, 0) : 0;
	sqlite3_finalize(count);
	if (!counted)
		return false;

	sqlite3_stmt* insert = nullptr;
	if (sqlite3_prepare_v2(sql, insert_select.c_str(), -1, &insert, nullptr) != SQLITE_OK)
		return false;
	sqlite3_bind_int64(insert, 2, std::max<sqlite3_int64>(1, total / 100));
	sqlite3_int64 next_rowid = std::numeric_limits<sqlite3_int64>::min();
	sqlite3_int64 copied = 0;
	bool ok = true;
	for (;;) {
		sqlite3_bind_int64(insert, 1, next_rowid);
		if (sqlite3_step(insert) != SQLITE_DONE) {
			ok = false;
			break;
		}
		const int batch = sqlite3_changes(sql);
		sqlite3_reset(insert);
		if (batch == 0)
			break;
		copied += batch;
		const sqlite3_int64 last_rowid = sqlite3_last_insert_rowid(sql); // ORDER BY rowid: the batch's last row
		if (!on_batch(copied, total)) {
			ok = false;
			break;
		}
		if (last_rowid == std::numeric_limits<sqlite3_int64>::max())
			break; // no rowid after it, and +1 would overflow
		next_rowid = last_rowid + 1;
	}
	sqlite3_finalize(insert);
	return ok;
}

// Reports a job made of steps as one percentage of the whole job. Each step's
// weight is roughly its share of the time; a step's share of 100 is its weight
// over the weights' sum. Only a rising percentage is passed on.
class StepProgress {
public:
	StepProgress(const MigrationProgress& progress, std::vector<int> weights)
		: progress_(progress), weights_(std::move(weights)) {
		for (int weight : weights_)
			total_ += weight;
	}
	// fraction (0-1) of step (an index into the weights) done; the steps
	// before it count as done.
	void report(size_t step, double fraction = 1.0) {
		double done = weights_[step] * fraction;
		for (size_t i = 0; i < step; ++i)
			done += weights_[i];
		const int percent = (int)(done * 100 / total_);
		if (progress_ && percent > reported_)
			progress_(reported_ = percent);
	}

private:
	MigrationProgress progress_;
	std::vector<int> weights_;
	int total_ = 0;
	int reported_ = 0;
};

// The steps of migrate_legacy_engine_columns()' rebuild, in order. The weights
// (kRebuildStepWeights) roughly follow their timing: copying the rows takes
// most of it.
enum RebuildStep { kRebuildCopy, kRebuildDrop, kRebuildCommit, kRebuildIndexes };
static const std::vector<int> kRebuildStepWeights = { 65, 10, 15, 10 };

// Rebuilds a trip_data made before engine_speed/engine_load existed without
// its N1/N2 columns, moving their values into those. Only jet rows
// (engine_type 1) get values, since N1/N2 is a jet's speed/load pair (see
// enginePowerSpec()). A helicopter turbine's (engine_type 3) speed is also N1,
// but its load, torque, was never recorded, and a row stores both or neither
// (db_history takes the smaller count), so its N1 is dropped and its rows get
// NULL like every other non-jet's. Copying into a new table writes each row
// once -- an UPDATE, then a DROP COLUMN per old column, would rewrite the
// whole table five times -- and lets progress report the share of rows copied.
//
// The new table has trip_data_columns()' definitions, except that a column
// the old table allowed NULL in (one migrate_table_columns() added) keeps
// allowing it, then any old column the definitions don't name, with its
// declared type but not its constraints. Rowids are kept. DROP TABLE takes
// the old indexes with it; create_schema() recreates them next
// (create_db_indexes()) and then reports that last step (kRebuildIndexes).
//
// progress gets the share of the whole rebuild done (see RebuildStep): after
// each copied batch, after the drop and after the commit.
//
// Runs while turb_eng_n1_1 exists, after migrate_table_columns() added the
// new columns, as one transaction: a failure or crash leaves the old table in
// place for the next start to redo it. cancelled, if set, is asked after each
// copied batch and once more before committing; true rolls the rebuild back
// the same way (the index step after the commit can't be cancelled). rebuilt
// is set once the rebuild commits. False on failure or cancel (logged) -- the
// INSERT no longer fills the old NOT NULL columns, so recording can't work
// until it succeeds.
static bool migrate_legacy_engine_columns(sqlite3* sql, StepProgress& progress, const MigrationCancelled& cancelled, bool& rebuilt) {
	static const char* const kLegacyColumns[] = { "turb_eng_n1_1", "turb_eng_n1_2", "turb_eng_n2_1", "turb_eng_n2_2" };
	const std::vector<TableColumn> old_columns = table_columns(sql, "trip_data");
	if (old_columns.empty())
		return false; // can't be read (logged), so it may still have the old columns
	if (find_column(old_columns, kLegacyColumns[0]) == old_columns.end())
		return true;
	log_cf(2, "DB", "Schema migration: trip_data -- moving N1/N2 into engine_speed/engine_load");
	if (sqlite3_create_function_v2(sql, "engine_pack", 3, SQLITE_UTF8 | SQLITE_DETERMINISTIC,
			nullptr, sql_engine_pack, nullptr, nullptr, nullptr) != SQLITE_OK) {
		log_cf(0, "DB", "Schema migration failed to register engine_pack: %s", sqlite3_errmsg(sql));
		return false;
	}

	std::string definitions, names, values;
	const auto add = [&](const std::string& definition, const std::string& name, const std::string& value) {
		const char* separator = definitions.empty() ? "" : ",";
		definitions += separator + definition;
		names += separator + name;
		values += separator + value;
	};
	for (const std::string& definition : trip_data_columns()) {
		const std::string name = column_name(definition);
		const auto old = find_column(old_columns, name);
		char nullable[512];
		strip_alter_column_constraints(definition.c_str(), nullable);
		const std::string value =
			name == "engine_speed" ? "CASE WHEN engine_type = 1 THEN engine_pack(number_of_engines, turb_eng_n1_1, turb_eng_n1_2) END" :
			name == "engine_load"  ? "CASE WHEN engine_type = 1 THEN engine_pack(number_of_engines, turb_eng_n2_1, turb_eng_n2_2) END" :
			name;
		add(old != old_columns.end() && !old->notNull ? nullable : definition, name, value);
	}
	for (const TableColumn& old : old_columns) {
		const bool legacy = std::find(std::begin(kLegacyColumns), std::end(kLegacyColumns), old.name) != std::end(kLegacyColumns);
		const bool defined = std::any_of(trip_data_columns().begin(), trip_data_columns().end(),
			[&old](const std::string& definition) { return column_name(definition) == old.name; });
		if (!legacy && !defined)
			add(old.name + " " + old.type, old.name, old.name);
	}

	const std::string create = "CREATE TABLE trip_data_new (" + definitions + ");";
	const std::string insert_select = "INSERT INTO trip_data_new (rowid," + names + ") SELECT rowid," + values
		+ " FROM trip_data WHERE rowid >= ?1 ORDER BY rowid LIMIT ?2;";
	bool stopped = false;
	const auto keep_going = [&] { return !(stopped = cancelled && cancelled()); };
	bool ok = exec_sql(sql, "BEGIN TRANSACTION;")
		&& exec_sql(sql, create.c_str())
		&& copy_rows_in_batches(sql, insert_select, [&](sqlite3_int64 copied, sqlite3_int64 total) {
			progress.report(kRebuildCopy, (double)copied / total);
			return keep_going();
		})
		&& exec_sql(sql, "DROP TABLE trip_data;"
			"ALTER TABLE trip_data_new RENAME TO trip_data;");
	if (ok) {
		progress.report(kRebuildDrop);
		ok = keep_going() && exec_sql(sql, "COMMIT TRANSACTION;");
	}
	if (!ok) {
		if (stopped)
			log_cf(2, "DB", "Schema migration cancelled; trip_data left unchanged");
		else
			log_cf(0, "DB", "Schema migration failed (%s); trip_data left unchanged", sqlite3_errmsg(sql));
		exec_sql(sql, "ROLLBACK TRANSACTION");
		return false;
	}
	progress.report(kRebuildCommit);
	rebuilt = true;
	log_cf(2, "DB", "Schema migration: trip_data -- N1/N2 moved, old columns dropped");
	return true;
}

// Builds before user_version 1 stored local times (DATETIME::format_date_time())
// with the UTC offset's sign reversed: UTC+2 was stored as "-02:00". This
// flips that sign in every stored local time, once, in one transaction, then
// sets user_version to 1 so it never runs again. "+00:00" and values not in
// that format are left as they are. False on failure (logged, rolled back):
// recording can't start, since rows written with the correct sign would be
// flipped by the next start's retry.
static bool migrate_local_time_offsets(sqlite3* sql) {
	sqlite3_stmt* version = nullptr;
	if (sqlite3_prepare_v2(sql, "PRAGMA user_version;", -1, &version, nullptr) != SQLITE_OK) {
		log_cf(0, "DB", "Schema migration failed to read user_version: %s", sqlite3_errmsg(sql));
		return false;
	}
	const bool read = sqlite3_step(version) == SQLITE_ROW;
	const int user_version = read ? sqlite3_column_int(version, 0) : 0;
	sqlite3_finalize(version);
	if (!read) {
		log_cf(0, "DB", "Schema migration failed to read user_version: %s", sqlite3_errmsg(sql));
		return false;
	}
	if (user_version >= 1)
		return true;

	static const std::pair<const char*, const char*> kLocalTimes[] = {
		{ "trips", "departure_local_time" }, { "trips", "destination_local_time" },
		{ "trip_data", "local_time" }, { "trip_events", "time_local" },
		{ contactTableName(CONTACT_TABLE::LIFTOFFS), "time_local" },
		{ contactTableName(CONTACT_TABLE::TOUCHDOWNS), "time_local" },
	};
	bool ok = exec_sql(sql, "BEGIN TRANSACTION;");
	for (const auto& [table, column] : kLocalTimes) {
		if (!ok)
			break;
		// "YYYY-MM-DDThh:mm:ss.sss" is 23 characters; the sign is the 24th.
		char stmt_txt[512];
		snprintf(stmt_txt, sizeof(stmt_txt),
			"UPDATE %s SET %s = substr(%s,1,23) || CASE substr(%s,24,1) WHEN '+' THEN '-' ELSE '+' END || substr(%s,25)"
			" WHERE %s GLOB '????-??-??T??:??:??.???[+-]??:??*' AND substr(%s,25,5) <> '00:00';",
			table, column, column, column, column, column, column);
		ok = exec_sql(sql, stmt_txt);
	}
	ok = ok && exec_sql(sql, "PRAGMA user_version = 1;") && exec_sql(sql, "COMMIT TRANSACTION;");
	if (!ok) {
		log_cf(0, "DB", "Schema migration failed (%s); local times left unchanged", sqlite3_errmsg(sql));
		exec_sql(sql, "ROLLBACK TRANSACTION");
		return false;
	}
	log_cf(2, "DB", "Schema migration: local time UTC offsets corrected");
	return true;
}

// Part of create_schema() (also recreating the index that
// migrate_legacy_engine_columns()' rebuild drops), so Trip History reads stay
// indexed even when the schema was created/updated by migrate_db() alone (SimConnect
// never connected this session -- see migrate_db()'s doc comment in db.h).
// trip_data/trip_events/trip_liftoffs/trip_touchdowns are all queried with
// "WHERE trip = ?" (db_history.cpp) -- without an index that's a full table
// scan across every sample ever recorded, for every trip load. trips is
// filtered by group_id when counting a group's trips and when a deleted
// group's trips are ungrouped (db_groups.cpp). IF NOT EXISTS makes this safe to
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
	for (const char* stmt_txt : index_stmts)
		if (!exec_sql(sql, stmt_txt))
			log_cf(1, "DB", "Failed to create index \"%s\": %s", stmt_txt, sqlite3_errmsg(sql));
}

// Creates any missing table, adds columns missing from tables made by an
// older build (see migrate_table_columns()), moves legacy N1/N2 columns (see
// migrate_legacy_engine_columns()), corrects old local-time offsets (see
// migrate_local_time_offsets()) and creates the indexes. False if a table
// couldn't be created or migrated (already logged). progress, cancelled: see
// migrate_db().
static bool create_schema(sqlite3* sql, const MigrationProgress& progress, const MigrationCancelled& cancelled) {
	bool ok = true;
	for (const TableDef& table : database_tables()) {
		const std::string stmt_txt = std::string("CREATE TABLE IF NOT EXISTS ") + table.name + " (" + table.fields + ");";
		if (!exec_sql(sql, stmt_txt.c_str())) {
			log_cf(0, "DB", "Failed to create table %s: %s", table.name, sqlite3_errmsg(sql));
			ok = false;
		}
	}
	for (const TableDef& table : database_tables())
		if (!migrate_table_columns(sql, table.name, table.fields.c_str()))
			ok = false;
	StepProgress rebuild_progress(progress, kRebuildStepWeights);
	bool rebuilt = false;
	if (!migrate_legacy_engine_columns(sql, rebuild_progress, cancelled, rebuilt))
		ok = false;
	if (!migrate_local_time_offsets(sql))
		ok = false;
	create_db_indexes(sql);
	if (rebuilt)
		rebuild_progress.report(kRebuildIndexes); // the rebuild's last step: its indexes are back
	return ok;
}

bool migrate_db(const MigrationProgress& progress, const MigrationCancelled& cancelled) {
	const std::string fn_db = db_file_path();
	log_cf(3, "DB", "migrate_db: checking schema for %s", fn_db.c_str());
	sqlite3* sql = nullptr;
	if (sqlite3_open_v2(fn_db.c_str(), &sql, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr) != SQLITE_OK) {
		log_cf(0, "DB", "migrate_db: cannot open database %s: %s", fn_db.c_str(), sql ? sqlite3_errmsg(sql) : "unknown error");
		if (sql) sqlite3_close(sql);
		return false;
	}
	sqlite3_busy_timeout(sql, 5000);
	const bool ok = create_schema(sql, progress, cancelled);
	log_cf(3, "DB", "migrate_db: schema check complete");
	sqlite3_close(sql);
	return ok;
}

void connect_db(struct STATUS* status) {
	const std::string fn_db = db_file_path();
	if (sqlite3_open_v2(fn_db.c_str(), &status->sql, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_NOMUTEX | SQLITE_OPEN_SHAREDCACHE, NULL) == SQLITE_OK)
		log_cf(2, "DB", "Opened database %s", fn_db.c_str());
	else {
		log_cf(0, "DB", "Cannot open database: %s", sqlite3_errmsg(status->sql));
		exit(1);
	}
	// Without this, a lock held by connect_db_readwrite() (group/delete-trip
	// operations from the Trip History UI) makes db_write_worker's writes fail
	// with SQLITE_BUSY immediately instead of waiting the few ms those short
	// operations actually take -- dropping recorded samples for no reason.
	sqlite3_busy_timeout(status->sql, 5000);

	if (!create_schema(status->sql, {}, {}))
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
