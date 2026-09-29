#pragma once

#include "sqlite3.h"

#include <utility>

class QWidget;

// Connections for the UI (Trip History, groups, the map's analysis reports)
// and background queries -- independent of the recorder's own write
// connection (connect_db() in db.h). Neither creates the database: both fail
// (nullptr) if it doesn't exist yet. Caller must sqlite3_close() -- or use
// DbConnection below, which does.

// Read-only, 2 s busy timeout. Opened without SQLITE_OPEN_NOMUTEX (unlike
// connect_db()), so SQLite's own per-connection mutex makes it safe to use
// from whichever single thread is using it at a time.
sqlite3* connect_db_readonly();
// Read-write, for explicit UI write operations (e.g. deleting a trip), 5 s
// busy timeout.
sqlite3* connect_db_readwrite();

// A connection that closes itself. False if opening failed. Move-only.
class DbConnection {
public:
	DbConnection() = default;
	static DbConnection readOnly() { return DbConnection(connect_db_readonly()); }
	static DbConnection readWrite() { return DbConnection(connect_db_readwrite()); }

	DbConnection(DbConnection&& other) noexcept : sql_(std::exchange(other.sql_, nullptr)) {}
	DbConnection& operator=(DbConnection&& other) noexcept {
		std::swap(sql_, other.sql_);
		return *this;
	}
	DbConnection(const DbConnection&) = delete;
	DbConnection& operator=(const DbConnection&) = delete;
	~DbConnection() {
		if (sql_ != nullptr)
			sqlite3_close(sql_);
	}

	sqlite3* get() const { return sql_; }
	explicit operator bool() const { return sql_ != nullptr; }

private:
	explicit DbConnection(sqlite3* sql) : sql_(sql) {}
	sqlite3* sql_ = nullptr;
};

// DbConnection::readWrite() for a UI action. If the database can't be
// opened, logs "Cannot <action>", shows an error box over parent and returns
// a closed connection.
DbConnection openForWriting(QWidget* parent, const char* action);
