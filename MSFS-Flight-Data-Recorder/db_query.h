#pragma once

#include <QString>

#include <functional>

struct sqlite3;
struct sqlite3_stmt;

// Statement plumbing shared by the UI-side database files (db_history.cpp,
// db_groups.cpp). context starts every warning these log, e.g.
// "queryEvents(trip 5)".

// A text column as a QString; empty for NULL.
QString columnText(sqlite3_stmt* stmt, int column);

// Prepares stmtText; nullptr (logged as "<context>: prepare failed for
// "<stmtText>": ...") on failure.
sqlite3_stmt* prepareStatement(sqlite3* sql, const char* stmtText, const QString& context);

// Steps a prepared, bound stmt through its rows, calling onRow for each until
// it returns false, then finalizes stmt. A step error is logged as
// "<context>: step failed: ...".
void forEachRow(sqlite3* sql, sqlite3_stmt* stmt, const QString& context,
	const std::function<bool(sqlite3_stmt*)>& onRow);

// Binds text (as UTF-8) to parameter index of stmt.
void bindText(sqlite3_stmt* stmt, int index, const QString& text);

// Runs a statement that returns no rows, after bind sets its parameters;
// false (logged) if it fails.
bool execStatement(sqlite3* sql, const char* stmtText, const QString& context,
	const std::function<void(sqlite3_stmt*)>& bind);

// Runs body between BEGIN and COMMIT TRANSACTION, rolling back if body
// returns false or the commit fails. False on any failure.
bool inTransaction(sqlite3* sql, const QString& context, const std::function<bool()>& body);
