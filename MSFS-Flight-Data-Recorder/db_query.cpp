#include "db_query.h"
#include "logger.h"

#include "sqlite3.h"

QString columnText(sqlite3_stmt* stmt, int column) {
	const unsigned char* text = sqlite3_column_text(stmt, column);
	return text ? QString::fromUtf8(reinterpret_cast<const char*>(text)) : QString();
}

sqlite3_stmt* prepareStatement(sqlite3* sql, const char* stmtText, const QString& context) {
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, stmtText, -1, &stmt, nullptr) != SQLITE_OK) {
		Logger::logf(Logger::Warning, "DB", "%s: prepare failed for \"%s\": %s",
			qUtf8Printable(context), stmtText, sqlite3_errmsg(sql));
		return nullptr;
	}
	return stmt;
}

void forEachRow(sqlite3* sql, sqlite3_stmt* stmt, const QString& context,
	const std::function<bool(sqlite3_stmt*)>& onRow) {
	int rc;
	while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
		if (!onRow(stmt)) {
			rc = SQLITE_DONE;
			break;
		}
	}
	if (rc != SQLITE_DONE)
		Logger::logf(Logger::Warning, "DB", "%s: step failed: %s", qUtf8Printable(context), sqlite3_errmsg(sql));
	sqlite3_finalize(stmt);
}

void bindText(sqlite3_stmt* stmt, int index, const QString& text) {
	const QByteArray utf8 = text.toUtf8();
	sqlite3_bind_text(stmt, index, utf8.constData(), utf8.size(), SQLITE_TRANSIENT);
}

bool execStatement(sqlite3* sql, const char* stmtText, const QString& context,
	const std::function<void(sqlite3_stmt*)>& bind) {
	sqlite3_stmt* stmt = prepareStatement(sql, stmtText, context);
	if (!stmt)
		return false;
	bind(stmt);
	const bool ok = sqlite3_step(stmt) == SQLITE_DONE;
	if (!ok)
		Logger::logf(Logger::Warning, "DB", "%s: step failed for \"%s\": %s",
			qUtf8Printable(context), stmtText, sqlite3_errmsg(sql));
	sqlite3_finalize(stmt);
	return ok;
}

bool inTransaction(sqlite3* sql, const QString& context, const std::function<bool()>& body) {
	if (sqlite3_exec(sql, "BEGIN TRANSACTION", nullptr, nullptr, nullptr) != SQLITE_OK) {
		Logger::logf(Logger::Warning, "DB", "%s: BEGIN TRANSACTION failed: %s", qUtf8Printable(context), sqlite3_errmsg(sql));
		return false;
	}
	if (!body()) {
		sqlite3_exec(sql, "ROLLBACK TRANSACTION", nullptr, nullptr, nullptr);
		return false;
	}
	if (sqlite3_exec(sql, "COMMIT TRANSACTION", nullptr, nullptr, nullptr) != SQLITE_OK) {
		Logger::logf(Logger::Warning, "DB", "%s: COMMIT TRANSACTION failed: %s", qUtf8Printable(context), sqlite3_errmsg(sql));
		sqlite3_exec(sql, "ROLLBACK TRANSACTION", nullptr, nullptr, nullptr);
		return false;
	}
	return true;
}
