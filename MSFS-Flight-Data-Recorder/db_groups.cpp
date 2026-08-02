#include "db_groups.h"
#include "logger.h"

#include "sqlite3.h"

std::vector<TripGroup> queryAllGroups(sqlite3* sql) {
	std::vector<TripGroup> groups;
	Logger::log(Logger::Trace, "DB", QStringLiteral("queryAllGroups: loading group list"));

	const char* stmt_txt =
		"SELECT g.id, g.name, (SELECT COUNT(*) FROM trips t WHERE t.group_id = g.id) "
		"FROM trip_groups g ORDER BY g.name COLLATE NOCASE";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK)
		return groups;

	int rc;
	while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
		TripGroup group;
		group.id = sqlite3_column_int(stmt, 0);
		const unsigned char* text = sqlite3_column_text(stmt, 1);
		group.name = text ? QString::fromUtf8(reinterpret_cast<const char*>(text)) : QString();
		group.tripCount = sqlite3_column_int(stmt, 2);
		groups.push_back(group);
	}
	if (rc != SQLITE_DONE)
		Logger::logf(Logger::Warning, "DB", "queryAllGroups: step failed: %s", sqlite3_errmsg(sql));
	Logger::logf(Logger::Trace, "DB", "queryAllGroups: loaded %d groups", (int)groups.size());
	sqlite3_finalize(stmt);
	return groups;
}

// True if trip_groups already has a (case-insensitive) match for name, other
// than excludeGroupId itself (pass 0 -- never a valid id -- for insertGroup's
// "no exclusion" case).
static bool groupNameExists(sqlite3* sql, const QString& name, int excludeGroupId) {
	// Comparing in SQL via "COLLATE NOCASE" (as the UNIQUE index backing
	// trip_groups.name in db.cpp also does) only case-folds ASCII A-Z/a-z --
	// SQLite has no built-in Unicode-aware collation. That let e.g. "Café" and
	// "café" (or "MÜNCHEN"/"münchen") both be inserted as distinct groups,
	// each satisfying the NOCASE index despite being the same name to a user.
	// Qt's QString::compare(..., Qt::CaseInsensitive) does full Unicode case
	// folding, so pull every existing name and compare in C++ instead.
	const char* stmt_txt = "SELECT id, name FROM trip_groups";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK) {
		// Fails open (reports "no duplicate") on a prepare error, which is safe:
		// the caller's subsequent INSERT/UPDATE will still hit the COLLATE
		// NOCASE UNIQUE index and fail instead of succeeding wrongly, for any
		// duplicate that index is able to catch. Just log it, since a prepare
		// failure here is otherwise silent.
		Logger::logf(Logger::Warning, "DB", "groupNameExists: prepare failed: %s", sqlite3_errmsg(sql));
		return false;
	}
	bool exists = false;
	int rc;
	while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
		int id = sqlite3_column_int(stmt, 0);
		if (id == excludeGroupId)
			continue;
		const unsigned char* text = sqlite3_column_text(stmt, 1);
		QString existingName = text ? QString::fromUtf8(reinterpret_cast<const char*>(text)) : QString();
		if (QString::compare(existingName, name, Qt::CaseInsensitive) == 0) {
			exists = true;
			break;
		}
	}
	if (!exists && rc != SQLITE_DONE)
		Logger::logf(Logger::Warning, "DB", "groupNameExists: step failed: %s", sqlite3_errmsg(sql));
	sqlite3_finalize(stmt);
	return exists;
}

int insertGroup(sqlite3* sql, const QString& name) {
	QString trimmed = name.trimmed();
	// Blank and duplicate (case-insensitive) names are rejected here, not just
	// by the current UI caller (manage_groups_dialog.cpp): trip_groups.name has
	// no UNIQUE/CHECK constraint of its own, so without this check two groups
	// named "Vacation" and "vacation" would both silently succeed and then be
	// indistinguishable in the group filter combo and per-trip "Set Group" menu.
	if (trimmed.isEmpty() || groupNameExists(sql, trimmed, 0)) {
		Logger::log(Logger::Trace, "DB", QStringLiteral("insertGroup: rejected (blank or duplicate name)"));
		return 0;
	}
	const char* stmt_txt = "INSERT INTO trip_groups (name) VALUES (?)";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK)
		return 0;
	QByteArray utf8 = trimmed.toUtf8();
	sqlite3_bind_text(stmt, 1, utf8.constData(), utf8.size(), SQLITE_TRANSIENT);
	bool ok = sqlite3_step(stmt) == SQLITE_DONE;
	sqlite3_finalize(stmt);
	if (ok)
		Logger::logf(Logger::Trace, "DB", "insertGroup: created group %d", (int)sqlite3_last_insert_rowid(sql));
	return ok ? (int)sqlite3_last_insert_rowid(sql) : 0;
}

bool renameGroup(sqlite3* sql, int groupId, const QString& newName) {
	QString trimmed = newName.trimmed();
	if (trimmed.isEmpty() || groupNameExists(sql, trimmed, groupId)) {
		Logger::logf(Logger::Trace, "DB", "renameGroup(%d): rejected (blank or duplicate name)", groupId);
		return false;
	}
	const char* stmt_txt = "UPDATE trip_groups SET name = ? WHERE id = ?";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK)
		return false;
	QByteArray utf8 = trimmed.toUtf8();
	sqlite3_bind_text(stmt, 1, utf8.constData(), utf8.size(), SQLITE_TRANSIENT);
	sqlite3_bind_int(stmt, 2, groupId);
	bool ok = sqlite3_step(stmt) == SQLITE_DONE;
	sqlite3_finalize(stmt);
	if (ok)
		Logger::logf(Logger::Trace, "DB", "renameGroup(%d): renamed to \"%s\"", groupId, utf8.constData());
	return ok;
}

bool deleteGroup(sqlite3* sql, int groupId) {
	// Ungroup member trips first, then remove the group itself. Both statements
	// must commit together -- without an explicit transaction, a failure on the
	// second leaves trips permanently pointing at a group_id that no longer
	// exists in trip_groups.
	const char* stmts[] = {
		"UPDATE trips SET group_id = NULL WHERE group_id = ?",
		"DELETE FROM trip_groups WHERE id = ?",
	};
	Logger::logf(Logger::Trace, "DB", "deleteGroup(%d): starting delete", groupId);
	if (sqlite3_exec(sql, "BEGIN TRANSACTION", nullptr, nullptr, nullptr) != SQLITE_OK) {
		Logger::logf(Logger::Warning, "DB", "deleteGroup(%d): BEGIN TRANSACTION failed: %s", groupId, sqlite3_errmsg(sql));
		return false;
	}
	for (const char* stmt_txt : stmts) {
		sqlite3_stmt* stmt = nullptr;
		if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK) {
			if (stmt) sqlite3_finalize(stmt);
			sqlite3_exec(sql, "ROLLBACK TRANSACTION", nullptr, nullptr, nullptr);
			return false;
		}
		sqlite3_bind_int(stmt, 1, groupId);
		bool ok = sqlite3_step(stmt) == SQLITE_DONE;
		sqlite3_finalize(stmt);
		if (!ok) {
			sqlite3_exec(sql, "ROLLBACK TRANSACTION", nullptr, nullptr, nullptr);
			return false;
		}
	}
	if (sqlite3_exec(sql, "COMMIT TRANSACTION", nullptr, nullptr, nullptr) != SQLITE_OK) {
		Logger::logf(Logger::Warning, "DB", "deleteGroup(%d): COMMIT TRANSACTION failed: %s", groupId, sqlite3_errmsg(sql));
		sqlite3_exec(sql, "ROLLBACK TRANSACTION", nullptr, nullptr, nullptr);
		return false;
	}
	Logger::logf(Logger::Trace, "DB", "deleteGroup(%d): delete committed", groupId);
	return true;
}

bool setTripGroup(sqlite3* sql, int tripId, int groupId) {
	const char* stmt_txt = "UPDATE trips SET group_id = ? WHERE id = ?";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK)
		return false;
	if (groupId > 0)
		sqlite3_bind_int(stmt, 1, groupId);
	else
		sqlite3_bind_null(stmt, 1);
	sqlite3_bind_int(stmt, 2, tripId);
	bool ok = sqlite3_step(stmt) == SQLITE_DONE;
	sqlite3_finalize(stmt);
	if (ok)
		Logger::logf(Logger::Trace, "DB", "setTripGroup(trip %d): group set to %d", tripId, groupId);
	return ok;
}
