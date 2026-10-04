#include "db_groups.h"
#include "db_query.h"
#include "logger.h"

#include "sqlite3.h"

std::vector<TripGroup> queryAllGroups(sqlite3* sql) {
	std::vector<TripGroup> groups;
	Logger::log(Logger::Trace, "DB", QStringLiteral("queryAllGroups: loading group list"));

	const QString context = QStringLiteral("queryAllGroups");
	sqlite3_stmt* stmt = prepareStatement(sql,
		"SELECT g.id, g.name, (SELECT COUNT(*) FROM trips t WHERE t.group_id = g.id) "
		"FROM trip_groups g ORDER BY g.sort_order, g.name COLLATE NOCASE", context);
	if (!stmt)
		return groups;
	forEachRow(sql, stmt, context, [&](sqlite3_stmt* row) {
		TripGroup group;
		group.id = sqlite3_column_int(row, 0);
		group.name = columnText(row, 1);
		group.tripCount = sqlite3_column_int(row, 2);
		groups.push_back(group);
		return true;
	});
	Logger::logf(Logger::Trace, "DB", "queryAllGroups: loaded %d groups", (int)groups.size());
	return groups;
}

bool groupNameExists(sqlite3* sql, const QString& name, int excludeGroupId) {
	// Comparing in SQL via "COLLATE NOCASE" (as the UNIQUE index backing
	// trip_groups.name in db.cpp also does) only case-folds ASCII A-Z/a-z --
	// SQLite has no built-in Unicode-aware collation, so e.g. "Café" and
	// "café" (or "MÜNCHEN"/"münchen") would both satisfy the NOCASE index
	// despite being the same name to a user. Qt's QString::compare(...,
	// Qt::CaseInsensitive) does full Unicode case folding, so pull every
	// existing name and compare in C++ instead.
	// Fails open (reports "no duplicate") on a prepare error, which is safe:
	// the caller's subsequent INSERT/UPDATE will still hit the COLLATE NOCASE
	// UNIQUE index and fail instead of succeeding wrongly, for any duplicate
	// that index is able to catch.
	const QString context = QStringLiteral("groupNameExists");
	sqlite3_stmt* stmt = prepareStatement(sql, "SELECT id, name FROM trip_groups", context);
	if (!stmt)
		return false;
	bool exists = false;
	forEachRow(sql, stmt, context, [&](sqlite3_stmt* row) {
		if (sqlite3_column_int(row, 0) != excludeGroupId
			&& QString::compare(columnText(row, 1), name, Qt::CaseInsensitive) == 0)
			exists = true;
		return !exists;
	});
	return exists;
}

int insertGroup(sqlite3* sql, const QString& name) {
	QString trimmed = name.trimmed();
	// Blank and duplicate (case-insensitive) names are rejected here, not just
	// by the current UI caller (manage_groups_dialog.cpp): the database has no
	// blank-name constraint and its UNIQUE index only folds ASCII case (see
	// groupNameExists()), so without this check e.g. "Café" and "café" would
	// both succeed and be indistinguishable in the group filter combo and
	// per-trip "Set Group" menu.
	if (trimmed.isEmpty() || groupNameExists(sql, trimmed, 0)) {
		Logger::log(Logger::Trace, "DB", QStringLiteral("insertGroup: rejected (blank or duplicate name)"));
		return 0;
	}
	// New groups go after every existing one rather than at sort_order 0 (which
	// would otherwise bury them at the top of an already-customized order).
	int nextSortOrder = 0;
	sqlite3_stmt* maxStmt = nullptr;
	if (sqlite3_prepare_v2(sql, "SELECT COALESCE(MAX(sort_order), -1) + 1 FROM trip_groups", -1, &maxStmt, nullptr) == SQLITE_OK) {
		if (sqlite3_step(maxStmt) == SQLITE_ROW)
			nextSortOrder = sqlite3_column_int(maxStmt, 0);
		sqlite3_finalize(maxStmt);
	}

	const bool ok = execStatement(sql, "INSERT INTO trip_groups (name, sort_order) VALUES (?, ?)", QStringLiteral("insertGroup"),
		[&](sqlite3_stmt* stmt) {
			bindText(stmt, 1, trimmed);
			sqlite3_bind_int(stmt, 2, nextSortOrder);
		});
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
	const bool ok = execStatement(sql, "UPDATE trip_groups SET name = ? WHERE id = ?", QStringLiteral("renameGroup(%1)").arg(groupId),
		[&](sqlite3_stmt* stmt) {
			bindText(stmt, 1, trimmed);
			sqlite3_bind_int(stmt, 2, groupId);
		});
	if (ok)
		Logger::logf(Logger::Trace, "DB", "renameGroup(%d): renamed to \"%s\"", groupId, qUtf8Printable(trimmed));
	return ok;
}

bool deleteGroup(sqlite3* sql, int groupId) {
	// Ungroup member trips first, then remove the group itself. Both statements
	// must commit together -- without an explicit transaction, a failure on the
	// second leaves trips permanently pointing at a group_id that no longer
	// exists in trip_groups.
	const QString context = QStringLiteral("deleteGroup(%1)").arg(groupId);
	Logger::logf(Logger::Trace, "DB", "deleteGroup(%d): starting delete", groupId);
	const bool ok = inTransaction(sql, context, [&]() {
		auto bindGroup = [&](sqlite3_stmt* stmt) { sqlite3_bind_int(stmt, 1, groupId); };
		return execStatement(sql, "UPDATE trips SET group_id = NULL WHERE group_id = ?", context, bindGroup)
			&& execStatement(sql, "DELETE FROM trip_groups WHERE id = ?", context, bindGroup);
	});
	if (ok)
		Logger::logf(Logger::Trace, "DB", "deleteGroup(%d): delete committed", groupId);
	return ok;
}

bool reorderGroups(sqlite3* sql, const std::vector<int>& orderedGroupIds) {
	Logger::logf(Logger::Trace, "DB", "reorderGroups: persisting order for %zu group(s)", orderedGroupIds.size());
	const QString context = QStringLiteral("reorderGroups");
	const bool ok = inTransaction(sql, context, [&]() {
		sqlite3_stmt* stmt = prepareStatement(sql, "UPDATE trip_groups SET sort_order = ? WHERE id = ?", context);
		if (!stmt)
			return false;
		bool stepped = true;
		for (int i = 0; stepped && i < (int)orderedGroupIds.size(); i++) {
			sqlite3_bind_int(stmt, 1, i);
			sqlite3_bind_int(stmt, 2, orderedGroupIds[i]);
			stepped = sqlite3_step(stmt) == SQLITE_DONE;
			sqlite3_reset(stmt);
		}
		sqlite3_finalize(stmt);
		return stepped;
	});
	if (ok)
		Logger::log(Logger::Trace, "DB", QStringLiteral("reorderGroups: order committed"));
	return ok;
}

bool setTripGroup(sqlite3* sql, int tripId, int groupId) {
	const bool ok = execStatement(sql, "UPDATE trips SET group_id = ? WHERE id = ?", QStringLiteral("setTripGroup(trip %1)").arg(tripId),
		[&](sqlite3_stmt* stmt) {
			if (groupId > 0)
				sqlite3_bind_int(stmt, 1, groupId);
			else
				sqlite3_bind_null(stmt, 1);
			sqlite3_bind_int(stmt, 2, tripId);
		});
	if (ok)
		Logger::logf(Logger::Trace, "DB", "setTripGroup(trip %d): group set to %d", tripId, groupId);
	return ok;
}
