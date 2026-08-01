#include "db_groups.h"

#include "sqlite3.h"

std::vector<TripGroup> queryAllGroups(sqlite3* sql) {
	std::vector<TripGroup> groups;

	const char* stmt_txt =
		"SELECT g.id, g.name, (SELECT COUNT(*) FROM trips t WHERE t.group_id = g.id) "
		"FROM trip_groups g ORDER BY g.name COLLATE NOCASE";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK)
		return groups;

	while (sqlite3_step(stmt) == SQLITE_ROW) {
		TripGroup group;
		group.id = sqlite3_column_int(stmt, 0);
		const unsigned char* text = sqlite3_column_text(stmt, 1);
		group.name = text ? QString::fromUtf8(reinterpret_cast<const char*>(text)) : QString();
		group.tripCount = sqlite3_column_int(stmt, 2);
		groups.push_back(group);
	}
	sqlite3_finalize(stmt);
	return groups;
}

int insertGroup(sqlite3* sql, const QString& name) {
	const char* stmt_txt = "INSERT INTO trip_groups (name) VALUES (?)";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK)
		return 0;
	QByteArray utf8 = name.toUtf8();
	sqlite3_bind_text(stmt, 1, utf8.constData(), utf8.size(), SQLITE_TRANSIENT);
	bool ok = sqlite3_step(stmt) == SQLITE_DONE;
	sqlite3_finalize(stmt);
	return ok ? (int)sqlite3_last_insert_rowid(sql) : 0;
}

bool renameGroup(sqlite3* sql, int groupId, const QString& newName) {
	const char* stmt_txt = "UPDATE trip_groups SET name = ? WHERE id = ?";
	sqlite3_stmt* stmt = nullptr;
	if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK)
		return false;
	QByteArray utf8 = newName.toUtf8();
	sqlite3_bind_text(stmt, 1, utf8.constData(), utf8.size(), SQLITE_TRANSIENT);
	sqlite3_bind_int(stmt, 2, groupId);
	bool ok = sqlite3_step(stmt) == SQLITE_DONE;
	sqlite3_finalize(stmt);
	return ok;
}

bool deleteGroup(sqlite3* sql, int groupId) {
	// Ungroup member trips first, then remove the group itself.
	const char* stmts[] = {
		"UPDATE trips SET group_id = NULL WHERE group_id = ?",
		"DELETE FROM trip_groups WHERE id = ?",
	};
	for (const char* stmt_txt : stmts) {
		sqlite3_stmt* stmt = nullptr;
		if (sqlite3_prepare_v2(sql, stmt_txt, -1, &stmt, nullptr) != SQLITE_OK) {
			if (stmt) sqlite3_finalize(stmt);
			return false;
		}
		sqlite3_bind_int(stmt, 1, groupId);
		bool ok = sqlite3_step(stmt) == SQLITE_DONE;
		sqlite3_finalize(stmt);
		if (!ok)
			return false;
	}
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
	return ok;
}
