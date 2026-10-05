#pragma once

#include "trip_dataset.h"

#include <vector>

struct sqlite3;

// Read/write queries against the trip_groups table and the trips.group_id
// column. Plain sqlite3 in, plain structs out -- mirrors db_history.h.
// Callers open their own connection (a DbConnection, see db_connection.h:
// openForReading() for reads, openForWriting() for writes), same convention
// as deleteTripData().

// Ordered by the user-customized sort_order (ties broken alphabetically --
// only relevant for groups created before the sort_order column existed,
// which the migration gave sort_order 0, and never reordered since); each
// group's tripCount is the number of trips currently assigned to it.
std::vector<TripGroup> queryAllGroups(sqlite3* sql);

// True if a group other than excludeGroupId (0: none) already has name,
// compared case-insensitively with full Unicode case folding (unlike
// SQLite's ASCII-only COLLATE NOCASE).
bool groupNameExists(sqlite3* sql, const QString& name, int excludeGroupId);
// Creates a new group, appended after every existing group's sort_order.
// Returns its new id, or 0 on failure -- including a blank (post-trim) name
// or one that already exists (case-insensitively).
int insertGroup(sqlite3* sql, const QString& name);

// Persists a new display order: assigns each id in orderedGroupIds a
// sort_order equal to its index in the vector. Silently ignores any id that
// no longer exists (e.g. deleted concurrently); returns false only if a
// statement itself fails to prepare/execute.
bool reorderGroups(sqlite3* sql, const std::vector<int>& orderedGroupIds);

// Returns false (without changing anything) for a blank (post-trim) name or
// one that collides case-insensitively with a different existing group.
bool renameGroup(sqlite3* sql, int groupId, const QString& newName);

// Deletes the group and un-assigns (sets group_id to NULL) any trips that
// were in it.
bool deleteGroup(sqlite3* sql, int groupId);

// Assigns tripId to groupId. Pass groupId = 0 to unassign (ungroup) the trip.
bool setTripGroup(sqlite3* sql, int tripId, int groupId);
