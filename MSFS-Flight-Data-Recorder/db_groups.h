#pragma once

#include "trip_dataset.h"

#include <vector>

struct sqlite3;

// Read/write queries against the trip_groups table and the trips.group_id
// column. Plain sqlite3 in, plain structs out -- mirrors db_history.h.
// Callers open/close their own connection (connect_db_readonly() for reads,
// connect_db_readwrite() for writes), same convention as deleteTripData().

// Ordered by the user-customized sort_order (ties broken alphabetically --
// only relevant for groups that have never been reordered, which all share
// sort_order 0); each group's tripCount is the number of trips currently
// assigned to it.
std::vector<TripGroup> queryAllGroups(sqlite3* sql);

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
