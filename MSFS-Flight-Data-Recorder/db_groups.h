#pragma once

#include "trip_dataset.h"

#include <vector>

struct sqlite3;

// Read/write queries against the trip_groups table and the trips.group_id
// column. Plain sqlite3 in, plain structs out -- mirrors db_history.h.
// Callers open/close their own connection (connect_db_readonly() for reads,
// connect_db_readwrite() for writes), same convention as deleteTripData().

// Ordered alphabetically by name; each group's tripCount is the number of
// trips currently assigned to it.
std::vector<TripGroup> queryAllGroups(sqlite3* sql);

// Creates a new group. Returns its new id, or 0 on failure.
int insertGroup(sqlite3* sql, const QString& name);

bool renameGroup(sqlite3* sql, int groupId, const QString& newName);

// Deletes the group and un-assigns (sets group_id to NULL) any trips that
// were in it.
bool deleteGroup(sqlite3* sql, int groupId);

// Assigns tripId to groupId. Pass groupId = 0 to unassign (ungroup) the trip.
bool setTripGroup(sqlite3* sql, int tripId, int groupId);
