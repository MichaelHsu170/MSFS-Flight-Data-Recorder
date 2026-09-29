#pragma once

#include "trip_dataset.h"

#include <vector>

struct sqlite3;

// Which table a liftoff/touchdown row lives in.
enum class CONTACT_TABLE { LIFTOFFS, TOUCHDOWNS };

// Queries against the existing trips/trip_data tables -- all read-only except
// deleteTripData() and saveAnalysisReport() below. Plain sqlite3 in, plain
// structs out -- no Qt UI dependency, so this is reusable outside the Trip
// History panel (e.g. from a test).
std::vector<TripSummary> queryAllTrips(sqlite3* sql, int liveTripId);
TripDataset queryTripData(sqlite3* sql, int tripId);
std::vector<LiftoffPoint> queryLiftoffs(sqlite3* sql, int tripId);
std::vector<TouchdownPoint> queryTouchdowns(sqlite3* sql, int tripId);
std::vector<TripEvent> queryEvents(sqlite3* sql, int tripId);

// trip_events only stores a timestamp, not a position -- resolves each
// event's position (and sampleIndex) to the first sample in dataset.points
// at or after the event's zuluTime (the last sample for an event after the
// final one; unchanged if there are no samples), comparing zuluTime strings
// lexicographically (every row shares the same "%04d-%02d-%02dT..." format
// and timezone). Call after populating dataset.points and dataset.events.
void resolveEventPositions(TripDataset& dataset);

// Deletes all rows in trip_data, trip_events, trip_liftoffs, trip_touchdowns,
// and trips for the given trip id. Caller opens and closes the (readwrite) connection.
// Returns true on success.
bool deleteTripData(sqlite3* sql, int tripId);

// Stores an AI analysis report on a liftoff/touchdown row (its
// analysis_report column), replacing any earlier one. rowId <= 0 is ignored
// (false); a row id that doesn't exist changes nothing. Logs the outcome;
// false on failure.
bool saveAnalysisReport(sqlite3* sql, CONTACT_TABLE table, int rowId, const QString& report);
