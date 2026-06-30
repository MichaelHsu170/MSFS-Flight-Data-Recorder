#pragma once

#include "trip_dataset.h"

#include <vector>

struct sqlite3;

// Read-only queries against the existing trips/trip_data tables. Plain sqlite3
// in, plain structs out — no Qt UI dependency, so this is reusable outside the
// Trip History panel (e.g. from a future CLI or test).
std::vector<TripSummary> queryAllTrips(sqlite3* sql, int liveTripId);
TripDataset queryTripData(sqlite3* sql, int tripId);
std::vector<TouchdownPoint> queryTouchdowns(sqlite3* sql, int tripId);
std::vector<TripEvent> queryEvents(sqlite3* sql, int tripId);
