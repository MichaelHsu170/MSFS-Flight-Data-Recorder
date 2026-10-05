#pragma once

#include <QString>

#include "trip_dataset.h"

// Serializes dataset's trajectory (3D line + time-animated gx:Track) and
// liftoff/touchdown/event placemarks into a KML document and writes it to
// fileName, for exploring/annotating a recorded trip in Google Earth.
// Returns true on success; on failure returns false and, if errorMessage is
// non-null, fills it with a human-readable reason.
bool exportTripDatasetToKmlFile(const TripDataset& dataset, const QString& fileName, QString* errorMessage = nullptr);
