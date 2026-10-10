#pragma once

#include "trip_dataset.h"

#include <QDateTime>
#include <QString>

// What the map popup and the AI analysis prompt know about each liftoff and
// touchdown beyond its own row: the conditions at that moment and the final
// trajectory around it, both derived from the trip's samples when the trip
// loads (completeTripDataset()), so they cover every trip ever recorded.

// The sun's elevation (degrees above the horizon, without refraction) at
// latitude/longitude at utc, by NOAA's solar position equations. Latitude is
// why this is computed rather than left to the model: at the same clock time
// a high-latitude airport can have the midnight sun or polar night.
double sunElevationDegrees(double latitude, double longitude, const QDateTime& utc);

// Density altitude (ft) by the standard approximation
// PA + 118.8 ft/degC x (OAT - ISA temperature at PA).
double densityAltitudeFeet(double pressureAltitudeFt, double temperatureC);

// Names for the SURFACE TYPE and SURFACE CONDITION enums (MSFS 2024 SDK,
// Aircraft Misc Variables); empty for a value outside them (255: none) or NaN.
QString surfaceTypeName(double value);
QString surfaceConditionName(double value);

// The AMBIENT PRECIP STATE mask (2 none, 4 rain, 8 snow) as "none", "rain",
// "snow" or "rain and snow"; empty for NaN or a mask with none of these bits.
QString precipitationName(double mask);

// Fills every liftoff's and touchdown's conditions and profile from
// dataset.points (the sample at or after its zuluTime, sampleAtOrAfter()).
// The profile's window ends at the trip's neighboring contacts: a touchdown's
// approach starts at the previous contact, a liftoff's climb-out ends at the
// next one, so a touch-and-go's circuit isn't counted twice.
void analyzeContacts(TripDataset& dataset);
