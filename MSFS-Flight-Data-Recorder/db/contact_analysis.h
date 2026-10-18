#pragma once

#include "trip_dataset.h"

#include <QDateTime>
#include <QString>

#include <optional>

// What the map popup and the AI analysis prompt know about each liftoff and
// touchdown beyond its own row, derived from the trip's samples, so they
// cover every trip ever recorded: the conditions at that moment, when the
// trip loads (completeTripDataset()), and the final trajectory around it,
// when its AI analysis asks for it (MapBridge).

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

// Fills every liftoff's and touchdown's conditions from dataset.points (the
// sample at or after its zuluTime, sampleAtOrAfter()), and every touchdown's
// touchdownRate and crashed: a reset within 10 s after the touchdown, with no
// other liftoff or touchdown before it. A touchdown whose own sample is a
// reset is one too, so a teleport or slew onto the ground also reads as a
// crash: the samples can't tell the two apart. First drops every liftoff whose
// sample is a reset (see contactProfile()) with no touchdown after it, at or
// before the reset (which makes it a bounce before a crash, kept without conditions:
// nothing of it was recorded before the reset): the recorder logs one when the
// simulator's reset after a crash puts the aircraft back in the air, but it
// was never a takeoff. Its row stays in the database; only the loaded trip
// (map, KML) leaves it out.
void analyzeContacts(TripDataset& dataset);

// contact's (one of dataset's touchdowns if touchdown, else liftoffs) final
// trajectory from dataset.points. Its gates end at the trip's neighboring
// contacts: a touchdown's approach starts at the previous contact, a
// liftoff's climb-out ends at the next one, so a touch-and-go's circuit isn't
// counted twice. So does its segment (SegmentWeather): the weather and ride
// from the last crossing of 1000 ft above a touchdown down to it, or from a
// liftoff up to the first crossing of 1000 ft above it, or to that
// neighboring contact if it comes first, and only within 5 nm of the
// contact. Its series covers fixed seconds around the contact (15 s before a
// touchdown or 10 s before a liftoff, through 10 s after either), whatever
// happened in them, so a bounce, a go-around or a touch-and-go shows in its
// on-ground state, height and engine values for the model to judge; its
// highest terrain, the 5 nm before or after it, past those contacts too.
// None of it reaches past a reset: a sample the
// aircraft jumped to rather than flew to, further from the one before it
// than both's vertical and ground speeds allow (the simulator's reset after a
// crash, which can't be turned off, a teleport or slew), so the samples
// after a crash, of a flight that was never flown, aren't judged with it;
// nor do its conditions' wind range (analyzeContacts()) and touchdownRate.
// A contact after the trip's last sample is
// measured from that sample (sampleAtOrAfter()), so its trajectory ends
// before it. nullopt when none can be built: a trip without samples, or a
// point whose seconds from the contact can't be read (its sample's or the
// contact's time unparsable), which would leave the model a trajectory it
// can't place in time, or a liftoff whose sample is a reset (a bounce just
// before a crash, analyzeContacts()).
std::optional<ContactProfile> contactProfile(const TripDataset& dataset, const RunwayContactPoint& contact, bool touchdown);
