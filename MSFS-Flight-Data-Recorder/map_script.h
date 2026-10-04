#pragma once

#include <QString>

#include <utility>
#include <vector>

#include "trip_dataset.h"

// The JavaScript calls MapWidget runs in resources/map.html, built from trip
// data with no WebEngine involved. Each returns one complete statement; data
// is passed as compact JSON, so any string is escaped correctly.

// Most trajectory points sent to the page: Leaflet bogs down rendering more
// than about this many polyline segments.
constexpr int MAP_MAX_TRAJECTORY_POINTS = 3000;

// "window._x=...;" -- sets a page global to a string.
QString mapSetStringJs(const QString& jsVariable, const QString& value);
// setTrajectory({lats, lngs, idxs, version}): the (lat, lng) points thinned
// to at most MAP_MAX_TRAJECTORY_POINTS (see decimatedIndices() in
// trip_dataset.h), as parallel arrays -- more compact than one object per
// point and faster to iterate in JS. idxs are the original sample indices, so
// the page reports correct indices back for the cursor and visible range;
// version comes back with each visible range (see MapBridge::rangeChanged()).
QString mapSetTrajectoryJs(const std::vector<std::pair<double, double>>& coords, int version);
// appendPoints([{lat, lng}, ...]): live points added to the trajectory.
QString mapAppendPointsJs(const std::vector<std::pair<double, double>>& coords);
QString mapSetLiftoffsJs(const std::vector<LiftoffPoint>& liftoffs);
QString mapSetTouchdownsJs(const std::vector<TouchdownPoint>& touchdowns);
QString mapSetEventsJs(const std::vector<TripEvent>& events);
QString mapSetEventsVisibleJs(bool visible);
// setOverview([...]): one departure->destination segment per trip, with its
// group ("Ungrouped" for group 0) for the legend and colors.
QString mapSetOverviewJs(const std::vector<TripSummary>& trips);
