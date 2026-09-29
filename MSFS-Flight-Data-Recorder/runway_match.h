#pragma once

#include "types.h"

#include <functional>
#include <vector>

// Which of one airport's runways a liftoff/touchdown point lies on, and where
// on it -- the geometry half of the airport lookup (airport_lookup.cpp), which
// owns the SimConnect requests around it.
struct RUNWAY_MATCH {
	// Every runway whose footprint (its strict length x width rectangle)
	// contains the point, with direction, operational heading and
	// threshold/centerline distances filled in. airport_lookup.cpp picks the one
	// best aligned with the direction of travel.
	std::vector<RUNWAY_OPERATION> candidates;
	// True if any runway's padded "margin rectangle" (RUNWAY_MARGIN_LENGTH_M
	// past each end, RUNWAY_MARGIN_WIDTH_M past each edge -- see
	// runway_match.cpp) contains the point. Unlike a strict hit this never
	// selects a runway: it only means "this airport, near a runway, but not
	// confidently on one".
	bool any_margin_hit = false;
};

// airport's runways must be populated; each one's start_points[] (the two
// physical runway ends) is computed and stored as a side effect. point is the
// liftoff/touchdown position; bearing_tra the true direction of travel,
// deciding which runway end is in use. is_touchdown applies the displaced-
// threshold correction (a takeoff may use the pavement before the threshold,
// a landing is measured from it). trace receives one diagnostic line per
// decision, for the debug log.
RUNWAY_MATCH match_runways(AIRPORT& airport, const COORDINATE& point, double bearing_tra, bool is_touchdown,
	const std::function<void(const char*)>& trace);
