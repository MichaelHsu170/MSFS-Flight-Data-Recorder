#include "runway_match.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>

namespace {

// Padding applied past each runway end / each runway edge to build the
// "margin rectangle" -- a touchdown/liftoff inside this but outside the strict
// runway rectangle is judged "on this airport, but not confidently on a
// runway" rather than a clean runway match. Loosely based on real-world
// runway safety-area dimensions.
const double RUNWAY_MARGIN_LENGTH_M = 200; // past each end
const double RUNWAY_MARGIN_WIDTH_M = 60;   // past each edge

void tracef(const std::function<void(const char*)>& trace, const char* fmt, ...) {
	char buf[512];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	trace(buf);
}

// Single-corner-referenced polar footprint check shared by the margin and
// strict rectangles: anchor is the rectangle's near end on the centerline,
// extending length along runway_heading and width/2 to either side. Sets
// distance (anchor to point, meters) and limit (how far the rectangle reaches
// in that direction); the point is inside when distance <= limit. Templated on
// the dimension type so the strict check computes in RUNWAY's float and the
// margin check in double.
template <typename Length>
void footprint(COORDINATE anchor, float runway_heading, Length length, Length width, const COORDINATE& point,
	double& distance, double& limit) {
	double angle = atan(width / 2 / length) / V_PI * 180;
	double bearing = anchor.bearing2Coordinate(point);
	distance = anchor.distanceInKm2Coordinate(point) * 1000;
	double diff_bearing = bearing_difference(bearing, runway_heading);
	limit = 0;
	if (diff_bearing >= 0 && diff_bearing <= angle)
		limit = length / cos(diff_bearing / 180 * V_PI);
	else if (diff_bearing > angle && diff_bearing <= 90)
		limit = width / 2 / sin(diff_bearing / 180 * V_PI);
}

}

RUNWAY_MATCH match_runways(AIRPORT& airport, const COORDINATE& point, double bearing_tra, bool is_touchdown,
	const std::function<void(const char*)>& trace) {
	RUNWAY_MATCH result;
	for (int i = 0; i < airport.n_runways; i++) {
		RUNWAY* rwy = &airport.runways[i];
		// Human-readable "06L/24R"-style id for trace output -- numbers[]/
		// designators[] alone (e.g. 6/24) don't carry the leading zero or
		// the L/R/C side letter that a pilot would recognize.
		std::string rwy_id = rwy->runway_code_generator(true) + "/" + rwy->runway_code_generator(false);
		double heading = rwy->heading;
		rwy->start_points[1] = rwy->coordinate.destinationWithDistanceAndBearing(rwy->length / 2000, heading);
		heading = wrap_bearing(heading - 180);
		rwy->start_points[0] = rwy->coordinate.destinationWithDistanceAndBearing(rwy->length / 2000, heading);

		if (!result.any_margin_hit) {
			// Same footprint check as the strict one below, but anchored on a
			// point shifted RUNWAY_MARGIN_LENGTH_M further past threshold 0
			// (rather than reusing start_points[0] as-is), with length/width
			// padded by RUNWAY_MARGIN_LENGTH_M/RUNWAY_MARGIN_WIDTH_M -- shifting
			// the anchor, not just growing the dimensions, is what lets this
			// catch points short of threshold 0 or beyond the runway's side
			// edges near its ends, not only points beyond the far threshold.
			double margin_length = rwy->length + 2 * RUNWAY_MARGIN_LENGTH_M;
			double margin_width = rwy->width + 2 * RUNWAY_MARGIN_WIDTH_M;
			COORDINATE margin_start = rwy->coordinate.destinationWithDistanceAndBearing(
				(rwy->length / 2 + RUNWAY_MARGIN_LENGTH_M) / 1000, heading);
			double margin_distance = 0, margin_distance2 = 0;
			footprint(margin_start, rwy->heading, margin_length, margin_width, point, margin_distance, margin_distance2);
			if (margin_distance <= margin_distance2) {
				result.any_margin_hit = true;
				tracef(trace, "Runway candidate %d/%d: %s margin-rectangle hit (distance=%.1fm, distance2=%.1fm)",
					i + 1, airport.n_runways, rwy_id.c_str(), margin_distance, margin_distance2);
			}
		}

		double distance = 0, distance2 = 0;
		footprint(rwy->start_points[0], rwy->heading, rwy->length, rwy->width, point, distance, distance2);

		tracef(trace, "Runway candidate %d/%d: %s (heading=%.1f, len=%.0f, width=%.0f): distance=%.1fm, distance2=%.1fm -> %s",
			i + 1, airport.n_runways, rwy_id.c_str(), rwy->heading, rwy->length, rwy->width,
			distance, distance2, (distance <= distance2) ? "pass" : "fail (outside runway footprint)");

		if (distance <= distance2) {
			RUNWAY_OPERATION candidate;
			candidate.index = i;
			candidate.is_primary = bearing_difference(bearing_tra, rwy->heading) < 90;

			heading = rwy->heading;
			int index = 0;
			if (!candidate.is_primary) {
				heading = wrap_bearing(heading - 180);
				index = 1;
			}
			candidate.heading = (int)wrap_bearing((int)(heading + 0.5));

			candidate.diff_bearing_tra = bearing_difference(bearing_tra, heading);

			// Along-track (from this end's threshold, down the runway) and
			// cross-track (+right/-left of the centerline) distances of the
			// point on the great circle through the threshold along heading.
			// The along-track one is atan(tan d * cos(dtheta)), the same value
			// as acos(cos d / cos xt) but exact for a point near the threshold
			// or on the centerline.
			COORDINATE& threshold = rwy->start_points[index];
			const double d = threshold.distanceInKm2Coordinate(point) / EARTHRADIUSKM;
			const double dtheta = (threshold.bearing2Coordinate(point) - heading) * V_PI / 180;
			const double radius_ft = EARTHRADIUSKM * 1000 * M_2_FT;
			candidate.distances[0] = atan2(sin(d) * cos(dtheta), cos(d)) * radius_ft;
			candidate.distances[1] = asin(sin(d) * sin(dtheta)) * radius_ft;
			candidate.distances_percent[0] = candidate.distances[0] / rwy->length / M_2_FT;
			candidate.distances_percent[1] = candidate.distances[1] / rwy->width * 2 / M_2_FT;
			// Displaced-threshold correction applies to touchdowns only: a
			// landing's "usable region" is genuinely bounded by the marked
			// threshold (touching down before it is a short/non-standard
			// landing, worth surfacing), but a takeoff roll may legitimately
			// start at the physical runway end -- the pre-threshold pavement
			// is still valid, usable surface for a departure, so a liftoff's
			// distance/percent stay measured from the physical end.
			if (is_touchdown) {
				// enable==0 means this runway has no threshold data, so the
				// offset must stay 0 rather than be applied. Intentionally not
				// clamped at 0 after subtraction: a touchdown short of the
				// marked threshold (e.g. on a blast pad) reporting a negative
				// distance is meaningful, not an error.
				const float primary_offset_m = rwy->primary_threshold_enable ? rwy->primary_threshold_offset_m : 0;
				const float secondary_offset_m = rwy->secondary_threshold_enable ? rwy->secondary_threshold_offset_m : 0;
				const float threshold_offset_m = candidate.is_primary ? primary_offset_m : secondary_offset_m;
				double distance_before_correction_ft = candidate.distances[0];
				candidate.distances[0] -= threshold_offset_m * M_2_FT;
				// Percent is of landing distance available (physical length
				// minus both ends' displaced-threshold offsets), not full
				// physical length, so 100% still means "the far threshold" now
				// that the numerator starts from the near threshold instead of
				// the near physical end.
				double landing_distance_available_m = rwy->length - primary_offset_m - secondary_offset_m;
				if (landing_distance_available_m <= 0)
					landing_distance_available_m = rwy->length;
				candidate.distances_percent[0] = candidate.distances[0] / (landing_distance_available_m * M_2_FT);
				// Logged unconditionally (even when offset==0, i.e. no threshold data)
				// so a captured debug log always shows what this feature did with a
				// given touchdown -- this is the line to check against a runway with a
				// known displaced threshold to confirm the popup's corrected "Threshold"
				// figure is right, independent of the raw PAVEMENT parsing logged in
				// airport_lookup.cpp.
				tracef(trace, "Runway candidate %d/%d: %s touchdown threshold correction: end=%s, offset=%.1fm, distance %.1fft -> %.1fft (%.1f%%), LDA=%.1fm",
					i + 1, airport.n_runways, rwy_id.c_str(), candidate.is_primary ? "primary" : "secondary",
					threshold_offset_m, distance_before_correction_ft, candidate.distances[0], candidate.distances_percent[0] * 100,
					landing_distance_available_m);
			}

			tracef(trace, "Runway candidate %d/%d: %s accepted, is_primary=%d, diff_bearing_tra=%.1f",
				i + 1, airport.n_runways, rwy_id.c_str(), candidate.is_primary ? 1 : 0, candidate.diff_bearing_tra);
			result.candidates.push_back(candidate);
		}
	}
	return result;
}
