#include "airport_lookup.h"
#include "gui_notify.h"
#include "runway_match.h"

#include <cstdlib>
#include <cstring>

#include <algorithm>
#include <string>
#include <vector>

void start_facility_lookup(struct STATUS* status, LOOKUP_TARGET target, const COORDINATE& position, int heading, const COORDINATE* approach) {
	status->lookup.target = target;
	status->lookup.pending = TRUE;
	status->lookup.trip_id = status->id_trip;
	status->lookup.coordinate = position;
	status->lookup.heading = heading;
	if (approach != nullptr)
		status->lookup.approach = *approach;
	else
		status->lookup.approach.clear();
	SimConnect_RequestFacilitiesList_EX1(status->hSimConnect, SIMCONNECT_FACILITY_LIST_TYPE_AIRPORT, REQUEST_AIRPORTS);
	SimConnect_GetLastSentPacketID(status->hSimConnect, &status->lookup.send_id);
}

// Resolves which AIRPORT slot the in-flight facility lookup (AIRPORT_LIST /
// FACILITY_DATA / FACILITY_DATA_END / EXCEPTION) targets. Deliberately reads
// only lookup.target -- captured once, by start_facility_lookup() --
// rather than any live/mutable state such as status->departure.runway_act.index.
// That field used to be used for this instead, but it can be reset to -1 by
// a *later* trip's status->departure.clear() while an older trip's liftoff-
// marker or destination lookup is still in flight, which would misattribute
// the stale response to &status->departure. See lookup.target in
// types.h for the full history.
AIRPORT* facility_lookup_target(struct STATUS* status) {
	if (status->lookup.target == LOOKUP_TARGET::DEPARTURE)
		return &status->departure;
	// Liftoff-marker and touchdown/destination lookups use separate scratch
	// objects (status->lookup.liftoff_scratch vs. status->destination) even though
	// they're never in flight at the same time -- see destination's/
	// liftoff_scratch's declarations in types.h for why they're kept apart
	// instead of sharing one field.
	return status->lookup.target == LOOKUP_TARGET::LIFTOFF ? &status->lookup.liftoff_scratch : &status->destination;
}

// Human-readable label for gui_log_printf tracing, matching whichever slot
// facility_lookup_target() returned for this same in-flight lookup.
static const char* facility_lookup_target_label(struct STATUS* status, AIRPORT* apt) {
	return (apt == &status->departure) ? "departure" : status->lookup.target == LOOKUP_TARGET::LIFTOFF ? "liftoff" : "destination";
}

// Issues the SimConnect facility-data (runway) request for
// lookup.top[idx] into the current lookup's scratch AIRPORT slot,
// and records idx as the candidate the walk is now on. Shared by the
// AIRPORT_LIST handler (first candidate) and the FACILITY_DATA_END handler
// (advancing to the next candidate after a candidate with no strict runway
// match) -- see the multi-candidate walk described where
// lookup.candidate_index is declared in types.h.
static void facility_lookup_request_candidate(struct STATUS* status, int idx) {
	status->lookup.candidate_index = idx;
	char* ident = status->lookup.top[idx].ident;
	char* region = status->lookup.top[idx].region;
	AIRPORT* apt = facility_lookup_target(status);
	copy_cstr(apt->icao, ident);
	copy_cstr(apt->region, region);
	gui_log_printf(status, GUI_LOG_TRACE, "Requesting facility data for candidate #%d %s (%s) into %s slot",
		idx + 1, apt->icao, apt->region, facility_lookup_target_label(status, apt));
	// The definition's fields are server-side, per-connection state -- only
	// need to be registered once per connection, not once per lookup (see
	// lookup.runway_definition_added in types.h).
	if (!status->lookup.runway_definition_added) {
		status->lookup.runway_definition_added = TRUE;
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "OPEN AIRPORT");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "NAME64");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "MAGVAR");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "N_RUNWAYS");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "OPEN RUNWAY");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "LENGTH");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "WIDTH");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "HEADING");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "PRIMARY_NUMBER");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "SECONDARY_NUMBER");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "PRIMARY_DESIGNATOR");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "SECONDARY_DESIGNATOR");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "LATITUDE");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "LONGITUDE");
		// Displaced-threshold offsets -- nested PAVEMENT child records, matched
		// back to this runway in the FACILITY_DATA_PAVEMENT case below via
		// ParentUniqueRequestId. Request order (primary before secondary)
		// is relied on there to tell the two apart.
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "OPEN PRIMARY_THRESHOLD");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "LENGTH");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "WIDTH");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "ENABLE");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "CLOSE PRIMARY_THRESHOLD");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "OPEN SECONDARY_THRESHOLD");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "LENGTH");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "WIDTH");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "ENABLE");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "CLOSE SECONDARY_THRESHOLD");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "CLOSE RUNWAY");
		SimConnect_AddToFacilityDefinition(status->hSimConnect, DEFINITION_RUNWAYS, "CLOSE AIRPORT");
	}
	SimConnect_RequestFacilityData_EX1(status->hSimConnect, DEFINITION_RUNWAYS, REQUEST_RUNWAYS, ident, region);
	SimConnect_GetLastSentPacketID(status->hSimConnect, &status->lookup.send_id);
}

namespace {

// Ends the in-flight lookup: clears lookup.pending and picks up a liftoff/
// touchdown that happened while it was in flight and had its own request
// skipped (see request_next_touchdown_facility_lookup() in flight_phase.cpp;
// a no-op if the trip has ended or nothing is queued).
void end_lookup(struct STATUS* status) {
	status->lookup.pending = FALSE;
	request_next_touchdown_facility_lookup(status);
}

// Frees the slot's runways buffer and ends the lookup however the handler
// exits -- including a db_exception thrown by one of the db_* writes in
// on_lookup_resolved(), which would otherwise unwind straight past the
// cleanup to MyDispatchProc's catch, leaking the runways and leaving
// lookup.pending stuck true.
struct LookupEnd {
	struct STATUS* status;
	AIRPORT* rep;  // whose runways to free; nullptr if none were requested
	// Set to advance the multi-candidate walk to lookup.top[]'s next entry
	// (see the no-strict-match handling in lookup_on_facility_data_end()) --
	// the walk is still the same logical lookup, so lookup.pending must stay
	// TRUE and the queue must not be drained until the walk actually
	// finishes (a strict match, a cached margin/identity/coordinate-only
	// fallback, or exhausting the list).
	bool more_candidates_pending = false;
	~LookupEnd() {
		if (rep != nullptr && rep->runways != NULL) {
			free(rep->runways);
			rep->runways = NULL;
		}
		if (!more_candidates_pending)
			end_lookup(status);
	}
};

}

void add_nearest_airports(AIRPORT_LOOKUP::CANDIDATE (&top)[AIRPORT_LOOKUP::TOP_N], COORDINATE position,
	const SIMCONNECT_DATA_FACILITY_AIRPORT* airports, int count) {
	for (int i = 0; i < count; i++) {
		const SIMCONNECT_DATA_FACILITY_AIRPORT& airport = airports[i];
		// Real-world ICAO airport codes are always exactly 4 letters. Idents
		// longer than that (e.g. "VOLC2") identify vertiports/heliports/other
		// non-airport facilities, which are not valid touchdown/liftoff targets --
		// exclude them from nearest-airport consideration entirely.
		if (strlen(airport.Ident) != 4)
			continue;
		COORDINATE airport_loc;
		airport_loc.latitude = airport.Latitude;
		airport_loc.longitude = airport.Longitude;
		double distance = abs(position.distanceInKm2Coordinate(airport_loc));
		if (distance < top[AIRPORT_LOOKUP::TOP_N - 1].distance) {
			int pos = AIRPORT_LOOKUP::TOP_N - 1;
			while (pos > 0 && top[pos - 1].distance > distance) {
				top[pos] = top[pos - 1];
				pos--;
			}
			top[pos].distance = distance;
			copy_cstr(top[pos].ident, airport.Ident);
			copy_cstr(top[pos].region, airport.Region);
		}
	}
}

void lookup_on_airport_list(struct STATUS* status, SIMCONNECT_RECV_AIRPORT_LIST* pWxData) {
	// Drop responses for a lookup issued by a trip that has since ended --
	// id_trip only ever changes on this same dispatch thread (stop_recording()/
	// new-trip-start), so this comparison is race-free. Applying it now would
	// write stale airport data into whatever trip is active today. A large
	// facility list is split across multiple AIRPORT_LIST callbacks sharing
	// one request (see dwEntryNumber/dwOutOf below) -- only run the
	// pending-clear + next-lookup cleanup once, on the last chunk, so an
	// earlier chunk's cleanup can't race a lookup it just started back into
	// "not pending" while that new lookup is genuinely still in flight.
	if (status->lookup.trip_id != status->id_trip) {
		// Like every other terminal path below: a touchdown/departure lookup
		// may have been queued behind this (now-stale) one and would otherwise
		// sit stranded until some unrelated lookup happens to drain it.
		if (pWxData->dwEntryNumber + 1 == pWxData->dwOutOf)
			end_lookup(status);
		return;
	}
	// SimConnect splits a large facility list (e.g. every airport in loaded
	// scenery, 1000+ entries) across multiple AIRPORT_LIST callbacks that
	// share one request (dwEntryNumber counts 0..dwOutOf-1). Deciding
	// "nearest airport" independently per chunk is wrong -- a later chunk
	// that happens to contain only distant airports would conclude "not
	// found" and terminate/overwrite the lookup a second time while the
	// real match from an earlier chunk was still being resolved. Instead,
	// accumulate the running top-N nearest across all chunks in status,
	// and only act once the last chunk has been folded in.
	if (pWxData->dwEntryNumber == 0) {
		for (int k = 0; k < AIRPORT_LOOKUP::TOP_N; k++) {
			status->lookup.top[k].distance = 1e9;
			status->lookup.top[k].ident[0] = '\0';
			status->lookup.top[k].region[0] = '\0';
		}
		status->lookup.candidate_index = 0;
		status->lookup.margin_cache.found = false;
	}
	add_nearest_airports(status->lookup.top, status->lookup.coordinate, pWxData->rgData, (int)pWxData->dwArraySize);
	// Wait for the rest of the (possibly multi-chunk) list before deciding.
	if (pWxData->dwEntryNumber + 1 < pWxData->dwOutOf)
		return;
	for (int k = 0; k < AIRPORT_LOOKUP::TOP_N && status->lookup.top[k].ident[0] != '\0'; k++) {
		gui_log_printf(status, GUI_LOG_TRACE, "AIRPORT_LIST nearest #%d: %s (%s) at %.2f km",
			k + 1, status->lookup.top[k].ident, status->lookup.top[k].region, status->lookup.top[k].distance);
	}
	if (status->lookup.top[0].ident[0] != '\0') {
		// Kick off the multi-candidate walk at the nearest airport. Actual
		// runway geometry -- not ARP distance -- decides whether this (or a
		// farther candidate) is a match; see FACILITY_DATA_END below.
		facility_lookup_request_candidate(status, 0);
	} else {
		gui_log_printf(status, GUI_LOG_TRACE, "AIRPORT_LIST: no airport candidates at all; using coordinate-only fallback");
		// Terminal outcome for this lookup -- no facility data request was made,
		// so FACILITY_DATA_END will never fire to end it.
		LookupEnd end{ status, nullptr };
		on_lookup_resolved(status, facility_lookup_target(status), LOOKUP_OUTCOME::NO_AIRPORT);
	}
}

void lookup_on_facility_data(struct STATUS* status, SIMCONNECT_RECV_FACILITY_DATA* pWxData) {
	// Same staleness guard as AIRPORT_LIST above -- but no need to clear
	// lookup.pending here: FACILITY_DATA_END always follows this
	// (possibly stale) response and clears it there.
	if (status->lookup.trip_id != status->id_trip)
		return;
	switch (pWxData->Type) {
	case SIMCONNECT_FACILITY_DATA_AIRPORT:
	{
		AIRPORT* tmp = facility_lookup_target(status);
		memcpy(tmp, &pWxData->Data, sizeof(tmp->name) + sizeof(tmp->magvar) + sizeof(tmp->n_runways));
		if (tmp->n_runways < 0) {
			gui_log_printf(status, GUI_LOG_WARNING, "FACILITY_DATA_AIRPORT: negative n_runways=%d from sim; treating as 0 runways", tmp->n_runways);
			tmp->n_runways = 0;
		}
		// calloc, not malloc: any slot whose SIMCONNECT_FACILITY_DATA_RUNWAY
		// response never arrives (e.g. n_runways overstates what MSFS actually
		// sends) must read back as zero, not uninitialized heap garbage, since
		// the FACILITY_DATA_END matching loop below iterates all n_runways
		// slots unconditionally.
		tmp->runways = (RUNWAY*)calloc((size_t)tmp->n_runways, sizeof(RUNWAY));
		if (tmp->runways == NULL) {
			gui_log_printf(status, GUI_LOG_WARNING, "FACILITY_DATA_AIRPORT: malloc failed for %d runways; treating as 0 runways", tmp->n_runways);
			tmp->n_runways = 0;
		}
		gui_log_printf(status, GUI_LOG_TRACE, "FACILITY_DATA_AIRPORT: %s slot, name=%s, n_runways=%d",
			facility_lookup_target_label(status, tmp), tmp->name, tmp->n_runways);
	}
	break;
	case SIMCONNECT_FACILITY_DATA_RUNWAY:
	{
		AIRPORT* apt = facility_lookup_target(status);
		RUNWAY* rep = apt->runways;
		// UniqueRequestId is logged here (and echoed by the FACILITY_DATA_PAVEMENT
		// case below on every match) specifically so a captured debug log can be
		// used to confirm SimConnect actually hands out a distinct id per nested
		// RUNWAY record -- see the correlation assumption noted in pending_request_id's
		// declaration in types.h. If every runway at a multi-runway airport logs the
		// same UniqueRequestId here, that assumption is false and PAVEMENT matching
		// below is unreliable.
		gui_log_printf(status, GUI_LOG_TRACE, "FACILITY_DATA_RUNWAY: %s slot, ItemIndex=%lu, n_runways=%d, UniqueRequestId=%lu",
			facility_lookup_target_label(status, apt), pWxData->ItemIndex, apt->n_runways, pWxData->UniqueRequestId);
		// apt->n_runways is guaranteed >= 0 (clamped in FACILITY_DATA_AIRPORT
		// above); ItemIndex is unsigned, so comparing it directly against a
		// non-negative n_runways (rather than casting ItemIndex down to a
		// possibly-negative int) can't be bypassed by an out-of-range ItemIndex.
		if (rep == NULL || pWxData->ItemIndex >= (unsigned int)apt->n_runways) {
			gui_log_printf(status, GUI_LOG_WARNING, "FACILITY_DATA_RUNWAY: no runways buffer for ItemIndex=%lu; dropping", pWxData->ItemIndex);
			break;
		}
		memset(&rep[pWxData->ItemIndex], 0, sizeof(RUNWAY));
		// Wire payload is only placeholder..coordinate -- start_points[] and
		// the threshold/correlation fields below it are computed/populated
		// locally (start_points by match_runways(), threshold fields by the nested
		// FACILITY_DATA_PAVEMENT case below), never sent over the wire, so
		// all of them must stay excluded from this copy's size.
		memcpy((char*)&rep[pWxData->ItemIndex] + sizeof(rep->placeholder), &pWxData->Data,
			sizeof(RUNWAY) - sizeof(rep->placeholder) - sizeof(rep->start_points)
			- sizeof(rep->primary_threshold_offset_m) - sizeof(rep->secondary_threshold_offset_m)
			- sizeof(rep->primary_threshold_enable) - sizeof(rep->secondary_threshold_enable)
			- sizeof(rep->pending_request_id) - sizeof(rep->threshold_pavement_seen));
		rep[pWxData->ItemIndex].pending_request_id = pWxData->UniqueRequestId;
		rep[pWxData->ItemIndex].threshold_pavement_seen = 0;
	}
	break;
	case SIMCONNECT_FACILITY_DATA_PAVEMENT:
	{
		AIRPORT* apt = facility_lookup_target(status);
		RUNWAY* rep = apt->runways;
		// PAVEMENT is a child of RUNWAY (used for PRIMARY_THRESHOLD/
		// SECONDARY_THRESHOLD, both requested per-runway) -- unlike
		// FACILITY_DATA_RUNWAY, there's no ItemIndex identifying which
		// runway this belongs to, so it's matched via ParentUniqueRequestId
		// against the UniqueRequestId captured when that runway's own
		// FACILITY_DATA_RUNWAY record arrived, just above.
		struct { float length; float width; int enable; } pavement;
		memcpy(&pavement, &pWxData->Data, sizeof(pavement));
		RUNWAY* match = NULL;
		int match_index = -1;
		if (rep != NULL) {
			for (int i = 0; i < apt->n_runways; i++) {
				if (rep[i].pending_request_id == pWxData->ParentUniqueRequestId) {
					match = &rep[i];
					match_index = i;
					break;
				}
			}
		}
		if (match == NULL) {
			gui_log_printf(status, GUI_LOG_TRACE, "FACILITY_DATA_PAVEMENT: no runway matches ParentUniqueRequestId=%lu; dropping", pWxData->ParentUniqueRequestId);
			break;
		}
		// rwy_id/match_index/ParentUniqueRequestId are logged on every branch below
		// (not just failures) specifically so a captured debug log can be checked
		// against a runway with a known real-world displaced threshold to confirm:
		// (1) this record landed on the right runway, and (2) primary-vs-secondary
		// (order-based, see below) came out the right way round.
		std::string rwy_id = match->runway_code_generator(true) + "/" + match->runway_code_generator(false);
		// Requested field order is OPEN PRIMARY_THRESHOLD before OPEN
		// SECONDARY_THRESHOLD (see facility_lookup_request_candidate), so
		// the first PAVEMENT record for a given runway is always primary,
		// the second always secondary. This relies on SimConnect delivering
		// a runway's nested children in request-definition order -- not
		// independently confirmable from this data, hence logging both
		// offsets here for cross-checking against a known runway.
		if (match->threshold_pavement_seen == 0) {
			match->primary_threshold_offset_m = pavement.length;
			match->primary_threshold_enable = pavement.enable;
			match->threshold_pavement_seen = 1;
			gui_log_printf(status, GUI_LOG_TRACE, "FACILITY_DATA_PAVEMENT: runway[%d] %s (ParentUniqueRequestId=%lu): primary threshold offset=%.1fm, enable=%d",
				match_index, rwy_id.c_str(), pWxData->ParentUniqueRequestId, pavement.length, pavement.enable);
		} else if (match->threshold_pavement_seen == 1) {
			match->secondary_threshold_offset_m = pavement.length;
			match->secondary_threshold_enable = pavement.enable;
			match->threshold_pavement_seen = 2;
			gui_log_printf(status, GUI_LOG_TRACE, "FACILITY_DATA_PAVEMENT: runway[%d] %s (ParentUniqueRequestId=%lu): secondary threshold offset=%.1fm, enable=%d",
				match_index, rwy_id.c_str(), pWxData->ParentUniqueRequestId, pavement.length, pavement.enable);
		} else {
			gui_log_printf(status, GUI_LOG_TRACE, "FACILITY_DATA_PAVEMENT: runway[%d] %s (ParentUniqueRequestId=%lu): unexpected extra pavement record; dropping",
				match_index, rwy_id.c_str(), pWxData->ParentUniqueRequestId);
		}
	}
	break;
	default:
		break;
	}
}

void lookup_on_facility_data_end(struct STATUS* status) {
	AIRPORT* rep = facility_lookup_target(status);
	LookupEnd cleanup_guard{ status, rep };
	// Drop a response for a lookup issued by a trip that has since ended --
	// see the identical check in lookup_on_airport_list() above. rep
	// may already belong to a newly-started trip's (freshly cleared) departure/
	// destination slot at this point, so nothing below may touch it.
	if (status->lookup.trip_id != status->id_trip) {
		gui_log_printf(status, GUI_LOG_TRACE, "Dropping stale facility lookup response for trip %d (current trip %d)",
			status->lookup.trip_id, status->id_trip);
		return;
	}
	gui_log_printf(status, GUI_LOG_TRACE, "FACILITY_DATA_END: %s slot, icao=%s, n_runways=%d",
		facility_lookup_target_label(status, rep), rep->icao, rep->n_runways);
	double bearing_tra = (double)status->lookup.heading - rep->magvar;
	if (bearing_tra <= 0)
		bearing_tra += 360;
	// Ground-track refinement only applies to a touchdown/destination
	// lookup: the aircraft can still be crabbed into wind right up to the
	// moment of touchdown, so the bearing from that touchdown's own frozen
	// final-approach loc_dh snapshot (see TOUCHDOWN_DATA::loc_dh in
	// types.h) to the touchdown point is a better estimate of the
	// direction of travel than instantaneous heading. Departure/liftoff
	// have no such crossing available beforehand -- the aircraft is still
	// on the ground before liftoff, so the only 50-100ft AGL crossing it
	// could ever have is during climb-out, *after* the event -- and using
	// that would describe the reverse of the actual departure direction.
	// The aircraft is also mechanically tracking the runway during the
	// ground roll (no crab yet), so its own heading is already correct
	// for departure/liftoff; it's used unrefined for both.
	bool is_touchdown = status->lookup.target == LOOKUP_TARGET::TOUCHDOWN;
	const bool approach_based = is_touchdown && status->lookup.approach.latitude != 360;
	if (approach_based)
		bearing_tra = status->lookup.approach.bearing2Coordinate(status->lookup.coordinate);
	gui_log_printf(status, GUI_LOG_TRACE, "Runway match: bearing_tra=%.1f (%s), evaluating %d runway(s) for %s slot",
		bearing_tra, approach_based ? "loc_dh-based" : "heading-based",
		rep->n_runways, facility_lookup_target_label(status, rep));
	RUNWAY_MATCH match = match_runways(*rep, status->lookup.coordinate, bearing_tra, is_touchdown,
		[status](const char* line) { gui_log_printf(status, GUI_LOG_TRACE, "%s", line); });
	std::vector<struct RUNWAY_OPERATION>& candidates = match.candidates;
	const bool any_margin_hit = match.any_margin_hit;
	gui_log_printf(status, GUI_LOG_TRACE, "Runway match: %zu candidate(s) for %s slot",
		candidates.size(), facility_lookup_target_label(status, rep));
	if (candidates.size() > 0) {
		auto it = std::min_element(
			candidates.begin(),
			candidates.end(),
			[](struct RUNWAY_OPERATION& rwy1, struct RUNWAY_OPERATION& rwy2) {
				return rwy1.diff_bearing_tra < rwy2.diff_bearing_tra;
			}
		);
		rep->runway_act = *it;
		gui_log_printf(status, GUI_LOG_TRACE, "Runway match: selected runway index=%d (diff_bearing_tra=%.1f) for %s slot",
			rep->runway_act.index, rep->runway_act.diff_bearing_tra, facility_lookup_target_label(status, rep));
	}
	if (rep->runway_act.index != -1) {
		on_lookup_resolved(status, rep, LOOKUP_OUTCOME::RUNWAY);
	} else {
		// No strict runway match for this candidate. Snapshot candidate 0's
		// name (needed by the final <5km identity-only fallback below, since
		// this shared scratch AIRPORT slot gets overwritten by later
		// candidates), cache the nearest margin-rectangle hit if this is the
		// first one seen, then either advance the walk to the next candidate
		// or -- if the walk is exhausted -- resolve using whatever the walk
		// found (cached margin hit, then nearest-candidate identity within
		// 5km, then pure coordinate-only).
		if (status->lookup.candidate_index == 0) {
			copy_cstr(status->lookup.candidate0_name, rep->name);
		}
		if (any_margin_hit && !status->lookup.margin_cache.found) {
			status->lookup.margin_cache.found = true;
			copy_cstr(status->lookup.margin_cache.name, rep->name);
			copy_cstr(status->lookup.margin_cache.icao, rep->icao);
			copy_cstr(status->lookup.margin_cache.region, rep->region);
			gui_log_printf(status, GUI_LOG_TRACE, "Runway match: cached margin-rectangle identity %s (%s) for %s slot",
				rep->icao, rep->name, facility_lookup_target_label(status, rep));
		}
		bool has_next_candidate = (status->lookup.candidate_index + 1 < AIRPORT_LOOKUP::TOP_N)
			&& status->lookup.top[status->lookup.candidate_index + 1].ident[0] != '\0';
		bool resolve_as_known_airport_no_runway = false;
		if (has_next_candidate) {
			gui_log_printf(status, GUI_LOG_TRACE, "Runway match: no strict match for candidate #%d (%s); advancing to candidate #%d",
				status->lookup.candidate_index + 1, rep->icao, status->lookup.candidate_index + 2);
			cleanup_guard.more_candidates_pending = true;
			facility_lookup_request_candidate(status, status->lookup.candidate_index + 1);
		} else if (status->lookup.margin_cache.found) {
			gui_log_printf(status, GUI_LOG_TRACE, "Runway match: candidate walk exhausted; resolving via cached margin-rectangle identity %s (%s)",
				status->lookup.margin_cache.icao, status->lookup.margin_cache.name);
			copy_cstr(rep->name, status->lookup.margin_cache.name);
			copy_cstr(rep->icao, status->lookup.margin_cache.icao);
			copy_cstr(rep->region, status->lookup.margin_cache.region);
			resolve_as_known_airport_no_runway = true;
		} else if (status->lookup.top[0].distance < 5) {
			gui_log_printf(status, GUI_LOG_TRACE, "Runway match: candidate walk exhausted with no margin hit; nearest candidate %.2f km away is within 5km, using its identity with no runway",
				status->lookup.top[0].distance);
			copy_cstr(rep->icao, status->lookup.top[0].ident);
			copy_cstr(rep->region, status->lookup.top[0].region);
			copy_cstr(rep->name, status->lookup.candidate0_name);
			resolve_as_known_airport_no_runway = true;
		} else {
			gui_log_printf(status, GUI_LOG_TRACE, "Runway match: candidate walk exhausted with no strict/margin match and nearest candidate %.2f km away exceeds 5km threshold; using coordinate-only fallback", status->lookup.top[0].distance);
			on_lookup_resolved(status, rep, LOOKUP_OUTCOME::NO_AIRPORT);
		}
		if (resolve_as_known_airport_no_runway)
			on_lookup_resolved(status, rep, LOOKUP_OUTCOME::AIRPORT);
	}
	// runways free + lookup.pending reset happen in cleanup_guard's
	// destructor above, regardless of which branch was taken. status->flight.loc_dh
	// is deliberately left untouched here -- it's reset by three things only:
	// a fresh 50-100ft AGL crossing (overwrites with new data), climbing back
	// above 100ft (clears to sentinel -- see flight_on_sample() in flight_phase.cpp),
	// or trip start. This lets repeated touchdowns from the same low bounce/
	// touch-and-go sequence (which never climbs above 100ft) keep reusing the
	// one real approach ground track instead of falling back to heading-based.
	// Safe to leave unreset here: a genuinely distinct later landing climbing
	// above 100ft is a real-world flying assumption, not something this code
	// enforces -- status->flight.airborne (set purely from sim_on_ground, with no
	// altitude term) can't tell a low bounce apart from a full circuit. A
	// genuine later landing either gets fresh data on the way back down, or
	// -- if that descent doesn't happen to resample the 50-100ft band --
	// finds loc_dh already cleared to the sentinel and falls back to
	// heading-based bearing, so a stale cross-approach position can never
	// reach a later touchdown either way.
}

void lookup_on_exception(struct STATUS* status, DWORD send_id) {
	if (!status->lookup.pending || send_id != status->lookup.send_id)
		return;
	const bool current_trip = status->lookup.trip_id == status->id_trip;
	AIRPORT* rep = current_trip ? facility_lookup_target(status) : nullptr;
	LookupEnd end{ status, rep };
	if (current_trip)
		on_lookup_resolved(status, rep, LOOKUP_OUTCOME::FAILED);
}

void reset_airport_lookup(struct STATUS* status) {
	status->lookup.pending = false;
	status->lookup.trip_id = -1;
	status->lookup.target = LOOKUP_TARGET::TOUCHDOWN;
	status->lookup.send_id = 0;
	status->lookup.runway_definition_added = false;
	status->lookup.liftoff_scratch.clear();
}
