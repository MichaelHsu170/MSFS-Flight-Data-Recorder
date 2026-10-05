#pragma once

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <Windows.h>
#include "sqlite3.h"

#include "event_filter.h"
#define DATABASE_NAME "flight_data"
#define V_PI 3.14159265358979323846
#define M_2_FT 3.2808399
#define EARTHRADIUSKM 6371.0

// bearing (degrees, at most one turn outside it) wrapped into (0, 360]: due
// north is 360, never 0.
inline double wrap_bearing(double bearing) {
	if (bearing <= 0)
		return bearing + 360;
	if (bearing > 360)
		return bearing - 360;
	return bearing;
}

// The angle between two bearings (degrees, within one turn of each other),
// 0-180.
inline double bearing_difference(double a, double b) {
	const double diff = fabs(a - b);
	return diff > 180 ? 360 - diff : diff;
}

class DATETIME {
public:
	double year;
	double month_of_year;
	double day_of_month;
	double day_of_week;
	double time_day;
	double timezone_offset;

	DATETIME() { clear(); }

	void clear() {
		year = 0;
		month_of_year = 0;
		day_of_month = 0;
		time_day = 0;
		day_of_week = 0;
		timezone_offset = 0;
	}

	std::string format_date_time() const {
		int hour = (int)time_day / 3600;
		int minute = ((int)time_day - 3600 * hour) / 60;
		double second = time_day - 3600 * hour - 60 * minute;

		char sign = '+';
		if (timezone_offset < 0)
			sign = '-';
		double timezone = abs(timezone_offset);
		timezone /= 3600;
		int timezone_hour = (int)timezone;
		int timezone_minute = (int)((timezone - timezone_hour) * 60);

		char ret[32];
		memset(ret, 0, sizeof(ret));
		snprintf(ret, sizeof(ret), "%04.0f-%02.0f-%02.0fT%02d:%02d:%06.3f%c%02d:%02d_%1.0f",
			year, month_of_year, day_of_month, hour, minute, second,
			sign, timezone_hour, timezone_minute, day_of_week);
		return std::string(ret);
	}
};

class COORDINATE {
public:
	enum COORDINATE_CAT {
		LATITUDE,
		LONGITUDE,
	};

	double latitude;
	double longitude;

	COORDINATE() { clear(); }

	void clear() {
		latitude = 360;
		longitude = 360;
	}

	// One axis as degrees/minutes/seconds, e.g. 43°30'00.0"N -- the format the
	// Data Table and the map popups (formatDMS in map.html) show, so a position
	// copied from either pastes the same way into Google Maps/Earth's search
	// box. Rounds to whole tenths of an arcsecond up front and decomposes with
	// integer division/modulo so seconds can't round up to "60.0" instead of
	// carrying into the next minute.
	std::string coordinate_decimal_to_dms(enum COORDINATE_CAT cat) const {
		const double value = cat == LATITUDE ? latitude : longitude;
		const char letter = cat == LATITUDE ? (value >= 0 ? 'N' : 'S') : (value >= 0 ? 'E' : 'W');
		long long tenths = llround(fabs(value) * 36000.0);
		const long long deg = tenths / 36000;
		tenths -= deg * 36000;
		const long long min = tenths / 600;
		tenths -= min * 600;
		char ret[32];
		snprintf(ret, sizeof(ret), "%lld\xC2\xB0%02lld'%02lld.%lld\"%c", deg, min, tenths / 10, tenths % 10, letter);
		return std::string(ret);
	}

	double distanceInKm2Coordinate(COORDINATE loc) {
		double phy1 = latitude * V_PI / 180.0;
		double phy2 = loc.latitude * V_PI / 180.0;
		double dPhy = (loc.latitude - latitude) * V_PI / 180.0;
		double dLambda = (loc.longitude - longitude) * V_PI / 180.0;
		double a = pow(sin(dPhy / 2), 2) + cos(phy1) * cos(phy2) * pow(sin(dLambda / 2), 2);
		double c = 2 * atan2(sqrt(a), sqrt(1 - a));
		return EARTHRADIUSKM * c;
	}

	double bearing2Coordinate(COORDINATE loc) {
		double phy1 = latitude * V_PI / 180.0;
		double phy2 = loc.latitude * V_PI / 180.0;
		double lambda1 = longitude * V_PI / 180.0;
		double lambda2 = loc.longitude * V_PI / 180.0;
		double y = sin(lambda2 - lambda1) * cos(phy2);
		double x = cos(phy1) * sin(phy2) - sin(phy1) * cos(phy2) * cos(lambda2 - lambda1);
		double theta = atan2(y, x);
		return wrap_bearing(theta * 180 / V_PI);
	}

	COORDINATE destinationWithDistanceAndBearing(double distance, double bearing) {
		COORDINATE ret;
		double phy = latitude * V_PI / 180.0;
		double lambda = longitude * V_PI / 180.0;
		double theta = bearing * V_PI / 180.0;
		ret.latitude = asin(sin(phy) * cos(distance / EARTHRADIUSKM) + cos(phy) * sin(distance / EARTHRADIUSKM) * cos(theta));
		ret.longitude = lambda + atan2(sin(theta) * sin(distance / EARTHRADIUSKM) * cos(phy), cos(distance / EARTHRADIUSKM) - sin(phy) * sin(ret.latitude));
		ret.latitude *= 180.0 / V_PI;
		ret.longitude *= 180.0 / V_PI;
		return ret;
	}
};

class RUNWAY {
public:
	char placeholder[4];
	float length;
	float width;
	float heading;
	int numbers[2];
	int designators[2];
	COORDINATE coordinate;
	COORDINATE start_points[2];

	// Displaced-threshold offset (meters, from the physical runway end to the
	// marked/usable threshold) for each end, from SimConnect's PRIMARY_THRESHOLD/
	// SECONDARY_THRESHOLD facility data (nested PAVEMENT child records -- see
	// the FACILITY_DATA_PAVEMENT handling in airport_lookup.cpp). *_enable mirrors
	// the sim's own ENABLE flag: 0 means this runway has no threshold data, so
	// the offset must be treated as 0, not used as-is.
	float primary_threshold_offset_m;
	float secondary_threshold_offset_m;
	int primary_threshold_enable;
	int secondary_threshold_enable;
	// Transient correlation state, valid only while a single FACILITY_DATA
	// response for this runway is streaming in -- not part of the wire copy,
	// not persisted. See SIMCONNECT_FACILITY_DATA_RUNWAY/_PAVEMENT handling.
	unsigned int pending_request_id;
	int threshold_pavement_seen; // 0=none yet, 1=primary received, 2=both received

	RUNWAY() { clear(); }

	void clear() {
		length = 0;
		width = 0;
		heading = 0;
		for (int i = 0; i < (int)(sizeof(numbers) / sizeof(int)); i++)
			numbers[i] = -1;
		for (int i = 0; i < (int)(sizeof(designators) / sizeof(int)); i++)
			designators[i] = -1;
		coordinate.clear();
		for (int i = 0; i < 2; i++)
			start_points[i].clear();
		primary_threshold_offset_m = 0;
		secondary_threshold_offset_m = 0;
		primary_threshold_enable = 0;
		secondary_threshold_enable = 0;
		pending_request_id = 0;
		threshold_pavement_seen = 0;
	}

	std::string runway_code_generator(bool is_primary) {
		int runway_number = is_primary ? numbers[0] : numbers[1];
		int runway_designator = is_primary ? designators[0] : designators[1];
		char designator = 0;
		switch (runway_designator) {
		case 1: designator = 'L'; break;
		case 2: designator = 'R'; break;
		case 3: designator = 'C'; break;
		case 4: designator = 'W'; break;
		case 5: designator = 'A'; break;
		case 6: designator = 'B'; break;
		default: break;
		}
		static const char* const numbers_dir[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
		char ret[4];
		memset(ret, 0, sizeof(ret));
		if (runway_number > 0 && runway_number <= 36)
			snprintf(ret, sizeof(ret), "%02d%c", runway_number, designator);
		else if (runway_number >= 37 && runway_number <= 44)
			snprintf(ret, sizeof(ret), "%s%c", numbers_dir[runway_number - 37], designator);
		return std::string(ret);
	}
};

struct RUNWAY_OPERATION {
	int index = -1;
	bool is_primary = TRUE;
	double diff_bearing_tra = 0;
	double distances[2];
	double distances_percent[2];
	// Operational heading (1-360, aviation convention -- due north is 360,
	// never 0) of the runway end actually used -- i.e.
	// AIRPORT::runways[index].heading, flipped 180° if !is_primary. Captured
	// and rounded once at match time (see match_runways() in
	// runway_match.cpp) instead of being re-derived from runways[] wherever it's
	// read, so it stays valid even after the source AIRPORT is cleared/reused.
	// -1 (unlike 1-360) means "not yet computed" -- see AIRPORT::clear(),
	// matching the -1 = unset convention used by distances[]/distances_percent[]
	// above.
	int heading = -1;
};

class AIRPORT {
public:
	char name[64];
	float magvar;
	int n_runways;
	RUNWAY* runways;
	struct RUNWAY_OPERATION runway_act;
	char icao[5];
	char region[3];

	AIRPORT() {
		runways = NULL;
		clear();
	}

	~AIRPORT() { clear(); }

	void clear() {
		memset(icao, 0, sizeof(icao));
		memset(region, 0, sizeof(region));
		memset(name, 0, sizeof(name));
		magvar = 0;
		n_runways = 0;
		if (runways != NULL)
			free(runways);
		runways = NULL;
		runway_act.index = -1;
		runway_act.is_primary = TRUE;
		runway_act.diff_bearing_tra = 0;
		for (int i = 0; i < (int)(sizeof(runway_act.distances) / sizeof(double)); i++)
			runway_act.distances[i] = -1;
		for (int i = 0; i < (int)(sizeof(runway_act.distances_percent) / sizeof(double)); i++)
			runway_act.distances_percent[i] = -1;
		runway_act.heading = -1;
	}

	void copy(AIRPORT* src) {
		memcpy(name, src->name, sizeof(src->name));
		memcpy(icao, src->icao, sizeof(src->icao));
		memcpy(region, src->region, sizeof(src->region));
		magvar = src->magvar;
		// Free any buffer this AIRPORT already owns before reassigning, so a
		// copy() onto an already-populated AIRPORT doesn't leak it.
		if (runways != NULL) {
			free(runways);
			runways = NULL;
		}
		n_runways = src->n_runways;
		if (src->runways != NULL) {
			runways = (RUNWAY*)malloc(sizeof(RUNWAY) * n_runways);
			if (runways != NULL)
				memcpy(runways, src->runways, sizeof(RUNWAY) * n_runways);
			else
				n_runways = 0;
		}
		runway_act = src->runway_act;
	}

	std::string runway_code_generator() {
		if (runway_act.index > -1)
			return runways[runway_act.index].runway_code_generator(runway_act.is_primary);
		return "";
	}
};

struct FLIGHT_DATA {
	int heading = 0;
	int altitude = 0;
	int speed = 0;
	int vertical_speed = 0;
	double g_force = 1;
	double pitch = 0;
	double bank = 0;
	int wind_direction = 0;
	int wind_velocity = 0;
	COORDINATE coordinate;
	DATETIME time_zulu;
	DATETIME time_local;
};

// What a liftoff and a touchdown record share (LIFTOFF_DATA, TOUCHDOWN_DATA).
struct CONTACT_RECORD {
	struct FLIGHT_DATA flight_data;
	AIRPORT airport;
	int db_id = -1;              // trip_liftoffs/trip_touchdowns row ID, set after immediate INSERT
	// Monotonically increasing across both liftoff_data and touchdown_data
	// (FLIGHT_PHASE::next_facility_lookup_seq), so request_next_touchdown_facility_lookup
	// can pick whichever of the two lists holds the chronologically earliest
	// unresolved lookup instead of always preferring one list over the other.
	int seq = 0;
};

// Every moment this trip's aircraft actually became airborne, as a marker
// occurrence (touch-and-goes included), independent of the trip's single,
// permanent "departure" record (STATUS::departure / FLIGHT_PHASE::departure_db_id),
// which always stays locked to the first liftoff only. See
// flight_on_sample()'s liftoff detection.
struct LIFTOFF_DATA : CONTACT_RECORD {
	struct LIFTOFF_DATA* next = NULL;
};

// Every touchdown of the trip.
struct TOUCHDOWN_DATA : CONTACT_RECORD {
	// Snapshot of FLIGHT_PHASE::loc_dh taken the instant this touchdown is recorded
	// (see flight_phase.cpp) rather than read live from FLIGHT_PHASE::loc_dh when this
	// touchdown's facility lookup eventually resolves. FLIGHT_PHASE::loc_dh is a
	// single shared scratch field that keeps getting overwritten by every
	// subsequent low-altitude pass (e.g. a go-around's second approach) --
	// only one facility lookup is in flight at a time, so a touchdown's own
	// lookup can still be queued (see request_next_touchdown_facility_lookup)
	// when a later approach's crossing overwrites it. At touchdown time the
	// shared field holds this approach's 50-100ft band position, or is unset
	// (latitude 360) if the descent skipped the band between samples:
	// flight_on_sample() clears it once the aircraft climbs above 100ft, so it
	// can't still hold an earlier approach's position.
	COORDINATE loc_dh;
	struct TOUCHDOWN_DATA* next = NULL;
};

// Forward declaration — full definition in simconnect_defs.h
struct FLIGHT_DATA_RECORD;

// Thread-safe FIFO feeding one persistent worker thread: producers push, the
// worker blocks in pop(). Base of SampleWriteQueue and EventWriteQueue.
template <typename T>
class WorkQueue {
public:
	// Blocks until an item is available. Returns false once stop() has been
	// called and the queue has fully drained -- the worker loop should exit.
	bool pop(T& item) {
		std::unique_lock<std::mutex> lock(mutex_);
		cv_.wait(lock, [this] { return !queue_.empty() || stopping_; });
		if (queue_.empty())
			return false;
		item = std::move(queue_.front());
		queue_.pop_front();
		return true;
	}

	// Tells the worker thread to exit once it has drained whatever is
	// currently queued (does not discard pending items).
	void stop() {
		{
			std::lock_guard<std::mutex> lock(mutex_);
			stopping_ = true;
		}
		cv_.notify_one();
	}

	// Re-arms the queue for a fresh worker thread after reconnecting.
	void reset() {
		std::lock_guard<std::mutex> lock(mutex_);
		stopping_ = false;
	}

protected:
	void enqueue(T item) {
		{
			std::lock_guard<std::mutex> lock(mutex_);
			queue_.push_back(std::move(item));
		}
		cv_.notify_one();
	}

private:
	std::mutex mutex_;
	std::condition_variable cv_;
	std::deque<T> queue_;
	bool stopping_ = false;
};

// One entry in STATUS::sample_write_queue. A null `data` with `trip_id` set
// marks the end of that trip's samples (a "barrier") so the DB-write worker
// can log "Recording stopped" and notify the GUI at the right point in the
// stream, without a new trip's samples racing ahead of the old trip's flush.
struct SAMPLE_QUEUE_ITEM {
	struct FLIGHT_DATA_RECORD* data;
	int trip_id;
};

// Thread-safe queue feeding a single persistent DB-write worker thread
// (db_write_worker in db.cpp). Every producer -- the SimConnect dispatch
// callback appending samples, and stop_recording() pushing an end-of-trip
// barrier -- just pushes onto this queue; only the worker thread ever touches
// STATUS::sql for sample flushes, so writes never race each other and a new
// trip starting while a previous trip's flush is still in flight needs no
// queue reset.
class SampleWriteQueue : public WorkQueue<SAMPLE_QUEUE_ITEM> {
public:
	void push(struct FLIGHT_DATA_RECORD* data, int trip_id) {
		enqueue({ data, trip_id });
	}
};

// One entry in STATUS::event_write_queue -- either an Insert (one new
// trip_events row) or a Delete (retract previously-inserted rows by
// event_seq, e.g. EventFloodFilter confirming a slow flood after already
// committing its occurrences -- see event_filter.h).
// Both kinds share one queue/worker so a Delete for seqs N..N+2 can never be
// dequeued and executed ahead of the Inserts that created those same rows --
// see EventWriteQueue below.
struct EVENT_QUEUE_ITEM {
	enum class Kind { Insert, Delete };
	Kind kind = Kind::Insert;

	// Insert fields. trip_id is captured explicitly at enqueue time (same
	// reasoning as SAMPLE_QUEUE_ITEM::trip_id above) rather than read from
	// status->id_trip by the worker thread, since a new trip can already be
	// live by the time this item is actually dequeued and written. seq is
	// the id EventFloodFilter gave this occurrence, stored alongside it in
	// trip_events so a later Delete can target it precisely.
	int trip_id = -1;
	std::string event;
	std::string time_zulu;
	std::string time_local;
	unsigned long long seq = 0;

	// Delete fields -- only meaningful when kind == Kind::Delete. A handful of
	// entries in practice (a slow flood's threshold), but not capped here.
	std::vector<unsigned long long> delete_seqs;
};

// Thread-safe queue feeding a single persistent event-write worker thread
// (event_write_worker in db.cpp), like SampleWriteQueue above. Moves
// db_insert_event's synchronous BEGIN/INSERT/COMMIT (including its fsync) off
// the SimConnect dispatch thread, which otherwise blocks the UI directly
// (a held lever can fire an event every frame).
class EventWriteQueue : public WorkQueue<EVENT_QUEUE_ITEM> {
public:
	void push(int trip_id, const std::string& event, const std::string& time_zulu, const std::string& time_local, unsigned long long seq) {
		EVENT_QUEUE_ITEM item;
		item.kind = EVENT_QUEUE_ITEM::Kind::Insert;
		item.trip_id = trip_id;
		item.event = event;
		item.time_zulu = time_zulu;
		item.time_local = time_local;
		item.seq = seq;
		enqueue(std::move(item));
	}

	// Enqueues a retraction of previously-inserted rows by event_seq -- see
	// EVENT_QUEUE_ITEM::Kind::Delete above.
	void push_delete(std::vector<unsigned long long> seqs) {
		EVENT_QUEUE_ITEM item;
		item.kind = EVENT_QUEUE_ITEM::Kind::Delete;
		item.delete_seqs = std::move(seqs);
		enqueue(std::move(item));
	}
};

// Copies src into a char array, cut to fit, null-terminated and zero-filled
// past the end -- strncpy's result plus the terminator it can leave out.
template <size_t N>
inline void copy_cstr(char (&dst)[N], const char* src) {
	const size_t len = strnlen(src, N - 1);
	memcpy(dst, src, len);
	memset(dst + len, 0, N - len);
}

// What a facility (airport/runway) lookup is resolving -- see
// AIRPORT_LOOKUP::target.
enum class LOOKUP_TARGET { DEPARTURE, LIFTOFF, TOUCHDOWN };

// The in-flight facility (airport/runway) lookup -- see airport_lookup.h.
// Only one lookup is in flight at a time.
struct AIRPORT_LOOKUP {
	// Scratch space for an in-flight liftoff-marker (touch-and-go, not the
	// trip's one departure) lookup -- mirrors STATUS::destination's scratch role
	// for a touchdown lookup, but kept as its own field rather than
	// sharing destination: even though the two are never populated
	// concurrently (only one lookup is in flight -- see pending below), they mean
	// different things -- destination is the trip's actual destination
	// airport, this is a transient candidate for whichever liftoff marker is
	// currently being resolved -- and collapsing them into one field would
	// make STATUS::destination silently hold liftoff-marker data during that
	// window, a landmine for any future code that reads it assuming it's
	// always the trip's destination. The lookup-handling logic these two
	// share is reused via facility_lookup_target()/facility_lookup_target_label()
	// in airport_lookup.cpp (code reuse), not by reusing this storage.
	AIRPORT liftoff_scratch;
	// Set right before SimConnect_RequestFacilitiesList_EX1() is called (on
	// becoming airborne or touchdown) and cleared once the async facility lookup it
	// starts (AIRPORT_LIST -> optional FACILITY_DATA(s) -> FACILITY_DATA_END)
	// terminates. Only one such lookup may be in flight at a time -- overlapping
	// lookups would race on the STATUS::departure/STATUS::destination and
	// liftoff_scratch scratch objects (see airport_lookup.cpp). trip_id records
	// which trip issued the in-flight lookup, so a response that arrives after
	// that trip has already ended (id_trip changed) can be recognized as stale
	// and dropped instead of being applied to whatever trip is active when it
	// lands.
	bool pending = FALSE;
	int trip_id = -1;
	// Which record the in-flight lookup is for, fixed when it starts (by
	// start_facility_lookup() in airport_lookup.cpp) and read by every
	// AIRPORT_LIST/FACILITY_DATA/FACILITY_DATA_END/EXCEPTION handler through
	// facility_lookup_target(): the trip's one departure, a later liftoff
	// (touch-and-go marker) or a touchdown. Not re-derived per callback from
	// "departure.runway_act.index == -1", which breaks across a trip
	// boundary: if a liftoff-marker or destination lookup is still in flight
	// when its trip ends, the new trip's STATUS::departure.clear() resets
	// runway_act.index to -1 out from under it, so the stale response would
	// be misattributed to &STATUS::departure, leaking its real target's
	// runways buffer (lookup_on_facility_data_end() frees the buffer of the
	// slot facility_lookup_target() names).
	LOOKUP_TARGET target = LOOKUP_TARGET::TOUCHDOWN;
	// SendID of the most recent SimConnect_RequestFacilitiesList_EX1/
	// RequestFacilityData_EX1 call belonging to the in-flight lookup (see
	// pending above), captured via SimConnect_GetLastSentPacketID
	// right after each call. SIMCONNECT_RECV_ID_EXCEPTION reports failed requests
	// asynchronously with no other correlation to the request that failed; matching
	// its dwSendID against this lets a rejected lookup request be recognized and
	// terminated instead of leaving pending stuck true forever.
	DWORD send_id = 0;
	// Guards SimConnect_AddToFacilityDefinition(DEFINITION_RUNWAYS, ...): those
	// fields describe the definition itself (server-side, per-connection state),
	// not any particular request, so they only need to be registered once per
	// connection -- re-adding the same fields on every lookup is wasteful and
	// risks eventually exceeding an internal SDK limit. Reset on reconnect
	// (RecorderBridge::tryConnect()) since a new SimConnect connection starts
	// with an empty definition table.
	bool runway_definition_added = FALSE;
	// SimConnect_RequestFacilitiesList_EX1() takes no lat/lon -- it always
	// returns facilities near the aircraft's CURRENT position at the moment
	// the request is sent, not any historical position. Since only one lookup
	// may be in flight at a time (see pending above), a
	// touchdown/departure lookup queued behind an earlier one can fire well
	// after the aircraft has moved from where that event actually happened
	// (e.g. a go-around after a bounced landing). coordinate
	// is set immediately before each SimConnect_RequestFacilitiesList_EX1
	// call to the *historical* coordinate the response should be evaluated
	// against (the touchdown's stored CONTACT_RECORD::flight_data.coordinate,
	// or STATUS::departure_data for a departure), and used
	// in place of STATUS::data.coordinate throughout the AIRPORT_LIST/
	// FACILITY_DATA_END handlers so a moved-since aircraft position can't
	// misattribute the response to the wrong airport/runway.
	COORDINATE coordinate;
	// Same staleness problem as coordinate above, but for the
	// heading used as the runway-bearing fallback when loc_dh (the low-altitude
	// decision-height position) isn't available: STATUS::data.heading reflects
	// the aircraft's heading at the moment FACILITY_DATA_END arrives, which can
	// be well after the actual touchdown/liftoff (e.g. queued behind an earlier
	// lookup, or the aircraft has already turned off the runway). Set alongside
	// coordinate from the same historical source (magnetic
	// heading, matching STATUS::data.heading's units) each time that field is.
	int heading = 0;
	// Running top-N nearest airports across every chunk of the current
	// AIRPORT_LIST response. SimConnect splits a large facility list (e.g.
	// every airport in loaded scenery, 1000+ entries) across multiple
	// AIRPORT_LIST callbacks that share one request (see dwEntryNumber/
	// dwOutOf in lookup_on_airport_list()) -- deciding "nearest airport" from any
	// single chunk in isolation is wrong, since a later chunk that happens
	// to contain only distant airports would otherwise conclude "not found"
	// and terminate/overwrite the lookup a second time while an earlier
	// chunk's real match was still being resolved. Reset to all-distance-1e9
	// when a fresh request's first chunk (dwEntryNumber == 0) arrives.
	struct CANDIDATE {
		double distance = 1e9;
		char ident[9] = {};
		char region[3] = {};
	};
	static const int TOP_N = 5;
	CANDIDATE top[TOP_N];
	// Which top[] slot the in-flight multi-candidate runway
	// walk is currently fetching/evaluating. Reset to 0 alongside
	// top[] itself, on a fresh request's first chunk.
	int candidate_index = 0;
	// Cached "known airport, no specific runway" identity from a margin-
	// rectangle hit (touchdown/liftoff near a runway but not strictly on it)
	// found while walking top[] nearest-to-farthest. Only
	// ever holds the nearest candidate that had a margin hit -- see the
	// "cache only if not already found" rule in
	// lookup_on_facility_data_end().
	struct MARGIN_CACHE {
		bool found = false;
		char name[64] = {};
		char icao[5] = {};
		char region[3] = {};
	} margin_cache;
	// Snapshot of top[0]'s AIRPORT::name, captured the first
	// time candidate 0's FACILITY_DATA_END is processed. Needed by the final
	// <5km identity-only fallback: by the time the candidate walk exhausts
	// top[], the one shared scratch AIRPORT slot has been
	// overwritten by later candidates' data, so candidate 0's name has to be
	// preserved separately.
	char candidate0_name[64] = {};
	// For a touchdown lookup, that touchdown's final-approach position
	// (TOUCHDOWN_DATA::loc_dh), captured when the lookup starts: the bearing
	// from it to the touchdown point estimates the direction of travel better
	// than a heading that may include crab. Latitude 360 (COORDINATE's "unset")
	// when unknown or not a touchdown.
	COORDINATE approach;
};

// The recording trip's flight-phase state -- see flight_phase.h.
struct FLIGHT_PHASE {
	// Heap copy of the most recently produced sample, owned outside the
	// queue so the dispatch callback can compute the next sample's delta_s
	// without touching whatever the DB-write worker is doing.
	struct FLIGHT_DATA_RECORD* last_sample = NULL;
	bool airborne = FALSE;
	// Every moment this trip's aircraft actually became airborne, as a marker
	// occurrence (touch-and-goes included), independent of the trip's single
	// permanent departure_db_id below/STATUS::departure -- see LIFTOFF_DATA. NOT
	// populated for the trip's first liftoff, which only ever updates
	// departure_db_id/STATUS::departure (see flight_on_sample()'s liftoff
	// detection).
	LIFTOFF_DATA* liftoff_data = NULL;
	LIFTOFF_DATA* liftoff_data_end = NULL;
	TOUCHDOWN_DATA* touchdown_data = NULL;
	TOUCHDOWN_DATA* touchdown_data_end = NULL;
	// Shared seq counter for CONTACT_RECORD::seq, so
	// request_next_touchdown_facility_lookup() can pick whichever of the two
	// lists holds the chronologically earliest unresolved lookup. Reset only
	// at true trip boundaries, same as departure_lookup_initiated.
	int next_facility_lookup_seq = 0;
	// trip_liftoffs row ID for this trip's single departure, set after the
	// immediate INSERT at the moment it becomes airborne (see flight_on_sample()'s
	// liftoff detection) and consumed later by on_lookup_resolved()'s
	// departure UPDATE, same db_id pattern as CONTACT_RECORD::db_id but for
	// the one-per-trip departure. Reset to -1 at trip start (recording-start
	// block in flight_on_sample()).
	int departure_db_id = -1;
	COORDINATE loc_dh;
	// Set when a departure's own facility lookup was skipped because
	// lookup.pending was already true (a previous trip's lookup was
	// still draining when this trip became airborne). request_next_touchdown_facility_lookup()
	// in flight_phase.cpp checks this before touchdown_data, so the departure lookup
	// is retried as soon as the shared slot frees up rather than being lost --
	// unlike touchdowns, a skipped departure has no "unresolved" marker of its own
	// to search for later. Cleared in stop_recording() so a lookup skipped by a
	// trip that ends before its retry turn can't be mistakenly fired for
	// whatever trip is active later.
	bool departure_lookup_needed = FALSE;
	// Set the instant this trip's first liftoff is detected (flight_phase.cpp),
	// whether or not the resulting lookup fires immediately or is deferred via
	// departure_lookup_needed above. A trip has exactly one departure
	// airport -- wherever that first liftoff happened -- so this must stay
	// TRUE for the rest of the trip, including through any number of later
	// touch-and-goes or full-stop taxi-back-and-liftoffs, none of which are a
	// new departure. departure.runway_act.index == -1 would be racy for this
	// same purpose: that field only flips once the async lookup
	// actually *resolves*, so becoming airborne before a slow (e.g.
	// multi-chunk AIRPORT_LIST) departure lookup resolves would still see -1
	// and be mistaken for a fresh departure, overwriting the captured liftoff
	// coordinate/heading. Reset only at true trip boundaries (trip start and
	// RecorderBridge::tryConnect()'s carry-over reset), never on landing.
	bool departure_lookup_initiated = FALSE;
	// What the trip's departure (first liftoff) recorded at the moment it
	// became airborne -- position, heading, time, speeds -- captured whether
	// or not its lookup fires immediately (see departure_lookup_needed
	// above): a deferred departure lookup has no other record of where and
	// when the liftoff happened once request_next_touchdown_facility_lookup()
	// finally sends it, and the lookup's result is logged with this time.
	FLIGHT_DATA departure_data;
};

struct STATUS {
	bool in_sim = FALSE;
	bool sim_running = FALSE;
	bool paused = FALSE;
	bool recording = FALSE;
	// User-facing gate on automatic recording start, toggled via the Recording
	// indicator in LiveStatusPanel and persisted through AppSettings. Distinct
	// from `recording` (which trip is actually mid-flight right now): this only
	// suppresses the start-on-engine-start check in flight_on_sample(), so flipping it
	// while a trip is already recording has no effect on that trip.
	bool recording_enabled = TRUE;
	bool quit = FALSE;
	HANDLE hSimConnect = NULL;
	sqlite3* sql = NULL;
	std::mutex mutex_db_commit;
	// Single-writer queue for periodic sample flushes -- see SampleWriteQueue
	// above. db_writer_thread is the one persistent thread draining it,
	// started in connect_db() and joined in wait_for_db_writers().
	SampleWriteQueue sample_write_queue;
	std::thread db_writer_thread;
	// Same pattern as sample_write_queue/db_writer_thread above, for
	// trip_events rows instead of trip_data samples. Started in connect_db()
	// and joined in wait_for_db_writers().
	EventWriteQueue event_write_queue;
	std::thread event_writer_thread;
	int sample_interval_ms = 500;
	int id_trip = -1;
	// The ids of trips whose tail samples may still be draining through
	// sample_write_queue after stop_recording() already reset id_trip to -1
	// on this (dispatch) thread. id_trip is reset synchronously so a new
	// trip's event logging can never be mistaken for the old one's (see
	// stop_recording()), but that also makes the ended trip look non-Live in
	// the UI immediately -- before db_write_worker has actually finished
	// flushing its samples on the DB-write thread. Without this,
	// TripHistoryPanel could let the user delete that trip's row while the
	// worker is still inserting trip_data rows for it, orphaning them. A set
	// rather than a single id: a short trip can stop (and be pushed here)
	// while a still-earlier trip's barrier hasn't reached the front of
	// sample_write_queue yet, so more than one id can be draining at once --
	// a single scalar would have the later stop_recording() call clobber the
	// earlier trip's id, leaving it wrongly deletable. Inserted by
	// stop_recording() right before it pushes the trip's end-of-trip barrier;
	// erased by db_write_worker (db.cpp) once that barrier is processed.
	// Guarded by flushing_trip_ids_mutex because it's written on the dispatch
	// thread, read from the GUI thread (RecorderBridge::isTripFlushing()),
	// and erased from the DB-write worker thread.
	mutable std::mutex flushing_trip_ids_mutex;
	std::set<int> flushing_trip_ids;
	FLIGHT_DATA data;
	AIRPORT departure;
	// The trip's destination airport, filled in once a touchdown's facility
	// lookup resolves (copied from here into the matching TOUCHDOWN_DATA
	// node -- see on_lookup_resolved()). Also doubles as scratch space for that
	// same lookup while it's still in flight (the in-progress AIRPORT_LIST
	// candidate search, before a specific runway is known to be the match),
	// which is safe because only one facility lookup is ever in flight at a
	// time (AIRPORT_LOOKUP::pending) and every read of this object
	// happens synchronously within the same FACILITY_DATA_END/AIRPORT_LIST
	// callback that just populated it.
	AIRPORT destination;
	AIRPORT_LOOKUP lookup;
	FLIGHT_PHASE flight;
	// Two-tier flood protection every cockpit event passes through before
	// commit_event() in recorder.cpp -- see event_filter.h. Lives for the
	// app's lifetime: each entry's own quiet period clears it, so nothing here
	// needs a trip-boundary reset.
	EventFloodFilter event_filter;
	// Per-event-name last-logged time for the "Event ignored (no active
	// trip)" TRACE line -- see EVENT_NO_TRIP_LOG_COOLDOWN and commit_event()
	// in recorder.cpp. Purely a log rate-limit, not a suppression: an entry
	// never blocks an occurrence from committing and needs no trip-boundary
	// reset -- it just ages out once EVENT_NO_TRIP_LOG_COOLDOWN elapses.
	std::unordered_map<std::string, std::chrono::steady_clock::time_point> no_trip_log_throttle;
	void* gui_context = nullptr;
};
