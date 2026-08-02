#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>
#include <Windows.h>
#include "sqlite3.h"

#include "version.h"
#define DATABASE_NAME "flight_data"
#define V_PI 3.14159265358979323846
#define M_2_FT 3.2808399
#define EARTHRADIUSKM 6371.0

class DATETIME {
public:
	double year;
	double month_of_year;
	double day_of_month;
	double day_of_week;
	double time_day;
	double timezone_offset;

	DATETIME() { clear(); }
	~DATETIME() { clear(); }

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
	~COORDINATE() { clear(); }

	void clear() {
		latitude = 360;
		longitude = 360;
	}

	std::string coordinate_decimal_to_dms(enum COORDINATE_CAT cat) {
		double coordinate = 0;
		char tmp1 = 'T';
		switch (cat) {
		case LATITUDE:
			coordinate = latitude;
			tmp1 = (coordinate < 0) ? 'S' : 'N';
			break;
		case LONGITUDE:
			coordinate = longitude;
			tmp1 = (coordinate < 0) ? 'W' : 'E';
			break;
		default:
			break;
		}
		coordinate = abs(coordinate);
		int degree = (int)coordinate;
		coordinate -= degree;
		coordinate *= 60;
		int minute = (int)coordinate;
		coordinate -= minute;
		coordinate *= 60;
		int second = (int)coordinate;
		char ret[12];
		memset(ret, 0, sizeof(ret));
		snprintf(ret, sizeof(ret), "%03d %02d %02d%c", degree, minute, second, tmp1);
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
		double ret = theta * 180 / V_PI;
		if (ret <= 0)
			ret += 360;
		return ret;
	}

	COORDINATE destinationWithDistanceAndBearing(double distance, double bearing) {
		COORDINATE ret;
		double phy = latitude * V_PI / 180.0;
		double lambda = longitude * V_PI / 180.0;
		double theta = bearing * V_PI / 180.0;
		ret.latitude = asin(sin(phy) * cos(distance / EARTHRADIUSKM) + cos(phy) * sin(distance / EARTHRADIUSKM) * cos(theta));
		ret.longitude = lambda + atan2(sin(theta) * sin(distance / EARTHRADIUSKM) * cos(phy), cos(distance / EARTHRADIUSKM) - sin(phy) * sin(phy));
		ret.latitude *= 180.0 / V_PI;
		ret.longitude *= 180.0 / V_PI;
		return ret;
	}

	COORDINATE intersectionCoordinate(double bearing1, COORDINATE loc, double bearing2) {
		COORDINATE ret;
		double phy1 = latitude * V_PI / 180.0;
		double phy2 = loc.latitude * V_PI / 180.0;
		double lambda1 = longitude * V_PI / 180.0;
		double lambda2 = loc.longitude * V_PI / 180.0;
		double theta1 = bearing1 * V_PI / 180.0;
		double theta2 = bearing2 * V_PI / 180.0;

		double delta12 = 2 * asin(sqrt((pow(sin((phy2 - phy1) / 2), 2) + cos(phy1) * cos(phy2) * pow(sin((lambda2 - lambda1) / 2), 2))));
		double thetaa = acos((sin(phy2) - sin(phy1) * cos(delta12)) / (sin(delta12) * cos(phy1)));
		double thetab = acos((sin(phy1) - sin(phy2) * cos(delta12)) / (sin(delta12) * cos(phy2)));
		double theta12 = 0;
		double theta21 = 0;
		if (sin(lambda2 - lambda1) > 0) {
			theta12 = thetaa;
			theta21 = 2 * V_PI - thetab;
		} else {
			theta12 = 2 * V_PI - thetaa;
			theta21 = thetab;
		}
		double alpha1 = theta1 - theta12;
		double alpha2 = theta21 - theta2;
		if ((sin(alpha1) == 0 && sin(alpha2) == 0) || sin(alpha1) * sin(alpha2) < 0) {
			ret.clear();
		} else {
			double alpha3 = acos(-1 * cos(alpha1) * cos(alpha2) + sin(alpha1) * sin(alpha2) * cos(delta12));
			double delta1 = atan2(sin(delta12) * sin(alpha1) * sin(alpha2), cos(alpha2) + cos(alpha1) * cos(alpha3));
			double phy3 = asin(sin(phy1) * cos(delta1) + cos(phy1) * sin(delta1) * cos(theta1));
			double delta_lambda1 = atan2(sin(theta1) * sin(delta1) * cos(phy1), cos(delta1) - sin(phy1) * sin(phy3));
			double lambda3 = lambda1 + delta_lambda1;
			ret.latitude = phy3 * 180.0 / V_PI;
			ret.longitude = lambda3 * 180.0 / V_PI;
		}
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

	RUNWAY() { clear(); }
	~RUNWAY() { clear(); }

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
		std::vector<std::string> numbers_dir = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
		char ret[4];
		memset(ret, 0, sizeof(ret));
		if (runway_number > 0 && runway_number <= 36)
			snprintf(ret, sizeof(ret), "%02d%c", runway_number, designator);
		else if (runway_number >= 37 && runway_number <= 44)
			snprintf(ret, sizeof(ret), "%s%c", numbers_dir[runway_number - 37].c_str(), designator);
		return std::string(ret);
	}
};

struct RUNWAY_OPERATION {
	int index = -1;
	bool is_primary = TRUE;
	double diff_bearing_tra = 0;
	double distances[2];
	double distances_percent[2];
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
	}

	void copy(AIRPORT* src) {
		memcpy(name, src->name, sizeof(src->name));
		memcpy(icao, src->icao, sizeof(src->icao));
		memcpy(region, src->region, sizeof(src->region));
		magvar = src->magvar;
		// Free any buffer this AIRPORT already owns before reassigning --
		// otherwise a copy() onto an already-populated AIRPORT (unlike this
		// header's one current caller, which always copies onto a freshly
		// clear()'d touchdown airport) would leak it.
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

struct TOUCHDOWN_DATA {
	struct FLIGHT_DATA flight_data;
	AIRPORT airport;
	int db_id = -1;             // trip_touchdowns row ID, set after immediate INSERT
	// Snapshot of STATUS::loc_dh taken the instant this touchdown is recorded
	// (see recorder.cpp) rather than read live from STATUS::loc_dh when this
	// touchdown's facility lookup eventually resolves. STATUS::loc_dh is a
	// single shared scratch field that keeps getting overwritten by every
	// subsequent low-altitude pass (e.g. a go-around's second approach) --
	// only one facility lookup is in flight at a time, so a touchdown's own
	// lookup can still be queued (see request_next_touchdown_facility_lookup)
	// when a later approach's crossing overwrites it. Capturing it here at
	// touchdown time is safe because a touchdown always passes through
	// STATUS::loc_dh's 50-100ft trigger band during its own final approach,
	// immediately beforehand -- so this field can never be stale for the
	// touchdown that captures it, unlike the shared field read later.
	COORDINATE loc_dh;
	struct TOUCHDOWN_DATA* next = NULL;
};

// Forward declaration — full definition in simconnect_defs.h
struct FLIGHT_DATA_RECORD;

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
// STATUS::sql for sample flushes. This removes the detached per-batch writer
// threads that used to race each other and the queue-reset code that ran when
// a new trip started while a previous trip's flush was still in flight.
class SampleWriteQueue {
public:
	void push(struct FLIGHT_DATA_RECORD* data, int trip_id) {
		{
			std::lock_guard<std::mutex> lock(mutex_);
			queue_.push_back({ data, trip_id });
		}
		cv_.notify_one();
	}

	// Blocks until an item is available. Returns false once stop() has been
	// called and the queue has fully drained -- the worker loop should exit.
	bool pop(SAMPLE_QUEUE_ITEM& item) {
		std::unique_lock<std::mutex> lock(mutex_);
		cv_.wait(lock, [this] { return !queue_.empty() || stopping_; });
		if (queue_.empty())
			return false;
		item = queue_.front();
		queue_.pop_front();
		return true;
	}

	// Tells the worker thread to exit once it has drained whatever is
	// currently queued (does not discard pending samples).
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

private:
	std::mutex mutex_;
	std::condition_variable cv_;
	std::deque<SAMPLE_QUEUE_ITEM> queue_;
	bool stopping_ = false;
};

struct STATUS {
	bool in_sim = FALSE;
	bool sim_running = FALSE;
	bool paused = FALSE;
	bool recording = FALSE;
	bool quit = FALSE;
	// Heap copy of the most recently produced sample, owned outside the
	// queue so the dispatch callback can compute the next sample's delta_s
	// without touching whatever the DB-write worker is doing.
	struct FLIGHT_DATA_RECORD* last_sample = NULL;
	HANDLE hSimConnect = NULL;
	sqlite3* sql = NULL;
	std::mutex mutex_db_commit;
	// Single-writer queue for periodic sample flushes -- see SampleWriteQueue
	// above. db_writer_thread is the one persistent thread draining it,
	// started in connect_db() and joined in wait_for_db_writers().
	SampleWriteQueue sample_write_queue;
	std::thread db_writer_thread;
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
	bool airborne = FALSE;
	TOUCHDOWN_DATA* touchdown_data = NULL;
	TOUCHDOWN_DATA* touchdown_data_end = NULL;
	FLIGHT_DATA data;
	COORDINATE loc_dh;
	AIRPORT departure;
	AIRPORT destination;
	// Set right before SimConnect_RequestFacilitiesList_EX1() is called (on
	// takeoff or touchdown) and cleared once the async facility lookup it
	// starts (AIRPORT_LIST -> optional FACILITY_DATA(s) -> FACILITY_DATA_END)
	// terminates. Only one such lookup may be in flight at a time -- overlapping
	// lookups would race on the departure/destination scratch objects above
	// (see MyDispatchProc in recorder.cpp). facility_lookup_trip_id records
	// which trip issued the in-flight lookup, so a response that arrives after
	// that trip has already ended (id_trip changed) can be recognized as stale
	// and dropped instead of being applied to whatever trip is active when it
	// lands.
	bool facility_lookup_pending = FALSE;
	int facility_lookup_trip_id = -1;
	// Set when a takeoff's own facility lookup was skipped because
	// facility_lookup_pending was already true (a previous trip's lookup was
	// still draining when this trip took off). request_next_touchdown_facility_lookup()
	// in recorder.cpp checks this before touchdown_data, so the departure lookup
	// is retried as soon as the shared slot frees up rather than being lost --
	// unlike touchdowns, a skipped takeoff has no "unresolved" marker of its own
	// to search for later. Cleared in stop_recording() so a lookup skipped by a
	// trip that ends before its retry turn can't be mistakenly fired for
	// whatever trip is active later.
	bool facility_lookup_departure_needed = FALSE;
	// Set the instant this trip's first takeoff is detected (recorder.cpp),
	// whether or not the resulting lookup fires immediately or is deferred via
	// facility_lookup_departure_needed above. A trip has exactly one departure
	// airport -- wherever that first takeoff happened -- so this must stay
	// TRUE for the rest of the trip, including through any number of later
	// touch-and-goes or full-stop taxi-back-and-takeoffs, none of which are a
	// new departure. Using departure.runway_act.index == -1 for this same
	// purpose used to be racy: that field only flips once the async lookup
	// actually *resolves*, so a takeoff occurring before a slow (e.g.
	// multi-chunk AIRPORT_LIST) departure lookup resolves would still see -1
	// and be mistaken for a fresh departure, overwriting the captured takeoff
	// coordinate/heading and eventually misrouting that stale lookup's
	// response into the destination slot once the real departure resolves.
	// Reset only at true trip boundaries: trip start and RecorderBridge::
	// tryConnect()'s carry-over reset (same places facility_lookup_departure_
	// needed etc. are reset), never on landing.
	bool departure_lookup_initiated = FALSE;
	// SendID of the most recent SimConnect_RequestFacilitiesList_EX1/
	// RequestFacilityData_EX1 call belonging to the in-flight lookup (see
	// facility_lookup_pending above), captured via SimConnect_GetLastSentPacketID
	// right after each call. SIMCONNECT_RECV_ID_EXCEPTION reports failed requests
	// asynchronously with no other correlation to the request that failed; matching
	// its dwSendID against this lets a rejected lookup request be recognized and
	// terminated instead of leaving facility_lookup_pending stuck true forever.
	DWORD facility_lookup_send_id = 0;
	// Guards SimConnect_AddToFacilityDefinition(DEFINITION_RUNWAYS, ...): those
	// fields describe the definition itself (server-side, per-connection state),
	// not any particular request, so they only need to be registered once per
	// connection -- re-adding the same fields on every lookup is wasteful and
	// risks eventually exceeding an internal SDK limit. Reset on reconnect
	// (RecorderBridge::tryConnect()) since a new SimConnect connection starts
	// with an empty definition table.
	bool facility_definition_runways_added = FALSE;
	// SimConnect_RequestFacilitiesList_EX1() takes no lat/lon -- it always
	// returns facilities near the aircraft's CURRENT position at the moment
	// the request is sent, not any historical position. Since only one lookup
	// may be in flight at a time (see facility_lookup_pending above), a
	// touchdown/departure lookup queued behind an earlier one can fire well
	// after the aircraft has moved from where that event actually happened
	// (e.g. a go-around after a bounced landing). facility_lookup_coordinate
	// is set immediately before each SimConnect_RequestFacilitiesList_EX1
	// call to the *historical* coordinate the response should be evaluated
	// against (the touchdown's stored TOUCHDOWN_DATA::flight_data.coordinate,
	// or facility_lookup_departure_coordinate below for a departure), and used
	// in place of status->data.coordinate throughout the AIRPORT_LIST/
	// FACILITY_DATA_END handlers so a moved-since aircraft position can't
	// misattribute the response to the wrong airport/runway.
	COORDINATE facility_lookup_coordinate;
	// The aircraft's coordinate at the moment of takeoff, captured whether or
	// not that takeoff's lookup fires immediately (see facility_lookup_departure_needed
	// above) -- a deferred departure lookup has no other record of where the
	// takeoff actually happened once request_next_touchdown_facility_lookup()
	// finally sends it.
	COORDINATE facility_lookup_departure_coordinate;
	// Same staleness problem as facility_lookup_coordinate above, but for the
	// heading used as the runway-bearing fallback when loc_dh (the low-altitude
	// decision-height position) isn't available: status->data.heading reflects
	// the aircraft's heading at the moment FACILITY_DATA_END arrives, which can
	// be well after the actual touchdown/takeoff (e.g. queued behind an earlier
	// lookup, or the aircraft has already turned off the runway). Set alongside
	// facility_lookup_coordinate from the same historical source (magnetic
	// heading, matching status->data.heading's units) each time that field is.
	int facility_lookup_heading = 0;
	// The aircraft's heading at the moment of takeoff -- see
	// facility_lookup_departure_coordinate above, same reasoning.
	int facility_lookup_departure_heading = 0;
	// This trip's own frozen low-altitude snapshot for the departure runway
	// match, analogous to TOUCHDOWN_DATA::loc_dh for a touchdown/destination
	// match. loc_dh below is a single field shared across every 50-100ft
	// AGL crossing of the whole trip (climb-outs and approaches alike), kept
	// continuously overwritten right up to whatever crossing happens last --
	// so if the departure lookup is deferred or queued behind another (see
	// facility_lookup_departure_needed above) and doesn't resolve until after
	// a later touchdown, touch-and-go climb-out, or go-around, loc_dh no
	// longer holds the original departure's position by the time
	// SIMCONNECT_RECV_ID_FACILITY_DATA_END reads it. Captured once, on the
	// first 50-100ft AGL crossing after this trip's first takeoff is
	// detected (departure_lookup_initiated above), and left untouched for
	// the rest of the trip -- latitude stays the COORDINATE-default 360
	// sentinel until that capture happens. Reset only at true trip
	// boundaries, same as departure_lookup_initiated.
	COORDINATE facility_lookup_departure_loc_dh;
	// Running top-N nearest airports across every chunk of the current
	// AIRPORT_LIST response. SimConnect splits a large facility list (e.g.
	// every airport in loaded scenery, 1000+ entries) across multiple
	// AIRPORT_LIST callbacks that share one request (see dwEntryNumber/
	// dwOutOf in MyDispatchProc) -- deciding "nearest airport" from any
	// single chunk in isolation is wrong, since a later chunk that happens
	// to contain only distant airports would otherwise conclude "not found"
	// and terminate/overwrite the lookup a second time while an earlier
	// chunk's real match was still being resolved. Reset to all-distance-1e9
	// when a fresh request's first chunk (dwEntryNumber == 0) arrives.
	struct FACILITY_LIST_CANDIDATE {
		double distance = 1e9;
		char ident[9] = {};
		char region[3] = {};
	};
	static const int FACILITY_LIST_TOP_N = 5;
	FACILITY_LIST_CANDIDATE facility_lookup_top[FACILITY_LIST_TOP_N];
	std::unordered_set<std::string> skip_events;
	// Names already announced via the one-time "Event skipped" trace line (see
	// the SIMCONNECT_RECV_ID_EVENT handler) -- some skip_events entries fire at
	// very high frequency (that's exactly why they're skipped in the first
	// place), so logging every occurrence would just move the log-bloat problem
	// from trip_events to msfs_fdr_debug.log instead of fixing it.
	std::unordered_set<std::string> skip_events_logged;
	// Same one-time-per-quiet-period throttle as skip_events_logged, but for the
	// "Event ignored (no active trip)" trace line -- SimConnect keeps delivering
	// event notifications (e.g. AUTOPILOT_OFF/APU_OFF_SWITCH toggled repeatedly
	// at the gate) even with no trip recording, and logging every one of those
	// floods msfs_fdr_debug.log just as badly as the skip_events case did.
	// Cleared whenever a trip starts or stops so each idle stretch gets its own
	// fresh one-time notice per event name.
	std::unordered_set<std::string> no_trip_events_logged;
	void* gui_context = nullptr;
};
