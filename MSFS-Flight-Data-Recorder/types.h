#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
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
	// Operational heading (1-360, aviation convention -- due north is 360,
	// never 0) of the runway end actually used -- i.e.
	// AIRPORT::runways[index].heading, flipped 180° if !is_primary. Captured
	// and rounded once at match time (see the runway-candidate loop in
	// recorder.cpp) instead of being re-derived from runways[] wherever it's
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
		// Free any buffer this AIRPORT already owns before reassigning --
		// otherwise a copy() onto an already-populated AIRPORT (unlike this
		// header's two current callers, which both copy onto a freshly
		// clear()'d LIFTOFF_DATA/TOUCHDOWN_DATA node airport -- see
		// FACILITY_DATA_END's liftoff-marker and touchdown branches) would
		// leak it.
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

// Every moment this trip's aircraft actually became airborne, as a marker
// occurrence (touch-and-goes included), independent of the trip's single,
// permanent "departure" record (STATUS::departure / departure_db_id), which
// always stays locked to the first liftoff only. See recorder.cpp's
// liftoff-detection block.
struct LIFTOFF_DATA {
	struct FLIGHT_DATA flight_data;
	AIRPORT airport;
	int db_id = -1;              // trip_liftoffs row ID, set after immediate INSERT
	// Monotonically increasing across both liftoff_data and touchdown_data
	// (STATUS::next_facility_lookup_seq), so request_next_touchdown_facility_lookup
	// can pick whichever of the two lists holds the chronologically earliest
	// unresolved lookup instead of always preferring one list over the other.
	int seq = 0;
	struct LIFTOFF_DATA* next = NULL;
};

// Mirrors LIFTOFF_DATA, but for touchdowns.
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
	int seq = 0;
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

// One entry in STATUS::event_write_queue -- either an Insert (one new
// trip_events row) or a Delete (retract previously-inserted rows by
// event_seq, e.g. tier 2 confirming a flood after already forwarding up to
// EVENT_TIER2_THRESHOLD occurrences -- see tier2_gate() in recorder.cpp).
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
	// this occurrence's STATUS::next_event_seq value, stored alongside it in
	// trip_events so a later Delete can target it precisely.
	int trip_id = -1;
	std::string event;
	std::string time_zulu;
	std::string time_local;
	unsigned long long seq = 0;

	// Delete fields -- only meaningful when kind == Kind::Delete. Never more
	// than EVENT_TIER2_THRESHOLD entries in practice, but not capped here.
	std::vector<unsigned long long> delete_seqs;
};

// Thread-safe queue feeding a single persistent event-write worker thread
// (event_write_worker in db.cpp), mirroring SampleWriteQueue above. Moves
// db_insert_event's synchronous BEGIN/INSERT/COMMIT (including its fsync) off
// the SimConnect dispatch thread, which otherwise blocks the UI directly --
// see EVENT_STREAK below for why that mattered in practice.
class EventWriteQueue {
public:
	void push(int trip_id, const std::string& event, const std::string& time_zulu, const std::string& time_local, unsigned long long seq) {
		EVENT_QUEUE_ITEM item;
		item.kind = EVENT_QUEUE_ITEM::Kind::Insert;
		item.trip_id = trip_id;
		item.event = event;
		item.time_zulu = time_zulu;
		item.time_local = time_local;
		item.seq = seq;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			queue_.push_back(std::move(item));
		}
		cv_.notify_one();
	}

	// Enqueues a retraction of previously-inserted rows by event_seq -- see
	// EVENT_QUEUE_ITEM::Kind::Delete above.
	void push_delete(std::vector<unsigned long long> seqs) {
		EVENT_QUEUE_ITEM item;
		item.kind = EVENT_QUEUE_ITEM::Kind::Delete;
		item.delete_seqs = std::move(seqs);
		{
			std::lock_guard<std::mutex> lock(mutex_);
			queue_.push_back(std::move(item));
		}
		cv_.notify_one();
	}

	bool pop(EVENT_QUEUE_ITEM& item) {
		std::unique_lock<std::mutex> lock(mutex_);
		cv_.wait(lock, [this] { return !queue_.empty() || stopping_; });
		if (queue_.empty())
			return false;
		item = queue_.front();
		queue_.pop_front();
		return true;
	}

	void stop() {
		{
			std::lock_guard<std::mutex> lock(mutex_);
			stopping_ = true;
		}
		cv_.notify_one();
	}

	void reset() {
		std::lock_guard<std::mutex> lock(mutex_);
		stopping_ = false;
	}

private:
	std::mutex mutex_;
	std::condition_variable cv_;
	std::deque<EVENT_QUEUE_ITEM> queue_;
	bool stopping_ = false;
};

// One buffered, not-yet-decided occurrence of a repeated event name -- see
// EVENT_STREAK. trip_id is captured per-occurrence (not read from
// status->id_trip at flush time) so a streak still pending across a trip
// boundary still attributes each occurrence to the trip it actually happened
// in once flushed.
struct EVENT_STREAK_OCCURRENCE {
	int trip_id;
	std::string time_zulu;
	std::string time_local;
};

// Tier 1 (fast-burst, blocking) per-event-name flood-detection state -- see
// record_event() in recorder.cpp. This is the FIRST of two independent flood
// gates an occurrence passes through before it can ever reach commit_event();
// see EVENT_TIER2_STATE below for the second. Occurrences of a
// non-whitelisted event are held here, not yet logged/recorded, until either
// EVENT_TIER1_WINDOW passes with no new occurrence (flushed as legitimate --
// each one then proceeds to tier 2) or EVENT_TIER1_THRESHOLD consecutive
// occurrences arrive less than EVENT_TIER1_WINDOW apart, at which point
// `suppressing` flips true: further occurrences are dropped silently (not
// buffered, not logged individually, never reaching tier 2 at all) for as
// long as they keep arriving within EVENT_TIER1_WINDOW of each other. The
// instant a gap of EVENT_TIER1_WINDOW or more elapses -- whether still
// accumulating `pending` or already `suppressing` -- the whole entry is
// erased and forgotten; the next occurrence of that name starts a brand new
// streak from scratch. There is deliberately no longer-lived memory of "this
// name floods" beyond that: a name that flooded once is fully eligible to be
// treated as legitimate again the moment the burst that triggered it stops
// (see record_event() in recorder.cpp).
struct EVENT_STREAK {
	std::chrono::steady_clock::time_point last_time;
	std::vector<EVENT_STREAK_OCCURRENCE> pending;
	bool suppressing = false;
	size_t suppressed_count = 0;
};

// Tier 2 (slow-drip, non-blocking-until-confirmed) per-event-name
// flood-detection state -- see tier2_gate() in recorder.cpp. Every occurrence
// tier 1 (EVENT_STREAK above) forwards as legitimate passes through here
// next. Unlike tier 1, tier 2 does NOT hold occurrences back: each one is
// passed to commit_event() immediately and tracked in `recent` regardless of
// whether that call actually wrote anything (commit_event() drops it silently
// if no trip is active -- see its comment in recorder.cpp), so tier 2 can
// tell whether EVENT_TIER2_THRESHOLD of them land within EVENT_TIER2_WINDOW
// of each other even across a no-trip stretch. The moment that count is
// reached, the flood is "confirmed": every occurrence still in `recent`
// (there are exactly EVENT_TIER2_THRESHOLD of them, by construction) is
// retracted -- deleted from trip_events and pulled back out of the Live
// Status list via their seq, harmlessly a no-op for any that were never
// actually written/shown in the first place -- since a confirmed-flooding
// event's occurrences carry no useful information once the pattern is known,
// and `suppressing` flips true so further occurrences are dropped with no
// commit, no retraction bookkeeping, and no UI line at all. Exactly like
// tier 1, this is self-healing purely from timing: the instant a gap of
// EVENT_TIER2_WINDOW passes with nothing arriving, the whole entry is
// erased and the next occurrence starts a brand new tier-2 window from
// scratch -- there is no longer-lived "this name floods" memory here either.
struct EVENT_TIER2_STATE {
	// One still-in-window occurrence that has passed through commit_event()
	// (which may or may not have actually written it, depending on trip
	// state -- see commit_event()'s comment in recorder.cpp), identified by
	// its STATUS::next_event_seq value so it can be retracted precisely --
	// see EVENT_QUEUE_ITEM::Kind::Delete. Cleared out (not merely aged off)
	// the moment `suppressing` engages, since these have just been retracted
	// (or were never written) and no longer need tracking.
	struct COMMIT {
		unsigned long long seq;
		std::chrono::steady_clock::time_point time;
	};
	std::deque<COMMIT> recent;
	// Last occurrence of any kind (tracked into `recent`, or silently
	// suppressed) -- distinct from the timestamps inside `recent`, which only
	// covers tracked occurrences and is fully cleared on suppression. This is
	// what lets self-healing detect a quiet gap while `suppressing` is true,
	// when `recent` itself is already empty.
	std::chrono::steady_clock::time_point last_time;
	bool suppressing = false;
	size_t suppressed_count = 0;
};

struct STATUS {
	bool in_sim = FALSE;
	bool sim_running = FALSE;
	bool paused = FALSE;
	bool recording = FALSE;
	// User-facing gate on automatic recording start, toggled via the Recording
	// indicator in LiveStatusPanel and persisted through AppSettings. Distinct
	// from `recording` (which trip is actually mid-flight right now): this only
	// suppresses the auto-start-on-liftoff check in recorder.cpp, so flipping it
	// while a trip is already recording has no effect on that trip.
	bool recording_enabled = TRUE;
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
	bool airborne = FALSE;
	// Every moment this trip's aircraft actually became airborne, as a marker
	// occurrence (touch-and-goes included), independent of the trip's single
	// permanent departure_db_id/departure below -- see LIFTOFF_DATA. NOT
	// populated for the trip's first liftoff, which only ever updates
	// departure_db_id/departure (see MyDispatchProc's liftoff-detection
	// block).
	LIFTOFF_DATA* liftoff_data = NULL;
	LIFTOFF_DATA* liftoff_data_end = NULL;
	TOUCHDOWN_DATA* touchdown_data = NULL;
	TOUCHDOWN_DATA* touchdown_data_end = NULL;
	// Shared seq counter for LIFTOFF_DATA::seq/TOUCHDOWN_DATA::seq, so
	// request_next_touchdown_facility_lookup() can pick whichever of the two
	// lists holds the chronologically earliest unresolved lookup. Reset only
	// at true trip boundaries, same as departure_lookup_initiated.
	int next_facility_lookup_seq = 0;
	// trip_liftoffs row ID for this trip's single departure, set after the
	// immediate INSERT at the moment it becomes airborne (see MyDispatchProc's
	// liftoff-detection block) and consumed later by FACILITY_DATA_END's
	// departure UPDATE, same db_id pattern as TOUCHDOWN_DATA::db_id but for
	// the one-per-trip departure. Reset to -1 at trip start (recording-start
	// block in recorder.cpp).
	int departure_db_id = -1;
	FLIGHT_DATA data;
	COORDINATE loc_dh;
	AIRPORT departure;
	// The trip's destination airport, filled in once a touchdown's facility
	// lookup resolves (copied from here into the matching TOUCHDOWN_DATA
	// node -- see FACILITY_DATA_END). Also doubles as scratch space for that
	// same lookup while it's still in flight (the in-progress AIRPORT_LIST
	// candidate search, before a specific runway is known to be the match),
	// which is safe because only one facility lookup is ever in flight at a
	// time (facility_lookup_pending below) and every read of this object
	// happens synchronously within the same FACILITY_DATA_END/AIRPORT_LIST
	// callback that just populated it.
	AIRPORT destination;
	// Scratch space for an in-flight liftoff-marker (touch-and-go, not the
	// trip's one departure) lookup -- mirrors destination's scratch role
	// above for a touchdown lookup, but kept as its own field rather than
	// sharing destination: even though the two are never populated
	// concurrently (same single-flight guarantee as above), they mean
	// different things -- destination is the trip's actual destination
	// airport, this is a transient candidate for whichever liftoff marker is
	// currently being resolved -- and collapsing them into one field would
	// make status->destination silently hold liftoff-marker data during that
	// window, a landmine for any future code that reads it assuming it's
	// always the trip's destination. The lookup-handling logic these two
	// share is reused via facility_lookup_target()/facility_lookup_target_label()
	// in recorder.cpp (code reuse), not by reusing this storage.
	AIRPORT liftoff_scratch;
	// Set right before SimConnect_RequestFacilitiesList_EX1() is called (on
	// becoming airborne or touchdown) and cleared once the async facility lookup it
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
	// Set alongside facility_lookup_pending whenever the in-flight lookup is
	// for a liftoff *marker* (touch-and-go) rather than the trip's departure
	// or a touchdown -- facility_lookup_is_departure below still distinguishes
	// "departure vs. not" on its own, but can no longer alone tell a
	// liftoff-marker lookup apart from a touchdown lookup within the "not
	// departure" case. Read throughout MyDispatchProc's AIRPORT_LIST/
	// FACILITY_DATA/FACILITY_DATA_END/EXCEPTION handlers (always via the
	// facility_lookup_target()/facility_lookup_target_label() helpers, not
	// re-derived) and set by request_next_touchdown_facility_lookup().
	bool facility_lookup_is_liftoff = FALSE;
	// Set alongside facility_lookup_pending/facility_lookup_is_liftoff above,
	// at each of the same three call sites, to whether the in-flight lookup
	// is for the trip's one departure. This used to be re-derived at each
	// async callback as "departure.runway_act.index == -1", which is exactly
	// the kind of live/mutable check departure_lookup_initiated's own comment
	// below already warns against for a different reason (a fast touch-and-go
	// racing a slow lookup) -- it has a second failure mode across a trip
	// boundary: if a liftoff-marker or destination lookup is still in flight
	// when its trip ends, the new trip's status->departure.clear() resets
	// runway_act.index back to -1 out from under it, so the stale response
	// gets misattributed to &status->departure instead of its real target,
	// permanently leaking that target's runways buffer (freed on the wrong
	// object by FacilityLookupCleanup in FACILITY_DATA_END). Capturing this
	// once at request time, like facility_lookup_is_liftoff already does,
	// makes slot selection depend only on state fixed at request time.
	bool facility_lookup_is_departure = FALSE;
	// Set when a departure's own facility lookup was skipped because
	// facility_lookup_pending was already true (a previous trip's lookup was
	// still draining when this trip became airborne). request_next_touchdown_facility_lookup()
	// in recorder.cpp checks this before touchdown_data, so the departure lookup
	// is retried as soon as the shared slot frees up rather than being lost --
	// unlike touchdowns, a skipped departure has no "unresolved" marker of its own
	// to search for later. Cleared in stop_recording() so a lookup skipped by a
	// trip that ends before its retry turn can't be mistakenly fired for
	// whatever trip is active later.
	bool facility_lookup_departure_needed = FALSE;
	// Set the instant this trip's first liftoff is detected (recorder.cpp),
	// whether or not the resulting lookup fires immediately or is deferred via
	// facility_lookup_departure_needed above. A trip has exactly one departure
	// airport -- wherever that first liftoff happened -- so this must stay
	// TRUE for the rest of the trip, including through any number of later
	// touch-and-goes or full-stop taxi-back-and-liftoffs, none of which are a
	// new departure. Using departure.runway_act.index == -1 for this same
	// purpose used to be racy: that field only flips once the async lookup
	// actually *resolves*, so becoming airborne before a slow (e.g.
	// multi-chunk AIRPORT_LIST) departure lookup resolves would still see -1
	// and be mistaken for a fresh departure, overwriting the captured liftoff
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
	// The aircraft's coordinate at the moment it became airborne, captured
	// whether or not that liftoff's lookup fires immediately (see
	// facility_lookup_departure_needed above) -- a deferred departure lookup
	// has no other record of where the liftoff actually happened once
	// request_next_touchdown_facility_lookup() finally sends it.
	COORDINATE facility_lookup_departure_coordinate;
	// Same staleness problem as facility_lookup_coordinate above, but for the
	// heading used as the runway-bearing fallback when loc_dh (the low-altitude
	// decision-height position) isn't available: status->data.heading reflects
	// the aircraft's heading at the moment FACILITY_DATA_END arrives, which can
	// be well after the actual touchdown/liftoff (e.g. queued behind an earlier
	// lookup, or the aircraft has already turned off the runway). Set alongside
	// facility_lookup_coordinate from the same historical source (magnetic
	// heading, matching status->data.heading's units) each time that field is.
	int facility_lookup_heading = 0;
	// The aircraft's heading at the moment it became airborne -- see
	// facility_lookup_departure_coordinate above, same reasoning.
	int facility_lookup_departure_heading = 0;
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
	// Per-event-name tier 1 flood-detection buffers -- see EVENT_STREAK and
	// record_event() in recorder.cpp. Runs continuously for the life of the
	// app (not scoped to a trip or reset when one starts/ends): each entry's
	// own quiet-period timeout is what clears it, so there's nothing here
	// that needs a trip boundary to reset.
	std::unordered_map<std::string, EVENT_STREAK> event_streaks;
	// Per-event-name tier 2 flood-detection state -- see EVENT_TIER2_STATE and
	// tier2_gate() in recorder.cpp. Same "no trip boundary reset" reasoning as
	// event_streaks above.
	std::unordered_map<std::string, EVENT_TIER2_STATE> tier2_state;
	// Assigns each occurrence committed by commit_event() a unique, monotonic,
	// never-reused id, stored as trip_events.event_seq. Needed because
	// time_zulu/time_local (status->data.time_zulu/time_local) are read from a
	// periodically-refreshed snapshot, not captured per-event -- two distinct
	// occurrences can share an identical timestamp string, which would make
	// tier 2's retraction (EVENT_QUEUE_ITEM::Kind::Delete) unsafe if it
	// matched on timestamp instead. Only ever incremented, never reset --
	// including across reconnects -- so a seq is always unambiguous even if
	// two occurrences happen to be assigned across a reconnect boundary.
	unsigned long long next_event_seq = 0;
	// Per-event-name last-logged time for the "Event ignored (no active
	// trip)" TRACE line -- see EVENT_NO_TRIP_LOG_COOLDOWN and commit_event()
	// in recorder.cpp. Purely a log rate-limit, not a suppression: unlike the
	// no_trip_events_logged blacklist this replaces, an entry never blocks an
	// occurrence from committing and needs no trip-boundary reset -- it just
	// ages out naturally once EVENT_NO_TRIP_LOG_COOLDOWN elapses.
	std::unordered_map<std::string, std::chrono::steady_clock::time_point> no_trip_log_throttle;
	void* gui_context = nullptr;
};
