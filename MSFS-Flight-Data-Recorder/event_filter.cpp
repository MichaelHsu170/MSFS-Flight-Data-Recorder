#include "event_filter.h"

#include "logger.h"

#include <unordered_set>

namespace {

// Held flap levers legitimately fire these in rapid bursts; subjecting them
// to the tiers would misclassify real flap input as a flood.
const std::unordered_set<std::string> EVENT_FLOOD_WHITELIST = {
	"FLAPS_INCR", "FLAPS_DECR",
};

const std::chrono::milliseconds EVENT_TIER1_WINDOW(500);
// 3, not 2: a single accidental double-fire (switch bounce, a fast
// double-click) doesn't silence a name. A real flood (observed ~126 ms
// between repeats) still trips this within ~250 ms.
const size_t EVENT_TIER1_THRESHOLD = 3;

// Catches a flood too slow to ever trip tier 1 (e.g. one spurious
// occurrence every 1-2 s): 3 within 5 s is well outside normal timing for a
// discrete user action.
const std::chrono::milliseconds EVENT_TIER2_WINDOW(5000);
const size_t EVENT_TIER2_THRESHOLD = 3;

// The "Event flood ..." lines go straight to Logger, not gui_log_printf():
// they keep their real INFO/WARNING severity in msfs_fdr_debug.log but must
// never reach the Live Status panel, which gui_log_printf() decides by level
// alone (see gui_notify_log() in recorder_bridge.cpp).
void log_flood_ended(const std::string& name, size_t suppressed_count) {
	Logger::logf(Logger::Info, "Recorder", "Event flood ended: %s (suppressed %zu occurrences)",
		name.c_str(), suppressed_count);
}

}

void EventFloodFilter::tier2(const std::string& name, const Occurrence& occurrence, const Output& out) {
	auto now = now_();
	Tier2State* state = &tier2_[name];

	if (state->suppressing) {
		if (now - state->last_time < EVENT_TIER2_WINDOW) {
			state->last_time = now;
			state->suppressed_count++;
			return;
		}
		// Quiet period elapsed while suppressing -- the flood is over; forget
		// it and treat this occurrence as the start of a new window.
		log_flood_ended(name, state->suppressed_count);
		tier2_.erase(name);
		state = &tier2_[name];
	}

	while (!state->recent.empty() && now - state->recent.front().time >= EVENT_TIER2_WINDOW)
		state->recent.pop_front();
	state->last_time = now;

	unsigned long long seq = ++next_seq_;
	state->recent.push_back({ seq, now });
	out.commit(name, occurrence.trip_id, occurrence.time_zulu, occurrence.time_local, seq);

	if (state->recent.size() >= EVENT_TIER2_THRESHOLD) {
		std::vector<unsigned long long> seqs;
		seqs.reserve(state->recent.size());
		for (const Tier2State::Commit& c : state->recent)
			seqs.push_back(c.seq);
		Logger::logf(Logger::Warning, "Recorder",
			"Event flood detected: %s repeated %zu times in under %lldms; retracting and suppressing further occurrences until it settles",
			name.c_str(), seqs.size(), (long long)EVENT_TIER2_WINDOW.count());
		out.retract(seqs);
		state->suppressing = true;
		state->suppressed_count = state->recent.size();
		state->recent.clear();
	}
}

void EventFloodFilter::flush_pending(const std::string& name, std::vector<Occurrence>& pending, const Output& out) {
	for (const Occurrence& occurrence : pending)
		tier2(name, occurrence, out);
	pending.clear();
}

void EventFloodFilter::flush_stale(const Output& out) {
	auto now = now_();
	for (auto it = tier1_.begin(); it != tier1_.end(); ) {
		if (now - it->second.last_time >= EVENT_TIER1_WINDOW) {
			if (it->second.suppressing)
				log_flood_ended(it->first, it->second.suppressed_count);
			else
				flush_pending(it->first, it->second.pending, out);
			it = tier1_.erase(it);
		} else {
			++it;
		}
	}
	// Tier 2 holds nothing back, so a stale entry is just forgotten.
	for (auto it = tier2_.begin(); it != tier2_.end(); ) {
		if (now - it->second.last_time >= EVENT_TIER2_WINDOW) {
			if (it->second.suppressing)
				log_flood_ended(it->first, it->second.suppressed_count);
			it = tier2_.erase(it);
		} else {
			++it;
		}
	}
}

void EventFloodFilter::flush_all(const Output& out) {
	for (auto it = tier1_.begin(); it != tier1_.end(); ++it) {
		if (!it->second.suppressing)
			flush_pending(it->first, it->second.pending, out);
	}
	tier1_.clear();
	tier2_.clear();
}

void EventFloodFilter::record(const std::string& name, int trip_id, const std::string& time_zulu,
	const std::string& time_local, const Output& out) {
	if (EVENT_FLOOD_WHITELIST.count(name)) {
		out.commit(name, trip_id, time_zulu, time_local, ++next_seq_);
		return;
	}

	auto now = now_();
	Tier1State& state = tier1_[name];

	if (state.suppressing) {
		if (now - state.last_time >= EVENT_TIER1_WINDOW) {
			// Quiet period elapsed while suppressing -- the burst is over;
			// this occurrence starts a new streak.
			log_flood_ended(name, state.suppressed_count);
			tier1_.erase(name);
			Tier1State& fresh = tier1_[name];
			fresh.last_time = now;
			fresh.pending.push_back({ trip_id, time_zulu, time_local });
			return;
		}
		state.last_time = now;
		state.suppressed_count++;
		return;
	}

	if (!state.pending.empty() && now - state.last_time >= EVENT_TIER1_WINDOW)
		flush_pending(name, state.pending, out);

	state.last_time = now;
	state.pending.push_back({ trip_id, time_zulu, time_local });

	if (state.pending.size() >= EVENT_TIER1_THRESHOLD) {
		Logger::logf(Logger::Warning, "Recorder",
			"Event flood detected: %s repeated %zu times in under %lldms; suppressing further occurrences until it settles",
			name.c_str(), state.pending.size(), (long long)EVENT_TIER1_WINDOW.count());
		state.suppressing = true;
		state.suppressed_count = state.pending.size();
		state.pending.clear();
	}
}
