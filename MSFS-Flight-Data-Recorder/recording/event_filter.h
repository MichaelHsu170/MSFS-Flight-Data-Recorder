#pragma once

#include <algorithm>
#include <chrono>
#include <deque>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

// Flood protection for SimConnect notification events: a simulator-side fault
// (e.g. a stuck autopilot mode oscillating on/off many times a second) can
// fire one event name thousands of times a minute, and nothing tells which
// name in advance. Every occurrence passes through two independent tiers
// before it is committed:
//
// - Tier 1 (fast burst, blocking): occurrences of a name are held back until
//   a quiet gap of 500 ms. Three occurrences less than 500 ms apart start
//   suppressing it -- further ones within 500 ms of the previous are dropped
//   silently; nothing of the burst is committed.
// - Tier 2 (slow drip, non-blocking): every occurrence tier 1 lets through is
//   committed immediately, but three within 5 s confirm a slow flood: those
//   three are retracted and further ones dropped until 5 s pass quietly.
//
// Neither tier remembers a name for longer than its window: the moment a
// quiet gap passes, the name is treated as ordinary again. FLAPS_INCR/
// FLAPS_DECR (held flap levers legitimately repeat quickly) bypass both
// tiers and commit immediately. Runs regardless of whether a trip is active;
// the caller's commit decides whether an occurrence is actually written.
//
// Quiet periods are only noticed when something calls in: record() for the
// same name, or flush_stale() (called on every sample tick).
class EventFloodFilter {
public:
	using TimePoint = std::chrono::steady_clock::time_point;

	struct Output {
		// One occurrence that passed (or bypassed) both tiers, with the
		// trip_id and timestamps captured when it happened and a unique,
		// never-reused seq for retracting it later.
		std::function<void(const std::string& name, int trip_id, const std::string& time_zulu,
			const std::string& time_local, unsigned long long seq)> commit;
		// Previously committed occurrences, by seq, now confirmed to be a
		// slow flood.
		std::function<void(const std::vector<unsigned long long>& seqs)> retract;
	};

	// One occurrence of name, as seen now. trip_id and the timestamps are
	// carried through to commit even if it commits later (e.g. after its trip
	// ended), so it lands in the trip it happened in.
	void record(const std::string& name, int trip_id, const std::string& time_zulu, const std::string& time_local,
		const Output& out);
	// Resolves every name whose quiet period has elapsed: commits tier 1's
	// held-back occurrences and forgets finished floods.
	void flush_stale(const Output& out);
	// Commits everything still held back and forgets all state, without
	// waiting for quiet periods -- for shutdown, when nothing will call
	// flush_stale() again. Deliberately doesn't log "flood ended" for floods
	// still being suppressed: they may still be arriving.
	void flush_all(const Output& out);
	// Makes every later seq greater than seq, so this run's seqs can't repeat
	// ones an earlier run stored (trip_events.event_seq, see connect_db()).
	// Never lowers the next seq.
	void continue_after(unsigned long long seq) { next_seq_ = std::max<unsigned long long>(next_seq_, seq); }

	// Replaces the clock (std::chrono::steady_clock::now by default), so
	// tests can move time forward instead of waiting.
	void set_clock(std::function<TimePoint()> now) { now_ = std::move(now); }

private:
	// One held-back tier 1 occurrence, with what it carries to commit.
	struct Occurrence {
		int trip_id;
		std::string time_zulu;
		std::string time_local;
	};
	struct Tier1State {
		TimePoint last_time;
		std::vector<Occurrence> pending;
		bool suppressing = false;
		size_t suppressed_count = 0;
	};
	struct Tier2State {
		struct Commit {
			unsigned long long seq;
			TimePoint time;
		};
		// Committed occurrences still inside the 5 s window; cleared (not
		// aged off) when suppression starts, since they were just retracted.
		std::deque<Commit> recent;
		// Last occurrence of any kind -- lets a quiet gap be detected while
		// suppressing, when recent is empty.
		TimePoint last_time;
		bool suppressing = false;
		size_t suppressed_count = 0;
	};

	void tier2(const std::string& name, const Occurrence& occurrence, const Output& out);
	void flush_pending(const std::string& name, std::vector<Occurrence>& pending, const Output& out);

	std::function<TimePoint()> now_ = std::chrono::steady_clock::now;
	std::unordered_map<std::string, Tier1State> tier1_;
	std::unordered_map<std::string, Tier2State> tier2_;
	// Only ever incremented -- including across reconnects -- so a seq never
	// identifies two occurrences. Needed because timestamps come from the
	// latest sample and two occurrences can share one. continue_after() moves
	// it past the seqs earlier runs stored.
	unsigned long long next_seq_ = 0;
};
