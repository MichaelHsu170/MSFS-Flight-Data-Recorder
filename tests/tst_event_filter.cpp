// Event flood filter (event_filter.cpp) on its own: occurrences in on a fake
// clock, commits and retractions out.
#include "event_filter.h"

#include <QtTest>

namespace {

struct Commit {
	std::string name;
	int tripId;
	std::string timeZulu;
	std::string timeLocal;
	unsigned long long seq;
};

// An EventFloodFilter on a clock that only advance() moves, recording what
// it outputs.
struct Harness {
	EventFloodFilter filter;
	EventFloodFilter::TimePoint clock = std::chrono::steady_clock::now();
	std::vector<Commit> commits;
	std::vector<std::vector<unsigned long long>> retractions;
	EventFloodFilter::Output out;

	Harness() {
		filter.set_clock([this] { return clock; });
		out.commit = [this](const std::string& name, int tripId, const std::string& zulu,
			const std::string& local, unsigned long long seq) {
			commits.push_back({ name, tripId, zulu, local, seq });
		};
		out.retract = [this](const std::vector<unsigned long long>& seqs) { retractions.push_back(seqs); };
	}

	void advance(int ms) { clock += std::chrono::milliseconds(ms); }
	void record(const std::string& name, int tripId = 1) {
		filter.record(name, tripId, "zulu", "local", out);
	}
	void flushStale() { filter.flush_stale(out); }
	int count(const std::string& name) const {
		int n = 0;
		for (const Commit& c : commits)
			n += c.name == name;
		return n;
	}
};

}

class TstEventFilter : public QObject {
	Q_OBJECT

private slots:
	void singleOccurrenceIsHeldUntilQuietPeriod() {
		Harness h;
		h.filter.record("GEAR_UP", 7, "2026-01-02 03:04:05", "2026-01-02 05:04:05", h.out);
		h.flushStale();
		h.advance(499);
		h.flushStale();
		QVERIFY(h.commits.empty());
		h.advance(1);
		h.flushStale();
		QCOMPARE(h.commits.size(), size_t(1));
		// What was captured at record() time is carried through.
		QCOMPARE(h.commits[0].name, std::string("GEAR_UP"));
		QCOMPARE(h.commits[0].tripId, 7);
		QCOMPARE(h.commits[0].timeZulu, std::string("2026-01-02 03:04:05"));
		QCOMPARE(h.commits[0].timeLocal, std::string("2026-01-02 05:04:05"));
		QCOMPARE(h.commits[0].seq, 1ULL);
	}

	void heldOccurrenceCommitsOnNextOccurrenceAfterQuiet() {
		Harness h;
		h.record("GEAR_UP", 1);
		h.advance(600);
		h.record("GEAR_UP", 2); // no flush_stale() in between
		QCOMPARE(h.commits.size(), size_t(1));
		QCOMPARE(h.commits[0].tripId, 1);
		h.advance(600);
		h.flushStale();
		QCOMPARE(h.commits.size(), size_t(2));
		QCOMPARE(h.commits[1].tripId, 2);
	}

	void twoQuickRepeatsAreBothCommitted() {
		Harness h;
		h.record("GEAR_TOGGLE");
		h.advance(100);
		h.record("GEAR_TOGGLE");
		h.advance(500);
		h.flushStale();
		QCOMPARE(h.count("GEAR_TOGGLE"), 2);
		QVERIFY(h.retractions.empty());
	}

	void threeQuickRepeatsAreSuppressed() {
		Harness h;
		for (int i = 0; i < 3; ++i) {
			h.record("AP_MASTER");
			h.advance(100);
		}
		h.advance(500);
		h.flushStale();
		QVERIFY(h.commits.empty());
		QVERIFY(h.retractions.empty());
	}

	void burstStaysSuppressedWhileItContinues() {
		Harness h;
		// 20 occurrences 400 ms apart: never a 500 ms gap, so the whole 8 s
		// burst is dropped.
		for (int i = 0; i < 20; ++i) {
			h.record("AP_MASTER");
			h.flushStale();
			h.advance(400);
		}
		h.advance(100);
		h.flushStale();
		QVERIFY(h.commits.empty());
		// Over: the next occurrence is ordinary again.
		h.record("AP_MASTER");
		h.advance(500);
		h.flushStale();
		QCOMPARE(h.count("AP_MASTER"), 1);
	}

	void occurrenceAfterBurstStartsNewStreakWithoutFlush() {
		Harness h;
		for (int i = 0; i < 3; ++i)
			h.record("AP_MASTER");
		h.advance(500);
		h.record("AP_MASTER"); // ends the burst itself, no flush_stale()
		QVERIFY(h.commits.empty());
		h.advance(500);
		h.flushStale();
		QCOMPARE(h.count("AP_MASTER"), 1);
	}

	void flapsBypassBothTiers() {
		Harness h;
		for (int i = 0; i < 6; ++i) {
			h.record("FLAPS_INCR");
			h.record("FLAPS_DECR");
		}
		QCOMPARE(h.count("FLAPS_INCR"), 6);
		QCOMPARE(h.count("FLAPS_DECR"), 6);
		h.advance(6000);
		h.flushStale();
		QCOMPARE(h.commits.size(), size_t(12));
		QVERIFY(h.retractions.empty());
	}

	void namesAreIndependent() {
		Harness h;
		for (int i = 0; i < 3; ++i)
			h.record("AP_MASTER");
		h.record("GEAR_UP");
		h.advance(500);
		h.flushStale();
		QCOMPARE(h.count("AP_MASTER"), 0);
		QCOMPARE(h.count("GEAR_UP"), 1);
	}

	void slowRepeatsAreRetractedThenSuppressedThenRecover() {
		Harness h;
		for (int i = 0; i < 3; ++i) {
			h.record("AP_HDG_HOLD");
			h.advance(600);
			h.flushStale();
		}
		// Each was committed, then the third confirmed the flood.
		QCOMPARE(h.count("AP_HDG_HOLD"), 3);
		QCOMPARE(h.retractions.size(), size_t(1));
		QCOMPARE(h.retractions[0], (std::vector<unsigned long long>{ 1, 2, 3 }));
		// Suppressed while it keeps coming within 5 s of the previous one.
		for (int i = 0; i < 3; ++i) {
			h.advance(4000);
			h.record("AP_HDG_HOLD");
			h.advance(600);
			h.flushStale();
		}
		QCOMPARE(h.count("AP_HDG_HOLD"), 3);
		// 5 s of quiet ends it.
		h.advance(5000);
		h.flushStale();
		h.record("AP_HDG_HOLD");
		h.advance(600);
		h.flushStale();
		QCOMPARE(h.count("AP_HDG_HOLD"), 4);
		QCOMPARE(h.retractions.size(), size_t(1));
	}

	void suppressedSlowFloodEndsOnNextOccurrenceWithoutFlush() {
		Harness h;
		for (int i = 0; i < 3; ++i) {
			h.record("AP_HDG_HOLD");
			h.advance(600);
			h.flushStale();
		}
		// No flush_stale() while the 5 s pass: the slow-flood state is still
		// there when this occurrence reaches it, and it ends the flood.
		h.advance(5000);
		h.record("AP_HDG_HOLD");
		h.advance(600);
		h.flushStale();
		QCOMPARE(h.retractions.size(), size_t(1));
		QCOMPARE(h.count("AP_HDG_HOLD"), 4);
	}

	void repeatsFiveSecondsApartAreNotAFlood() {
		Harness h;
		for (int i = 0; i < 4; ++i) {
			h.record("GEAR_TOGGLE");
			h.advance(2500);
			h.flushStale();
		}
		QCOMPARE(h.count("GEAR_TOGGLE"), 4);
		QVERIFY(h.retractions.empty());
	}

	void doubleThenSingleWithinFiveSecondsIsASlowFlood() {
		Harness h;
		// A quick double passes tier 1 as two occurrences, both counting
		// towards tier 2's three.
		h.record("GEAR_TOGGLE");
		h.record("GEAR_TOGGLE");
		h.advance(500);
		h.flushStale();
		h.advance(1000);
		h.record("GEAR_TOGGLE");
		h.advance(500);
		h.flushStale();
		QCOMPARE(h.count("GEAR_TOGGLE"), 3);
		QCOMPARE(h.retractions.size(), size_t(1));
		QCOMPARE(h.retractions[0].size(), size_t(3));
	}

	void flushAllCommitsHeldOccurrencesButNotSuppressedOnes() {
		Harness h;
		h.record("GEAR_UP", 3);
		for (int i = 0; i < 3; ++i)
			h.record("AP_MASTER", 3);
		h.filter.flush_all(h.out);
		QCOMPARE(h.count("GEAR_UP"), 1);
		QCOMPARE(h.count("AP_MASTER"), 0);
		QCOMPARE(h.commits[0].tripId, 3);
		// Everything is forgotten: nothing more comes out later.
		h.advance(6000);
		h.flushStale();
		QCOMPARE(h.commits.size(), size_t(1));
		// And a name that was suppressed is ordinary again at once.
		h.record("AP_MASTER");
		h.advance(500);
		h.flushStale();
		QCOMPARE(h.count("AP_MASTER"), 1);
	}

	void seqsAreUniqueAndIncreasingAcrossNames() {
		Harness h;
		h.record("FLAPS_INCR");
		h.record("GEAR_UP");
		h.record("PARKING_BRAKES");
		h.advance(500);
		h.flushStale();
		h.filter.flush_all(h.out);
		h.record("FLAPS_DECR");
		QCOMPARE(h.commits.size(), size_t(4));
		for (size_t i = 0; i < h.commits.size(); ++i)
			QCOMPARE(h.commits[i].seq, (unsigned long long)(i + 1));
	}
};

QTEST_GUILESS_MAIN(TstEventFilter)
#include "tst_event_filter.moc"
