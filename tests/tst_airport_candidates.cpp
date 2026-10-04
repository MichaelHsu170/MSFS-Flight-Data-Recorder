// Nearest-airport candidates (add_nearest_airports() in airport_lookup.cpp)
// on their own: made-up AIRPORT_LIST entries in, the nearest-first
// candidate list out.
#include "airport_lookup.h"

#include <QtTest>

#include <cstring>
#include <string>
#include <vector>

namespace {

using Candidates = AIRPORT_LOOKUP::CANDIDATE[AIRPORT_LOOKUP::TOP_N];

// An airport ident at latitude 45 + northKm/111.2 (about northKm north of
// the reference point), same longitude.
SIMCONNECT_DATA_FACILITY_AIRPORT airport(const char* ident, double northKm, const char* region = "K1") {
	SIMCONNECT_DATA_FACILITY_AIRPORT a = {};
	strncpy(a.Ident, ident, sizeof(a.Ident) - 1);
	strncpy(a.Region, region, sizeof(a.Region) - 1);
	a.Latitude = 45.0 + northKm / 111.2;
	a.Longitude = 7.0;
	return a;
}

COORDINATE reference() {
	COORDINATE c;
	c.latitude = 45.0;
	c.longitude = 7.0;
	return c;
}

void add(Candidates& top, const std::vector<SIMCONNECT_DATA_FACILITY_AIRPORT>& list) {
	add_nearest_airports(top, reference(), list.data(), (int)list.size());
}

std::vector<std::string> idents(const Candidates& top) {
	std::vector<std::string> out;
	for (const AIRPORT_LOOKUP::CANDIDATE& c : top)
		if (c.ident[0] != '\0')
			out.push_back(c.ident);
	return out;
}

}

class TstAirportCandidates : public QObject {
	Q_OBJECT

private slots:
	void emptyListAddsNothing() {
		Candidates top;
		add_nearest_airports(top, reference(), nullptr, 0);
		QVERIFY(idents(top).empty());
	}

	void keepsTheNearestFiveInOrder() {
		Candidates top;
		add(top, { airport("AAAA", 30), airport("BBBB", 5), airport("CCCC", 50), airport("DDDD", 1),
			airport("EEEE", 20), airport("FFFF", 10), airport("GGGG", 40) });
		QCOMPARE(idents(top), (std::vector<std::string>{ "DDDD", "BBBB", "FFFF", "EEEE", "AAAA" }));
		for (int i = 1; i < AIRPORT_LOOKUP::TOP_N; ++i)
			QVERIFY(top[i - 1].distance <= top[i].distance);
		QVERIFY(qAbs(top[0].distance - 1.0) < 0.05);
		QVERIFY(qAbs(top[4].distance - 30.0) < 0.5);
	}

	void onlyFourLetterIdentsCount() {
		Candidates top;
		add(top, { airport("VOLC2", 0.1), airport("H1", 0.2), airport("ABC", 0.3), airport("KSEA", 9) });
		QCOMPARE(idents(top), std::vector<std::string>{ "KSEA" });
	}

	void carriesRegion() {
		Candidates top;
		add(top, { airport("LSZH", 3, "LS") });
		QCOMPARE(std::string(top[0].region), std::string("LS"));
	}

	void accumulatesAcrossChunks() {
		// A long list arrives in several AIRPORT_LIST chunks: the nearest
		// overall win, whichever chunk they were in.
		Candidates top;
		add(top, { airport("FAR1", 80), airport("FAR2", 90), airport("FAR3", 70) });
		add(top, { airport("NEA1", 2), airport("FAR4", 95) });
		add(top, { airport("NEA2", 4), airport("NEA3", 3), airport("FAR5", 99) });
		QCOMPARE(idents(top), (std::vector<std::string>{ "NEA1", "NEA3", "NEA2", "FAR3", "FAR1" }));
	}

	void fartherThanAFullListIsIgnored() {
		Candidates top;
		add(top, { airport("AAAA", 1), airport("BBBB", 2), airport("CCCC", 3), airport("DDDD", 4), airport("EEEE", 5) });
		add(top, { airport("ZZZZ", 6) });
		QCOMPARE(idents(top), (std::vector<std::string>{ "AAAA", "BBBB", "CCCC", "DDDD", "EEEE" }));
	}

	void southIsAsNearAsNorth() {
		Candidates top;
		add(top, { airport("NRTH", 10), airport("SUTH", -5) });
		QCOMPARE(idents(top), (std::vector<std::string>{ "SUTH", "NRTH" }));
		QVERIFY(top[0].distance > 0);
	}
};

QTEST_APPLESS_MAIN(TstAirportCandidates)
#include "tst_airport_candidates.moc"
