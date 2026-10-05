// Small shared helpers: trip_dataset.h (time parsing, file-name pieces,
// decimation) and trip_data_fields.h (field labels, field lists, bool bit
// packing).
#include "simconnect_defs.h"
#include "trip_data_fields.h"
#include "trip_dataset.h"

#include <QtTest>

#include <algorithm>

class TstTripDataset : public QObject {
	Q_OBJECT

private slots:
	void parsesRecorderTimestamps() {
		const QDateTime t = parseZuluTime("2026-01-02T10:00:00.500+00:00_5");
		QVERIFY(t.isValid());
		QCOMPARE(t.toUTC().toString(Qt::ISODateWithMs), QStringLiteral("2026-01-02T10:00:00.500Z"));
		const QDateTime offset = parseZuluTime("2026-01-02T12:30:00.000+02:00_5");
		QCOMPARE(offset.toUTC().toString(Qt::ISODateWithMs), QStringLiteral("2026-01-02T10:30:00.000Z"));
		QVERIFY(!parseZuluTime("").isValid());
		QVERIFY(!parseZuluTime("not a time").isValid());
	}

	void airportPairNames() {
		QCOMPARE(airportPairName("AAAA", "BBBB", "x"), QStringLiteral("AAAA-BBBB"));
		QCOMPARE(airportPairName("AAAA", "", "x"), QStringLiteral("AAAA"));
		QCOMPARE(airportPairName("", "BBBB", "x"), QStringLiteral("BBBB"));
		QCOMPARE(airportPairName("", "", "fallback"), QStringLiteral("fallback"));
	}

	void airportLabels() {
		QCOMPARE(airportLabel("EGLL", "Heathrow"), QStringLiteral("EGLL (Heathrow)"));
		QCOMPARE(airportLabel("EGLL", ""), QStringLiteral("EGLL"));
		QCOMPARE(airportLabel("", "Heathrow"), QString());
	}

	void airportPairFromFirstLiftoffAndLastTouchdown() {
		LiftoffPoint lo1, lo2;
		lo1.icao = "AAAA";
		lo2.icao = "CCCC";
		TouchdownPoint td1, td2;
		td1.icao = "DDDD";
		td2.icao = "BBBB";
		QCOMPARE(airportPairName(std::vector<LiftoffPoint>{ lo1, lo2 }, std::vector<TouchdownPoint>{ td1, td2 }, "x"), QStringLiteral("AAAA-BBBB"));
		QCOMPARE(airportPairName(std::vector<LiftoffPoint>{}, std::vector<TouchdownPoint>{}, "Trip 3"), QStringLiteral("Trip 3"));
	}

	void departureTimestampSuffix() {
		QCOMPARE(appendDepartureTimestamp("AAAA-BBBB", "2026-01-02T10:00:00.500+00:00_5"), QStringLiteral("AAAA-BBBB_20260102100000"));
		QCOMPARE(appendDepartureTimestamp("trip", ""), QStringLiteral("trip"));
	}

	void fieldLabels() {
		QCOMPARE(tripFieldLabel("plane_touchdown_latitude"), QStringLiteral("Plane Touchdown Latitude"));
		QCOMPARE(tripFieldLabel("turb_eng_ignition_switch_ex1_1"), QStringLiteral("Turb Eng Ignition Switch Ex1 1"));
		QCOMPARE(tripFieldLabel("g_force"), QStringLiteral("G Force"));
	}

	void fieldListsHaveNoDuplicates() {
		QSet<QString> names;
		int count = 0;
#define ADD_NUM(dbColumn, memberExpr, sqlType) names.insert(QStringLiteral(#dbColumn)); ++count;
		TRIP_DATA_NUM_FIELDS(ADD_NUM)
#undef ADD_NUM
#define ADD_BOOL(name, group, bit) names.insert(QStringLiteral(#name)); ++count;
		TRIP_DATA_BOOL_FIELDS(ADD_BOOL)
#undef ADD_BOOL
		QCOMPARE(names.size(), count);
		QCOMPARE(count, 136 + 96);
	}

	void boolFieldsUseEachBitOnce() {
		QSet<int> usedBits;
		int count = 0;
#define ADD_BIT(name, group, bit) \
		QVERIFY((group) >= 1 && (group) <= 3 && (bit) >= 0 && (bit) <= 31); \
		usedBits.insert((group) * 32 + (bit)); ++count;
		TRIP_DATA_BOOL_FIELDS(ADD_BIT)
#undef ADD_BIT
		QCOMPARE(usedBits.size(), count);
	}

	// tripBoolGroups()'s packing formula against independently hand-computed
	// hex literals (field -> group/bit from TRIP_DATA_BOOL_FIELDS above), not
	// a recomputation of the shift-and-OR it performs.
	void tripBoolGroupsPacksFieldsIntoExpectedBits() {
		FLIGHT_DATA_RECORD r{};
		r.autopilot_airspeed_hold = 1; // group 1, bit 0
		r.autopilot_master = 1;        // group 1, bit 18
		r.aileron_trim_disabled = 1;   // group 1, bit 30
		r.flap_damage_by_speed = 1;    // group 2, bit 0
		r.eng_failed_2 = 1;            // group 2, bit 20
		r.sim_on_ground = 1;           // group 3, bit 30
		r.kohlsman_setting_std = 1;    // group 3, bit 31

		const std::array<uint32_t, 4> groups = tripBoolGroups(r);
		QCOMPARE(groups[0], 0u);
		QCOMPARE(groups[1], 0x40040001u);
		QCOMPARE(groups[2], 0x00100001u);
		QCOMPARE(groups[3], 0xC0000000u);
	}

	void decimatedIndicesKeepEverySampleWithinBudget() {
		QCOMPARE(decimatedIndices(0, 4, 10), (std::vector<int>{ 0, 1, 2, 3, 4 }));
		QCOMPARE(decimatedIndices(3, 5, 3), (std::vector<int>{ 3, 4, 5 }));
		QCOMPARE(decimatedIndices(7, 7, 10), (std::vector<int>{ 7 }));
		QVERIFY(decimatedIndices(0, -1, 10).empty());
		QVERIFY(decimatedIndices(5, 4, 10).empty());
	}

	void decimatedIndicesThinByStrideAndKeepTheLastSample() {
		// 10 samples, budget 4: stride 3.
		QCOMPARE(decimatedIndices(0, 9, 4), (std::vector<int>{ 0, 3, 6, 9 }));
		// 11 samples: stride 3, and the last (10) isn't on the stride.
		QCOMPARE(decimatedIndices(0, 10, 4), (std::vector<int>{ 0, 3, 6, 9, 10 }));
		// A slice starts at lo.
		QCOMPARE(decimatedIndices(100, 110, 4), (std::vector<int>{ 100, 103, 106, 109, 110 }));
		// A long trip stays within the budget: 60001 samples need stride 21,
		// giving 2858 plus the last.
		const std::vector<int> many = decimatedIndices(0, 60000, 3000);
		QCOMPARE(many.size(), size_t(2859));
		QCOMPARE(many.front(), 0);
		QCOMPARE(many.back(), 60000);
		QVERIFY(std::is_sorted(many.begin(), many.end()));
	}
};

QTEST_APPLESS_MAIN(TstTripDataset)
#include "tst_trip_dataset.moc"
