// Engine power (engine_power.cpp): which SimVars each engine type records as
// its speed/load, the engine count, which engines' combustion counts, and
// reading the stored values back.
#include "engine_power.h"
#include "simconnect_defs.h"

#include <QtTest>

#include <cstring>
#include <limits>

namespace {

// A record with every engine SimVar of engine i set to a distinct value:
// RPM 1000+i, MP 20+i, N1 50+i, N2 60+i, torque 70+i, prop RPM 2000+i.
FLIGHT_DATA_RECORD record(double engineType, double engines) {
	FLIGHT_DATA_RECORD r;
	std::memset(static_cast<void*>(&r), 0, sizeof(r));
	r.engine_type = engineType;
	r.number_of_engines = engines;
	for (int i = 0; i < MAX_ENGINES; ++i) {
		r.general_eng_rpm[i] = 1000 + i;
		r.recip_eng_manifold_pressure[i] = 20 + i;
		r.turb_eng_n1[i] = 50 + i;
		r.turb_eng_n2[i] = 60 + i;
		r.turb_eng_max_torque_percent[i] = 70 + i;
		r.prop_rpm[i] = 2000 + i;
	}
	return r;
}

}

class TstEnginePower : public QObject {
	Q_OBJECT

private slots:
	void eachEngineTypeRecordsItsOwnSimVars_data() {
		QTest::addColumn<int>("engineType");
		QTest::addColumn<float>("speed1");
		QTest::addColumn<float>("load2");
		QTest::addColumn<QString>("speedLabel");
		QTest::addColumn<QString>("loadUnit");
		QTest::newRow("piston") << 0 << 1000.0f << 21.0f << "RPM" << "inHg";
		QTest::newRow("jet") << 1 << 50.0f << 61.0f << "N1" << "%";
		QTest::newRow("helo turbine") << 3 << 50.0f << 71.0f << "N1" << "%";
		QTest::newRow("turboprop") << 5 << 2000.0f << 71.0f << "Prop RPM" << "%";
	}
	void eachEngineTypeRecordsItsOwnSimVars() {
		QFETCH(int, engineType);
		QFETCH(float, speed1);
		QFETCH(float, load2);
		QFETCH(QString, speedLabel);
		QFETCH(QString, loadUnit);
		const EnginePower power = enginePowerFromRecord(record(engineType, 2));
		QCOMPARE(power.engineType, engineType);
		QCOMPARE(power.count, 2);
		QCOMPARE(power.speed[0], speed1);
		QCOMPARE(power.speed[1], speed1 + 1);
		QCOMPARE(power.load[0], load2 - 1);
		QCOMPARE(power.load[1], load2);
		// Engines past the count are left 0.
		QCOMPARE(power.speed[2], 0.0f);
		QCOMPARE(power.load[2], 0.0f);
		const EnginePowerSpec* spec = enginePowerSpec(engineType);
		QVERIFY(spec);
		QCOMPARE(QString::fromUtf8(spec->speed.label), speedLabel);
		QCOMPARE(QString::fromUtf8(spec->load.unit), loadUnit);
	}

	void onlyJetN1AndN2HaveAFixedAxis() {
		QCOMPARE(enginePowerSpec(1)->speed.axisMax, 110.0);
		QCOMPARE(enginePowerSpec(1)->load.axisMax, 110.0);
		QCOMPARE(enginePowerSpec(3)->speed.axisMax, 110.0);
		QCOMPARE(enginePowerSpec(3)->load.axisMax, 0.0);
		QCOMPARE(enginePowerSpec(0)->speed.axisMax, 0.0);
		QCOMPARE(enginePowerSpec(5)->load.axisMax, 0.0);
	}

	void otherEngineTypesRecordNothing() {
		// None (2), unsupported (4) and electric (6) report none of these SimVars.
		for (int type : { 2, 4, 6, -1, 7 }) {
			QVERIFY(!enginePowerSpec(type));
			const EnginePower power = enginePowerFromRecord(record(type, 2));
			QCOMPARE(power.engineType, type);
			QCOMPARE(power.count, 0);
			QCOMPARE(power.speed[0], 0.0f);
		}
	}

	void anEngineTypeNoIntHoldsReadsAsUnknown() {
		for (double type : { std::numeric_limits<double>::quiet_NaN(), 1e300, -1e300 }) {
			const EnginePower power = enginePowerFromRecord(record(type, 2));
			QCOMPARE(power.engineType, -1);
			QCOMPARE(power.count, 0);
		}
	}

	void theEngineCountIsClampedToTheRecordedEngines() {
		QCOMPARE(enginePowerFromRecord(record(1, 4)).count, 4);
		QCOMPARE(enginePowerFromRecord(record(1, 6)).count, 4);
		QCOMPARE(enginePowerFromRecord(record(1, 0)).count, 0);
		QCOMPARE(enginePowerFromRecord(record(1, -1)).count, 0);
		QCOMPARE(enginePowerFromRecord(record(1, 6)).speed[3], 53.0f);
	}

	void engineCountIsClampedToTheRecordedEngines() {
		QCOMPARE(engineCount(record(2, 3)), 3);
		QCOMPARE(engineCount(record(2, 6)), 4);
		QCOMPARE(engineCount(record(2, -1)), 0);
		QCOMPARE(engineCount(record(2, std::numeric_limits<double>::quiet_NaN())), 0);
		QCOMPARE(engineCount(record(2, 1e300)), 0);
	}

	void onlyTheAircraftsEnginesCountAsCombusting() {
		FLIGHT_DATA_RECORD r = record(1, 4);
		QVERIFY(!anyEngineCombusting(r));
		r.eng_combustion_4 = 1;
		QVERIFY(anyEngineCombusting(r));
		r.number_of_engines = 3; // engine 4 isn't one of its engines
		QVERIFY(!anyEngineCombusting(r));
		r.eng_combustion_3 = 1;
		QVERIFY(anyEngineCombusting(r));
		r = record(1, 2);
		r.eng_combustion_1 = 1;
		QVERIFY(anyEngineCombusting(r));
		r.number_of_engines = 0;
		QVERIFY(!anyEngineCombusting(r));
	}

	void packWritesTheRecordedEnginesAsLittleEndianFloats() {
		const std::array<float, MAX_ENGINES> values{ 85.5f, 90.25f, 1, 2 };
		// 85.5 = 42AB0000, 90.25 = 42B48000, low byte first.
		QCOMPARE(QByteArray(packEngineValues(values, 2).data(), 8).toHex(), QByteArray("0000ab420080b442"));
		QCOMPARE(packEngineValues(values, 2).size(), size_t(8));
		QCOMPARE(packEngineValues(values, 9).size(), size_t(16)); // clamped to MAX_ENGINES
		QVERIFY(packEngineValues(values, 0).empty());      // stored as NULL
		QVERIFY(packEngineValues(values, -1).empty());

		std::array<float, MAX_ENGINES> out{};
		const std::string_view blob = packEngineValues(values, 3);
		QCOMPARE(unpackEngineValues(blob.data(), (int)blob.size(), out), 3);
		QCOMPARE(out, (std::array<float, MAX_ENGINES>{ 85.5f, 90.25f, 1, 0 }));
	}

	void unpackReadsUpToFourFloats() {
		const float stored[] = { 85.5f, 90.25f, 1, 2, 3 };
		// Engines past the count are cleared, not left from the previous row.
		std::array<float, MAX_ENGINES> out{ 9, 9, 9, 9 };
		QCOMPARE(unpackEngineValues(stored, 2 * sizeof(float), out), 2);
		QCOMPARE(out, (std::array<float, MAX_ENGINES>{ 85.5f, 90.25f, 0, 0 }));
		// A partial trailing float is ignored; more than 4 are cut off.
		out = { 9, 9, 9, 9 };
		QCOMPARE(unpackEngineValues(stored, 6, out), 1);
		QCOMPARE(out[1], 0.0f);
		QCOMPARE(unpackEngineValues(stored, sizeof(stored), out), 4);
		QCOMPARE(out, (std::array<float, MAX_ENGINES>{ 85.5f, 90.25f, 1, 2 }));
	}

	void unpackOfNothingReadsNoEngines() {
		const std::array<float, MAX_ENGINES> stale{ 7, 7, 7, 7 };
		const std::array<float, MAX_ENGINES> zeros{};
		std::array<float, MAX_ENGINES> out = stale;
		QCOMPARE(unpackEngineValues(nullptr, 8, out), 0);  // NULL: not recorded
		QCOMPARE(out, zeros);
		const float stored[] = { 1 };
		out = stale;
		QCOMPARE(unpackEngineValues(stored, 0, out), 0);
		QCOMPARE(out, zeros);
		out = stale;
		QCOMPARE(unpackEngineValues(stored, -4, out), 0);
		QCOMPARE(out, zeros);
	}
};

QTEST_GUILESS_MAIN(TstEnginePower)
#include "tst_engine_power.moc"
