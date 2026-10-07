// Engine power (engine_power.cpp): which recorded value each engine type
// shows as its speed/load, the engine count, and which engines' combustion
// counts.
#include "engine_power.h"
#include "simconnect_defs.h"

#include <QtTest>

#include <cstring>
#include <limits>

namespace {

// A zeroed record of an aircraft with this many engines.
FLIGHT_DATA_RECORD record(double engines) {
	FLIGHT_DATA_RECORD r;
	std::memset(static_cast<void*>(&r), 0, sizeof(r));
	r.number_of_engines = engines;
	return r;
}

}

class TstEnginePower : public QObject {
	Q_OBJECT

private slots:
	void eachEngineTypeShowsItsOwnValues_data() {
		QTest::addColumn<int>("engineType");
		QTest::addColumn<int>("speedField");
		QTest::addColumn<int>("loadField");
		QTest::addColumn<QString>("speedLabel");
		QTest::addColumn<QString>("loadUnit");
		QTest::newRow("piston") << 0 << (int)TRIP_ENGINE_general_eng_rpm << (int)TRIP_ENGINE_recip_eng_manifold_pressure << "RPM" << "inHg";
		QTest::newRow("jet") << 1 << (int)TRIP_ENGINE_turb_eng_n1 << (int)TRIP_ENGINE_turb_eng_n2 << "N1" << "%";
		QTest::newRow("helo turbine") << 3 << (int)TRIP_ENGINE_turb_eng_n1 << (int)TRIP_ENGINE_turb_eng_max_torque_percent << "N1" << "%";
		QTest::newRow("turboprop") << 5 << (int)TRIP_ENGINE_prop_rpm << (int)TRIP_ENGINE_turb_eng_max_torque_percent << "Prop RPM" << "%";
	}
	void eachEngineTypeShowsItsOwnValues() {
		QFETCH(int, engineType);
		QFETCH(int, speedField);
		QFETCH(int, loadField);
		QFETCH(QString, speedLabel);
		QFETCH(QString, loadUnit);
		const EnginePowerSpec* spec = enginePowerSpec(engineType);
		QVERIFY(spec);
		QCOMPARE((int)spec->speed.field, speedField);
		QCOMPARE((int)spec->load.field, loadField);
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

	void otherEngineTypesShowNothing() {
		// None (2), unsupported (4), electric (6), and no type at all.
		for (int type : { 2, 4, 6, -1, 7 })
			QVERIFY(!enginePowerSpec(type));
	}

	void engineCountIsClampedToTheEngineIndexes() {
		QCOMPARE(engineCount(record(3)), 3);
		QCOMPARE(engineCount(record(16)), 16);
		QCOMPARE(engineCount(record(17)), 16);
		QCOMPARE(engineCount(record(-1)), 0);
		QCOMPARE(engineCount(record(std::numeric_limits<double>::quiet_NaN())), 0);
		QCOMPARE(engineCount(record(1e300)), 0);
	}

	void onlyTheAircraftsEnginesCountAsCombusting() {
		FLIGHT_DATA_RECORD r = record(6);
		QVERIFY(!anyEngineCombusting(r));
		r.eng_combustion[5] = 1;
		QVERIFY(anyEngineCombusting(r));
		r.number_of_engines = 5; // engine 6 isn't one of its engines
		QVERIFY(!anyEngineCombusting(r));
		r.eng_combustion[4] = 1;
		QVERIFY(anyEngineCombusting(r));
		r = record(2);
		r.eng_combustion[0] = 1;
		QVERIFY(anyEngineCombusting(r));
		r.number_of_engines = 0;
		QVERIFY(!anyEngineCombusting(r));
		// MSFS's last engine index counts.
		r = record(16);
		r.eng_combustion[15] = 1;
		QVERIFY(anyEngineCombusting(r));
	}
};

QTEST_GUILESS_MAIN(TstEnginePower)
#include "tst_engine_power.moc"
