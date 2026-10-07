// Engine power (engine_power.cpp): which recorded values each engine type
// shows as its N1/N2, the engine count, and which engines' combustion
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
	void eachEngineTypeShowsItsN1AndN2Equivalents_data() {
		QTest::addColumn<int>("engineType");
		QTest::addColumn<int>("n1Field");
		QTest::addColumn<int>("n2Field");
		QTest::addColumn<QString>("n1Label");
		QTest::addColumn<QString>("n2Label");
		QTest::addColumn<QString>("unit");
		QTest::addColumn<int>("decimals");
		QTest::addColumn<QString>("axisTitle");
		QTest::addColumn<double>("axisMax");
		QTest::newRow("piston") << 0 << (int)TRIP_ENGINE_general_eng_rpm << (int)TRIP_ENGINE_prop_rpm
			<< "RPM" << "Prop RPM" << "rpm" << 0 << "RPM" << 0.0;
		for (const auto& [name, type] : { std::pair{ "jet", 1 }, std::pair{ "helo turbine", 3 }, std::pair{ "turboprop", 5 } })
			QTest::newRow(name) << type << (int)TRIP_ENGINE_turb_eng_n1 << (int)TRIP_ENGINE_turb_eng_n2
				<< "N1" << "N2" << "%" << 1 << "N1 / N2 (%)" << 110.0;
	}
	void eachEngineTypeShowsItsN1AndN2Equivalents() {
		QFETCH(int, engineType);
		QFETCH(int, n1Field);
		QFETCH(int, n2Field);
		QFETCH(QString, n1Label);
		QFETCH(QString, n2Label);
		QFETCH(QString, unit);
		QFETCH(int, decimals);
		QFETCH(QString, axisTitle);
		QFETCH(double, axisMax);
		const EnginePowerSpec* spec = enginePowerSpec(engineType);
		QVERIFY(spec);
		QCOMPARE((int)spec->n1Field, n1Field);
		QCOMPARE((int)spec->n2Field, n2Field);
		QCOMPARE(QString::fromUtf8(spec->n1Label), n1Label);
		QCOMPARE(QString::fromUtf8(spec->n2Label), n2Label);
		QCOMPARE(QString::fromUtf8(spec->unit), unit);
		QCOMPARE(spec->decimals, decimals);
		QCOMPARE(QString::fromUtf8(spec->axisTitle), axisTitle);
		QCOMPARE(spec->axisMax, axisMax);
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
