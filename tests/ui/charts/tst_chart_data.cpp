// Chart data module (chart_data.cpp) on its own: made-up samples in, series
// points, axis ranges and hover values out.
#include "chart_data.h"
#include "local_time_zone.h"

#include <QFile>
#include <QRegularExpression>
#include <QTimeZone>
#include <QtTest>

#include <cmath>
#include <set>

namespace {

QString zulu(int hour, int minute, int second, int ms = 0) {
	return QStringLiteral("2026-03-04T%1:%2:%3.%4+00:00_3")
		.arg(hour, 2, 10, QLatin1Char('0')).arg(minute, 2, 10, QLatin1Char('0'))
		.arg(second, 2, 10, QLatin1Char('0')).arg(ms, 3, 10, QLatin1Char('0'));
}

// What chartTimeMs() should give for zulu(): that UTC instant in epoch ms.
double utcMs(int hour, int minute, int second, int ms = 0) {
	return (double)QDateTime(QDate(2026, 3, 4), QTime(hour, minute, second, ms), QTimeZone::UTC).toMSecsSinceEpoch();
}

// A sample of an aircraft of engineType with engines 1..engines, every
// engine value not recorded (NaN) until set().
struct EnginePoint {
	TripSamplePoint p;
	EnginePoint(int engineType, int engines) {
		p.engineType = engineType;
		p.engineValues.assign((size_t)engines * TRIP_ENGINE_FIELD_COUNT, std::nan(""));
	}
	EnginePoint& set(int engine, TripEngineField field, double value) {
		p.engineValues[(size_t)(engine - 1) * TRIP_ENGINE_FIELD_COUNT + field] = value;
		return *this;
	}
};

ChartValues valuesWith(double vs, double ias, double gs, double alt, double fuel, double pitch, double bank) {
	ChartValues v{};
	v[CHART_VERTICAL_SPEED] = vs;
	v[CHART_AIRSPEED] = ias;
	v[CHART_GROUND_SPEED] = gs;
	v[CHART_ALTITUDE] = alt;
	v[CHART_FUEL_WEIGHT] = fuel;
	v[CHART_PITCH] = pitch;
	v[CHART_BANK] = bank;
	return v;
}

// Every series' value for sample i is i * 10 + the series id, so a mix-up
// between samples or series shows.
ChartSample sample(const QString& time, int i) {
	ChartSample s;
	s.zuluTime = time;
	for (int k = 0; k < CHART_SERIES_COUNT; ++k)
		s.values[k] = i * 10 + k;
	return s;
}

}

class TstChartData : public QObject {
	Q_OBJECT

private slots:
	// Every test runs in a local time zone with DST changes, so none can rely
	// on local time matching zulu time or on a day without a skipped hour.
	void initTestCase() { TestSupport::usePacificLocalTime(); }

	void seriesTableMatchesTheQml() {
		QFile qml(QStringLiteral(CHARTS_QML));
		QVERIFY(qml.open(QIODevice::ReadOnly));
		const QString text = QString::fromUtf8(qml.readAll());
		std::set<std::string> names, keys;
		for (const ChartSeriesDef& def : CHART_SERIES) {
			QVERIFY2(text.contains(QStringLiteral("objectName: \"%1\"").arg(def.objectName)), def.objectName);
			// A series entry's key, or an element of engineSeries()' speed/load
			// key arrays -- not just the quoted name anywhere in the file.
			const QString key = QRegularExpression::escape(QString::fromLatin1(def.valueKey));
			const QRegularExpression keyed(QStringLiteral("key:\\s*\"%1\"|(speed|load):\\s*\\[[^\\]]*\"%1\"").arg(key));
			QVERIFY2(text.contains(keyed), def.valueKey);
			names.insert(def.objectName);
			keys.insert(def.valueKey);
		}
		QCOMPARE(names.size(), size_t(CHART_SERIES_COUNT));
		QCOMPARE(keys.size(), size_t(CHART_SERIES_COUNT));
		// ...and no LineSeries in the QML is missing from the table.
		QCOMPARE((int)text.count(QRegularExpression(QStringLiteral("LineSeries\\s*\\{"))), (int)CHART_SERIES_COUNT);
		// Only the on-ground series are flags.
		for (int s = 0; s < CHART_SERIES_COUNT; ++s)
			QCOMPARE(CHART_SERIES[s].isFlag, s >= CHART_GEAR_ON_GROUND_0 && s <= CHART_GEAR_ON_GROUND_2);
	}

	void valuesComeFromTheirSampleFields() {
		// A 5-engine jet: N1/N2 of engines 1-4 (5 has no series), 0 for engine
		// 3's N2, which wasn't recorded. Prop RPM is a turboprop's, not shown.
		EnginePoint jet(1, 5);
		for (int e = 1; e <= 5; ++e)
			jet.set(e, TRIP_ENGINE_turb_eng_n1, e).set(e, TRIP_ENGINE_turb_eng_n2, 4 + e).set(e, TRIP_ENGINE_prop_rpm, 99);
		jet.set(3, TRIP_ENGINE_turb_eng_n2, std::nan(""));
		TripSamplePoint& p = jet.p;
		p.verticalSpeed = -5; p.airspeed = 6; p.groundSpeed = 7; p.altitude = 8;
		p.gearHandlePosition = 9;
		p.gearPosition[0] = 10; p.gearPosition[1] = 11; p.gearPosition[2] = 12;
		p.gearOnGround[0] = true; p.gearOnGround[1] = false; p.gearOnGround[2] = true;
		p.brakeIndicator = 13; p.flapsHandleIndex = 14; p.spoilersHandlePosition = 15;
		p.fuelTotalQuantityWeight = 16; p.pitchDegrees = -17.5; p.bankDegrees = 18.25;
		const ChartValues v = chartValues(p);
		const ChartValues expected = { 1, 2, 3, 4, 5, 6, 0, 8, -5, 6, 7, 8, 9, 10, 11, 12, 1, 0, 1, 13, 14, 15, 16, -17.5, 18.25 };
		for (int s = 0; s < CHART_SERIES_COUNT; ++s)
			QCOMPARE(v[s], expected[s]);
	}

	void engineValuesAreTheOnesTheEngineTypeShows() {
		// A turboprop shows prop RPM and torque; an engine past the sample's
		// own engines is 0.
		EnginePoint turboprop(5, 1);
		turboprop.set(1, TRIP_ENGINE_prop_rpm, 2100).set(1, TRIP_ENGINE_turb_eng_max_torque_percent, 25.5)
			.set(1, TRIP_ENGINE_turb_eng_n1, 99);
		ChartValues v = chartValues(turboprop.p);
		QCOMPARE(v[CHART_ENG_SPEED_1], 2100.0);
		QCOMPARE(v[CHART_ENG_LOAD_1], 25.5);
		QCOMPARE(v[CHART_ENG_SPEED_2], 0.0);
		// An engine type that shows no power: every engine series is 0.
		EnginePoint none(2, 2);
		none.set(1, TRIP_ENGINE_turb_eng_n1, 50).set(1, TRIP_ENGINE_general_eng_rpm, 50);
		v = chartValues(none.p);
		for (int s = CHART_ENG_SPEED_1; s < CHART_VERTICAL_SPEED; ++s)
			QCOMPARE(v[s], 0.0);
	}

	void chartTimeIsTheUtcInstant() {
		QCOMPARE(chartTimeMs(zulu(10, 20, 30, 450)), 1772619630450.0);
		QCOMPARE(chartTimeMs(zulu(23, 59, 59, 999)), 1772668799999.0);
	}

	// US Pacific time skips 02:00-03:00 local on 2026-03-08. Read as local
	// time, zulu times in that hour would be shifted an hour and the hover
	// time with them; as UTC instants they keep their order and their time.
	void chartTimesIgnoreTheLocalDstChange() {
		QVERIFY(QDateTime(QDate(2026, 3, 8), QTime(2, 30)).time() != QTime(2, 30)); // the zone is in effect
		QCOMPARE(chartTimeMs(QStringLiteral("2026-03-08T01:59:00.000+00:00_0")), 1772935140000.0);
		QCOMPARE(chartTimeMs(QStringLiteral("2026-03-08T02:30:00.000+00:00_0")), 1772937000000.0);
		QCOMPARE(chartTimeMs(QStringLiteral("2026-03-08T03:00:00.000+00:00_0")), 1772938800000.0);
		QCOMPARE(chartValueMap(1772937000000.0, ChartValues{}).value("timeStr").toString(),
			QStringLiteral("02:30:00.000 UTC"));
	}

	void malformedTimeIsNaN() {
		QVERIFY(qIsNaN(chartTimeMs(QString())));
		QVERIFY(qIsNaN(chartTimeMs(QStringLiteral("2026-13-04T10:20:30.000+00:00"))));
		QVERIFY(qIsNaN(chartTimeMs(QStringLiteral("2026-03-04T25:20:30.000+00:00"))));
		QVERIFY(qIsNaN(chartTimeMs(QStringLiteral("garbage garbage garbage!"))));
	}

	void niceAxisMaxHasHeadroomAndRoundSteps() {
		QCOMPARE(niceAxisMax(0), 1.0);
		QCOMPARE(niceAxisMax(-50), 1.0);
		QCOMPARE(niceAxisMax(80), 100.0);    // 100 exactly, step 20
		QCOMPARE(niceAxisMax(100), 150.0);   // 125 -> step 50
		QCOMPARE(niceAxisMax(250), 400.0);   // 312.5 -> step 100
		QCOMPARE(niceAxisMax(38000), 50000.0);
	}

	void niceSignedAxisRangeHasMarginsAndRoundSteps() {
		QCOMPARE(niceSignedAxisRange(-500, 1500), std::make_pair(-1000.0, 2000.0));
		QCOMPARE(niceSignedAxisRange(-2000, 0), std::make_pair(-3000.0, 1000.0));
		// A flat or narrow range is widened to 2 around its middle.
		QCOMPARE(niceSignedAxisRange(0, 0), std::make_pair(-2.0, 2.0));
		QCOMPARE(niceSignedAxisRange(10, 10.5), std::make_pair(8.0, 12.0));
	}

	void extentsTrackMinMax() {
		ChartExtents e;
		QVERIFY(!e.valid);
		e.add(valuesWith(-300, 120, 130, 5000, 900, 3, -10));
		QVERIFY(e.valid);
		QCOMPARE(e.vsMin, -300.0); QCOMPARE(e.vsMax, -300.0);
		QCOMPARE(e.speedMax, 130.0);   // the higher of airspeed and ground speed
		QCOMPARE(e.altMax, 5000.0);
		QCOMPARE(e.fuelMax, 900.0);
		QCOMPARE(e.pitchMin, 3.0); QCOMPARE(e.pitchMax, 3.0);
		QCOMPARE(e.bankMin, -10.0); QCOMPARE(e.bankMax, -10.0);
		e.add(valuesWith(700, 100, 100, 4000, 800, 3, -10));
		QCOMPARE(e.vsMin, -300.0); QCOMPARE(e.vsMax, 700.0);
		QCOMPARE(e.altMax, 5000.0);   // a lower value keeps the max
		e.add(valuesWith(0, 0, 0, 0, 0, -5, 25));
		QCOMPARE(e.pitchMin, -5.0); QCOMPARE(e.bankMax, 25.0);
		QCOMPARE(e.speedMax, 130.0);
	}

	void engineExtentsAreTheMaxOfAnyEngine() {
		ChartExtents e;
		ChartValues v{};
		v[CHART_ENG_SPEED_1] = 50; v[CHART_ENG_SPEED_4] = 80;
		v[CHART_ENG_LOAD_2] = 30; v[CHART_ENG_LOAD_3] = 20;
		e.add(v);
		QCOMPARE(e.engSpeedMax, 80.0);
		QCOMPARE(e.engLoadMax, 30.0);
		v[CHART_ENG_LOAD_4] = 40;
		e.add(v);
		QCOMPARE(e.engLoadMax, 40.0);
		v[CHART_ENG_SPEED_2] = 90;
		e.add(v);
		QCOMPARE(e.engSpeedMax, 90.0);
	}

	void chartEngineIsTheFirstPointWithPower() {
		// No engines; an engine type that shows no power; engine 1's speed
		// not recorded (a migrated trip's other quantity) -- none count.
		const TripSamplePoint noEngines = EnginePoint(1, 0).p;
		const TripSamplePoint noPower = EnginePoint(2, 2).set(1, TRIP_ENGINE_turb_eng_n1, 50).p;
		const TripSamplePoint notRecorded = EnginePoint(1, 2).set(1, TRIP_ENGINE_turb_eng_n2, 50).p;
		QCOMPARE(chartEngine({}).count, 0);
		QCOMPARE(chartEngine({ noEngines, noPower, notRecorded }).count, 0);
		const TripSamplePoint piston = EnginePoint(0, 2).set(1, TRIP_ENGINE_general_eng_rpm, 2400).p;
		const TripSamplePoint jet = EnginePoint(1, 1).set(1, TRIP_ENGINE_turb_eng_n1, 50).p;
		ChartEngine e = chartEngine({ noEngines, notRecorded, piston, jet });
		QCOMPARE(e.engineType, 0);
		QCOMPARE(e.count, 2);
		// More engines than the chart has series for: capped.
		e = chartEngine({ EnginePoint(1, 6).set(1, TRIP_ENGINE_turb_eng_n1, 50).p });
		QCOMPARE(e.engineType, 1);
		QCOMPARE(e.count, CHART_ENGINES);
	}

	void chartEngineSpecLabelsByEngineType() {
		const QVariantMap jet = chartEngineSpec({ 1, 2 });
		QCOMPARE(jet.value("count").toInt(), 2);
		QCOMPARE(jet.value("speedLabel").toString(), QStringLiteral("N1"));
		QCOMPARE(jet.value("loadLabel").toString(), QStringLiteral("N2"));
		QCOMPARE(jet.value("speedUnit").toString(), QStringLiteral("%"));
		QCOMPARE(jet.value("speedDecimals").toInt(), 1);
		QCOMPARE(jet.value("loadAxisTitle").toString(), QStringLiteral("N2 (%)"));

		const QVariantMap piston = chartEngineSpec({ 0, 1 });
		QCOMPARE(piston.value("speedAxisTitle").toString(), QStringLiteral("RPM"));
		QCOMPARE(piston.value("loadUnit").toString(), QStringLiteral("inHg"));
		QCOMPARE(piston.value("loadDecimals").toInt(), 1);

		// No power recorded, or an engine type that isn't recorded: count 0 only.
		QCOMPARE(chartEngineSpec({ 1, 0 }), (QVariantMap{ { "count", 0 } }));
		QCOMPARE(chartEngineSpec({ 2, 2 }), (QVariantMap{ { "count", 0 } }));
		QCOMPARE(chartEngineSpec({}), (QVariantMap{ { "count", 0 } }));
	}

	void maxOnlyExtentsStartAtZero() {
		ChartExtents e;
		e.add(valuesWith(0, -5, -5, -100, -1, 0, 0));
		QCOMPARE(e.speedMax, 0.0);
		QCOMPARE(e.altMax, 0.0);
		QCOMPARE(e.fuelMax, 0.0);
	}

	void buildsOnePointPerSampleForEverySeries() {
		const std::vector<ChartSample> samples = {
			sample(zulu(10, 0, 0), 0), sample(zulu(10, 0, 1), 1), sample(zulu(10, 0, 2), 2),
		};
		const ChartSeriesData data = buildChartSeries(samples);
		QCOMPARE(data.pointTimesMs, (std::vector<double>{ utcMs(10, 0, 0), utcMs(10, 0, 1), utcMs(10, 0, 2) }));
		QCOMPARE(data.axisLo.toMSecsSinceEpoch(), (qint64)utcMs(10, 0, 0));
		QCOMPARE(data.axisHi.toMSecsSinceEpoch(), (qint64)utcMs(10, 0, 2));
		for (int s = 0; s < CHART_SERIES_COUNT; ++s) {
			QCOMPARE(data.series[s].size(), 3);
			for (int i = 0; i < 3; ++i)
				QCOMPARE(data.series[s][i], QPointF(data.pointTimesMs[i], i * 10 + s));
		}
		QVERIFY(data.extents.valid);
		QCOMPARE(data.extents.vsMin, (double)CHART_VERTICAL_SPEED);
		QCOMPARE(data.extents.vsMax, 20.0 + CHART_VERTICAL_SPEED);
		QCOMPARE(data.extents.speedMax, 20.0 + CHART_GROUND_SPEED);
	}

	void malformedTimesAreFilledFromThePreviousValidOne() {
		const std::vector<ChartSample> samples = {
			sample(QStringLiteral("bad"), 0), sample(zulu(10, 0, 5), 1),
			sample(QString(), 2), sample(zulu(10, 0, 9), 3), sample(QStringLiteral("x"), 4),
		};
		const ChartSeriesData data = buildChartSeries(samples);
		// Nothing dropped (indices are shared with the map and data table).
		QCOMPARE(data.pointTimesMs.size(), size_t(5));
		QCOMPARE(data.series[CHART_ALTITUDE].size(), 5);
		// A leading bad one gets the first valid time; the axis ignores bad ones.
		QCOMPARE(data.pointTimesMs, (std::vector<double>{ utcMs(10, 0, 5), utcMs(10, 0, 5), utcMs(10, 0, 5),
			utcMs(10, 0, 9), utcMs(10, 0, 9) }));
		QCOMPARE(data.axisLo.toMSecsSinceEpoch(), (qint64)utcMs(10, 0, 5));
		QCOMPARE(data.axisHi.toMSecsSinceEpoch(), (qint64)utcMs(10, 0, 9));
	}

	void singleSampleGetsAOneSecondAxis() {
		const ChartSeriesData data = buildChartSeries({ sample(zulu(8, 0, 0), 0) });
		QCOMPARE(data.axisHi.toMSecsSinceEpoch() - data.axisLo.toMSecsSinceEpoch(), qint64(1000));
	}

	void noValidTimeUsesTheCurrentTime() {
		const qint64 before = QDateTime::currentMSecsSinceEpoch();
		const ChartSeriesData data = buildChartSeries({ sample(QStringLiteral("bad"), 0), sample(QString(), 1) });
		const qint64 after = QDateTime::currentMSecsSinceEpoch();
		QVERIFY(data.axisLo.toMSecsSinceEpoch() >= before && data.axisLo.toMSecsSinceEpoch() <= after);
		QCOMPARE(data.axisHi.toMSecsSinceEpoch() - data.axisLo.toMSecsSinceEpoch(), qint64(1000));
		QCOMPARE(data.pointTimesMs[0], (double)data.axisLo.toMSecsSinceEpoch());
		QCOMPARE(data.pointTimesMs[1], (double)data.axisLo.toMSecsSinceEpoch());
	}

	void extentsOfASlice() {
		std::vector<ChartSample> samples;
		for (int i = 0; i < 6; ++i)
			samples.push_back(sample(zulu(9, 0, i), i));
		const ChartSeriesData data = buildChartSeries(samples);
		const ChartExtents e = chartExtents(data.series, 2, 3);
		QCOMPARE(e.vsMin, 20.0 + CHART_VERTICAL_SPEED);
		QCOMPARE(e.vsMax, 30.0 + CHART_VERTICAL_SPEED);
		QCOMPARE(e.altMax, 30.0 + CHART_ALTITUDE);
		QCOMPARE(e.bankMin, 20.0 + CHART_BANK);
	}

	void valuesAtReadsBackOneSample() {
		const ChartSeriesData data = buildChartSeries({
			sample(zulu(9, 0, 0), 0), sample(zulu(9, 0, 1), 1), sample(zulu(9, 0, 2), 2) });
		const ChartValues v = chartValuesAt(data.series, 1);
		for (int s = 0; s < CHART_SERIES_COUNT; ++s)
			QCOMPARE(v[s], 10.0 + s);
	}

	void decimateSeriesKeepsFirstAndLast() {
		QList<QPointF> full;
		for (int i = 0; i < 10; ++i)
			full.append(QPointF(i, i * 2));
		QCOMPARE(decimateSeries(full, 0, 9, 100), full);
		QCOMPARE(decimateSeries(full, 0, 9, 4), (QList<QPointF>{ { 0, 0 }, { 3, 6 }, { 6, 12 }, { 9, 18 } }));
		QCOMPARE(decimateSeries(full, 2, 4, 100), (QList<QPointF>{ { 2, 4 }, { 3, 6 }, { 4, 8 } }));
		QVERIFY(decimateSeries(full, 0, 10, 100).isEmpty());
		QVERIFY(decimateSeries(full, -1, 3, 100).isEmpty());
		QVERIFY(decimateSeries({}, 0, 0, 100).isEmpty());
	}

	void nearestSampleIndexPicksTheClosestTime() {
		const std::vector<double> t = { 1000, 2000, 4000 };
		QCOMPARE(nearestSampleIndex(t, 0), 0);
		QCOMPARE(nearestSampleIndex(t, 1000), 0);
		QCOMPARE(nearestSampleIndex(t, 1400), 0);
		QCOMPARE(nearestSampleIndex(t, 1500), 1);   // tie: the later one
		QCOMPARE(nearestSampleIndex(t, 1600), 1);
		QCOMPARE(nearestSampleIndex(t, 3100), 2);
		QCOMPARE(nearestSampleIndex(t, 9000), 2);
		QCOMPARE(nearestSampleIndex({ 5 }, 100), 0);
	}

	void valueMapHasTimeAndEverySeries() {
		ChartValues v{};
		for (int s = 0; s < CHART_SERIES_COUNT; ++s)
			v[s] = s + 0.5;
		v[CHART_GEAR_ON_GROUND_0] = 1;
		v[CHART_GEAR_ON_GROUND_1] = 0;
		const QVariantMap m = chartValueMap(utcMs(14, 5, 6, 789), v);
		QCOMPARE(m.value("timeStr").toString(), QStringLiteral("14:05:06.789 UTC"));
		QCOMPARE((int)m.size(), CHART_SERIES_COUNT + 1);
		QCOMPARE(m.value("ias").toDouble(), CHART_AIRSPEED + 0.5);
		QCOMPARE(m.value("bank").toDouble(), CHART_BANK + 0.5);
		QCOMPARE(m.value("onGnd0").typeId(), QMetaType::Bool);
		QCOMPARE(m.value("onGnd0").toBool(), true);
		QCOMPARE(m.value("onGnd1").toBool(), false);
		QCOMPARE(m.value("onGnd2").toBool(), true);   // 18.5 > 0.5
	}
};

QTEST_GUILESS_MAIN(TstChartData)
#include "tst_chart_data.moc"
