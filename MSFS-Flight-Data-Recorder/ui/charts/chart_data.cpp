#include "chart_data.h"
#include "trip_dataset.h"

#include <QTimeZone>
#include <QtMath>

#include <algorithm>
#include <cmath>

namespace {

std::array<ChartSeriesDef, CHART_SERIES_COUNT> makeChartSeries() {
	std::array<ChartSeriesDef, CHART_SERIES_COUNT> series;
	for (int i = 0; i < SIM_ENGINE_INDEXES; ++i) {
		for (const auto& [first, prefix] : { std::pair{ CHART_ENG_N1_1, "engN1_" }, std::pair{ CHART_ENG_N2_1, "engN2_" } }) {
			const QString key = QString::fromLatin1(prefix) + QString::number(i + 1);
			series[first + i] = { key + QStringLiteral("Series"), key, false };
		}
	}
#define CHART_SERIES_DEF(id, objectName, valueKey, isFlag) series[id] = { QStringLiteral(objectName), QStringLiteral(valueKey), isFlag };
	CHART_SERIES_LIST(CHART_SERIES_DEF)
#undef CHART_SERIES_DEF
	return series;
}

}

const std::array<ChartSeriesDef, CHART_SERIES_COUNT> CHART_SERIES = makeChartSeries();

ChartValues chartValues(const TripSamplePoint& p) {
	ChartValues v{};
	if (const EnginePowerSpec* spec = enginePowerSpec(p.engineType)) {
		const auto value = [&p](int engine, TripEngineField field) {
			const double raw = p.engineValue(engine, field);
			return std::isnan(raw) ? 0.0 : raw;
		};
		for (int i = 0; i < std::min(p.engineCount(), SIM_ENGINE_INDEXES); ++i) {
			v[CHART_ENG_N1_1 + i] = value(i + 1, spec->n1Field);
			v[CHART_ENG_N2_1 + i] = value(i + 1, spec->n2Field);
		}
	}
	v[CHART_VERTICAL_SPEED] = p.verticalSpeed;
	v[CHART_AIRSPEED] = p.airspeed;
	v[CHART_GROUND_SPEED] = p.groundSpeed;
	v[CHART_ALTITUDE] = p.altitude;
	v[CHART_GEAR_HANDLE] = p.gearHandlePosition;
	v[CHART_GEAR_POS_0] = p.gearPosition[0];
	v[CHART_GEAR_POS_1] = p.gearPosition[1];
	v[CHART_GEAR_POS_2] = p.gearPosition[2];
	v[CHART_GEAR_ON_GROUND_0] = p.gearOnGround[0] ? 1 : 0;
	v[CHART_GEAR_ON_GROUND_1] = p.gearOnGround[1] ? 1 : 0;
	v[CHART_GEAR_ON_GROUND_2] = p.gearOnGround[2] ? 1 : 0;
	v[CHART_BRAKE] = p.brakeIndicator;
	v[CHART_FLAPS] = p.flapsHandleIndex;
	v[CHART_SPOILERS] = p.spoilersHandlePosition;
	v[CHART_FUEL_WEIGHT] = p.fuelTotalQuantityWeight;
	v[CHART_PITCH] = p.pitchDegrees;
	v[CHART_BANK] = p.bankDegrees;
	return v;
}

ChartEngine chartEngine(const std::vector<TripSamplePoint>& points) {
	for (const TripSamplePoint& p : points) {
		const EnginePowerSpec* spec = enginePowerSpec(p.engineType);
		if (!spec)
			continue;
		const bool n1 = !std::isnan(p.engineValue(1, spec->n1Field));
		const bool n2 = !std::isnan(p.engineValue(1, spec->n2Field));
		if (n1 || n2)
			return { p.engineType, std::min(p.engineCount(), SIM_ENGINE_INDEXES), n1, n2 };
	}
	return {};
}

QVariantMap chartEngineSpec(const ChartEngine& engine) {
	QVariantMap m;
	const EnginePowerSpec* spec = enginePowerSpec(engine.engineType);
	const int count = spec ? engine.count : 0;
	m[QStringLiteral("count")] = count;
	if (count == 0)
		return m;
	m[QStringLiteral("n1Recorded")] = engine.n1Recorded;
	m[QStringLiteral("n2Recorded")] = engine.n2Recorded;
	m[QStringLiteral("n1Label")] = QString::fromUtf8(spec->n1Label);
	m[QStringLiteral("n2Label")] = QString::fromUtf8(spec->n2Label);
	m[QStringLiteral("unit")] = QString::fromUtf8(spec->unit);
	m[QStringLiteral("decimals")] = spec->decimals;
	m[QStringLiteral("axisTitle")] = QString::fromUtf8(spec->axisTitle);
	return m;
}

double chartTimeMs(const QString& zuluTime) {
	const QDateTime zulu = parseZuluTime(zuluTime);
	if (!zulu.isValid())
		return qQNaN();
	return (double)zulu.toMSecsSinceEpoch();
}

namespace {

// Rounds rawStep up to 1/2/5/10 x its power of 10.
double niceStep(double rawStep) {
	double mag  = qPow(10.0, qFloor(qLn(rawStep) / qLn(10.0)));
	double norm = rawStep / mag;
	return (norm <= 1.0) ? mag
	     : (norm <= 2.0) ? 2.0 * mag
	     : (norm <= 5.0) ? 5.0 * mag
	     :                 10.0 * mag;
}

}

double niceAxisMax(double value) {
	if (value <= 0.0)
		return 1.0;
	double target = value * 1.25;
	double step = niceStep(target / 5.0);
	return qCeil(target / step) * step;
}

std::pair<double, double> niceSignedAxisRange(double minVal, double maxVal) {
	double range = maxVal - minVal;
	if (range < 2.0) {
		double mid = (minVal + maxVal) * 0.5;
		minVal = mid - 1.0;
		maxVal = mid + 1.0;
		range = 2.0;
	}
	double margin = range * 0.25;
	double lo = minVal - margin;
	double hi = maxVal + margin;
	double step = niceStep((hi - lo) / 5.0);
	return { qFloor(lo / step) * step, qCeil(hi / step) * step };
}

void ChartExtents::add(const ChartValues& v) {
	const double vs = v[CHART_VERTICAL_SPEED];
	const double pitch = v[CHART_PITCH];
	const double bank = v[CHART_BANK];
	if (!valid) {
		vsMin = vsMax = vs;
		pitchMin = pitchMax = pitch;
		bankMin = bankMax = bank;
		valid = true;
	} else {
		vsMin = qMin(vsMin, vs);
		vsMax = qMax(vsMax, vs);
		pitchMin = qMin(pitchMin, pitch);
		pitchMax = qMax(pitchMax, pitch);
		bankMin = qMin(bankMin, bank);
		bankMax = qMax(bankMax, bank);
	}
	speedMax = qMax(speedMax, qMax(v[CHART_AIRSPEED], v[CHART_GROUND_SPEED]));
	altMax = qMax(altMax, v[CHART_ALTITUDE]);
	fuelMax = qMax(fuelMax, v[CHART_FUEL_WEIGHT]);
	for (int s = CHART_ENG_N1_1; s <= CHART_ENG_N2_LAST; ++s)
		engineMax = qMax(engineMax, v[s]);
}

ChartValues chartValuesAt(const ChartSeriesLists& series, int index) {
	ChartValues v{};
	for (int s = 0; s < CHART_SERIES_COUNT; ++s) {
		if (!series[s].isEmpty())
			v[s] = series[s][index].y();
	}
	return v;
}

ChartExtents chartExtents(const ChartSeriesLists& series, int lo, int hi) {
	ChartExtents extents;
	for (int i = lo; i <= hi; ++i)
		extents.add(chartValuesAt(series, i));
	return extents;
}

ChartSeriesData buildChartSeries(const std::vector<ChartSample>& samples, int engineCount) {
	ChartSeriesData data;
	data.pointTimesMs.reserve(samples.size());
	for (const ChartSample& s : samples)
		data.pointTimesMs.push_back(chartTimeMs(s.zuluTime));

	auto firstValid = std::find_if(data.pointTimesMs.cbegin(), data.pointTimesMs.cend(),
		[](double ms) { return !qIsNaN(ms); });
	auto lastValid = std::find_if(data.pointTimesMs.crbegin(), data.pointTimesMs.crend(),
		[](double ms) { return !qIsNaN(ms); });
	data.axisLo = firstValid == data.pointTimesMs.cend()
		? QDateTime::currentDateTime()
		: QDateTime::fromMSecsSinceEpoch((qint64)*firstValid);
	data.axisHi = lastValid == data.pointTimesMs.crend()
		? data.axisLo.addSecs(1)
		: QDateTime::fromMSecsSinceEpoch((qint64)*lastValid);
	if (data.axisHi <= data.axisLo)
		data.axisHi = data.axisLo.addSecs(1);

	// After the axis range above, so the fill values can't skew it.
	double lastGood = (double)data.axisLo.toMSecsSinceEpoch();
	for (double& ms : data.pointTimesMs) {
		if (qIsNaN(ms))
			ms = lastGood;
		else
			lastGood = ms;
	}

	std::vector<int> charted;
	for (int s = 0; s < CHART_SERIES_COUNT; ++s) {
		const int engine = s <= CHART_ENG_N2_LAST ? (s - CHART_ENG_N1_1) % SIM_ENGINE_INDEXES + 1 : 0;
		if (engine <= engineCount)
			charted.push_back(s);
	}
	for (int s : charted)
		data.series[s].reserve((int)samples.size());
	for (size_t i = 0; i < samples.size(); ++i) {
		for (int s : charted)
			data.series[s].append(QPointF(data.pointTimesMs[i], samples[i].values[s]));
	}
	data.extents = chartExtents(data.series, 0, (int)samples.size() - 1);
	return data;
}

QList<QPointF> decimateSeries(const QList<QPointF>& full, int lo, int hi, int maxPoints) {
	QList<QPointF> result;
	if (lo < 0 || hi >= full.size())
		return result;
	const std::vector<int> indices = decimatedIndices(lo, hi, maxPoints);
	result.reserve((int)indices.size());
	for (int i : indices)
		result.append(full[i]);
	return result;
}

int nearestSampleIndex(const std::vector<double>& timesMs, double timeMs) {
	auto it = std::lower_bound(timesMs.begin(), timesMs.end(), timeMs);
	int idx = (int)(it - timesMs.begin());
	if (idx >= (int)timesMs.size())
		idx = (int)timesMs.size() - 1;
	// lower_bound lands on the first time >= timeMs; the one before may be closer.
	if (idx > 0 && (timesMs[idx] - timeMs > timeMs - timesMs[idx - 1]))
		idx--;
	return idx;
}

QVariantMap chartValueMap(double sampleTimeMs, const ChartValues& values) {
	QVariantMap m;
	m[QStringLiteral("timeStr")] = QDateTime::fromMSecsSinceEpoch((qint64)sampleTimeMs, QTimeZone::UTC)
		.toString(QStringLiteral("HH:mm:ss.zzz")) + QStringLiteral(" UTC");
	for (int s = 0; s < CHART_SERIES_COUNT; ++s) {
		const QString& key = CHART_SERIES[s].valueKey;
		if (CHART_SERIES[s].isFlag)
			m[key] = values[s] > 0.5;
		else
			m[key] = values[s];
	}
	return m;
}
