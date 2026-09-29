#include "chart_data.h"

#include <QtMath>

#include <algorithm>

const std::array<ChartSeriesDef, CHART_SERIES_COUNT> CHART_SERIES = { {
	{ "n1_1Series", "n1_1", false },
	{ "n1_2Series", "n1_2", false },
	{ "n2_1Series", "n2_1", false },
	{ "n2_2Series", "n2_2", false },
	{ "verticalSpeedSeries", "vs", false },
	{ "airspeedSeries", "ias", false },
	{ "groundSpeedSeries", "gs", false },
	{ "altitudeSeries", "alt", false },
	{ "gearHandleSeries", "gearHandle", false },
	{ "gearPosition0Series", "gearPos0", false },
	{ "gearPosition1Series", "gearPos1", false },
	{ "gearPosition2Series", "gearPos2", false },
	{ "gearOnGround0Series", "onGnd0", true },
	{ "gearOnGround1Series", "onGnd1", true },
	{ "gearOnGround2Series", "onGnd2", true },
	{ "brakeSeries", "brake", false },
	{ "flapsSeries", "flaps", false },
	{ "spoilersSeries", "spoilers", false },
	{ "fuelWeightSeries", "fuel", false },
	{ "pitchSeries", "pitch", false },
	{ "bankSeries", "bank", false },
} };

ChartValues chartValues(const TripSamplePoint& p) {
	ChartValues v{};
	v[CHART_N1_1] = p.n1_1;
	v[CHART_N1_2] = p.n1_2;
	v[CHART_N2_1] = p.n2_1;
	v[CHART_N2_2] = p.n2_2;
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

double chartTimeMs(const QString& zuluTime) {
	if (zuluTime.size() < 23)
		return qQNaN();
	QDate d(zuluTime.mid(0, 4).toInt(), zuluTime.mid(5, 2).toInt(), zuluTime.mid(8, 2).toInt());
	QTime t(zuluTime.mid(11, 2).toInt(), zuluTime.mid(14, 2).toInt(),
	        zuluTime.mid(17, 2).toInt(), zuluTime.mid(20, 3).toInt());
	if (!d.isValid() || !t.isValid())
		return qQNaN();
	return (double)QDateTime(d, t).toMSecsSinceEpoch(); // local time
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

bool ChartExtents::add(const ChartValues& v) {
	const ChartExtents before = *this;
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
	return !before.valid
		|| vsMin != before.vsMin || vsMax != before.vsMax
		|| speedMax != before.speedMax || altMax != before.altMax || fuelMax != before.fuelMax
		|| pitchMin != before.pitchMin || pitchMax != before.pitchMax
		|| bankMin != before.bankMin || bankMax != before.bankMax;
}

ChartExtents chartExtents(const ChartSeriesLists& series, int lo, int hi) {
	ChartExtents extents;
	for (int i = lo; i <= hi; ++i) {
		ChartValues v{};
		for (int s = 0; s < CHART_SERIES_COUNT; ++s)
			v[s] = series[s][i].y();
		extents.add(v);
	}
	return extents;
}

ChartSeriesData buildChartSeries(const std::vector<ChartSample>& samples) {
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

	for (QList<QPointF>& list : data.series)
		list.reserve((int)samples.size());
	for (size_t i = 0; i < samples.size(); ++i) {
		for (int s = 0; s < CHART_SERIES_COUNT; ++s)
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
	m[QStringLiteral("timeStr")] = QDateTime::fromMSecsSinceEpoch((qint64)sampleTimeMs)
		.toString(QStringLiteral("HH:mm:ss.zzz")) + QStringLiteral(" UTC");
	for (int s = 0; s < CHART_SERIES_COUNT; ++s) {
		const QString key = QString::fromLatin1(CHART_SERIES[s].valueKey);
		if (CHART_SERIES[s].isFlag)
			m[key] = values[s] > 0.5;
		else
			m[key] = values[s];
	}
	return m;
}
