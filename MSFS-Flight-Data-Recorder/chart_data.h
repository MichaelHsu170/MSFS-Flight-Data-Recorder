#pragma once

#include <QDateTime>
#include <QList>
#include <QPointF>
#include <QString>
#include <QVariantMap>

#include <array>
#include <utility>
#include <vector>

#include "trip_dataset.h"

// The data behind ChartsPanel's timeline charts, with no Qt Graphs or QML
// objects involved: which series exist, their points and axis ranges, and
// the hover readout. ChartsPanel only moves these into the QML series/axes.

enum ChartSeriesId {
	CHART_N1_1, CHART_N1_2, CHART_N2_1, CHART_N2_2,
	CHART_VERTICAL_SPEED, CHART_AIRSPEED, CHART_GROUND_SPEED, CHART_ALTITUDE,
	CHART_GEAR_HANDLE, CHART_GEAR_POS_0, CHART_GEAR_POS_1, CHART_GEAR_POS_2,
	CHART_GEAR_ON_GROUND_0, CHART_GEAR_ON_GROUND_1, CHART_GEAR_ON_GROUND_2,
	CHART_BRAKE, CHART_FLAPS, CHART_SPOILERS, CHART_FUEL_WEIGHT,
	CHART_PITCH, CHART_BANK,
	CHART_SERIES_COUNT
};

struct ChartSeriesDef {
	// The LineSeries' objectName in resources/charts_panel.qml.
	const char* objectName;
	// Its key in chartValueMap(), read by charts_panel.qml's hover readout.
	const char* valueKey;
	// A 0/1 series, reported as a bool in chartValueMap().
	bool isFlag;
};
// Indexed by ChartSeriesId.
extern const std::array<ChartSeriesDef, CHART_SERIES_COUNT> CHART_SERIES;

// One sample's value for every series, indexed by ChartSeriesId.
using ChartValues = std::array<double, CHART_SERIES_COUNT>;
ChartValues chartValues(const TripSamplePoint& point);

// A zulu-time string ("yyyy-MM-ddTHH:mm:ss.zzz..."; see DATETIME::
// format_date_time() in types.h) as chart X-axis epoch ms. Qt Graphs'
// DateTimeAxis always labels in local time, so the zulu components are read
// as a local time -- the labels then show zulu. NaN for a malformed/short
// string rather than epoch 0 (1970), so a bad point can't drag an axis back
// to 1970.
double chartTimeMs(const QString& zuluTime);

// Axis max for a series peaking at value: 25% headroom, rounded up to a
// "nice" step (1/2/5 x power of 10) giving about 5 grid intervals. 1 for
// value <= 0.
double niceAxisMax(double value);
// [lo, hi] for a signed axis spanning [minVal, maxVal] (widened to at least
// 2 around its middle): 25% margin each side, floored/ceiled to a nice step.
std::pair<double, double> niceSignedAxisRange(double minVal, double maxVal);

// What the Y axes are sized from: min/max of vertical speed, pitch and bank
// (the signed axes), max of speed (airspeed or ground speed), altitude and
// fuel weight (the axes starting at 0; their max starts at 0 too).
struct ChartExtents {
	bool valid = false; // false until the first add()
	double vsMin = 0, vsMax = 0;
	double speedMax = 0, altMax = 0, fuelMax = 0;
	double pitchMin = 0, pitchMax = 0;
	double bankMin = 0, bankMax = 0;
	// Widens the extents to include values; true if anything changed.
	bool add(const ChartValues& values);
};

// Every series' points: X = chartTimeMs(), Y = the value. Indexed by
// ChartSeriesId; every list has one point per sample.
using ChartSeriesLists = std::array<QList<QPointF>, CHART_SERIES_COUNT>;
// Extents of samples lo..hi of series.
ChartExtents chartExtents(const ChartSeriesLists& series, int lo, int hi);

// What the chart worker thread needs of one sample: copied on the GUI thread
// so the worker never touches the dataset, and without the rest of a
// TripSamplePoint (e.g. its ~1 KB rawNums).
struct ChartSample {
	QString zuluTime;
	ChartValues values;
};

struct ChartSeriesData {
	// One per sample. A malformed timestamp is replaced by the previous valid
	// one (axisLo if none): samples can't be dropped, since the map, charts
	// and data table share sample indices, and the times must stay sorted and
	// NaN-free for nearestSampleIndex().
	std::vector<double> pointTimesMs;
	// First and last valid timestamps -- the current time if there are none;
	// axisHi is at least 1 s after axisLo.
	QDateTime axisLo;
	QDateTime axisHi;
	ChartExtents extents;
	ChartSeriesLists series;
};
// Everything ChartsPanel::setDataset() shows for a historical trip. Pure, so
// it runs on a worker thread.
ChartSeriesData buildChartSeries(const std::vector<ChartSample>& samples);

// Samples lo..hi of full, thinned to at most maxPoints (see
// decimatedIndices() in trip_dataset.h). Empty if lo..hi isn't inside full.
QList<QPointF> decimateSeries(const QList<QPointF>& full, int lo, int hi, int maxPoints);

// Index of the time in timesMs (sorted, non-empty) nearest to timeMs; the
// later one on a tie.
int nearestSampleIndex(const std::vector<double>& timesMs, double timeMs);
// The hover readout for one sample at sampleTimeMs: "timeStr" ("HH:mm:ss.zzz
// UTC") plus every series' valueKey.
QVariantMap chartValueMap(double sampleTimeMs, const ChartValues& values);
