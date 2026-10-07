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

// The engines the first chart has series for; engines past it aren't
// charted.
constexpr int CHART_ENGINES = 4;

// The data behind ChartsPanel's timeline charts, with no Qt Graphs or QML
// objects involved: which series exist, their points and axis ranges, and
// the hover readout. ChartsPanel only moves these into the QML series/axes.

// Every chart series, in ChartSeriesId order -- the one list both the enum
// and CHART_SERIES are built from: X(id, objectName, valueKey, isFlag), as
// in ChartSeriesDef below. Engines 1..CHART_ENGINES' speed and load
// (enginePowerSpec() in engine_power.h) come first.
#define CHART_SERIES_LIST(X) \
	X(CHART_ENG_SPEED_1,      "engSpeed1Series",     "engSpeed1",  false) \
	X(CHART_ENG_SPEED_2,      "engSpeed2Series",     "engSpeed2",  false) \
	X(CHART_ENG_SPEED_3,      "engSpeed3Series",     "engSpeed3",  false) \
	X(CHART_ENG_SPEED_4,      "engSpeed4Series",     "engSpeed4",  false) \
	X(CHART_ENG_LOAD_1,       "engLoad1Series",      "engLoad1",   false) \
	X(CHART_ENG_LOAD_2,       "engLoad2Series",      "engLoad2",   false) \
	X(CHART_ENG_LOAD_3,       "engLoad3Series",      "engLoad3",   false) \
	X(CHART_ENG_LOAD_4,       "engLoad4Series",      "engLoad4",   false) \
	X(CHART_VERTICAL_SPEED,   "verticalSpeedSeries", "vs",         false) \
	X(CHART_AIRSPEED,         "airspeedSeries",      "ias",        false) \
	X(CHART_GROUND_SPEED,     "groundSpeedSeries",   "gs",         false) \
	X(CHART_ALTITUDE,         "altitudeSeries",      "alt",        false) \
	X(CHART_GEAR_HANDLE,      "gearHandleSeries",    "gearHandle", false) \
	X(CHART_GEAR_POS_0,       "gearPosition0Series", "gearPos0",   false) \
	X(CHART_GEAR_POS_1,       "gearPosition1Series", "gearPos1",   false) \
	X(CHART_GEAR_POS_2,       "gearPosition2Series", "gearPos2",   false) \
	X(CHART_GEAR_ON_GROUND_0, "gearOnGround0Series", "onGnd0",     true)  \
	X(CHART_GEAR_ON_GROUND_1, "gearOnGround1Series", "onGnd1",     true)  \
	X(CHART_GEAR_ON_GROUND_2, "gearOnGround2Series", "onGnd2",     true)  \
	X(CHART_BRAKE,            "brakeSeries",         "brake",      false) \
	X(CHART_FLAPS,            "flapsSeries",         "flaps",      false) \
	X(CHART_SPOILERS,         "spoilersSeries",      "spoilers",   false) \
	X(CHART_FUEL_WEIGHT,      "fuelWeightSeries",    "fuel",       false) \
	X(CHART_PITCH,            "pitchSeries",         "pitch",      false) \
	X(CHART_BANK,             "bankSeries",          "bank",       false)

enum ChartSeriesId {
#define CHART_SERIES_ID(id, objectName, valueKey, isFlag) id,
	CHART_SERIES_LIST(CHART_SERIES_ID)
#undef CHART_SERIES_ID
	CHART_SERIES_COUNT
};

struct ChartSeriesDef {
	// The LineSeries' objectName in ui/charts/charts_panel.qml.
	const char* objectName;
	// Its key in chartValueMap(), read by charts_panel.qml's hover readout.
	const char* valueKey;
	// A 0/1 series, reported as a bool in chartValueMap().
	bool isFlag;
};
// Indexed by ChartSeriesId.
extern const std::array<ChartSeriesDef, CHART_SERIES_COUNT> CHART_SERIES;

static_assert(CHART_ENG_LOAD_1 - CHART_ENG_SPEED_1 == CHART_ENGINES && CHART_VERTICAL_SPEED - CHART_ENG_LOAD_1 == CHART_ENGINES,
	"one engine speed and one engine load series per engine");

// One sample's value for every series, indexed by ChartSeriesId. An engine's
// speed and load are the values enginePowerSpec() names for the sample's
// engine type; 0 for one not recorded (an engine past the sample's
// engineCount(), a type without a spec, or a value an older build didn't
// record).
using ChartValues = std::array<double, CHART_SERIES_COUNT>;
ChartValues chartValues(const TripSamplePoint& point);

// What the first chart is labeled by: an ENGINE TYPE and how many engines'
// series it shows.
struct ChartEngine {
	int engineType = -1;
	int count = 0;  // 0..CHART_ENGINES; 0 = no engine power recorded
};
// The first point's that recorded engine 1's speed for an engine type with a
// spec (count 0 if none did), its count capped at CHART_ENGINES. A trip keeps
// one aircraft, so later points have the same engine type.
ChartEngine chartEngine(const std::vector<TripSamplePoint>& points);
// charts_panel.qml's root engineSpec for engine: count, plus speed/load
// Label, Unit, Decimals and AxisTitle (enginePowerSpec()) -- only count (0)
// if engine recorded no power, which shows the no-data message.
QVariantMap chartEngineSpec(const ChartEngine& engine);

// A zulu-time string (parseZuluTime() in trip_dataset.h) as chart X-axis
// epoch ms: the real UTC instant, which charts_panel.qml's time axis labels in
// UTC, so a trip across the PC's DST change plots and reads as zulu time
// without a skipped or repeated hour. NaN for a string parseZuluTime()
// rejects rather than epoch 0 (1970), so a bad point can't drag an axis back
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
// (the signed axes), max of speed (airspeed or ground speed), altitude, fuel
// weight and engine speed/load (any engine) (the axes starting at 0; their
// max starts at 0 too).
struct ChartExtents {
	bool valid = false; // false until the first add()
	double vsMin = 0, vsMax = 0;
	double speedMax = 0, altMax = 0, fuelMax = 0;
	double engSpeedMax = 0, engLoadMax = 0;
	double pitchMin = 0, pitchMax = 0;
	double bankMin = 0, bankMax = 0;
	// Widens the extents to include values.
	void add(const ChartValues& values);
};

// Every series' points: X = chartTimeMs(), Y = the value. Indexed by
// ChartSeriesId; every list has one point per sample.
using ChartSeriesLists = std::array<QList<QPointF>, CHART_SERIES_COUNT>;
// Sample index's values, read back from series.
ChartValues chartValuesAt(const ChartSeriesLists& series, int index);
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

// Samples lo..hi of full, thinned to at most maxPoints plus sample hi (see
// decimatedIndices() in trip_dataset.h). Empty if lo..hi isn't inside full.
QList<QPointF> decimateSeries(const QList<QPointF>& full, int lo, int hi, int maxPoints);

// Index of the time in timesMs (sorted, non-empty) nearest to timeMs; the
// later one on a tie.
int nearestSampleIndex(const std::vector<double>& timesMs, double timeMs);
// The hover readout for one sample at sampleTimeMs: "timeStr" ("HH:mm:ss.zzz
// UTC") plus every series' valueKey.
QVariantMap chartValueMap(double sampleTimeMs, const ChartValues& values);
