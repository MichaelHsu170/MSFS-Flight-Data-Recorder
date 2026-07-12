#include "charts_panel.h"

#include <QQuickWidget>
#include <QQuickItem>
#include <QQmlContext>
#include <QVBoxLayout>
#include <QUrl>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>
#include <QElapsedTimer>
#include "logger.h"
#include <algorithm>

#include <QtGraphs/qlineseries.h>
#include <QtGraphs/qdatetimeaxis.h>
#include <QtGraphs/qvalueaxis.h>

#include <QtMath>

namespace {

// Convert a "yyyy-MM-ddTHH:mm:ss.zzz+HH:MM_D" zulu-time string to an epoch ms
// value that, when Qt Graphs displays it as local time, shows the UTC (zulu)
// components. Qt Graphs DateTimeAxis always renders labels in local time regardless
// of setTimeZone(), so we store epoch ms = QDateTime(utcDate, utcTime, LocalTime)
// — the point whose local time LOOKS like the zulu time we actually want.
static double epochMillisFor(const QString& zuluTime) {
	if (zuluTime.size() < 23) return 0.0;
	QDate d(zuluTime.mid(0, 4).toInt(), zuluTime.mid(5, 2).toInt(), zuluTime.mid(8, 2).toInt());
	QTime t(zuluTime.mid(11, 2).toInt(), zuluTime.mid(14, 2).toInt(),
	        zuluTime.mid(17, 2).toInt(), zuluTime.mid(20, 3).toInt());
	if (!d.isValid() || !t.isValid()) return 0.0;
	return (double)QDateTime(d, t, QTimeZone::LocalTime).toMSecsSinceEpoch();
}

QLineSeries* findSeries(QQuickItem* root, const char* objectName) {
	return root->findChild<QLineSeries*>(QString::fromLatin1(objectName));
}

QDateTimeAxis* findXAxis(QQuickItem* root, const char* objectName) {
	return root->findChild<QDateTimeAxis*>(QString::fromLatin1(objectName));
}

QValueAxis* findYAxis(QQuickItem* root, const char* objectName) {
	return root->findChild<QValueAxis*>(QString::fromLatin1(objectName));
}

// Compute a Y axis max for value: apply 25% headroom, then round up to the
// nearest "nice" step (1/2/5 × power-of-10) sized to give ~5 grid intervals.
static double yAxisMax(double value) {
	if (value <= 0.0)
		return 1.0;
	double target  = value * 1.25;
	double rawStep = target / 5.0;
	double mag     = qPow(10.0, qFloor(qLn(rawStep) / qLn(10.0)));
	double norm    = rawStep / mag;
	double step    = (norm <= 1.0) ? mag
	               : (norm <= 2.0) ? 2.0 * mag
	               : (norm <= 5.0) ? 5.0 * mag
	               :                 10.0 * mag;
	return qCeil(target / step) * step;
}

// Returns [lo, hi] for a signed axis spanning [minVal, maxVal] with 25% margin
// on each side, then floor/ceil to the nearest nice tick step (~5 grid lines).
static std::pair<double, double> signedAxisRange(double minVal, double maxVal) {
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
	double rawStep = (hi - lo) / 5.0;
	double mag  = qPow(10.0, qFloor(qLn(rawStep) / qLn(10.0)));
	double norm = rawStep / mag;
	double step = (norm <= 1.0) ? mag
	            : (norm <= 2.0) ? 2.0 * mag
	            : (norm <= 5.0) ? 5.0 * mag
	            :                 10.0 * mag;
	lo = qFloor(lo / step) * step;
	hi = qCeil(hi  / step) * step;
	return {lo, hi};
}

// Lightweight copy of one TripSamplePoint carrying only the fields the chart
// worker needs, avoiding the rawNums vector (~1 KB per point) that
// DataTablePanel uses but charts never need.
struct LightPoint {
	QString zuluTime;
	double n1_1, n1_2, n2_1, n2_2;
	int verticalSpeed, airspeed, groundSpeed, altitude;
	double gearHandlePosition;
	int gearPosition[3];
	bool gearOnGround[3];
	int brakeIndicator;
	double flapsHandleIndex, spoilersHandlePosition, fuelTotalQuantityWeight;
	double pitchDegrees, bankDegrees;
};

// Everything setDataset() needs to push into the QML series/axis, built on a
// worker thread. None of these types touch QML/GUI objects, so this is safe
// to build off the GUI thread -- only handing the finished lists to the
// actual QLineSeries objects (in setDataset()'s watcher callback) needs to
// happen back on the GUI thread.
struct ChartSeriesData {
	std::vector<double> pointTimesMs;
	QDateTime axisLo;
	QDateTime axisHi;
	double vsMin = 0.0, vsMax = 0.0;
	double speedYMax = 0.0;
	double altYMax   = 0.0;
	double fuelYMax  = 0.0;
	double pitchMin = 0.0, pitchMax = 0.0;
	double bankMin  = 0.0, bankMax  = 0.0;
	qint64 computeNs = 0;
	QList<QPointF> n1_1, n1_2, n2_1, n2_2;
	QList<QPointF> verticalSpeed;
	QList<QPointF> airspeed, groundSpeed;
	QList<QPointF> altitude;
	QList<QPointF> gearHandle, gearPos0, gearPos1, gearPos2, gearOnGround0, gearOnGround1, gearOnGround2;
	QList<QPointF> brake, flaps, spoilers;
	QList<QPointF> fuelWeight;
	QList<QPointF> pitch, bank;
};

}

ChartsPanel::ChartsPanel(QWidget* parent) : QWidget(parent) {
	view_ = new QQuickWidget(this);
	view_->setResizeMode(QQuickWidget::SizeRootObjectToView);
	view_->rootContext()->setContextProperty(QStringLiteral("chartsBridge"), this);
	view_->setSource(QUrl(QStringLiteral("qrc:/charts/charts_panel.qml")));

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(view_);
}

void ChartsPanel::setAllXAxisRange(const QDateTime& lo, const QDateTime& hi) {
	QQuickItem* root = view_->rootObject();
	if (!root) return;
	for (QDateTimeAxis* ax : root->findChildren<QDateTimeAxis*>()) {
		ax->setMin(lo);
		ax->setMax(hi);
	}
}

void ChartsPanel::buildSeriesCache() {
	if (cache_.valid)
		return;
	QQuickItem* root = view_->rootObject();
	if (!root)
		return;
	cache_.n1_1          = findSeries(root, "n1_1Series");
	cache_.n1_2          = findSeries(root, "n1_2Series");
	cache_.n2_1          = findSeries(root, "n2_1Series");
	cache_.n2_2          = findSeries(root, "n2_2Series");
	cache_.verticalSpeed = findSeries(root, "verticalSpeedSeries");
	cache_.airspeed      = findSeries(root, "airspeedSeries");
	cache_.groundSpeed   = findSeries(root, "groundSpeedSeries");
	cache_.altitude      = findSeries(root, "altitudeSeries");
	cache_.gearHandle    = findSeries(root, "gearHandleSeries");
	cache_.gearPos0      = findSeries(root, "gearPosition0Series");
	cache_.gearPos1      = findSeries(root, "gearPosition1Series");
	cache_.gearPos2      = findSeries(root, "gearPosition2Series");
	cache_.gearOnGround0 = findSeries(root, "gearOnGround0Series");
	cache_.gearOnGround1 = findSeries(root, "gearOnGround1Series");
	cache_.gearOnGround2 = findSeries(root, "gearOnGround2Series");
	cache_.brake         = findSeries(root, "brakeSeries");
	cache_.flaps         = findSeries(root, "flapsSeries");
	cache_.spoilers      = findSeries(root, "spoilersSeries");
	cache_.fuelWeight    = findSeries(root, "fuelWeightSeries");
	cache_.pitch         = findSeries(root, "pitchSeries");
	cache_.bank          = findSeries(root, "bankSeries");
	cache_.xAxis         = findXAxis(root, "sharedXAxis");
	cache_.vsYAxis       = findYAxis(root, "vsYAxis");
	cache_.speedYAxis    = findYAxis(root, "speedYAxis");
	cache_.altYAxis      = findYAxis(root, "altYAxis");
	cache_.fuelYAxis     = findYAxis(root, "fuelYAxis");
	cache_.pitchYAxis    = findYAxis(root, "pitchYAxis");
	cache_.bankYAxis     = findYAxis(root, "bankYAxis");
	cache_.valid         = (cache_.n1_1 != nullptr);
}

void ChartsPanel::setDataset(const TripDataset& dataset) {
	++datasetVersion_;
	liveVsMin_ = liveVsMax_ = 0.0;
	liveVsValid_ = false;
	liveSpeedMax_   = 0.0;
	liveAltMax_     = 0.0;
	liveFuelMax_    = 0.0;
	livePitchMin_ = livePitchMax_ = 0.0;
	liveBankMin_  = liveBankMax_  = 0.0;
	livePitchBankValid_ = false;
	full_.ready      = false;
	lastRangeStart_  = INT_MIN;
	lastRangeEnd_    = INT_MIN;

	QQuickItem* root = view_->rootObject();
	if (root == nullptr) {
		emit seriesLoaded();
		return;
	}

	// Empty dataset (Deselect / overview mode): make charts appear empty by
	// resetting the X axis to a 1-second window at current time. The stale
	// series points are left in memory but become invisible outside the axis
	// range. Avoids QLineSeries::clear() here because clear() on a populated
	// series blocks the main thread on a Qt Graphs render-thread sync. The next
	// trip load replaces stale points via s->replace() with no intermediate clear.
	if (dataset.points.empty()) {
		pointTimesMs_.clear();
		pointCount_ = 0;
		full_.ready = false;
		buildSeriesCache();
		if (cache_.xAxis) {
			QDateTime now = QDateTime::currentDateTime();
			setAllXAxisRange(now, now.addSecs(1));
		}
		emit seriesLoaded();
		return;
	}

	QElapsedTimer copyTimer; copyTimer.start();
	std::vector<LightPoint> points;
	points.reserve(dataset.points.size());
	for (const TripSamplePoint& p : dataset.points) {
		LightPoint lp;
		lp.zuluTime          = p.zuluTime;
		lp.n1_1              = p.n1_1;
		lp.n1_2              = p.n1_2;
		lp.n2_1              = p.n2_1;
		lp.n2_2              = p.n2_2;
		lp.verticalSpeed     = p.verticalSpeed;
		lp.airspeed          = p.airspeed;
		lp.groundSpeed       = p.groundSpeed;
		lp.altitude          = p.altitude;
		lp.gearHandlePosition    = p.gearHandlePosition;
		lp.gearPosition[0]       = p.gearPosition[0];
		lp.gearPosition[1]       = p.gearPosition[1];
		lp.gearPosition[2]       = p.gearPosition[2];
		lp.gearOnGround[0]       = p.gearOnGround[0];
		lp.gearOnGround[1]       = p.gearOnGround[1];
		lp.gearOnGround[2]       = p.gearOnGround[2];
		lp.brakeIndicator        = p.brakeIndicator;
		lp.flapsHandleIndex      = p.flapsHandleIndex;
		lp.spoilersHandlePosition    = p.spoilersHandlePosition;
		lp.fuelTotalQuantityWeight   = p.fuelTotalQuantityWeight;
		lp.pitchDegrees              = p.pitchDegrees;
		lp.bankDegrees               = p.bankDegrees;
		points.push_back(std::move(lp));
	}
	Logger::logf(Logger::Profile, "Charts", "copy: %lld ms  (%zu pts)", copyTimer.nsecsElapsed() / 1000000, points.size());

	int ver = datasetVersion_;
	auto* watcher = new QFutureWatcher<ChartSeriesData>(this);
	connect(watcher, &QFutureWatcher<ChartSeriesData>::finished, this, [this, watcher, ver]() {
		Logger::logf(Logger::Profile, "Charts", "finished lambda: ver=%d cur=%d", ver, datasetVersion_);
		watcher->deleteLater();
		// A newer setDataset call superseded this one — discard stale results
		// rather than writing old trip data into series that were already cleared.
		if (ver != datasetVersion_) return;
		// Non-const so we can std::move the QList members into full_ below,
		// avoiding a second 15 MB copy into the QLineSeries objects.
		ChartSeriesData data = watcher->result();
		Logger::logf(Logger::Profile, "Charts", "compute (bg): %lld ms", data.computeNs / 1000000);

		QElapsedTimer applyTimer; applyTimer.start();
		QQuickItem* root = view_->rootObject();
		if (root == nullptr)
			return;

		pointTimesMs_ = std::move(data.pointTimesMs);
		pointCount_ = (int)pointTimesMs_.size();

		setAllXAxisRange(data.axisLo, data.axisHi);
		{
			auto [vsLo, vsHi] = signedAxisRange(data.vsMin, data.vsMax);
			if (QValueAxis* ax = findYAxis(root, "vsYAxis")) { ax->setMin(vsLo); ax->setMax(vsHi); }
		}
		if (QValueAxis* ax = findYAxis(root, "speedYAxis"))
			ax->setMax(yAxisMax(data.speedYMax));
		if (QValueAxis* ax = findYAxis(root, "altYAxis"))
			ax->setMax(yAxisMax(data.altYMax));
		if (QValueAxis* ax = findYAxis(root, "fuelYAxis"))
			ax->setMax(yAxisMax(data.fuelYMax));
		{
			auto [pLo, pHi] = signedAxisRange(data.pitchMin, data.pitchMax);
			auto [bLo, bHi] = signedAxisRange(data.bankMin,  data.bankMax);
			if (QValueAxis* ax = findYAxis(root, "pitchYAxis")) { ax->setMin(pLo); ax->setMax(pHi); }
			if (QValueAxis* ax = findYAxis(root, "bankYAxis"))  { ax->setMin(bLo); ax->setMax(bHi); }
		}
		liveVsMin_ = data.vsMin; liveVsMax_ = data.vsMax;
		liveVsValid_ = true;
		liveSpeedMax_ = data.speedYMax;
		liveAltMax_   = data.altYMax;
		liveFuelMax_  = data.fuelYMax;
		livePitchMin_ = data.pitchMin; livePitchMax_ = data.pitchMax;
		liveBankMin_  = data.bankMin;  liveBankMax_  = data.bankMax;
		livePitchBankValid_ = true;
		root->setProperty("isFullRangeVisible", true);

		// Move full-resolution data into full_ so setVisibleRange can serve
		// exact slices on zoom rather than leaving all 49k points in Qt Graphs.
		displayStride_ = std::max(1, (pointCount_ + kDisplayPoints - 1) / kDisplayPoints);
		full_.n1_1          = std::move(data.n1_1);
		full_.n1_2          = std::move(data.n1_2);
		full_.n2_1          = std::move(data.n2_1);
		full_.n2_2          = std::move(data.n2_2);
		full_.verticalSpeed = std::move(data.verticalSpeed);
		full_.airspeed      = std::move(data.airspeed);
		full_.groundSpeed   = std::move(data.groundSpeed);
		full_.altitude      = std::move(data.altitude);
		full_.gearHandle    = std::move(data.gearHandle);
		full_.gearPos0      = std::move(data.gearPos0);
		full_.gearPos1      = std::move(data.gearPos1);
		full_.gearPos2      = std::move(data.gearPos2);
		full_.gearOnGround0 = std::move(data.gearOnGround0);
		full_.gearOnGround1 = std::move(data.gearOnGround1);
		full_.gearOnGround2 = std::move(data.gearOnGround2);
		full_.brake         = std::move(data.brake);
		full_.flaps         = std::move(data.flaps);
		full_.spoilers      = std::move(data.spoilers);
		full_.fuelWeight    = std::move(data.fuelWeight);
		full_.pitch         = std::move(data.pitch);
		full_.bank          = std::move(data.bank);
		full_.ready = true;

		// Load a decimated view into each series. Qt Graphs renders every
		// loaded QPointF per frame; keeping this to ~kDisplayPoints points
		// makes the initial paint and subsequent zoom-out fast.
		// replace() atomically swaps all points in one scene-graph notification
		// instead of clear()+append() which sends two and causes the main thread
		// to block on a render-thread sync for each call on a populated series.
		buildSeriesCache();
		auto loadDec = [&](QLineSeries* s, const QList<QPointF>& full) {
			if (!s) return;
			if (displayStride_ <= 1) { s->replace(full); return; }
			QList<QPointF> dec;
			dec.reserve(full.size() / displayStride_ + 1);
			for (int i = 0; i < full.size(); i += displayStride_)
				dec.append(full[i]);
			s->replace(dec);
		};

		loadDec(cache_.n1_1,          full_.n1_1);
		loadDec(cache_.n1_2,          full_.n1_2);
		loadDec(cache_.n2_1,          full_.n2_1);
		loadDec(cache_.n2_2,          full_.n2_2);
		loadDec(cache_.verticalSpeed, full_.verticalSpeed);
		loadDec(cache_.airspeed,      full_.airspeed);
		loadDec(cache_.groundSpeed,   full_.groundSpeed);
		loadDec(cache_.altitude,      full_.altitude);
		loadDec(cache_.gearHandle,    full_.gearHandle);
		loadDec(cache_.gearPos0,      full_.gearPos0);
		loadDec(cache_.gearPos1,      full_.gearPos1);
		loadDec(cache_.gearPos2,      full_.gearPos2);
		loadDec(cache_.gearOnGround0, full_.gearOnGround0);
		loadDec(cache_.gearOnGround1, full_.gearOnGround1);
		loadDec(cache_.gearOnGround2, full_.gearOnGround2);
		loadDec(cache_.brake,         full_.brake);
		loadDec(cache_.flaps,         full_.flaps);
		loadDec(cache_.spoilers,      full_.spoilers);
		loadDec(cache_.fuelWeight,    full_.fuelWeight);
		loadDec(cache_.pitch,         full_.pitch);
		loadDec(cache_.bank,          full_.bank);

		Logger::logf(Logger::Profile, "Charts", "apply (GUI): %lld ms  (stride %d, ~%d pts/series)",
		             applyTimer.nsecsElapsed() / 1000000, displayStride_,
		             pointCount_ / std::max(1, displayStride_));
		emit seriesLoaded();
	});

	watcher->setFuture(QtConcurrent::run([this, points = std::move(points)]() {
		QElapsedTimer computeTimer; computeTimer.start();
		ChartSeriesData data;
		data.pointTimesMs.reserve(points.size());
		for (const LightPoint& p : points)
			data.pointTimesMs.push_back(epochMillisFor(p.zuluTime));

		data.axisLo = data.pointTimesMs.empty()
			? QDateTime::currentDateTime()
			: QDateTime::fromMSecsSinceEpoch((qint64)data.pointTimesMs.front());
		data.axisHi = data.pointTimesMs.empty()
			? data.axisLo.addSecs(1)
			: QDateTime::fromMSecsSinceEpoch((qint64)data.pointTimesMs.back());
		if (data.axisHi <= data.axisLo)
			data.axisHi = data.axisLo.addSecs(1);

		auto fill = [&](QList<QPointF>& target, auto valueOf) {
			target.reserve((int)points.size());
			for (int i = 0; i < (int)points.size(); ++i)
				target.append(QPointF(data.pointTimesMs[i], (qreal)valueOf(points[i])));
		};

		fill(data.n1_1,         [](const LightPoint& p) { return p.n1_1; });
		fill(data.n1_2,         [](const LightPoint& p) { return p.n1_2; });
		fill(data.n2_1,         [](const LightPoint& p) { return p.n2_1; });
		fill(data.n2_2,         [](const LightPoint& p) { return p.n2_2; });
		fill(data.verticalSpeed,[](const LightPoint& p) { return p.verticalSpeed; });
		fill(data.airspeed,     [](const LightPoint& p) { return p.airspeed; });
		fill(data.groundSpeed,  [](const LightPoint& p) { return p.groundSpeed; });
		fill(data.altitude,     [](const LightPoint& p) { return p.altitude; });
		fill(data.gearHandle,   [](const LightPoint& p) { return p.gearHandlePosition; });
		fill(data.gearPos0,     [](const LightPoint& p) { return p.gearPosition[0]; });
		fill(data.gearPos1,     [](const LightPoint& p) { return p.gearPosition[1]; });
		fill(data.gearPos2,     [](const LightPoint& p) { return p.gearPosition[2]; });
		fill(data.gearOnGround0,[](const LightPoint& p) { return p.gearOnGround[0] ? 1 : 0; });
		fill(data.gearOnGround1,[](const LightPoint& p) { return p.gearOnGround[1] ? 1 : 0; });
		fill(data.gearOnGround2,[](const LightPoint& p) { return p.gearOnGround[2] ? 1 : 0; });
		fill(data.brake,        [](const LightPoint& p) { return p.brakeIndicator; });
		fill(data.flaps,        [](const LightPoint& p) { return p.flapsHandleIndex; });
		fill(data.spoilers,     [](const LightPoint& p) { return p.spoilersHandlePosition; });
		fill(data.fuelWeight,   [](const LightPoint& p) { return p.fuelTotalQuantityWeight; });
		fill(data.pitch,        [](const LightPoint& p) { return p.pitchDegrees; });
		fill(data.bank,         [](const LightPoint& p) { return p.bankDegrees; });

		bool firstPB = true;
		for (const LightPoint& p : points) {
			double spd = qMax((double)p.airspeed, (double)p.groundSpeed);
			if (spd > data.speedYMax)
				data.speedYMax = spd;
			if (p.altitude > data.altYMax)
				data.altYMax = p.altitude;
			if (p.fuelTotalQuantityWeight > data.fuelYMax)
				data.fuelYMax = p.fuelTotalQuantityWeight;
			if (firstPB) {
				data.vsMin    = data.vsMax    = p.verticalSpeed;
				data.pitchMin = data.pitchMax = p.pitchDegrees;
				data.bankMin  = data.bankMax  = p.bankDegrees;
				firstPB = false;
			} else {
				data.vsMin    = qMin(data.vsMin,    (double)p.verticalSpeed);
				data.vsMax    = qMax(data.vsMax,    (double)p.verticalSpeed);
				data.pitchMin = qMin(data.pitchMin, p.pitchDegrees);
				data.pitchMax = qMax(data.pitchMax, p.pitchDegrees);
				data.bankMin  = qMin(data.bankMin,  p.bankDegrees);
				data.bankMax  = qMax(data.bankMax,  p.bankDegrees);
			}
		}

		data.computeNs = computeTimer.nsecsElapsed();
		return data;
	}));
}

void ChartsPanel::setCursorIndex(int index) {
	QQuickItem* root = view_->rootObject();
	if (root == nullptr)
		return;
	double cursorTime = (index >= 0 && index < (int)pointTimesMs_.size()) ? pointTimesMs_[index] : -1.0;
	root->setProperty("cursorTime", cursorTime);
}

void ChartsPanel::appendLivePoint(const TripSamplePoint& point) {
	full_.ready = false;  // live points are not in full_; disable zoom-slice path
	if (!cache_.valid)
		buildSeriesCache();
	if (!cache_.valid)
		return;

	pointCount_++;
	double t = epochMillisFor(point.zuluTime);
	pointTimesMs_.push_back(t);

	QDateTime hi = QDateTime::fromMSecsSinceEpoch((qint64)t);
	if (cache_.xAxis) {
		if (pointTimesMs_.size() == 1)
			setAllXAxisRange(hi.addSecs(-1), hi);
		else
			setAllXAxisRange(QDateTime::fromMSecsSinceEpoch((qint64)pointTimesMs_.front()), hi);
	}

	double spd = qMax((double)point.airspeed, (double)point.groundSpeed);
	if (spd > liveSpeedMax_) {
		liveSpeedMax_ = spd;
		if (cache_.speedYAxis)
			cache_.speedYAxis->setMax(yAxisMax(liveSpeedMax_));
	}
	if (point.altitude > liveAltMax_) {
		liveAltMax_ = point.altitude;
		if (cache_.altYAxis)
			cache_.altYAxis->setMax(yAxisMax(liveAltMax_));
	}
	if (point.fuelTotalQuantityWeight > liveFuelMax_) {
		liveFuelMax_ = point.fuelTotalQuantityWeight;
		if (cache_.fuelYAxis)
			cache_.fuelYAxis->setMax(yAxisMax(liveFuelMax_));
	}

	if (!liveVsValid_) {
		liveVsMin_ = liveVsMax_ = point.verticalSpeed;
		liveVsValid_ = true;
		if (cache_.vsYAxis) {
			auto [lo, hi] = signedAxisRange(liveVsMin_, liveVsMax_);
			cache_.vsYAxis->setMin(lo); cache_.vsYAxis->setMax(hi);
		}
	} else if (point.verticalSpeed < liveVsMin_ || point.verticalSpeed > liveVsMax_) {
		liveVsMin_ = qMin(liveVsMin_, (double)point.verticalSpeed);
		liveVsMax_ = qMax(liveVsMax_, (double)point.verticalSpeed);
		if (cache_.vsYAxis) {
			auto [lo, hi] = signedAxisRange(liveVsMin_, liveVsMax_);
			cache_.vsYAxis->setMin(lo); cache_.vsYAxis->setMax(hi);
		}
	}

	cache_.n1_1->append(t, point.n1_1);
	cache_.n1_2->append(t, point.n1_2);
	cache_.n2_1->append(t, point.n2_1);
	cache_.n2_2->append(t, point.n2_2);
	cache_.verticalSpeed->append(t, point.verticalSpeed);
	cache_.airspeed->append(t, point.airspeed);
	cache_.groundSpeed->append(t, point.groundSpeed);
	cache_.altitude->append(t, point.altitude);
	cache_.gearHandle->append(t, point.gearHandlePosition);
	cache_.gearPos0->append(t, point.gearPosition[0]);
	cache_.gearPos1->append(t, point.gearPosition[1]);
	cache_.gearPos2->append(t, point.gearPosition[2]);
	cache_.gearOnGround0->append(t, point.gearOnGround[0] ? 1 : 0);
	cache_.gearOnGround1->append(t, point.gearOnGround[1] ? 1 : 0);
	cache_.gearOnGround2->append(t, point.gearOnGround[2] ? 1 : 0);
	cache_.brake->append(t, point.brakeIndicator);
	cache_.flaps->append(t, point.flapsHandleIndex);
	cache_.spoilers->append(t, point.spoilersHandlePosition);
	cache_.fuelWeight->append(t, point.fuelTotalQuantityWeight);
	if (!livePitchBankValid_) {
		livePitchMin_ = livePitchMax_ = point.pitchDegrees;
		liveBankMin_  = liveBankMax_  = point.bankDegrees;
		livePitchBankValid_ = true;
	} else {
		bool pitchChanged = (point.pitchDegrees < livePitchMin_ || point.pitchDegrees > livePitchMax_);
		bool bankChanged  = (point.bankDegrees  < liveBankMin_  || point.bankDegrees  > liveBankMax_);
		livePitchMin_ = qMin(livePitchMin_, point.pitchDegrees);
		livePitchMax_ = qMax(livePitchMax_, point.pitchDegrees);
		liveBankMin_  = qMin(liveBankMin_,  point.bankDegrees);
		liveBankMax_  = qMax(liveBankMax_,  point.bankDegrees);
		if (pitchChanged && cache_.pitchYAxis) {
			auto [lo, hi] = signedAxisRange(livePitchMin_, livePitchMax_);
			cache_.pitchYAxis->setMin(lo); cache_.pitchYAxis->setMax(hi);
		}
		if (bankChanged && cache_.bankYAxis) {
			auto [lo, hi] = signedAxisRange(liveBankMin_, liveBankMax_);
			cache_.bankYAxis->setMin(lo); cache_.bankYAxis->setMax(hi);
		}
	}
	cache_.pitch->append(t, point.pitchDegrees);
	cache_.bank->append(t, point.bankDegrees);
}

void ChartsPanel::setVisibleRange(int startIndex, int endIndex) {
	// Leaflet fires both zoomend and moveend on every zoom interaction -- skip
	// the second call when both events produce the same range.
	if (startIndex == lastRangeStart_ && endIndex == lastRangeEnd_)
		return;
	lastRangeStart_ = startIndex;
	lastRangeEnd_   = endIndex;

	QElapsedTimer rangeTimer; rangeTimer.start();
	buildSeriesCache();
	if (!cache_.valid || !cache_.xAxis)
		return;

	QQuickItem* root = view_->rootObject();
	if (root == nullptr)
		return;

	if (startIndex < 0 || endIndex < 0 || pointTimesMs_.empty()) {
		QDateTime lo = pointTimesMs_.empty()
			? QDateTime::currentDateTime()
			: QDateTime::fromMSecsSinceEpoch((qint64)pointTimesMs_.front());
		QDateTime hi = pointTimesMs_.empty() ? lo.addSecs(1) : QDateTime::fromMSecsSinceEpoch((qint64)pointTimesMs_.back());
		setAllXAxisRange(lo, hi);
		root->setProperty("isFullRangeVisible", true);
		if (liveVsValid_ && cache_.vsYAxis) {
			auto [vsLo, vsHi] = signedAxisRange(liveVsMin_, liveVsMax_);
			cache_.vsYAxis->setMin(vsLo); cache_.vsYAxis->setMax(vsHi);
		}
		if (cache_.speedYAxis)
			cache_.speedYAxis->setMax(yAxisMax(liveSpeedMax_));
		if (cache_.altYAxis)
			cache_.altYAxis->setMax(yAxisMax(liveAltMax_));
		if (cache_.fuelYAxis)
			cache_.fuelYAxis->setMax(yAxisMax(liveFuelMax_));
		if (livePitchBankValid_) {
			auto [pLo, pHi] = signedAxisRange(livePitchMin_, livePitchMax_);
			auto [bLo, bHi] = signedAxisRange(liveBankMin_,  liveBankMax_);
			if (cache_.pitchYAxis) { cache_.pitchYAxis->setMin(pLo); cache_.pitchYAxis->setMax(pHi); }
			if (cache_.bankYAxis)  { cache_.bankYAxis->setMin(bLo);  cache_.bankYAxis->setMax(bHi); }
		}

		// Reload the decimated view so Qt Graphs renders ~kDisplayPoints points
		// instead of the full-resolution zoomed slice that was previously loaded.
		if (full_.ready) {
			auto reloadDec = [&](QLineSeries* s, const QList<QPointF>& full) {
				if (!s || full.isEmpty()) return;
				s->clear();
				if (displayStride_ <= 1) { s->append(full); return; }
				QList<QPointF> dec;
				dec.reserve(full.size() / displayStride_ + 1);
				for (int i = 0; i < full.size(); i += displayStride_)
					dec.append(full[i]);
				s->append(dec);
			};
			reloadDec(cache_.n1_1,          full_.n1_1);
			reloadDec(cache_.n1_2,          full_.n1_2);
			reloadDec(cache_.n2_1,          full_.n2_1);
			reloadDec(cache_.n2_2,          full_.n2_2);
			reloadDec(cache_.verticalSpeed, full_.verticalSpeed);
			reloadDec(cache_.airspeed,      full_.airspeed);
			reloadDec(cache_.groundSpeed,   full_.groundSpeed);
			reloadDec(cache_.altitude,      full_.altitude);
			reloadDec(cache_.gearHandle,    full_.gearHandle);
			reloadDec(cache_.gearPos0,      full_.gearPos0);
			reloadDec(cache_.gearPos1,      full_.gearPos1);
			reloadDec(cache_.gearPos2,      full_.gearPos2);
			reloadDec(cache_.gearOnGround0, full_.gearOnGround0);
			reloadDec(cache_.gearOnGround1, full_.gearOnGround1);
			reloadDec(cache_.gearOnGround2, full_.gearOnGround2);
			reloadDec(cache_.brake,         full_.brake);
			reloadDec(cache_.flaps,         full_.flaps);
			reloadDec(cache_.spoilers,      full_.spoilers);
			reloadDec(cache_.fuelWeight,    full_.fuelWeight);
			reloadDec(cache_.pitch,         full_.pitch);
			reloadDec(cache_.bank,          full_.bank);
		}
	} else {
		startIndex = qBound(0, startIndex, (int)pointTimesMs_.size() - 1);
		endIndex = qBound(0, endIndex, (int)pointTimesMs_.size() - 1);
		int lo = qMin(startIndex, endIndex);
		int hi = qMax(startIndex, endIndex);
		double loMs = pointTimesMs_[lo];
		double hiMs = pointTimesMs_[hi];
		if (hiMs <= loMs)
			hiMs = loMs + 1000.0;
		setAllXAxisRange(QDateTime::fromMSecsSinceEpoch((qint64)loMs),
					 QDateTime::fromMSecsSinceEpoch((qint64)hiMs));
		root->setProperty("isFullRangeVisible", false);

		if (full_.ready) {
			double speedMax = 0.0, altMax = 0.0, fuelMax = 0.0;
			double sliceVsMin = 0.0, sliceVsMax = 0.0;
			bool firstVS = true;
			for (int i = lo; i <= hi; ++i) {
				if (i < full_.airspeed.size())   speedMax = qMax(speedMax, full_.airspeed[i].y());
				if (i < full_.groundSpeed.size()) speedMax = qMax(speedMax, full_.groundSpeed[i].y());
				if (i < full_.altitude.size())    altMax   = qMax(altMax,   full_.altitude[i].y());
				if (i < full_.fuelWeight.size())  fuelMax  = qMax(fuelMax,  full_.fuelWeight[i].y());
				if (i < full_.verticalSpeed.size()) {
					double v = full_.verticalSpeed[i].y();
					if (firstVS) { sliceVsMin = sliceVsMax = v; firstVS = false; }
					else { sliceVsMin = qMin(sliceVsMin, v); sliceVsMax = qMax(sliceVsMax, v); }
				}
			}
			if (!firstVS && cache_.vsYAxis) {
				auto [vsLo, vsHi] = signedAxisRange(sliceVsMin, sliceVsMax);
				cache_.vsYAxis->setMin(vsLo); cache_.vsYAxis->setMax(vsHi);
			}
			if (cache_.speedYAxis)
				cache_.speedYAxis->setMax(yAxisMax(speedMax));
			if (cache_.altYAxis)
				cache_.altYAxis->setMax(yAxisMax(altMax));
			if (cache_.fuelYAxis)
				cache_.fuelYAxis->setMax(yAxisMax(fuelMax));
			double slicePitchMin = 0.0, slicePitchMax = 0.0;
			double sliceBankMin  = 0.0, sliceBankMax  = 0.0;
			bool firstPB = true;
			for (int i = lo; i <= hi; ++i) {
				if (i < full_.pitch.size()) {
					double v = full_.pitch[i].y();
					if (firstPB) { slicePitchMin = slicePitchMax = v; }
					else { slicePitchMin = qMin(slicePitchMin, v); slicePitchMax = qMax(slicePitchMax, v); }
				}
				if (i < full_.bank.size()) {
					double v = full_.bank[i].y();
					if (firstPB) { sliceBankMin = sliceBankMax = v; firstPB = false; }
					else { sliceBankMin = qMin(sliceBankMin, v); sliceBankMax = qMax(sliceBankMax, v); }
				}
				firstPB = false;
			}
			if (!firstPB) {
				auto [pLo, pHi] = signedAxisRange(slicePitchMin, slicePitchMax);
				auto [bLo, bHi] = signedAxisRange(sliceBankMin,  sliceBankMax);
				if (cache_.pitchYAxis) { cache_.pitchYAxis->setMin(pLo); cache_.pitchYAxis->setMax(pHi); }
				if (cache_.bankYAxis)  { cache_.bankYAxis->setMin(bLo);  cache_.bankYAxis->setMax(bHi); }
			}
		}

		// Load a decimated view of the visible slice — same kDisplayPoints cap
		// as the full-range view. A large but partial viewport (half the flight)
		// would otherwise dump 25k raw points into each series; stride keeps
		// rendering cost constant regardless of zoom level. stride=1 (full res)
		// is only used when the visible window is small enough to fit in budget.
		if (full_.ready) {
			int count = hi - lo + 1;
			int sliceStride = std::max(1, (count + kDisplayPoints - 1) / kDisplayPoints);
			auto loadSliceDec = [&](QLineSeries* s, const QList<QPointF>& full) {
				if (!s || hi >= full.size()) return;
				s->clear();
				if (sliceStride <= 1) {
					s->append(full.mid(lo, count));
					return;
				}
				QList<QPointF> dec;
				dec.reserve(count / sliceStride + 2);
				for (int i = lo; i <= hi; i += sliceStride)
					dec.append(full[i]);
				if ((hi - lo) % sliceStride != 0)
					dec.append(full[hi]);
				s->append(dec);
			};
			loadSliceDec(cache_.n1_1,          full_.n1_1);
			loadSliceDec(cache_.n1_2,          full_.n1_2);
			loadSliceDec(cache_.n2_1,          full_.n2_1);
			loadSliceDec(cache_.n2_2,          full_.n2_2);
			loadSliceDec(cache_.verticalSpeed, full_.verticalSpeed);
			loadSliceDec(cache_.airspeed,      full_.airspeed);
			loadSliceDec(cache_.groundSpeed,   full_.groundSpeed);
			loadSliceDec(cache_.altitude,      full_.altitude);
			loadSliceDec(cache_.gearHandle,    full_.gearHandle);
			loadSliceDec(cache_.gearPos0,      full_.gearPos0);
			loadSliceDec(cache_.gearPos1,      full_.gearPos1);
			loadSliceDec(cache_.gearPos2,      full_.gearPos2);
			loadSliceDec(cache_.gearOnGround0, full_.gearOnGround0);
			loadSliceDec(cache_.gearOnGround1, full_.gearOnGround1);
			loadSliceDec(cache_.gearOnGround2, full_.gearOnGround2);
			loadSliceDec(cache_.brake,         full_.brake);
			loadSliceDec(cache_.flaps,         full_.flaps);
			loadSliceDec(cache_.spoilers,      full_.spoilers);
			loadSliceDec(cache_.fuelWeight,    full_.fuelWeight);
			loadSliceDec(cache_.pitch,         full_.pitch);
			loadSliceDec(cache_.bank,          full_.bank);
		}
	}
	Logger::logf(Logger::Profile, "Charts", "setVisibleRange: %lld µs  (%d pts in series, stride %d)",
	             rangeTimer.nsecsElapsed() / 1000,
	             (startIndex < 0 || endIndex < 0 || pointTimesMs_.empty())
	                 ? (pointCount_ / std::max(1, displayStride_))
	                 : std::min(qAbs(lastRangeEnd_ - lastRangeStart_) + 1, kDisplayPoints),
	             (startIndex < 0 || endIndex < 0 || pointTimesMs_.empty())
	                 ? displayStride_
	                 : std::max(1, (qAbs(lastRangeEnd_ - lastRangeStart_) + 1) / kDisplayPoints));
}

QVariantMap ChartsPanel::valueAt(double timeMs) const {
	if (pointTimesMs_.empty())
		return {};

	// Find the sample index closest to timeMs.
	auto it = std::lower_bound(pointTimesMs_.begin(), pointTimesMs_.end(), timeMs);
	int idx = (int)(it - pointTimesMs_.begin());
	if (idx >= (int)pointTimesMs_.size())
		idx = (int)pointTimesMs_.size() - 1;
	// lower_bound lands on the first element >= timeMs; check if idx-1 is closer.
	if (idx > 0 && (pointTimesMs_[idx] - timeMs > timeMs - pointTimesMs_[idx - 1]))
		idx--;

	QString timeStr = QDateTime::fromMSecsSinceEpoch((qint64)pointTimesMs_[idx])
	                      .toString(QStringLiteral("HH:mm:ss.zzz"))
	                  + QStringLiteral(" UTC");

	auto fromFull = [&](const QList<QPointF>& v) -> double {
		return (idx < v.size()) ? v[idx].y() : 0.0;
	};
	auto fromSeries = [&](const QLineSeries* s) -> double {
		if (!s) return 0.0;
		auto pts = s->points();
		return (idx < pts.size()) ? pts[idx].y() : 0.0;
	};

	QVariantMap m;
	m[QStringLiteral("timeStr")] = timeStr;

	if (full_.ready) {
		m[QStringLiteral("n1_1")]       = fromFull(full_.n1_1);
		m[QStringLiteral("n1_2")]       = fromFull(full_.n1_2);
		m[QStringLiteral("n2_1")]       = fromFull(full_.n2_1);
		m[QStringLiteral("n2_2")]       = fromFull(full_.n2_2);
		m[QStringLiteral("vs")]         = fromFull(full_.verticalSpeed);
		m[QStringLiteral("ias")]        = fromFull(full_.airspeed);
		m[QStringLiteral("gs")]         = fromFull(full_.groundSpeed);
		m[QStringLiteral("alt")]        = fromFull(full_.altitude);
		m[QStringLiteral("gearHandle")] = fromFull(full_.gearHandle);
		m[QStringLiteral("gearPos0")]   = fromFull(full_.gearPos0);
		m[QStringLiteral("gearPos1")]   = fromFull(full_.gearPos1);
		m[QStringLiteral("gearPos2")]   = fromFull(full_.gearPos2);
		m[QStringLiteral("onGnd0")]     = fromFull(full_.gearOnGround0) > 0.5;
		m[QStringLiteral("onGnd1")]     = fromFull(full_.gearOnGround1) > 0.5;
		m[QStringLiteral("onGnd2")]     = fromFull(full_.gearOnGround2) > 0.5;
		m[QStringLiteral("brake")]      = fromFull(full_.brake);
		m[QStringLiteral("flaps")]      = fromFull(full_.flaps);
		m[QStringLiteral("spoilers")]   = fromFull(full_.spoilers);
		m[QStringLiteral("fuel")]       = fromFull(full_.fuelWeight);
		m[QStringLiteral("pitch")]      = fromFull(full_.pitch);
		m[QStringLiteral("bank")]       = fromFull(full_.bank);
	} else if (cache_.valid) {
		// Live mode: cache series are appended one-by-one (no decimation), so
		// their point index matches pointTimesMs_ directly.
		m[QStringLiteral("n1_1")]       = fromSeries(cache_.n1_1);
		m[QStringLiteral("n1_2")]       = fromSeries(cache_.n1_2);
		m[QStringLiteral("n2_1")]       = fromSeries(cache_.n2_1);
		m[QStringLiteral("n2_2")]       = fromSeries(cache_.n2_2);
		m[QStringLiteral("vs")]         = fromSeries(cache_.verticalSpeed);
		m[QStringLiteral("ias")]        = fromSeries(cache_.airspeed);
		m[QStringLiteral("gs")]         = fromSeries(cache_.groundSpeed);
		m[QStringLiteral("alt")]        = fromSeries(cache_.altitude);
		m[QStringLiteral("gearHandle")] = fromSeries(cache_.gearHandle);
		m[QStringLiteral("gearPos0")]   = fromSeries(cache_.gearPos0);
		m[QStringLiteral("gearPos1")]   = fromSeries(cache_.gearPos1);
		m[QStringLiteral("gearPos2")]   = fromSeries(cache_.gearPos2);
		m[QStringLiteral("onGnd0")]     = fromSeries(cache_.gearOnGround0) > 0.5;
		m[QStringLiteral("onGnd1")]     = fromSeries(cache_.gearOnGround1) > 0.5;
		m[QStringLiteral("onGnd2")]     = fromSeries(cache_.gearOnGround2) > 0.5;
		m[QStringLiteral("brake")]      = fromSeries(cache_.brake);
		m[QStringLiteral("flaps")]      = fromSeries(cache_.flaps);
		m[QStringLiteral("spoilers")]   = fromSeries(cache_.spoilers);
		m[QStringLiteral("fuel")]       = fromSeries(cache_.fuelWeight);
		m[QStringLiteral("pitch")]      = fromSeries(cache_.pitch);
		m[QStringLiteral("bank")]       = fromSeries(cache_.bank);
	}
	return m;
}
