#include "charts_panel.h"

#include <QQuickWidget>
#include <QQuickItem>
#include <QVBoxLayout>
#include <QUrl>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>
#include <QDebug>
#include <QElapsedTimer>

#include <QtGraphs/qlineseries.h>
#include <QtGraphs/qdatetimeaxis.h>
#include <QtGraphs/qvalueaxis.h>

#include <QtMath>

namespace {

// Convert a "yyyy-MM-ddTHH:mm:ss.zzz..." zulu-time string to epoch milliseconds,
// adjusted so that Qt's buggy DateTimeAxis (which renders epoch-ms as local wall
// time instead of UTC) displays the correct Zulu values.
//
// The original implementation used QDateTime(QDate,QTime).toMSecsSinceEpoch()
// which triggers a Windows timezone DST lookup on every call (~16 µs each,
// ~800 ms for 50k points). This version does equivalent pure arithmetic:
//   1. Gregorian → Julian Day Number (integer formula, no system calls)
//   2. JDN + time → UTC epoch ms
//   3. Subtract localOffsetMs so Qt's local-time display shows the Zulu values
static double epochMillisFor(const QString& zuluTime, qint64 localOffsetMs) {
	if (zuluTime.size() < 23) return 0.0;
	const QChar* c = zuluTime.constData();
	auto d1 = [&](int i) { return c[i].digitValue(); };
	int y  = d1(0)*1000 + d1(1)*100 + d1(2)*10 + d1(3);
	int mo = d1(5)*10  + d1(6);
	int dy = d1(8)*10  + d1(9);
	int h  = d1(11)*10 + d1(12);
	int mi = d1(14)*10 + d1(15);
	int s  = d1(17)*10 + d1(18);
	int ms = d1(20)*100 + d1(21)*10 + d1(22);
	// Proleptic Gregorian calendar → Julian Day Number (no Qt/system calls)
	int a  = (14 - mo) / 12;
	int yy = y + 4800 - a;
	int mm = mo + 12*a - 3;
	qint64 jd = (qint64)dy + (153*mm + 2)/5 + 365LL*yy + yy/4 - yy/100 + yy/400 - 32045LL;
	const qint64 kUnixEpochJd = 2440588LL;
	qint64 utcMs = (jd - kUnixEpochJd) * 86400000LL
	             + (qint64)h * 3600000LL + (qint64)mi * 60000LL + (qint64)s * 1000LL + ms;
	return (double)(utcMs - localOffsetMs);
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

// Round value up to the nearest multiple of step with 10% headroom.
// Returns the default fallback if value <= 0.
static double yAxisMax(double value, double step, double fallback) {
	if (value <= 0.0)
		return fallback;
	return qCeil(value * 1.25 / step) * step;
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
	double speedYMax = 0.0;
	double fuelYMax  = 0.0;
	qint64 computeNs = 0;
	QList<QPointF> n1_1, n1_2, n2_1, n2_2;
	QList<QPointF> verticalSpeed;
	QList<QPointF> airspeed, groundSpeed;
	QList<QPointF> altitude;
	QList<QPointF> gearHandle, gearPos0, gearPos1, gearPos2, gearOnGround0, gearOnGround1, gearOnGround2;
	QList<QPointF> brake, flaps, spoilers;
	QList<QPointF> fuelWeight;
};

}

ChartsPanel::ChartsPanel(QWidget* parent) : QWidget(parent) {
	localOffsetMs_ = (qint64)QDateTime::currentDateTime().offsetFromUtc() * 1000LL;
	view_ = new QQuickWidget(this);
	view_->setResizeMode(QQuickWidget::SizeRootObjectToView);
	view_->setSource(QUrl(QStringLiteral("qrc:/charts/charts_panel.qml")));

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(view_);
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
	cache_.xAxis         = findXAxis(root, "sharedXAxis");
	cache_.speedYAxis    = findYAxis(root, "speedYAxis");
	cache_.fuelYAxis     = findYAxis(root, "fuelYAxis");
	cache_.valid         = (cache_.n1_1 != nullptr);
}

void ChartsPanel::setDataset(const TripDataset& dataset) {
	liveSpeedMax_   = 0.0;
	liveFuelMax_    = 0.0;
	localOffsetMs_  = (qint64)QDateTime::currentDateTime().offsetFromUtc() * 1000LL;
	full_.ready      = false;
	lastRangeStart_  = INT_MIN;
	lastRangeEnd_    = INT_MIN;

	QQuickItem* root = view_->rootObject();
	if (root == nullptr) {
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
		points.push_back(std::move(lp));
	}
	qDebug("[Gen/Chrt] copy: %lld ms  (%zu pts)", copyTimer.nsecsElapsed() / 1000000, points.size());

	auto* watcher = new QFutureWatcher<ChartSeriesData>(this);
	connect(watcher, &QFutureWatcher<ChartSeriesData>::finished, this, [this, watcher]() {
		// Non-const so we can std::move the QList members into full_ below,
		// avoiding a second 15 MB copy into the QLineSeries objects.
		ChartSeriesData data = watcher->result();
		watcher->deleteLater();
		qDebug("[Gen/Chrt] compute (bg): %lld ms", data.computeNs / 1000000);

		QElapsedTimer applyTimer; applyTimer.start();
		QQuickItem* root = view_->rootObject();
		if (root == nullptr)
			return;

		pointTimesMs_ = std::move(data.pointTimesMs);
		pointCount_ = (int)pointTimesMs_.size();

		// Drive the axis via the "driver" DateTimeAxis (found by objectName).
		// Each chart's per-chart axis binds to it in QML, keeping all in sync.
		if (QDateTimeAxis* xAxis = findXAxis(root, "sharedXAxis")) {
			xAxis->setMin(data.axisLo);
			xAxis->setMax(data.axisHi);
		}
		if (QValueAxis* ax = findYAxis(root, "speedYAxis"))
			ax->setMax(yAxisMax(data.speedYMax, 50.0, 400.0));
		if (QValueAxis* ax = findYAxis(root, "fuelYAxis"))
			ax->setMax(yAxisMax(data.fuelYMax, 1000.0, 30000.0));
		liveSpeedMax_ = data.speedYMax;
		liveFuelMax_  = data.fuelYMax;
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
		full_.ready = true;

		// Load a decimated view into each series. Qt Graphs renders every
		// loaded QPointF per frame; keeping this to ~kDisplayPoints points
		// makes the initial paint and subsequent zoom-out fast.
		buildSeriesCache();
		auto loadDec = [&](QLineSeries* s, const QList<QPointF>& full) {
			if (!s) return;
			s->clear();
			if (displayStride_ <= 1) { s->append(full); return; }
			QList<QPointF> dec;
			dec.reserve(full.size() / displayStride_ + 1);
			for (int i = 0; i < full.size(); i += displayStride_)
				dec.append(full[i]);
			s->append(dec);
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

		qDebug("[Gen/Chrt] apply (GUI): %lld ms  (stride %d, ~%d pts/series)",
		       applyTimer.nsecsElapsed() / 1000000, displayStride_,
		       pointCount_ / std::max(1, displayStride_));
		emit seriesLoaded();
	});

	watcher->setFuture(QtConcurrent::run([this, points = std::move(points), offsetMs = localOffsetMs_]() {
		QElapsedTimer computeTimer; computeTimer.start();
		ChartSeriesData data;
		data.pointTimesMs.reserve(points.size());
		for (const LightPoint& p : points)
			data.pointTimesMs.push_back(epochMillisFor(p.zuluTime, offsetMs));

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

		for (const LightPoint& p : points) {
			double spd = qMax((double)p.airspeed, (double)p.groundSpeed);
			if (spd > data.speedYMax)
				data.speedYMax = spd;
			if (p.fuelTotalQuantityWeight > data.fuelYMax)
				data.fuelYMax = p.fuelTotalQuantityWeight;
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
	double t = epochMillisFor(point.zuluTime, localOffsetMs_);
	pointTimesMs_.push_back(t);

	QDateTime hi = QDateTime::fromMSecsSinceEpoch((qint64)t);
	if (cache_.xAxis) {
		if (pointTimesMs_.size() == 1)
			cache_.xAxis->setMin(hi.addSecs(-1));
		cache_.xAxis->setMax(hi);
	}

	double spd = qMax((double)point.airspeed, (double)point.groundSpeed);
	if (spd > liveSpeedMax_) {
		liveSpeedMax_ = spd;
		if (cache_.speedYAxis)
			cache_.speedYAxis->setMax(yAxisMax(liveSpeedMax_, 50.0, 400.0));
	}
	if (point.fuelTotalQuantityWeight > liveFuelMax_) {
		liveFuelMax_ = point.fuelTotalQuantityWeight;
		if (cache_.fuelYAxis)
			cache_.fuelYAxis->setMax(yAxisMax(liveFuelMax_, 1000.0, 30000.0));
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
		cache_.xAxis->setMin(lo);
		cache_.xAxis->setMax(hi);
		root->setProperty("isFullRangeVisible", true);

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
		cache_.xAxis->setMin(QDateTime::fromMSecsSinceEpoch((qint64)loMs));
		cache_.xAxis->setMax(QDateTime::fromMSecsSinceEpoch((qint64)hiMs));
		root->setProperty("isFullRangeVisible", false);

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
		}
	}
	qDebug("[Range   ] setVisibleRange: %lld µs  (%d pts in series, stride %d)",
	       rangeTimer.nsecsElapsed() / 1000,
	       (startIndex < 0 || endIndex < 0 || pointTimesMs_.empty())
	           ? (pointCount_ / std::max(1, displayStride_))
	           : std::min(qAbs(lastRangeEnd_ - lastRangeStart_) + 1, kDisplayPoints),
	       (startIndex < 0 || endIndex < 0 || pointTimesMs_.empty())
	           ? displayStride_
	           : std::max(1, (qAbs(lastRangeEnd_ - lastRangeStart_) + 1) / kDisplayPoints));
}
