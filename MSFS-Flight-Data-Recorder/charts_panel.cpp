#include "charts_panel.h"

#include <QQuickWidget>
#include <QQuickItem>
#include <QVBoxLayout>
#include <QUrl>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>

#include <QtGraphs/qlineseries.h>
#include <QtGraphs/qdatetimeaxis.h>

namespace {

QLineSeries* findSeries(QQuickItem* root, const char* objectName) {
	return root->findChild<QLineSeries*>(QString::fromLatin1(objectName));
}

QDateTimeAxis* findXAxis(QQuickItem* root, const char* objectName) {
	return root->findChild<QDateTimeAxis*>(QString::fromLatin1(objectName));
}

// Lightweight copy of one TripSamplePoint carrying only the fields the chart
// worker needs. Skips allFields (50+ QString pairs per point) which is the
// bulk of the copy cost -- 10k points × 50 pairs × 2 QStrings = ~1M atomic
// ref-count ops at 100 ms for a flight with 10k samples.
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
	view_ = new QQuickWidget(this);
	view_->setResizeMode(QQuickWidget::SizeRootObjectToView);
	view_->setSource(QUrl(QStringLiteral("qrc:/charts/charts_panel.qml")));

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(view_);
}

double ChartsPanel::epochMillisFor(const QString& zuluTime) {
	// Format: "yyyy-MM-ddTHH:mm:ss.zzz..." (fixed positions, always ASCII digits).
	// QDateTime::fromString with a format string does locale-aware parsing and
	// costs ~0.7 ms per call -- ~3.7 s for 5k points. Parse directly from char
	// positions instead: no allocation, no format scanning, no locale lookup.
	if (zuluTime.size() < 23) return 0.0;
	const QChar* c = zuluTime.constData();
	auto d1 = [&](int i) { return c[i].digitValue(); };
	int year  = d1(0)*1000 + d1(1)*100 + d1(2)*10 + d1(3);
	int month = d1(5)*10  + d1(6);
	int day   = d1(8)*10  + d1(9);
	int hour  = d1(11)*10 + d1(12);
	int min   = d1(14)*10 + d1(15);
	int sec   = d1(17)*10 + d1(18);
	int ms    = d1(20)*100 + d1(21)*10 + d1(22);
	// QDateTimeAxis renders epoch-ms values in local timezone regardless of axis
	// timezone (Qt 6.11 bug). Build a LocalTime QDateTime from the raw Zulu
	// components so the axis labels display the correct Zulu values.
	return (double)QDateTime(QDate(year, month, day), QTime(hour, min, sec, ms))
	                   .toMSecsSinceEpoch();
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
	cache_.valid         = (cache_.n1_1 != nullptr);
}

void ChartsPanel::setDataset(const TripDataset& dataset) {
	QQuickItem* root = view_->rootObject();
	if (root == nullptr) {
		emit seriesLoaded();
		return;
	}

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

	auto* watcher = new QFutureWatcher<ChartSeriesData>(this);
	connect(watcher, &QFutureWatcher<ChartSeriesData>::finished, this, [this, watcher]() {
		const ChartSeriesData data = watcher->result();
		watcher->deleteLater();

		QQuickItem* root = view_->rootObject();
		if (root == nullptr)
			return;

		pointTimesMs_ = data.pointTimesMs;
		pointCount_ = (int)data.pointTimesMs.size();

		// Drive the axis via the "driver" DateTimeAxis (found by objectName).
		// Each chart's per-chart axis binds to it in QML, keeping all in sync.
		if (QDateTimeAxis* xAxis = findXAxis(root, "sharedXAxis")) {
			xAxis->setMin(data.axisLo);
			xAxis->setMax(data.axisHi);
		}
		root->setProperty("isFullRangeVisible", true);

		auto apply = [&](const char* seriesName, const QList<QPointF>& values) {
			QLineSeries* series = findSeries(root, seriesName);
			if (series == nullptr)
				return;
			series->clear();
			series->append(values);
		};

		apply("n1_1Series", data.n1_1);
		apply("n1_2Series", data.n1_2);
		apply("n2_1Series", data.n2_1);
		apply("n2_2Series", data.n2_2);
		apply("verticalSpeedSeries", data.verticalSpeed);
		apply("airspeedSeries", data.airspeed);
		apply("groundSpeedSeries", data.groundSpeed);
		apply("altitudeSeries", data.altitude);
		apply("gearHandleSeries", data.gearHandle);
		apply("gearPosition0Series", data.gearPos0);
		apply("gearPosition1Series", data.gearPos1);
		apply("gearPosition2Series", data.gearPos2);
		apply("gearOnGround0Series", data.gearOnGround0);
		apply("gearOnGround1Series", data.gearOnGround1);
		apply("gearOnGround2Series", data.gearOnGround2);
		apply("brakeSeries", data.brake);
		apply("flapsSeries", data.flaps);
		apply("spoilersSeries", data.spoilers);
		apply("fuelWeightSeries", data.fuelWeight);

		buildSeriesCache();
		emit seriesLoaded();
	});

	watcher->setFuture(QtConcurrent::run([this, points = std::move(points)]() {
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
			cache_.xAxis->setMin(hi.addSecs(-1));
		cache_.xAxis->setMax(hi);
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
	} else {
		startIndex = qBound(0, startIndex, (int)pointTimesMs_.size() - 1);
		endIndex = qBound(0, endIndex, (int)pointTimesMs_.size() - 1);
		double loMs = pointTimesMs_[qMin(startIndex, endIndex)];
		double hiMs = pointTimesMs_[qMax(startIndex, endIndex)];
		if (hiMs <= loMs)
			hiMs = loMs + 1000.0;
		cache_.xAxis->setMin(QDateTime::fromMSecsSinceEpoch((qint64)loMs));
		cache_.xAxis->setMax(QDateTime::fromMSecsSinceEpoch((qint64)hiMs));
		root->setProperty("isFullRangeVisible", false);
	}
}
