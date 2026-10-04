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

#include <QtGraphs/qlineseries.h>
#include <QtGraphs/qdatetimeaxis.h>
#include <QtGraphs/qvalueaxis.h>


namespace {

QValueAxis* findYAxis(QQuickItem* root, const char* objectName) {
	return root->findChild<QValueAxis*>(QString::fromLatin1(objectName));
}

void setAxisRange(QValueAxis* axis, std::pair<double, double> range) {
	if (!axis)
		return;
	axis->setMin(range.first);
	axis->setMax(range.second);
}

}

ChartsPanel::ChartsPanel(QWidget* parent) : QWidget(parent) {
	// fdr_core is a static library: Qt only auto-registers a .qrc's compiled
	// resource data when it's linked into an executable/shared library
	// directly, so charts.qrc needs this explicit init here (ran from both
	// the app and any test linking fdr_core) or qrc:/charts/... 404s.
	Q_INIT_RESOURCE(charts);
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
	for (int s = 0; s < CHART_SERIES_COUNT; ++s)
		cache_.series[s] = root->findChild<QLineSeries*>(QString::fromLatin1(CHART_SERIES[s].objectName));
	cache_.xAxis      = root->findChild<QDateTimeAxis*>(QStringLiteral("sharedXAxis"));
	cache_.engSpeedYAxis = findYAxis(root, "engSpeedYAxis");
	cache_.engLoadYAxis  = findYAxis(root, "engLoadYAxis");
	cache_.vsYAxis    = findYAxis(root, "vsYAxis");
	cache_.speedYAxis = findYAxis(root, "speedYAxis");
	cache_.altYAxis   = findYAxis(root, "altYAxis");
	cache_.fuelYAxis  = findYAxis(root, "fuelYAxis");
	cache_.pitchYAxis = findYAxis(root, "pitchYAxis");
	cache_.bankYAxis  = findYAxis(root, "bankYAxis");
	cache_.valid      = (cache_.series[CHART_ENG_SPEED_1] != nullptr);
}

void ChartsPanel::setYAxes(const ChartExtents& extents) {
	// No spec (no power recorded): sized to the data like the other axes, so
	// the previous trip's engine scale doesn't stay. A fixed max that the data
	// goes past (an overspeed) is sized to the data too, not cut off.
	const EnginePowerSpec* spec = engine_.count > 0 ? enginePowerSpec(engine_.engineType) : nullptr;
	const auto engineAxisMax = [](double fixedMax, double dataMax) {
		return fixedMax > 0 && dataMax <= fixedMax ? fixedMax : niceAxisMax(dataMax);
	};
	if (cache_.engSpeedYAxis)
		cache_.engSpeedYAxis->setMax(engineAxisMax(spec ? spec->speed.axisMax : 0, extents.engSpeedMax));
	if (cache_.engLoadYAxis)
		cache_.engLoadYAxis->setMax(engineAxisMax(spec ? spec->load.axisMax : 0, extents.engLoadMax));
	if (cache_.speedYAxis)
		cache_.speedYAxis->setMax(niceAxisMax(extents.speedMax));
	if (cache_.altYAxis)
		cache_.altYAxis->setMax(niceAxisMax(extents.altMax));
	if (cache_.fuelYAxis)
		cache_.fuelYAxis->setMax(niceAxisMax(extents.fuelMax));
	if (!extents.valid)
		return;
	setAxisRange(cache_.vsYAxis, niceSignedAxisRange(extents.vsMin, extents.vsMax));
	setAxisRange(cache_.pitchYAxis, niceSignedAxisRange(extents.pitchMin, extents.pitchMax));
	setAxisRange(cache_.bankYAxis, niceSignedAxisRange(extents.bankMin, extents.bankMax));
}

void ChartsPanel::setEngine(const EnginePower& engine) {
	engine_ = engine;
	if (QQuickItem* root = view_->rootObject())
		root->setProperty("engineSpec", chartEngineSpec(engine));
}

void ChartsPanel::loadFullSlice(int lo, int hi) {
	// replace() swaps all points in one scene-graph notification; clear() +
	// append() would send two.
	for (int s = 0; s < CHART_SERIES_COUNT; ++s) {
		QLineSeries* series = cache_.series[s];
		if (!series)
			continue;
		const QList<QPointF> points = decimateSeries(full_[s], lo, hi, kDisplayPoints);
		if (!points.isEmpty())
			series->replace(points);
	}
}

void ChartsPanel::setDataset(const TripDataset& dataset) {
	++datasetVersion_;
	fullExtents_ = ChartExtents();
	loading_         = false;
	pendingRange_.reset();  // meant for the trip being replaced
	pendingCursorIndex_ = -1;
	lastRangeStart_  = INT_MIN;
	lastRangeEnd_    = INT_MIN;

	QQuickItem* root = view_->rootObject();
	if (root == nullptr) {
		Logger::log(Logger::Trace, "Charts", QStringLiteral("setDataset: QML root not ready yet; skipping series update, emitting seriesLoaded immediately"));
		emit seriesLoaded();
		return;
	}
	// The cursor belongs to the previous dataset. MapWidget::setDataset resets
	// its own cursor without emitting cursorIndexChanged, so clear it here or
	// the old cursor line stays drawn over a reloaded or overlapping trip.
	// The old lines and engineSpec (their labels) stay until a new trip's
	// load finishes and replaces both together.
	setCursorIndex(-1);

	// Empty dataset (Deselect / overview mode, or a trip with no point): drop
	// every line, and with engineSpec unset the QML hides the axes and shows
	// "No trip selected" in each chart -- or "No data recorded" for a trip
	// (tripId set). Clearing doesn't wait on rendering: after a
	// 200,000-sample trip (62,525 points shown, as each line is thinned to
	// kDisplayPoints) it took 0.3-0.9 ms of a 10-17 ms deselect, most of which is the QML hiding
	// the axes. The Profile "clear" log reports it.
	if (dataset.points.empty()) {
		Logger::log(Logger::Trace, "Charts", QStringLiteral("setDataset: empty dataset (Deselect/overview, or a trip with no point); clearing every series"));
		pointTimesMs_.clear();
		root->setProperty("engineSpec", QVariant());
		root->setProperty("noDataText", dataset.tripId >= 0 ? QStringLiteral("No data recorded") : QStringLiteral("No trip selected"));
		buildSeriesCache();
		QElapsedTimer clearTimer; clearTimer.start();
		for (QLineSeries* series : cache_.series) {
			if (series)
				series->clear();
		}
		Logger::logf(Logger::Profile, "Charts", "clear: %lld µs", clearTimer.nsecsElapsed() / 1000);
		emit seriesLoaded();
		return;
	}
	// Shown only while no old lines are left to show (e.g. after a deselect).
	root->setProperty("noDataText", QStringLiteral("Loading…"));
	loading_ = true;

	QElapsedTimer copyTimer; copyTimer.start();
	std::vector<ChartSample> samples;
	samples.reserve(dataset.points.size());
	for (const TripSamplePoint& p : dataset.points)
		samples.push_back({ p.zuluTime, chartValues(p) });
	Logger::logf(Logger::Profile, "Charts", "copy: %lld ms  (%zu pts)", copyTimer.nsecsElapsed() / 1000000, samples.size());
	const EnginePower engine = chartEngine(dataset.points);

	int ver = datasetVersion_;
	auto* watcher = new QFutureWatcher<ChartSeriesData>(this);
	connect(watcher, &QFutureWatcher<ChartSeriesData>::finished, this, [this, watcher, ver, engine]() {
		Logger::logf(Logger::Profile, "Charts", "finished lambda: ver=%d cur=%d", ver, datasetVersion_);
		watcher->deleteLater();
		// A newer setDataset call superseded this one — discard stale results
		// rather than writing old trip data into series that were already cleared.
		if (ver != datasetVersion_) {
			Logger::logf(Logger::Trace, "Charts", "setDataset: dataset superseded (ver=%d cur=%d); discarding stale computed series", ver, datasetVersion_);
			return;
		}
		loading_ = false;
		// Non-const so the point lists can be moved into full_ below, avoiding
		// a second 15 MB copy.
		ChartSeriesData data = watcher->result();

		QElapsedTimer applyTimer; applyTimer.start();
		QQuickItem* root = view_->rootObject();
		if (root == nullptr) {
			// Matches the synchronous early-return above: always emit seriesLoaded()
			// so TrajectoryView::pendingRenders_ reaches 0 even if the QML root was
			// torn down while this background computation was in flight, instead of
			// leaving the trip table permanently disabled with a stuck loading spinner.
			Logger::log(Logger::Trace, "Charts", QStringLiteral("setDataset: QML root torn down while computing series (bg); discarding result, emitting seriesLoaded"));
			emit seriesLoaded();
			return;
		}

		pointTimesMs_ = std::move(data.pointTimesMs);
		const int pointCount = (int)pointTimesMs_.size();

		buildSeriesCache();
		setAllXAxisRange(data.axisLo, data.axisHi);
		fullExtents_ = data.extents;
		setEngine(engine);
		setYAxes(fullExtents_);
		root->setProperty("isFullRangeVisible", true);

		// Keep the full-resolution points in full_ so setVisibleRange can serve
		// exact slices on zoom, and load only a thinned view into the series:
		// Qt Graphs renders every loaded point per frame.
		full_ = std::move(data.series);
		loadFullSlice(0, pointCount - 1);

		Logger::logf(Logger::Profile, "Charts", "apply (GUI): %lld ms  (%d pts, ~%d pts/series shown)",
		             applyTimer.nsecsElapsed() / 1000000, pointCount, qMin(pointCount, kDisplayPoints));
		// What was just loaded is what setVisibleRange(-1, -1) shows, so a
		// full range (sent while loading or after) is a repeat; a zoom isn't.
		lastRangeStart_ = lastRangeEnd_ = -1;
		if (const auto range = std::exchange(pendingRange_, std::nullopt))
			setVisibleRange(range->first, range->second);
		setCursorIndex(std::exchange(pendingCursorIndex_, -1));
		emit seriesLoaded();
	});

	watcher->setFuture(QtConcurrent::run([samples = std::move(samples)]() {
		QElapsedTimer computeTimer; computeTimer.start();
		ChartSeriesData data = buildChartSeries(samples);
		Logger::logf(Logger::Profile, "Charts", "compute (bg): %lld ms", computeTimer.nsecsElapsed() / 1000000);
		return data;
	}));
}

void ChartsPanel::setCursorIndex(int index) {
	QQuickItem* root = view_->rootObject();
	if (root == nullptr)
		return;
	// Loading: kept for when the lines are in (see loading_).
	if (loading_) {
		pendingCursorIndex_ = index;
		return;
	}
	double cursorTime = (index >= 0 && index < (int)pointTimesMs_.size()) ? pointTimesMs_[index] : -1.0;
	root->setProperty("cursorTime", cursorTime);
}

void ChartsPanel::setVisibleRange(int startIndex, int endIndex) {
	QElapsedTimer rangeTimer; rangeTimer.start();
	buildSeriesCache();
	if (!cache_.valid || !cache_.xAxis)
		return;

	QQuickItem* root = view_->rootObject();
	if (root == nullptr)
		return;

	// Loading: kept for when the lines are in (see loading_).
	if (loading_) {
		pendingRange_ = { startIndex, endIndex };
		return;
	}
	// No trip: the axes are hidden and have no range to show.
	if (pointTimesMs_.empty())
		return;

	// Leaflet fires both zoomend and moveend on every zoom interaction -- skip
	// the second call when both events produce the same range. Checked only
	// now so a range skipped above isn't taken as already applied.
	if (startIndex == lastRangeStart_ && endIndex == lastRangeEnd_) {
		Logger::log(Logger::Trace, "Charts", QStringLiteral("setVisibleRange: duplicate range (Leaflet zoomend+moveend); ignoring"));
		return;
	}
	lastRangeStart_ = startIndex;
	lastRangeEnd_   = endIndex;

	if (startIndex < 0 || endIndex < 0) {
		Logger::log(Logger::Trace, "Charts", QStringLiteral("setVisibleRange: full range requested (zoomed all the way out); reloading full-resolution decimated view"));
		setAllXAxisRange(QDateTime::fromMSecsSinceEpoch((qint64)pointTimesMs_.front()),
		                 QDateTime::fromMSecsSinceEpoch((qint64)pointTimesMs_.back()));
		root->setProperty("isFullRangeVisible", true);
		setYAxes(fullExtents_);
		// Replace the zoomed slice with the whole trip's thinned view.
		loadFullSlice(0, (int)pointTimesMs_.size() - 1);
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

		// The Y axes fit the visible slice, which is thinned to the same
		// kDisplayPoints budget as the full view: a large partial viewport
		// (half the flight) would otherwise load 25k points into each series.
		Logger::logf(Logger::Trace, "Charts", "setVisibleRange: zoomed slice [%d..%d] (%d pts, target ~%d pts/series)", lo, hi, hi - lo + 1, kDisplayPoints);
		setYAxes(chartExtents(full_, lo, hi));
		loadFullSlice(lo, hi);
	}
	Logger::logf(Logger::Profile, "Charts", "setVisibleRange: %lld µs", rangeTimer.nsecsElapsed() / 1000);
}

QVariantMap ChartsPanel::valueAt(double timeMs) const {
	if (pointTimesMs_.empty())
		return {};

	const int idx = nearestSampleIndex(pointTimesMs_, timeMs);
	ChartValues values{};
	for (int s = 0; s < CHART_SERIES_COUNT; ++s)
		values[s] = full_[s][idx].y();
	return chartValueMap(pointTimesMs_[idx], values);
}
