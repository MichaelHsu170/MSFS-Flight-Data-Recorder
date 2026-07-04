#include "map_widget.h"
#include "map_bridge.h"
#include "app_settings.h"

#include <QTimer>
#include <QWebEngineView>
#include <QWebEnginePage>
#include <QWebEngineProfile>
#include <QWebChannel>
#include <QVBoxLayout>
#include <QToolButton>
#include <QResizeEvent>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonDocument>
#include <QUrl>
#include <QElapsedTimer>
#include <QFutureWatcher>
#include "logger.h"
#include <QtConcurrent/QtConcurrentRun>

#include <vector>

namespace {

QJsonObject pointToJson(double lat, double lng) {
	QJsonObject obj;
	obj["lat"] = lat;
	obj["lng"] = lng;
	return obj;
}

QJsonObject touchdownToJson(const TouchdownPoint& t) {
	QJsonObject obj;
	obj["lat"] = t.latitude;
	obj["lng"] = t.longitude;
	obj["icao"] = t.icao;
	obj["airportName"] = t.airportName;
	obj["runway"] = t.runway;
	obj["airspeed"] = t.airspeed;
	obj["verticalSpeed"] = t.verticalSpeed;
	obj["gForce"] = t.gForce;
	obj["pitchDegrees"] = t.pitchDegrees;
	obj["bankDegrees"] = t.bankDegrees;
	obj["headingDegrees"] = t.headingDegrees;
	obj["distanceLength"] = t.distanceLength;
	obj["distanceWidth"] = t.distanceWidth;
	obj["distanceLengthPercent"] = t.distanceLengthPercent;
	obj["distanceWidthPercent"] = t.distanceWidthPercent;
	obj["windDirection"] = t.windDirection;
	obj["windVelocity"] = t.windVelocity;
	obj["zuluTime"] = t.zuluTime;
	obj["localTime"] = t.localTime;
	return obj;
}

QJsonObject eventToJson(const TripEvent& e) {
	QJsonObject obj;
	obj["lat"] = e.latitude;
	obj["lng"] = e.longitude;
	obj["event"] = e.event;
	obj["zuluTime"] = e.zuluTime;
	return obj;
}

// Routes the page's JS console (including Leaflet's tileerror/tileload and
// our own console.log/error calls in map.html) to qDebug so failures are
// visible without opening Chromium DevTools by hand.
class LoggingPage : public QWebEnginePage {
public:
	explicit LoggingPage(QObject* parent = nullptr) : QWebEnginePage(parent) {}

protected:
	void javaScriptConsoleMessage(JavaScriptConsoleMessageLevel level, const QString& message, int lineNumber, const QString& sourceID) override {
		Logger::Level logLevel = (level == ErrorMessageLevel || level == WarningMessageLevel)
		    ? Logger::Warning : Logger::Info;
		Logger::log(logLevel, "MapJS",
		    QStringLiteral("%1:%2 %3").arg(sourceID).arg(lineNumber).arg(message));
	}
};

}

MapWidget::MapWidget(QWidget* parent) : QWidget(parent) {
	// OSM's tile usage policy requires a valid User-Agent identifying the
	// application -- QtWebEngine's default UA is a generic Chromium string
	// that doesn't, which tile servers can reject.
	QWebEngineProfile::defaultProfile()->setHttpUserAgent(QStringLiteral("MSFS-Flight-Data-Recorder v2.0.0"));

	view_ = new QWebEngineView(this);
	view_->setPage(new LoggingPage(view_));
	channel_ = new QWebChannel(this);
	bridge_ = new MapBridge(this);

	channel_->registerObject(QStringLiteral("mapBridge"), bridge_);
	view_->page()->setWebChannel(channel_);
	connect(bridge_, &MapBridge::cursorIndexChanged, this, &MapWidget::cursorIndexChanged);
	connect(bridge_, &MapBridge::visibleRangeChanged, this, &MapWidget::visibleRangeChanged);
	connect(view_, &QWebEngineView::loadFinished, this, &MapWidget::onLoadFinished);

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(view_);

	// Floats free over the map (not part of the layout above) so it doesn't
	// take a separate row of vertical space the way a "Show Events" button
	// above the map used to.
	eventsToggle_ = new QToolButton(this);
	eventsToggle_->setText(QStringLiteral("⚑"));
	eventsToggle_->setCheckable(true);
	eventsToggle_->setChecked(true);
	eventsToggle_->setToolTip(QStringLiteral("Show/hide cockpit event markers (gear, flaps, spoilers, etc.) on the map"));
	eventsToggle_->setFixedSize(28, 28);
	eventsToggle_->setStyleSheet(QStringLiteral(
		"QToolButton { background: rgba(255,255,255,200); border: 1px solid #888888; border-radius: 4px; font-size: 14px; }"
		"QToolButton:checked { background: rgba(120,170,255,220); border: 1px solid #3366cc; }"
		"QToolButton:hover { border: 1px solid #3366cc; }"));
	eventsToggle_->raise();
	connect(eventsToggle_, &QToolButton::toggled, this, &MapWidget::setEventsVisible);

	view_->load(QUrl(QStringLiteral("qrc:/map/map.html")));

	liveUpdateTimer_ = new QTimer(this);
	liveUpdateTimer_->setInterval(250);
	connect(liveUpdateTimer_, &QTimer::timeout, this, &MapWidget::flushLivePoints);
}

void MapWidget::resizeEvent(QResizeEvent* event) {
	QWidget::resizeEvent(event);
	const int margin = 8;
	eventsToggle_->move(width() - eventsToggle_->width() - margin, margin);
}

void MapWidget::setDataset(const TripDataset& dataset) {
	mapGenTimer_.start();
	liveUpdateTimer_->stop();
	pendingLiveCoords_.clear();
	trajCoords_.clear();
	trajCoords_.reserve(dataset.points.size());
	QElapsedTimer copyTimer; copyTimer.start();
	for (const TripSamplePoint& p : dataset.points)
		trajCoords_.emplace_back(p.latitude, p.longitude);
	Logger::logf(Logger::Profile, "Map", "coord copy: %lld µs  (%zu pts)", copyTimer.nsecsElapsed() / 1000, trajCoords_.size());
	touchdowns_ = dataset.touchdowns;
	events_ = dataset.events;
	aircraftTitle_ = dataset.aircraftTitle;
	if (pageReady_) {
		// Inject the aircraft title before pushing touchdowns so
		// touchdownPopupHtml() sees the correct value when it runs setTouchdowns.
		QJsonArray arr;
		arr.append(QJsonValue(aircraftTitle_));
		QString encoded = QString::fromUtf8(QJsonDocument(arr).toJson(QJsonDocument::Compact));
		runJs(QStringLiteral("window._aircraftTitle=%1[0];").arg(encoded));
		pushTrajectory();
		pushTouchdownsAndEvents();
	} else {
		// Page still loading; trajectory will be pushed in refreshProvider() when
		// ready. Signal immediately so TrajectoryView's pending counter doesn't stall.
		emit trajectoryLoaded();
	}
}

void MapWidget::appendLivePoint(const TripSamplePoint& point) {
	trajCoords_.emplace_back(point.latitude, point.longitude);
	pendingLiveCoords_.emplace_back(point.latitude, point.longitude);
	if (!pageReady_)
		return;
	if (!liveUpdateTimer_->isActive())
		liveUpdateTimer_->start();
}

void MapWidget::flushLivePoints() {
	if (pendingLiveCoords_.empty()) {
		liveUpdateTimer_->stop();
		return;
	}
	QJsonArray pts;
	for (const auto& [lat, lng] : pendingLiveCoords_)
		pts.append(pointToJson(lat, lng));
	pendingLiveCoords_.clear();
	QJsonDocument doc(pts);
	runJs(QStringLiteral("appendPoints(%1);").arg(QString::fromUtf8(doc.toJson(QJsonDocument::Compact))));
}

void MapWidget::onLoadFinished(bool ok) {
	if (!ok) {
		Logger::log(Logger::Warning, "Map", QStringLiteral("map.html failed to load"));
		return;
	}
	pageReady_ = true;

	// Deliver the Gemini API key and current aircraft title as JS globals so
	// the in-popup streaming analysis can reach them without round-tripping
	// through QWebChannel. JSON-encode inside single-element arrays so any
	// special characters are escaped correctly.
	auto injectString = [this](const QString& jsVar, const QString& value) {
		QJsonArray arr;
		arr.append(QJsonValue(value));
		QString encoded = QString::fromUtf8(QJsonDocument(arr).toJson(QJsonDocument::Compact));
		runJs(QStringLiteral("%1=%2[0];").arg(jsVar, encoded));
	};
	injectString(QStringLiteral("window._geminiApiKey"), AppSettings::instance().geminiApiKey());
	injectString(QStringLiteral("window._aircraftTitle"), aircraftTitle_);

	refreshProvider();
}

void MapWidget::refreshProvider() {
	if (!pageReady_)
		return;
	liveUpdateTimer_->stop();
	pendingLiveCoords_.clear();
	runJs(QStringLiteral("initProvider();"));
	pushTrajectory();
	pushTouchdownsAndEvents();
}

void MapWidget::setEventsVisible(bool visible) {
	if (!pageReady_)
		return;
	runJs(QStringLiteral("setEventsVisible(%1);").arg(visible ? QStringLiteral("true") : QStringLiteral("false")));
}

void MapWidget::pushTrajectory() {
	// Build + decimate off the main thread: a long flight can have 60k+ sample
	// points, but Leaflet bogs down rendering more than ~3000 polyline segments.
	// Stride-decimate so at most MAX_POINTS are sent, preserving the first and
	// last point exactly. Each JSON point carries its original sample index (idx)
	// so the JS side can report correct indices back to C++ for cursor and range.
	std::vector<std::pair<double,double>> coords = trajCoords_;
	auto* watcher = new QFutureWatcher<QString>(this);
	connect(watcher, &QFutureWatcher<QString>::finished, this, [this, watcher]() {
		QElapsedTimer t; t.start();
		runJs(watcher->result());
		Logger::logf(Logger::Profile, "Map", "runJs trajectory: %lld µs  (async — JS render time not captured)", t.nsecsElapsed() / 1000);
		watcher->deleteLater();
		emit trajectoryLoaded();
	});
	watcher->setFuture(QtConcurrent::run([coords = std::move(coords)]() {
		QElapsedTimer t; t.start();
		const int MAX_POINTS = 3000;
		int n = (int)coords.size();
		int stride = std::max(1, (n + MAX_POINTS - 1) / MAX_POINTS);
		// Parallel flat arrays are more compact JSON than [{lat,lng,idx},...]
		// (no repeated field-name strings) and faster to iterate in JS.
		QJsonArray lats, lngs, idxs;
		for (int i = 0; i < n; i += stride) {
			lats.append(coords[i].first);
			lngs.append(coords[i].second);
			idxs.append(i);
		}
		if (n > 0 && (n - 1) % stride != 0) {
			lats.append(coords[n - 1].first);
			lngs.append(coords[n - 1].second);
			idxs.append(n - 1);
		}
		QJsonObject data;
		data[QStringLiteral("lats")] = lats;
		data[QStringLiteral("lngs")] = lngs;
		data[QStringLiteral("idxs")] = idxs;
		QJsonDocument doc(data);
		Logger::logf(Logger::Profile, "Map", "JSON build (bg): %lld ms  (%d → %d pts decimated)", t.nsecsElapsed() / 1000000, n, (int)lats.size());
		return QStringLiteral("setTrajectory(%1);").arg(QString::fromUtf8(doc.toJson(QJsonDocument::Compact)));
	}));
}

void MapWidget::pushTouchdownsAndEvents() {
	auto* touchdownWatcher = new QFutureWatcher<QString>(this);
	connect(touchdownWatcher, &QFutureWatcher<QString>::finished, this, [this, touchdownWatcher]() {
		QElapsedTimer t; t.start();
		runJs(touchdownWatcher->result());
		Logger::logf(Logger::Profile, "Map", "runJs touchdowns: %lld µs", t.nsecsElapsed() / 1000);
		touchdownWatcher->deleteLater();
	});
	touchdownWatcher->setFuture(QtConcurrent::run([touchdowns = touchdowns_]() {
		QElapsedTimer t; t.start();
		QJsonArray arr;
		for (const TouchdownPoint& td : touchdowns)
			arr.append(touchdownToJson(td));
		QJsonDocument doc(arr);
		Logger::logf(Logger::Profile, "Map", "touchdowns JSON (bg): %lld µs  (%d touchdowns)", t.nsecsElapsed() / 1000, (int)arr.size());
		return QStringLiteral("setTouchdowns(%1);").arg(QString::fromUtf8(doc.toJson(QJsonDocument::Compact)));
	}));

	auto* eventWatcher = new QFutureWatcher<QString>(this);
	connect(eventWatcher, &QFutureWatcher<QString>::finished, this, [this, eventWatcher]() {
		QElapsedTimer t; t.start();
		runJs(eventWatcher->result());
		Logger::logf(Logger::Profile, "Map", "runJs events: %lld µs", t.nsecsElapsed() / 1000);
		eventWatcher->deleteLater();
	});
	eventWatcher->setFuture(QtConcurrent::run([events = events_]() {
		QElapsedTimer t; t.start();
		QJsonArray arr;
		for (const TripEvent& e : events)
			arr.append(eventToJson(e));
		QJsonDocument doc(arr);
		Logger::logf(Logger::Profile, "Map", "events JSON (bg): %lld µs  (%d events)", t.nsecsElapsed() / 1000, (int)arr.size());
		return QStringLiteral("setEvents(%1);").arg(QString::fromUtf8(doc.toJson(QJsonDocument::Compact)));
	}));
}

void MapWidget::runJs(const QString& script) {
	view_->page()->runJavaScript(script);
}
