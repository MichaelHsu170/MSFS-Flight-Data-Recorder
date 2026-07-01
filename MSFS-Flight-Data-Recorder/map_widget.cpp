#include "map_widget.h"
#include "map_bridge.h"

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
#include <QDebug>
#include <QFutureWatcher>
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
	obj["runway"] = t.runway;
	obj["airspeed"] = t.airspeed;
	obj["verticalSpeed"] = t.verticalSpeed;
	obj["gForce"] = t.gForce;
	obj["pitchDegrees"] = t.pitchDegrees;
	obj["bankDegrees"] = t.bankDegrees;
	obj["headingDegrees"] = t.headingDegrees;
	obj["zuluTime"] = t.zuluTime;
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
		qDebug().noquote() << "[map.js]" << sourceID << ":" << lineNumber << "-" << message << "(level" << level << ")";
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
	liveUpdateTimer_->stop();
	pendingLiveCoords_.clear();
	trajCoords_.clear();
	trajCoords_.reserve(dataset.points.size());
	for (const TripSamplePoint& p : dataset.points)
		trajCoords_.emplace_back(p.latitude, p.longitude);
	touchdowns_ = dataset.touchdowns;
	events_ = dataset.events;
	if (pageReady_) {
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
		qWarning() << "MapWidget: map.html failed to load";
		return;
	}
	pageReady_ = true;
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
		runJs(watcher->result());
		watcher->deleteLater();
		emit trajectoryLoaded();
	});
	watcher->setFuture(QtConcurrent::run([coords = std::move(coords)]() {
		const int MAX_POINTS = 3000;
		int n = (int)coords.size();
		int stride = std::max(1, n / MAX_POINTS);
		QJsonArray arr;
		for (int i = 0; i < n; i += stride) {
			QJsonObject obj;
			obj[QStringLiteral("lat")] = coords[i].first;
			obj[QStringLiteral("lng")] = coords[i].second;
			obj[QStringLiteral("idx")] = i;
			arr.append(obj);
		}
		if (n > 0 && (n - 1) % stride != 0) {
			QJsonObject obj;
			obj[QStringLiteral("lat")] = coords[n - 1].first;
			obj[QStringLiteral("lng")] = coords[n - 1].second;
			obj[QStringLiteral("idx")] = n - 1;
			arr.append(obj);
		}
		QJsonDocument doc(arr);
		return QStringLiteral("setTrajectory(%1);").arg(QString::fromUtf8(doc.toJson(QJsonDocument::Compact)));
	}));
}

void MapWidget::pushTouchdownsAndEvents() {
	auto* touchdownWatcher = new QFutureWatcher<QString>(this);
	connect(touchdownWatcher, &QFutureWatcher<QString>::finished, this, [this, touchdownWatcher]() {
		runJs(touchdownWatcher->result());
		touchdownWatcher->deleteLater();
	});
	touchdownWatcher->setFuture(QtConcurrent::run([touchdowns = touchdowns_]() {
		QJsonArray arr;
		for (const TouchdownPoint& t : touchdowns)
			arr.append(touchdownToJson(t));
		QJsonDocument doc(arr);
		return QStringLiteral("setTouchdowns(%1);").arg(QString::fromUtf8(doc.toJson(QJsonDocument::Compact)));
	}));

	auto* eventWatcher = new QFutureWatcher<QString>(this);
	connect(eventWatcher, &QFutureWatcher<QString>::finished, this, [this, eventWatcher]() {
		runJs(eventWatcher->result());
		eventWatcher->deleteLater();
	});
	eventWatcher->setFuture(QtConcurrent::run([events = events_]() {
		QJsonArray arr;
		for (const TripEvent& e : events)
			arr.append(eventToJson(e));
		QJsonDocument doc(arr);
		return QStringLiteral("setEvents(%1);").arg(QString::fromUtf8(doc.toJson(QJsonDocument::Compact)));
	}));
}

void MapWidget::runJs(const QString& script) {
	view_->page()->runJavaScript(script);
}
