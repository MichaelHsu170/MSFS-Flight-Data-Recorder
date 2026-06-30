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
#include <QPointF>
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
	dataset_ = dataset;
	if (pageReady_) {
		pushTrajectory();
		pushTouchdownsAndEvents();
	}
}

void MapWidget::appendLivePoint(const TripSamplePoint& point) {
	dataset_.points.push_back(point);
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

namespace {

QString trajectoryScript(const std::vector<QPointF>& coords) {
	QJsonArray points;
	for (const QPointF& c : coords)
		points.append(pointToJson(c.y(), c.x()));
	QJsonDocument doc(points);
	return QStringLiteral("setTrajectory(%1);").arg(QString::fromUtf8(doc.toJson(QJsonDocument::Compact)));
}

}

void MapWidget::pushTrajectory() {
	// A long flight can have many thousands of sample points -- building the
	// JSON array and formatting it into a single multi-megabyte script string
	// is pure CPU work with no Qt GUI objects involved, so it's safe (and
	// necessary to avoid freezing the window) to do it on a worker thread and
	// only hop back to the GUI thread to actually call runJavaScript().
	std::vector<QPointF> coords;
	coords.reserve(dataset_.points.size());
	for (const TripSamplePoint& p : dataset_.points)
		coords.emplace_back(p.longitude, p.latitude);

	auto* watcher = new QFutureWatcher<QString>(this);
	connect(watcher, &QFutureWatcher<QString>::finished, this, [this, watcher]() {
		runJs(watcher->result());
		watcher->deleteLater();
	});
	watcher->setFuture(QtConcurrent::run(trajectoryScript, std::move(coords)));
}

void MapWidget::pushTouchdownsAndEvents() {
	std::vector<TouchdownPoint> touchdowns(dataset_.touchdowns.begin(), dataset_.touchdowns.end());
	std::vector<TripEvent> events(dataset_.events.begin(), dataset_.events.end());

	auto* touchdownWatcher = new QFutureWatcher<QString>(this);
	connect(touchdownWatcher, &QFutureWatcher<QString>::finished, this, [this, touchdownWatcher]() {
		runJs(touchdownWatcher->result());
		touchdownWatcher->deleteLater();
	});
	touchdownWatcher->setFuture(QtConcurrent::run([touchdowns = std::move(touchdowns)]() {
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
	eventWatcher->setFuture(QtConcurrent::run([events = std::move(events)]() {
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
