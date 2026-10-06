// Map widget (map_widget.cpp): the QWebEngineView wrapper around the
// Leaflet/OSM trajectory map -- setDataset()/showOverview()/
// resetZoom()/setEventsVisible() driving the real page, and the page's cursor
// and range forwarded only for the current trajectory, the range the page
// measures, AI analysis with fetch() stubbed, the right-click menu
// and an overview route clicked with the mouse, and a page reload; not the
// pure JS string-building math (already covered standalone in
// tst_map_script.cpp).
//
// Needs a custom main(), not QTEST_MAIN: QWebEngineView requires
// Qt::AA_ShareOpenGLContexts to be set before QApplication is constructed
// (see main.cpp), and QTEST_MAIN's generated main() constructs QApplication
// itself with no hook to do that first. This target also must NOT run under
// QT_QPA_PLATFORM=offscreen (unlike every other fdr_add_test() target) --
// WebEngine's GPU process hits a fatal DCHECK (!m_scopedOverlayReadAccess)
// trying to share a GL context with an offscreen-platform window.
//
// Deliberately only ever constructs ONE MapWidget for the whole test binary
// (in initTestCase(), reused by every slot below): in this sandboxed
// environment a second QWebEngineView-backed widget, constructed later in
// the same process even after the first was destroyed, reliably never
// reaches "page ready" (confirmed experimentally -- its load just hangs).
// A single instance loads and runs fine, so every slot after the one that
// establishes readiness shares that one instance instead of making its own.
#include "kml_export.h"
#include "logger.h"
#include "map_bridge.h"
#include "map_widget.h"
#include "test_support.h"

#include <QApplication>
#include <QClipboard>
#include <QFile>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMenu>
#include <QMessageBox>
#include <QRegularExpression>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QWebEngineView>
#include <QtTest>

#include <functional>

using namespace TestSupport;

namespace {

LiftoffPoint liftoffAt(const QString& icao) {
	LiftoffPoint p;
	p.icao = icao;
	return p;
}

TouchdownPoint touchdownAt(const QString& icao) {
	TouchdownPoint p;
	p.icao = icao;
	return p;
}

TripSamplePoint samplePoint(double lat, double lon) {
	TripSamplePoint p;
	p.latitude = lat;
	p.longitude = lon;
	return p;
}

// One streamed Gemini response object carrying parts ({text, thought}) and,
// if finishReason isn't empty, the candidate's finishReason.
QString aiChunk(const QList<QPair<QString, bool>>& parts, const QString& finishReason = QString()) {
	QJsonArray jsonParts;
	for (const auto& [text, thought] : parts) {
		QJsonObject part{ { "text", text } };
		if (thought)
			part["thought"] = true;
		jsonParts.append(part);
	}
	QJsonObject candidate{ { "content", QJsonObject{ { "parts", jsonParts }, { "role", "model" } } } };
	if (!finishReason.isEmpty())
		candidate["finishReason"] = finishReason;
	return QString::fromUtf8(QJsonDocument(QJsonObject{ { "candidates", QJsonArray{ candidate } } }).toJson(QJsonDocument::Compact));
}

// A whole streamed response, as the API sends it: a JSON array of chunks.
QString aiStream(const QStringList& chunks) {
	return QLatin1Char('[') + chunks.join(QStringLiteral(",\r\n")) + QLatin1Char(']');
}

}

class TstMapWidget : public QObject {
	Q_OBJECT

	MapWidget* widget_ = nullptr;
	QTemporaryDir logDir_;
	QTemporaryDir filesDir_; // files saved through the map's menu
	QString logPath_;
	TripDataset shown_; // see showTrip()

	// Runs the page's AI analysis of a liftoff with fetch() stubbed: attempt n
	// gets attempts[n] (the last one repeated), each delivered in the given
	// pieces with HTTP status statuses[n] (the last one repeated; 200 if
	// none; 0 for a request that never reaches the service, as when offline;
	// -1 for a 200 whose connection drops after its pieces); saving the report succeeds if saveOk. Returns what it did: calls
	// (fetches made), saved (the report saved, null if none), text (the answer
	// shown), thinkingShown.
	QVariantMap runAiAnalysisWith(const QList<QStringList>& attempts, bool saveOk = true, const QList<int>& statuses = {}) {
		QJsonArray json;
		for (const QStringList& pieces : attempts)
			json.append(QJsonArray::fromStringList(pieces));
		QJsonArray jsonStatuses;
		for (int status : statuses)
			jsonStatuses.append(status);
		evalPageJs(widget_, QStringLiteral(R"JS(
			(function (attempts, statuses) {
			    window._ai = { calls: 0, saved: null, done: false };
			    var realFetch = window.fetch;
			    window.fetch = function () {
			        var n = window._ai.calls++;
			        var status = statuses.length ? statuses[Math.min(n, statuses.length - 1)] : 200;
			        if (status === 0) return Promise.reject(new TypeError('Failed to fetch'));
			        var drops = status === -1;
			        if (drops) status = 200;
			        var pieces = attempts[Math.min(n, attempts.length - 1)].slice();
			        var reader = { read: function () {
			            if (pieces.length) return Promise.resolve({ done: false, value: new TextEncoder().encode(pieces.shift()) });
			            return drops ? Promise.reject(new TypeError('network error')) : Promise.resolve({ done: true });
			        } };
			        return Promise.resolve({ ok: status < 300, status: status,
			            text: function () { return Promise.resolve(pieces.join('')); },
			            body: { getReader: function () { return reader; } } });
			    };
			    var box = document.getElementById('ai-test');
			    if (!box) { box = document.createElement('div'); box.id = 'ai-test'; document.body.appendChild(box); }
			    box.innerHTML = '<button id="td-btn-ai"></button><span id="td-spin-ai"></span><div id="td-result-ai"></div>';
			    runAiAnalysis('ai', { rowId: 5 }, function () { return 'prompt'; },
			        function (rowId, report, onSaved) { window._ai.saved = report; onSaved(%2); }, 'Analyze Liftoff')
			        .then(function () { window.fetch = realFetch; window._ai.done = true; });
			})(%1, %3))JS").arg(QString::fromUtf8(QJsonDocument(json).toJson(QJsonDocument::Compact)),
				saveOk ? QStringLiteral("true") : QStringLiteral("false"),
				QString::fromUtf8(QJsonDocument(jsonStatuses).toJson(QJsonDocument::Compact))));
		if (!QTest::qWaitFor([this] { return evalPageJs(widget_, QStringLiteral("window._ai.done")).toBool(); }, 5000))
			return {};
		return evalPageJs(widget_, QStringLiteral(
			"({ calls: window._ai.calls, saved: window._ai.saved,"
			"   text: document.getElementById('td-th-final-ai').textContent,"
			"   thinkingShown: document.getElementById('td-th-det-ai').style.display !== 'none' })")).toMap();
	}


	QWebEngineView* mapView() { return widget_->findChild<QWebEngineView*>(); }
	// The widget a user's mouse input reaches the page through.
	QWidget* mapInput() {
		QWidget* proxy = mapView()->focusProxy();
		return proxy ? proxy : mapView();
	}

	// A trip from (10, 20) to (11, 21), KJFK to KLAX, departing
	// 2026-01-02 10:00:00.5Z.
	static TripDataset tripToSave() {
		TripDataset dataset;
		dataset.points = { samplePoint(10, 20), samplePoint(11, 21) };
		LiftoffPoint liftoff = liftoffAt(QStringLiteral("KJFK"));
		liftoff.latitude = 10;
		liftoff.longitude = 20;
		TouchdownPoint touchdown = touchdownAt(QStringLiteral("KLAX"));
		touchdown.latitude = 11;
		touchdown.longitude = 21;
		dataset.liftoffPoints = { liftoff };
		dataset.touchdowns = { touchdown };
		dataset.departureZuluTime = QStringLiteral("2026-01-02T10:00:00.500+00:00_5");
		return dataset;
	}

	// An overview route from (10, 20) to (11, 21).
	static TripSummary overviewTrip(int id) {
		TripSummary trip;
		trip.id = id;
		trip.departureLat = 10;
		trip.departureLng = 20;
		trip.destinationLat = 11;
		trip.destinationLng = 21;
		return trip;
	}

	// Shows a copy of dataset: MapWidget keeps a pointer to the dataset it
	// shows (as the app's TrajectoryView owns it), so it must outlive the
	// test that passed it.
	void showTrip(const TripDataset& dataset) {
		shown_ = dataset;
		widget_->setDataset(shown_);
	}

	// Shows dataset and waits until the page has drawn it and the animated
	// fit has settled.
	void loadTrip(const TripDataset& dataset) {
		showTrip(dataset);
		QTRY_COMPARE_WITH_TIMEOUT(mapTrajectoryPointCount(widget_), int(dataset.points.size()), 10000);
		double south = 90, west = 180, north = -90, east = -180;
		for (const TripSamplePoint& p : dataset.points) {
			south = std::min(south, p.latitude);
			north = std::max(north, p.latitude);
			west = std::min(west, p.longitude);
			east = std::max(east, p.longitude);
		}
		QTRY_VERIFY_WITH_TIMEOUT(mapFittedTo(widget_, south, west, north, east), 5000);
	}

	// The middle of the straight line from (lat1, lng1) to (lat2, lng2) as
	// the page draws it, in the view's coordinates.
	QPoint midpointOnScreen(double lat1, double lng1, double lat2, double lng2) {
		const QVariantList xy = evalPageJs(widget_, QStringLiteral(
			"var a = leafletMapInstance.latLngToContainerPoint([%1, %2]), b = leafletMapInstance.latLngToContainerPoint([%3, %4]);"
			"[(a.x + b.x) / 2, (a.y + b.y) / 2]").arg(lat1).arg(lng1).arg(lat2).arg(lng2)).toList();
		return QPoint(qRound(xy.value(0).toDouble()), qRound(xy.value(1).toDouble()));
	}

	// Right-clicks the map with the mouse at pos and returns the menu that
	// pops up; nullptr if none did.
	QMenu* openRightClickMenu(const QPoint& pos) {
		QTest::mouseClick(mapInput(), Qt::RightButton, Qt::NoModifier, pos);
		QMenu* menu = nullptr;
		return QTest::qWaitFor([&menu] { return (menu = qobject_cast<QMenu*>(QApplication::activePopupWidget())) != nullptr; }, 5000)
			? menu : nullptr;
	}

	// Puts an element with this inner HTML over the map's top left corner
	// (removed by removeTestElement()) and returns its middle, in the view's
	// coordinates.
	QPoint addTestElement(const QString& html) {
		const QVariantList xy = evalPageJs(widget_, QStringLiteral(
			"var d = document.createElement('div'); d.id = 'menu-test'; d.innerHTML = '%1';"
			"d.style.cssText = 'position:absolute;left:10px;top:10px;z-index:2000;background:white';"
			"document.body.appendChild(d); var r = d.getBoundingClientRect(); [r.left + r.width / 2, r.top + r.height / 2]").arg(html)).toList();
		return QPoint(qRound(xy.value(0).toDouble()), qRound(xy.value(1).toDouble()));
	}
	void removeTestElement() {
		evalPageJs(widget_, QStringLiteral("document.getElementById('menu-test').remove(); true"));
	}

	// Right-clicks the map with the mouse, away from the trip, and returns
	// the items of the menu that pops up; then chooses choose from it, or
	// closes it if that's empty. onDialog runs on the dialog choosing it
	// opens.
	QStringList rightClickMenu(const QString& choose = QString(), const std::function<void(QWidget*)>& onDialog = {}) {
		QMenu* menu = openRightClickMenu(QPoint(60, 250));
		if (!menu)
			return {};
		QStringList items;
		for (QAction* action : menu->actions())
			if (!action->isSeparator())
				items << action->text();
		if (choose.isEmpty()) {
			menu->close();
			return items;
		}
		if (onDialog)
			onNextModal(onDialog);
		chooseMenuItem(menu, choose);
		return items;
	}

	// image (a grab of the map showing tripToSave()) is the whole view, with
	// the trajectory's blue line across its middle.
	void checkShowsTheTrajectory(const QImage& image) {
		const qreal scale = image.devicePixelRatio();
		QCOMPARE(image.size(), mapView()->size() * scale);
		const QColor middle = image.pixelColor(midpointOnScreen(10, 20, 11, 21) * scale);
		QVERIFY2(middle.blue() > 200 && middle.red() < 60 && middle.green() < 60, qPrintable(middle.name()));
	}

	// Reloads map.html, as if it was opened anew, and waits until it loaded.
	void reloadPage() {
		QSignalSpy loaded(mapView(), &QWebEngineView::loadFinished);
		mapView()->reload();
		QVERIFY(loaded.wait(15000));
		QVERIFY(loaded.value(0).value(0).toBool());
	}

private slots:
	// Logger::init() takes effect once per process, so it runs here.
	void initTestCase() {
		isolateFiles(); // the database and settings.ini the map's menu reads
		// Save dialogs a test can answer (see saveFileDialogAs()).
		QCoreApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
		logPath_ = logDir_.filePath(QStringLiteral("map.log"));
		Logger::init(Logger::Info, logPath_);
		widget_ = new MapWidget;
		widget_->resize(400, 300);
		widget_->show();
	}

	void cleanupTestCase() {
		delete widget_;
	}
	void cleanup() {
		cancelPendingModals(); // no dialog action outlives its test
		// reopeningAPopupMidAnalysisDoesNotRunItTwice() stubs the page's fetch
		// and report save; if it failed midway, put the real ones back.
		evalPageJs(widget_, QStringLiteral(
			"if (window._dup) { window._dup.restore(); window._dup.marker.closePopup(); window._dup = null; } true"));
	}

	// Must run first (QTest runs slots in declaration order): right after
	// initTestCase() constructs the widget, the page cannot possibly be ready
	// yet -- QWebEngineView::load() always needs a round trip through the
	// Chromium render process -- so every "not ready yet" early-return branch
	// has to be exercised here, before anything else gets a chance to wait
	// for the page to finish loading.
	void beforePageReadyEveryEarlyReturnBranchIsTakenAndSetDatasetEmitsOnceImmediately() {
		// Remembered and applied once the page loads (checked in the next slot).
		widget_->setEventsVisible(false);

		QSignalSpy spy(widget_, &MapWidget::trajectoryLoaded);
		TripDataset dataset;
		dataset.points = { samplePoint(3, 4) };
		showTrip(dataset);
		QCOMPARE(spy.count(), 1); // page not ready: setDataset's immediate-emit path

		// Let the page actually finish loading and refreshProvider() re-push the
		// same dataset; suppressNextTrajectoryLoaded_ should swallow that second,
		// deferred emit so callers never see the same load reported twice. The
		// push decides on that emit right after queueing the trajectory's script
		// (MapWidget::pushTrip()), so it has once the page shows the point. Also
		// leaves the shared widget_ in the "page ready" state for every slot below.
		QTRY_COMPARE_WITH_TIMEOUT(mapTrajectoryPointCount(widget_), 1, 15000);
		QCOMPARE(spy.count(), 1);
	}

	void afterPageReadySetDatasetDrawsEverythingAndEmitsOnce() {
		QSignalSpy spy(widget_, &MapWidget::trajectoryLoaded);
		TripDataset dataset;
		dataset.points = { samplePoint(1, 2), samplePoint(3, 4) };
		dataset.liftoffPoints = { liftoffAt(QStringLiteral("KJFK")) };
		dataset.touchdowns = { touchdownAt(QStringLiteral("KLAX")) };
		TripEvent event;
		event.event = QStringLiteral("GEAR_UP");
		dataset.events = { event };
		showTrip(dataset);

		QVERIFY(spy.wait(10000));
		QCOMPARE(spy.count(), 1);
		QCOMPARE(mapTrajectoryPointCount(widget_), 2);
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "liftoff-icon"), 1, 5000);
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "touchdown-icon"), 1, 5000);

		// Events were hidden before the page loaded (previous slot): the page
		// keeps them off the map until they're shown again.
		QTest::qWait(500); // the events push runs alongside the two above
		QCOMPARE(mapElementCount(widget_, "event-icon"), 0);
		widget_->setEventsVisible(true);
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "event-icon"), 1, 5000);
		widget_->setEventsVisible(false);
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "event-icon"), 0, 5000);
		widget_->setEventsVisible(true); // later slots expect the default
	}

	void aSupersededSetDatasetIsDiscardedWithoutEmittingTrajectoryLoadedTwice() {
		QSignalSpy spy(widget_, &MapWidget::trajectoryLoaded);
		TripDataset first;
		first.points = { samplePoint(1, 2) };
		TripDataset second;
		second.points = { samplePoint(3, 4) };
		showTrip(first);
		showTrip(second); // supersedes the first before its background compute can finish

		QVERIFY(spy.wait(10000));
		QTest::qWait(300); // give the superseded watcher a chance to finish too
		QCOMPARE(spy.count(), 1); // the superseded load never emits
	}

	void aVisibleRangeIsForwardedOnlyForTheCurrentTrajectory() {
		MapBridge* bridge = widget_->findChild<MapBridge*>();
		QVERIFY(bridge);
		QSignalSpy pageRange(bridge, &MapBridge::visibleRangeChanged);
		QSignalSpy forwarded(widget_, &MapWidget::visibleRangeChanged);
		TripDataset dataset;
		dataset.points = { samplePoint(10, 20), samplePoint(11, 21) };
		showTrip(dataset);

		// The page reports the range it fit the new trajectory to, tagged with
		// that trajectory's version.
		QVERIFY(pageRange.wait(10000));
		const int version = pageRange.last().value(2).toInt();
		QVERIFY(QTest::qWaitFor([&]() { return forwarded.count() == pageRange.count(); }, 2000));

		forwarded.clear();
		bridge->rangeChanged(0, 1, version - 1); // measured on the previous trajectory
		QCOMPARE(forwarded.count(), 0);
		bridge->rangeChanged(0, 1, version);
		QCOMPARE(forwarded.count(), 1);
		QCOMPARE(forwarded.value(0).value(0).toInt(), 0);
		QCOMPARE(forwarded.value(0).value(1).toInt(), 1);
	}

	// The page reports the part of the trip on screen as the samples in view
	// plus one on each side (the line runs on to them off screen); the whole
	// trip as (-1, -1); and, zoomed in between samples, the sample nearest
	// the middle of the view plus one on each side.
	void theVisibleRangeIsTheSamplesInViewPlusOneOnEachSide() {
		MapBridge* bridge = widget_->findChild<MapBridge*>();
		QVERIFY(bridge);
		QSignalSpy pageRange(bridge, &MapBridge::visibleRangeChanged);
		TripDataset dataset;
		dataset.points = { samplePoint(10, 20), samplePoint(10.5, 20.5), samplePoint(11, 21),
			samplePoint(11.5, 21.5), samplePoint(12, 22) };
		loadTrip(dataset);
		const auto lastRange = [&pageRange] {
			return pageRange.isEmpty() ? QString()
				: QStringLiteral("%1,%2").arg(pageRange.last().value(0).toInt()).arg(pageRange.last().value(1).toInt());
		};
		QTRY_COMPARE_WITH_TIMEOUT(lastRange(), QStringLiteral("-1,-1"), 5000);

		// At zoom 9 the 400 x 300 view is about 1.1 x 0.8 degrees: only the
		// sample at (11.5, 21.5) is in it.
		evalPageJs(widget_, QStringLiteral("leafletMapInstance.setView([11.5, 21.5], 9, {animate: false})"));
		QTRY_COMPARE_WITH_TIMEOUT(lastRange(), QStringLiteral("2,4"), 5000);

		// At zoom 14 the view is about 0.03 degrees across: no sample is in
		// it, and (11, 21) is the nearest to its middle.
		evalPageJs(widget_, QStringLiteral("leafletMapInstance.setView([11.1, 21.1], 14, {animate: false})"));
		QTRY_COMPARE_WITH_TIMEOUT(lastRange(), QStringLiteral("1,3"), 5000);
	}

	void aCursorIndexIsForwardedOnlyForTheCurrentTrajectory() {
		MapBridge* bridge = widget_->findChild<MapBridge*>();
		QVERIFY(bridge);
		QSignalSpy loaded(widget_, &MapWidget::trajectoryLoaded);
		QSignalSpy pageCursor(bridge, &MapBridge::cursorIndexChanged);
		QSignalSpy forwarded(widget_, &MapWidget::cursorIndexChanged);
		TripDataset dataset;
		dataset.points = { samplePoint(10, 20), samplePoint(11, 21), samplePoint(12, 22) };
		showTrip(dataset);
		QVERIFY(loaded.wait(10000)); // the page has drawn the new trajectory

		// A click on the line moves the cursor to the nearest sample, tagged
		// with the trajectory's version.
		evalPageJs(widget_, QStringLiteral(
			"leafletMapInstance.eachLayer(function (l) { if (l instanceof L.Polyline) l.fire('click', { latlng: L.latLng(12, 22) }); })"));
		QTRY_COMPARE_WITH_TIMEOUT(forwarded.count(), 1, 5000);
		QCOMPARE(forwarded.value(0).value(0).toInt(), 2);
		const int version = pageCursor.value(0).value(1).toInt();

		forwarded.clear();
		bridge->markerMoved(1, version - 1); // measured on the previous trajectory
		QCOMPARE(forwarded.count(), 0);
		bridge->markerMoved(1, version);
		QCOMPARE(forwarded.count(), 1);
		QCOMPARE(forwarded.value(0).value(0).toInt(), 1);
	}

	void defaultMapImageFileNameTracksOverviewVsLoadedTripState() {
		TripDataset dataset;
		dataset.liftoffPoints = { liftoffAt(QStringLiteral("KJFK")) };
		dataset.touchdowns = { touchdownAt(QStringLiteral("KLAX")) };
		dataset.departureZuluTime = QStringLiteral("2024-03-15T10:30:00.000+00:00_0");
		showTrip(dataset);
		QCOMPARE(widget_->defaultMapImageFileName(), QStringLiteral("KJFK-KLAX_20240315103000.png"));

		widget_->showOverview({});
		QCOMPARE(widget_->defaultMapImageFileName(), QStringLiteral("trips.png"));
	}

	// The overview replaces the trip: its line, cursor, liftoff/touchdown/event
	// markers and their popups' stored points all go, and the route is drawn.
	void showOverviewRemovesTheTripFromTheMap() {
		TripDataset dataset;
		dataset.points = { samplePoint(10, 20), samplePoint(11, 21) };
		dataset.liftoffPoints = { liftoffAt(QStringLiteral("KJFK")) };
		dataset.touchdowns = { touchdownAt(QStringLiteral("KLAX")) };
		TripEvent event;
		event.event = QStringLiteral("GEAR_UP");
		dataset.events = { event };
		showTrip(dataset);
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "liftoff-icon"), 1, 10000);
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "touchdown-icon"), 1, 5000);
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "event-icon"), 1, 5000);
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "leaflet-marker-draggable"), 1, 5000);

		TripSummary trip;
		trip.id = 1;
		trip.departureLat = 10;
		trip.departureLng = 20;
		trip.destinationLat = 11;
		trip.destinationLng = 21;
		widget_->showOverview({ trip });
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "overview-endpoint"), 2, 5000);
		QCOMPARE(mapTrajectoryPointCount(widget_), 0);
		QCOMPARE(mapElementCount(widget_, "leaflet-marker-draggable"), 0);
		QCOMPARE(mapElementCount(widget_, "liftoff-icon"), 0);
		QCOMPARE(mapElementCount(widget_, "touchdown-icon"), 0);
		QCOMPARE(mapElementCount(widget_, "event-icon"), 0);
		QCOMPARE(evalPageJs(widget_, QStringLiteral(
			"Object.keys(_liftoffStore).length + Object.keys(_touchdownStore).length")).toInt(), 0);
	}

	void resetZoomRefitsTheMapToTheTrajectory() {
		TripDataset dataset;
		dataset.points = { samplePoint(10, 20), samplePoint(11, 21) };
		showTrip(dataset);
		// Waits for this trip on the page, not for trajectoryLoaded: the
		// previous slot's loads, never waited for, can still emit that late.
		QTRY_COMPARE_WITH_TIMEOUT(mapTrajectoryPointCount(widget_), 2, 10000);
		QTRY_VERIFY_WITH_TIMEOUT(mapFittedTo(widget_, 10, 20, 11, 21), 5000);
		const int fitted = mapZoom(widget_);
		QVERIFY(fitted > 2);

		evalPageJs(widget_, QStringLiteral("leafletMapInstance.setZoom(2, {animate: false})"));
		QCOMPARE(mapZoom(widget_), 2);
		widget_->resetZoom();
		QTRY_COMPARE_WITH_TIMEOUT(mapZoom(widget_), fitted, 5000);
	}

	// Leaflet drops an animated zoom asked for during another one, so a trip
	// selected while the previous trip's fit is still zooming must still be
	// fitted once that zoom ends.
	void aTripLoadedDuringTheLastFitsZoomIsStillFitted() {
		evalPageJs(widget_, QStringLiteral("leafletMapInstance.setView([11, 21], 9, {animate: false})"));
		// The second trip is loaded once the first one's zoom (9 -> 7) has
		// started animating.
		evalPageJs(widget_, QStringLiteral(
			"leafletMapInstance.once('zoomanim', function () {"
			"  setTrajectory({lats: [10, 11], lngs: [20, 21], idxs: [0, 1], version: 0});"
			"});"
			"setTrajectory({lats: [10, 12], lngs: [20, 22], idxs: [0, 1], version: 0});"));
		QTRY_VERIFY_WITH_TIMEOUT(mapFittedTo(widget_, 10, 20, 11, 21), 5000);
		// 1 x 1 degree at 10 N in the 400 x 300 view less 20 px padding:
		// ~182 px a side at zoom 8, ~364 px (too wide) at zoom 9.
		QCOMPARE(mapZoom(widget_), 8);
	}

	// A trip with no samples has nowhere to put the cursor, so the previous
	// trip's draggable cursor marker must not stay on the map.
	void aTripWithNoSamplesRemovesThePreviousCursorMarker() {
		QSignalSpy spy(widget_, &MapWidget::trajectoryLoaded);
		TripDataset withPoints;
		withPoints.points = { samplePoint(10, 20), samplePoint(11, 21) };
		showTrip(withPoints);
		QVERIFY(spy.wait(10000));
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "leaflet-marker-draggable"), 1, 5000);

		TripDataset empty;
		empty.tripId = 9;
		showTrip(empty);
		QVERIFY(spy.wait(10000));
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "leaflet-marker-draggable"), 0, 5000);
	}

	// Recorded names and times show in the popups as typed: markup in them is
	// text, not elements.
	void popupValuesShowLiterally() {
		const QString contact = evalPageJs(widget_, QStringLiteral(
			"(function () { var d = document.createElement('div');"
			"  d.innerHTML = runwayContactPopupHtml('x', {icao: 'T&<i>X</i>', lat: 1, lng: 2, airspeed: 1, verticalSpeed: 0,"
			"    pitchDegrees: 0, bankDegrees: 0, headingDegrees: 0, windDirection: 0, windVelocity: 0, zuluTime: '<i>z</i>'},"
			"    'Liftoff', null, 'f', 'L').html;"
			"  var v = d.querySelectorAll('.td-val');"
			"  return [v[0].textContent, v[v.length - 1].textContent, d.querySelectorAll('i').length].join('|'); })()")).toString();
		QCOMPARE(contact, QStringLiteral("T&<i>X</i>|<i>z</i>|0"));

		const QString event = evalPageJs(widget_, QStringLiteral(
			"(function () { var d = document.createElement('div');"
			"  d.innerHTML = eventPopupHtml([{event: 'A & <i>B</i>', zuluTime: '<i>z</i>'}]);"
			"  return d.textContent + '|' + d.querySelectorAll('i').length; })()")).toString();
		QCOMPARE(event, QStringLiteral("Event• A & <i>B</i><i>z</i>|0"));
	}

	// With no runway matched the stored distances read back as 0 and mean
	// nothing, so neither the popup nor the AI prompt shows them; with one
	// matched, both do.
	void distancesShowOnlyWithAMatchedRunway() {
		const QString js = QStringLiteral(
			"(function (runway) { var t = {icao: 'EGLL', runway: runway, runwayHeading: -1, lat: 1, lng: 2, airspeed: 140,"
			"    verticalSpeed: 0, pitchDegrees: 0, bankDegrees: 0, headingDegrees: 90, distanceLength: 0, distanceWidth: 0,"
			"    distanceLengthPercent: 0, distanceWidthPercent: 0, windDirection: 0, windVelocity: 0, zuluTime: 'z'};"
			"  var d = document.createElement('div');"
			"  d.innerHTML = runwayContactPopupHtml('x', t, 'Liftoff', null, 'f', 'L').html;"
			"  var keys = Array.prototype.map.call(d.querySelectorAll('.td-key'), function (k) { return k.textContent; });"
			"  return keys.join(',') + '|' + /Centerline offset/.test(buildLiftoffPrompt(t)); })(%1)");
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("''"))).toString(),
			QStringLiteral("Airport,Coordinate,Airspeed,V/S,Pitch,Bank,Heading,Wind,Zulu|false"));
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("'09'"))).toString(),
			QStringLiteral("Airport,Runway,Coordinate,Airspeed,V/S,Pitch,Bank,Heading,Threshold,Centerline,Wind,Zulu|true"));
	}

	// A threshold distance halfway between whole numbers rounds away from
	// zero in the popup and the AI prompt, so a negative one reads the same
	// as its positive mirror (and as in the KML): -2.5 ft is -3 ft, -12.5% is -13%.
	void negativeThresholdDistanceRoundsAwayFromZero() {
		const QString js = QStringLiteral(
			"(function () { var t = {icao: 'EGLL', runway: '27L', runwayHeading: -1, lat: 1, lng: 2, airspeed: 140,"
			"    verticalSpeed: 0, pitchDegrees: 0, bankDegrees: 0, headingDegrees: 90, distanceLength: -2.5, distanceWidth: 0,"
			"    distanceLengthPercent: -0.125, distanceWidthPercent: 0, windDirection: 0, windVelocity: 0, zuluTime: 'z'};"
			"  var d = document.createElement('div');"
			"  d.innerHTML = runwayContactPopupHtml('x', t, 'Liftoff', null, 'f', 'L').html;"
			"  var threshold = Array.prototype.filter.call(d.querySelectorAll('.td-row'), function (r) {"
			"    return r.querySelector('.td-key').textContent === 'Threshold'; })[0].querySelector('.td-val').textContent;"
			"  return threshold + '|' + /: -3 ft \\(-13% of/.test(buildLiftoffPrompt(t)); })()");
		QCOMPARE(evalPageJs(widget_, js).toString(), QStringLiteral("-3 ft (-13%)|true"));
	}

	// The AI prompt names the airport and runway with the popup's labels:
	// "ICAO (Name)" and "runway (heading°)", each part only when known.
	void analysisPromptNamesTheAirportAndRunway() {
		const QString js = QStringLiteral(
			"(function (name, heading) { var t = {icao: 'EGLL', airportName: name, runway: '27L', runwayHeading: heading,"
			"    lat: 1, lng: 2, airspeed: 140, verticalSpeed: 0, pitchDegrees: 0, bankDegrees: 0, headingDegrees: 90,"
			"    distanceLength: 0, distanceWidth: 0, distanceLengthPercent: 0, distanceWidthPercent: 0, windDirection: 0,"
			"    windVelocity: 0, zuluTime: 'z', gForce: 1};"
			"  return buildTouchdownPrompt(t).split('\\n').filter(function (l) {"
			"    return /^  (Airport|Runway): /.test(l); }).join('|'); })(%1)");
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("'Heathrow', 270"))).toString(),
			QStringLiteral("  Airport: EGLL (Heathrow)|  Runway: 27L (270°)"));
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("'', -1"))).toString(),
			QStringLiteral("  Airport: EGLL|  Runway: 27L"));
	}

	// The map's touchdown popup and the KML export's placemark description
	// are built separately (map.html, kml_export.cpp) but list the same
	// fields with the same values; only the popup adds the coordinate, which
	// in the KML is the placemark's own position.
	void touchdownPopupMatchesTheKmlDescription() {
		TouchdownPoint td;
		td.latitude = 51.47;
		td.longitude = -0.45;
		td.icao = QStringLiteral("EGLL");
		td.airportName = QStringLiteral("Heathrow");
		td.runway = QStringLiteral("27L");
		td.runwayHeading = 270;
		td.airspeed = 135;
		td.verticalSpeed = -180;
		td.gForce = 1.234;
		td.pitchDegrees = 3.24;
		td.bankDegrees = -1.46;
		td.headingDegrees = 271;
		td.distanceLength = -2.5;  // halves, where rounding conventions differ
		td.distanceLengthPercent = -0.125;
		td.distanceWidth = -5.6;
		td.distanceWidthPercent = -0.12;
		td.windDirection = 250;
		td.windVelocity = 12;
		td.zuluTime = QStringLiteral("2024-03-15T10:30:00.000+00:00_5");
		td.localTime = QStringLiteral("2024-03-15T10:30:00.000+00:00_5");
		TripDataset dataset;
		dataset.points = { samplePoint(51.47, -0.45) };
		dataset.touchdowns = { td };
		showTrip(dataset);

		const QString popupRowsJs = QStringLiteral(
			"(function () { var rows = [];"
			"  leafletMapInstance.eachLayer(function (l) {"
			"    if (!(l instanceof L.Marker) || l.options.icon.options.className !== 'touchdown-icon') return;"
			"    var d = document.createElement('div'); d.innerHTML = l.getPopup().getContent();"
			"    d.querySelectorAll('.td-row').forEach(function (r) {"
			"      rows.push(r.querySelector('.td-key').textContent + ': ' + r.querySelector('.td-val').textContent); }); });"
			"  return rows; })()");
		QStringList popup;
		QVERIFY(QTest::qWaitFor([&] {
			popup = evalPageJs(widget_, popupRowsJs).toStringList();
			return !popup.isEmpty() && popup.first().startsWith(QStringLiteral("Airport: EGLL"));
		}, 10000));
		QCOMPARE(popup.first(), QStringLiteral("Airport: EGLL (Heathrow)"));
		QVERIFY2(popup.value(2).startsWith(QStringLiteral("Coordinate: ")), qPrintable(popup.value(2)));
		popup.removeAt(2);

		QTemporaryDir dir;
		const QString path = dir.filePath(QStringLiteral("trip.kml"));
		QVERIFY(exportTripDatasetToKmlFile(dataset, path));
		QFile file(path);
		QVERIFY(file.open(QIODevice::ReadOnly));
		const QString kml = QString::fromUtf8(file.readAll());
		const qsizetype folder = kml.indexOf(QStringLiteral("<name>Touchdowns</name>"));
		QVERIFY(folder >= 0);
		const qsizetype start = kml.indexOf(QStringLiteral("<![CDATA["), folder) + 9;
		const QString description = kml.mid(start, kml.indexOf(QStringLiteral("]]>"), start) - start);
		QStringList placemark;
		static const QRegularExpression row(QStringLiteral("<b>(.*?):</b> (.*?)<br/>"));
		for (const QRegularExpressionMatch& m : row.globalMatch(description))
			placemark.append(m.captured(1) + QStringLiteral(": ") + m.captured(2));

		QCOMPARE(popup, placemark);
	}

	// A finished answer from a model that doesn't think is complete: saved
	// as it is, with no thinking panel.
	void aiAnswerWithoutThinkingIsSaved() {
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Grade: A", false } }), aiChunk({ { "\nGood.", false } }, "STOP") }) } });
		QCOMPARE(r.value("calls").toInt(), 1);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Grade: A\nGood."));
		QCOMPARE(r.value("text").toString(), QStringLiteral("Grade: AGood."));
		QCOMPARE(r.value("thinkingShown").toBool(), false);
	}

	// Thinking, then the answer, split across reads mid-object: saved with
	// its thinking section, which the panel shows.
	void aiAnswerWithThinkingIsSavedWithIt() {
		const QString stream = aiStream({ aiChunk({ { "Wind from the left.", true } }), aiChunk({ { "Grade: B", false } }, "STOP") });
		const QVariantMap r = runAiAnalysisWith({ { stream.left(30), stream.mid(30, 40), stream.mid(70) } });
		QCOMPARE(r.value("calls").toInt(), 1);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("<thinking>Wind from the left.</thinking>Grade: B"));
		QCOMPARE(r.value("text").toString(), QStringLiteral("Grade: B"));
		QCOMPARE(r.value("thinkingShown").toBool(), true);
	}

	// An answer stopped early (token limit), or a stream that ends with no
	// finish at all, isn't complete even after thinking: it's retried.
	void anAiAnswerCutOffIsRetried() {
		const QVariantMap r = runAiAnalysisWith({
			{ aiStream({ aiChunk({ { "Thought", true }, { "Grade: half", false } }, "MAX_TOKENS") }) },
			{ aiStream({ aiChunk({ { "Thought", true }, { "Grade: half", false } }) }) },
			{ aiStream({ aiChunk({ { "Grade: C", false } }, "STOP") }) } });
		QCOMPARE(r.value("calls").toInt(), 3);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Grade: C"));
		QCOMPARE(r.value("thinkingShown").toBool(), false);
	}

	// Three incomplete answers in a row: nothing is saved and the message
	// says so; a finish with no answer text counts as incomplete too.
	void threeIncompleteAiAnswersSaveNothing() {
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Thought", true } }, "STOP") }) } });
		QCOMPARE(r.value("calls").toInt(), 3);
		QVERIFY(r.value("saved").isNull());
		QVERIFY(r.value("text").toString().startsWith(QStringLiteral("The AI didn't return a complete analysis.")));
		QCOMPARE(r.value("thinkingShown").toBool(), false);
	}

	// Braces and quotes in the model's text are text, not JSON structure.
	void aiAnswerTextWithBracesIsKeptWhole() {
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Use {x and \"}\" ", false } }),
			aiChunk({ { "then \\ {", false } }, "STOP") }) } });
		QCOMPARE(r.value("calls").toInt(), 1);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Use {x and \"}\" then \\ {"));
	}

	// A real response recorded from the service (84 streamed objects ending in
	// STOP), however the network splits it: the answer, with an unmatched
	// brace, quotes and a backslash in it, is saved whole after all of its
	// thinking.
	void aRecordedAiResponseIsSavedWhole_data() {
		QTest::addColumn<int>("pieceSize");
		QTest::newRow("1 byte") << 1;
		QTest::newRow("7 bytes") << 7;
		QTest::newRow("64 bytes") << 64;
		QTest::newRow("4 KiB") << 4096;
		QTest::newRow("whole") << 0;
	}
	void aRecordedAiResponseIsSavedWhole() {
		QFETCH(int, pieceSize);
		QFile file(QStringLiteral(AI_STREAM_RESPONSE));
		QVERIFY(file.open(QIODevice::ReadOnly));
		const QString response = QString::fromUtf8(file.readAll());
		QString thinking;
		for (const QJsonValue chunk : QJsonDocument::fromJson(response.toUtf8()).array())
			for (const QJsonValue part : chunk["candidates"][0]["content"]["parts"].toArray())
				if (part["thought"].toBool())
					thinking += part["text"].toString();
		QCOMPARE(thinking.size(), 6502);
		QStringList pieces;
		for (qsizetype i = 0; i < response.size(); i += pieceSize ? pieceSize : response.size())
			pieces.append(response.mid(i, pieceSize ? pieceSize : -1));

		const QVariantMap r = runAiAnalysisWith({ pieces });
		const QString answer = QStringLiteral(R"(The plane glides steady {x. The pilot initiates a "flare" \. The landing is buttery smooth}.)");
		QCOMPARE(r.value("calls").toInt(), 1);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("<thinking>") + thinking + QStringLiteral("</thinking>") + answer);
		QCOMPARE(r.value("text").toString(), answer);
		QCOMPARE(r.value("thinkingShown").toBool(), true);
	}

	// A report the database didn't take stays shown, with a note under it
	// that it wasn't saved.
	void anAiAnswerThatCouldNotBeSavedSaysSo() {
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } }, false);
		QCOMPARE(r.value("calls").toInt(), 1);
		const QString text = r.value("text").toString();
		QVERIFY2(text.startsWith(QStringLiteral("Grade: A")), qPrintable(text));
		QVERIFY2(text.contains(QStringLiteral("Couldn't save this analysis")), qPrintable(text));
	}

	// Closing and reopening a popup while its analysis runs rebuilds the
	// popup: the new Analyze button shows the run still going, a second run
	// isn't started, and the answer shows in the reopened popup when it's in.
	void reopeningAPopupMidAnalysisDoesNotRunItTwice() {
		evalPageJs(widget_, QStringLiteral("window._geminiApiKey = 'test-key'")); // the popups are built with it
		TripDataset dataset = tripToSave();
		dataset.liftoffPoints[0].rowId = 5;
		loadTrip(dataset);
		evalPageJs(widget_, QStringLiteral(R"JS(
			window._geminiApiKey = '';
			var id = Object.keys(_liftoffStore)[0];
			var marker = null;
			leafletMapInstance.eachLayer(function (l) {
			    if (l.getPopup && l.getPopup() && String(l.getPopup().getContent()).indexOf('td-btn-' + id + '\x22') >= 0) marker = l;
			});
			var realFetch = window.fetch, realSave = mapBridge.saveLiftoffAnalysisReport;
			var ai = window._dup = { id: id, marker: marker, calls: 0, saves: 0, release: null };
			window.fetch = function () {
			    ai.calls++;
			    return new Promise(function (resolve) { ai.release = resolve; });
			};
			mapBridge.saveLiftoffAnalysisReport = function (rowId, report, onSaved) { ai.saves++; onSaved(true); };
			ai.restore = function () { window.fetch = realFetch; mapBridge.saveLiftoffAnalysisReport = realSave; };
			marker.openPopup();
			ai.firstButton = document.getElementById('td-btn-' + id);
			ai.firstButton.click();
		)JS"));
		QCOMPARE(evalPageJs(widget_, QStringLiteral("window._dup.calls")).toInt(), 1);

		const QString reopened = evalPageJs(widget_, QStringLiteral(R"JS(
			var ai = window._dup;
			ai.marker.closePopup();
			ai.marker.openPopup();
			var btn = document.getElementById('td-btn-' + ai.id);
			analyzeLiftoff(ai.id); // what clicking it would do were it enabled
			[btn !== ai.firstButton, btn.disabled, document.getElementById('td-spin-' + ai.id).style.display, ai.calls].join('|')
		)JS")).toString();
		QCOMPARE(reopened, QStringLiteral("true|true|inline-block|1"));

		evalPageJs(widget_, QStringLiteral(R"JS(
			var stream = %1, ai = window._dup;
			ai.release({ ok: true, status: 200, body: { getReader: function () {
			    var sent = false;
			    return { read: function () {
			        if (sent) return Promise.resolve({ done: true });
			        sent = true;
			        return Promise.resolve({ done: false, value: new TextEncoder().encode(stream) });
			    } };
			} } });
		)JS").arg(QString::fromUtf8(QJsonDocument(QJsonArray{ aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) })
			.toJson(QJsonDocument::Compact)).chopped(1).mid(1)));
		QTRY_VERIFY(!evalPageJs(widget_, QStringLiteral("_liftoffStore[window._dup.id].analyzing")).toBool());
		const QString done = evalPageJs(widget_, QStringLiteral(R"JS(
			var ai = window._dup;
			ai.restore();
			var shown = [ai.calls, ai.saves, document.getElementById('td-btn-' + ai.id).disabled,
			             document.getElementById('td-result-' + ai.id).textContent].join('|');
			ai.marker.closePopup();
			shown
		)JS")).toString();
		QCOMPARE(done, QStringLiteral("1|1|false|Grade: A"));
	}

	// A popup closed while its analysis runs is gone from the page, but the
	// analysis isn't: an answer cut off after its thinking is retried there,
	// the next answer is saved, and the reopened popup shows it with its
	// Analyze button enabled again.
	void anAnalysisRetriedAfterItsPopupClosedIsSaved() {
		evalPageJs(widget_, QStringLiteral("window._geminiApiKey = 'test-key'")); // the popups are built with it
		TripDataset dataset = tripToSave();
		dataset.liftoffPoints[0].rowId = 5;
		loadTrip(dataset);
		const QString streams = QString::fromUtf8(QJsonDocument(QJsonArray{
			aiStream({ aiChunk({ { "Thought", true }, { "Grade: half", false } }) }),
			aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) }).toJson(QJsonDocument::Compact));
		evalPageJs(widget_, QStringLiteral(R"JS(
			window._geminiApiKey = '';
			var id = Object.keys(_liftoffStore)[0];
			var marker = null;
			leafletMapInstance.eachLayer(function (l) {
			    if (l.getPopup && l.getPopup() && String(l.getPopup().getContent()).indexOf('td-btn-' + id + '\x22') >= 0) marker = l;
			});
			var streams = %1;
			var realFetch = window.fetch, realSave = mapBridge.saveLiftoffAnalysisReport;
			var ai = window._closed = { id: id, marker: marker, calls: 0, saved: null, release: null };
			function response(stream) {
			    return { ok: true, status: 200, body: { getReader: function () {
			        var sent = false;
			        return { read: function () {
			            if (sent) return Promise.resolve({ done: true });
			            sent = true;
			            return Promise.resolve({ done: false, value: new TextEncoder().encode(stream) });
			        } };
			    } } };
			}
			window.fetch = function () {
			    var stream = streams[ai.calls++];
			    if (ai.calls > 1) return Promise.resolve(response(stream));
			    return new Promise(function (resolve) { ai.release = function () { resolve(response(stream)); }; });
			};
			mapBridge.saveLiftoffAnalysisReport = function (rowId, report, onSaved) { ai.saved = report; onSaved(true); };
			ai.restore = function () { window.fetch = realFetch; mapBridge.saveLiftoffAnalysisReport = realSave; };
			marker.openPopup();
			document.getElementById('td-btn-' + id).click();
			marker.closePopup();
		)JS").arg(streams));
		// Leaflet fades a closed popup out before it removes it from the page.
		QTRY_VERIFY(evalPageJs(widget_, QStringLiteral("!document.getElementById('td-result-' + window._closed.id)")).toBool());
		evalPageJs(widget_, QStringLiteral("window._closed.release()"));
		QTRY_VERIFY(!evalPageJs(widget_, QStringLiteral("_liftoffStore[window._closed.id].analyzing")).toBool());
		const QString done = evalPageJs(widget_, QStringLiteral(R"JS(
			var ai = window._closed;
			ai.restore();
			ai.marker.openPopup();
			var shown = [ai.calls, ai.saved, document.getElementById('td-btn-' + ai.id).disabled,
			             document.getElementById('td-result-' + ai.id).textContent].join('|');
			ai.marker.closePopup();
			shown
		)JS")).toString();
		QCOMPARE(done, QStringLiteral("2|Grade: A|false|Grade: A"));
	}

	// A server error or rate limit is often gone a moment later: the request
	// is retried, and the answer that follows is saved.
	void aServerErrorOrRateLimitIsRetried_data() {
		QTest::addColumn<int>("status");
		QTest::newRow("internal error") << 500;
		QTest::newRow("unavailable") << 503;
		QTest::newRow("rate limit") << 429;
	}
	void aServerErrorOrRateLimitIsRetried() {
		QFETCH(int, status);
		const QVariantMap r = runAiAnalysisWith({
			{ QStringLiteral(R"([{"error":{"code":%1,"message":"Try later."}}])").arg(status) },
			{ aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } }, true, { status, 200 });
		QCOMPARE(r.value("calls").toInt(), 2);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Grade: A"));
		QCOMPARE(r.value("text").toString(), QStringLiteral("Grade: A"));
	}

	// Three server errors in a row: the last one's message is shown and
	// nothing is saved.
	void threeServerErrorsShowTheLastOne() {
		const QVariantMap r = runAiAnalysisWith({ { QStringLiteral(R"([{"error":{"code":500,"message":"Internal error encountered."}}])") } },
			true, { 500 });
		QCOMPARE(r.value("calls").toInt(), 3);
		QVERIFY(r.value("saved").isNull());
		QCOMPARE(r.value("text").toString(), QStringLiteral("API error 500: Internal error encountered."));
	}

	// A rejected request (bad key, no access) won't change on retry: it's
	// shown at once, pointing at the key in settings.ini.
	void aRejectedRequestIsNotRetried() {
		const QVariantMap r = runAiAnalysisWith({ { QStringLiteral(R"([{"error":{"code":400,"message":"API key not valid."}}])") } },
			true, { 400 });
		QCOMPARE(r.value("calls").toInt(), 1);
		QVERIFY(r.value("saved").isNull());
		const QString text = r.value("text").toString();
		QVERIFY2(text.startsWith(QStringLiteral("API error 400: API key not valid.")), qPrintable(text));
		QVERIFY2(text.contains(QStringLiteral("gemini_api_key")), qPrintable(text));
	}

	// A service that can't be reached (offline) is tried three times, then
	// the message says so and nothing is saved.
	void anUnreachableServiceIsTriedThreeTimesThenSaysSo() {
		const QVariantMap r = runAiAnalysisWith({ { QString() } }, true, { 0 });
		QCOMPARE(r.value("calls").toInt(), 3);
		QVERIFY(r.value("saved").isNull());
		QCOMPARE(r.value("text").toString(), QStringLiteral(
			"Couldn't reach the AI service after 3 attempts. Check your internet connection and try again."));
		QCOMPARE(r.value("thinkingShown").toBool(), false);
	}

	// A connection that drops mid-answer, after thinking arrived, is retried
	// the same way; after the third, the message replaces the thinking panel,
	// whose "Thinking…" would otherwise never finish.
	void aConnectionDroppedAfterThinkingHidesTheThinkingPanel() {
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Thought", true } }) }) } }, true, { -1 });
		QCOMPARE(r.value("calls").toInt(), 3);
		QVERIFY(r.value("saved").isNull());
		QCOMPARE(r.value("text").toString(), QStringLiteral(
			"Couldn't reach the AI service after 3 attempts. Check your internet connection and try again."));
		QCOMPARE(r.value("thinkingShown").toBool(), false);
	}

	// A touchdown analyzed before shows its saved report when its popup
	// opens, with or without an API key: the answer, and the thinking behind
	// a closed "Show thinking" panel only if the report has some.
	void aSavedReportIsShownWhenThePopupOpens_data() {
		QTest::addColumn<QString>("report");
		QTest::addColumn<QString>("shown");
		QTest::newRow("with thinking") << QStringLiteral("<thinking>Wind from the left.</thinking>\nGrade: B")
			<< QStringLiteral("Grade: B|Wind from the left.|closed");
		QTest::newRow("without thinking") << QStringLiteral("Grade: A") << QStringLiteral("Grade: A||none");
	}
	void aSavedReportIsShownWhenThePopupOpens() {
		QFETCH(QString, report);
		QFETCH(QString, shown);
		TripDataset dataset = tripToSave();
		dataset.touchdowns[0].analysisReport = report;
		loadTrip(dataset);
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "touchdown-icon"), 1, 5000);
		const QString popup = evalPageJs(widget_, QStringLiteral(R"JS(
			(function () {
			    var marker = null;
			    leafletMapInstance.eachLayer(function (l) {
			        if (l instanceof L.Marker && l.options.icon.options.className === 'touchdown-icon') marker = l;
			    });
			    marker.openPopup();
			    var id = Object.keys(_touchdownStore)[0];
			    var det = document.getElementById('td-th-det-' + id);
			    var body = document.getElementById('td-th-body-' + id);
			    var shown = [document.getElementById('td-th-final-' + id).textContent, body ? body.textContent : '',
			                 det ? (det.open ? 'open' : 'closed') : 'none'].join('|');
			    marker.closePopup();
			    return shown;
			})())JS")).toString();
		QCOMPARE(popup, shown);
	}

	// The page's console goes to the debug log under MapJS: errors and
	// warnings as WARN, anything else as INFO.
	void pageConsoleMessagesAreLoggedAtTheirLevel() {
		evalPageJs(widget_, QStringLiteral(
			"console.error('console-error-1'); console.warn('console-warn-2'); console.log('console-log-3'); true"));
		const QString log = logPath_;
		QVERIFY(QTest::qWaitFor([&log] { return lineLogged(log, "INFO ", { "MapJS", "console-log-3" }); }, 5000));
		QVERIFY(warningLogged(log, { "MapJS", "console-error-1" }));
		QVERIFY(warningLogged(log, { "MapJS", "console-warn-2" }));
		QVERIFY(!lineLogged(log, "INFO ", { "console-error-1" }));
		QVERIFY(!lineLogged(log, "WARN ", { "console-log-3" }));
	}

	// --- Through the real mouse and the map's right-click menu ---

	// With a trip shown the right-click menu offers Export to KML; the
	// overview of every trip has no single trip to export.
	void rightClickMenuOffersExportOnlyForATrip() {
		loadTrip(tripToSave());
		QCOMPARE(rightClickMenu(), (QStringList{ "Reset Zoom", "Save Image", "Copy Image", "Export to KML" }));
		widget_->showOverview({ overviewTrip(7) });
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "overview-endpoint"), 2, 5000);
		QCOMPARE(rightClickMenu(), (QStringList{ "Reset Zoom", "Save Image", "Copy Image" }));
	}

	// Save Image suggests the trip's file name and saves the whole map as
	// the view shows it: the trajectory line is in the image.
	void saveImageWritesTheMapAsAPng() {
		loadTrip(tripToSave());
		const QString path = filesDir_.filePath(QStringLiteral("map.png"));
		QString suggested;
		rightClickMenu(QStringLiteral("Save Image"), [&](QWidget* dialog) { suggested = saveFileDialogAs(dialog, path); });
		QCOMPARE(suggested, QStringLiteral("KJFK-KLAX_20260102100000.png"));
		const QImage image(path);
		QVERIFY(!image.isNull());
		checkShowsTheTrajectory(image);
	}

	void aMapImageThatCantBeSavedSaysSo() {
		loadTrip(tripToSave());
		const QString path = filesDir_.filePath(QStringLiteral("no-such-folder/map.png"));
		QString error;
		rightClickMenu(QStringLiteral("Save Image"), [&](QWidget* dialog) {
			onNextModal([&error](QWidget* box) {
				error = static_cast<QMessageBox*>(box)->text();
				box->close();
			});
			saveFileDialogAs(dialog, path);
		});
		// Waits out the box handler too, so it can't outlive error.
		QVERIFY(waitFor([&error] { return !error.isEmpty(); }));
		QCOMPARE(error, QStringLiteral("Failed to save the map image to %1.").arg(path));
	}

	// Copy Image puts the same picture on the clipboard.
	void copyImagePutsTheMapOnTheClipboard() {
		loadTrip(tripToSave());
		const ClipboardGuard keepUsersClipboard;
		QClipboard* clipboard = QGuiApplication::clipboard();
		clipboard->clear();
		rightClickMenu(QStringLiteral("Copy Image"));
		const QImage image = clipboard->image();
		QVERIFY(!image.isNull());
		checkShowsTheTrajectory(image);
	}

	// With text selected on the page the menu starts with Copy, which copies
	// the selection and then clears it.
	void copyCopiesTheSelectedTextAndClearsTheSelection() {
		const ClipboardGuard keepUsersClipboard;
		QClipboard* clipboard = QGuiApplication::clipboard();
		clipboard->clear();
		const QPoint middle = addTestElement(QStringLiteral("Selected words"));
		evalPageJs(widget_, QStringLiteral("getSelection().selectAllChildren(document.getElementById('menu-test')); true"));
		QMenu* menu = openRightClickMenu(middle);
		QVERIFY(menu);
		QCOMPARE(menu->actions().value(0)->text(), QStringLiteral("Copy"));
		chooseMenuItem(menu, QStringLiteral("Copy"));
		QTRY_COMPARE_WITH_TIMEOUT(clipboard->text(), QStringLiteral("Selected words"), 5000);
		QTRY_COMPARE_WITH_TIMEOUT(evalPageJs(widget_, QStringLiteral("getSelection().toString()")).toString(), QString(), 5000);
		removeTestElement();
	}

	// Right-clicking a link on the page adds Copy Link, which copies its address.
	void copyLinkCopiesTheLinksAddress() {
		const ClipboardGuard keepUsersClipboard;
		QClipboard* clipboard = QGuiApplication::clipboard();
		clipboard->clear();
		const QPoint middle = addTestElement(QStringLiteral("<a href=\"https://example.com/route\">a link</a>"));
		QMenu* menu = openRightClickMenu(middle);
		QVERIFY(menu);
		const QString copyLink = mapView()->pageAction(QWebEnginePage::CopyLinkToClipboard)->text();
		QCOMPARE(menu->actions().last()->text(), copyLink);
		chooseMenuItem(menu, copyLink);
		QTRY_COMPARE_WITH_TIMEOUT(clipboard->text(), QStringLiteral("https://example.com/route"), 5000);
		removeTestElement();
	}

	// The menu is a popup, so the map can switch to the overview while it's
	// open: Export to KML then has no trip to export and does nothing.
	void exportToKmlChosenAfterSwitchingToTheOverviewDoesNothing() {
		loadTrip(tripToSave());
		QMenu* menu = openRightClickMenu(QPoint(60, 250));
		QVERIFY(menu);
		widget_->showOverview({ overviewTrip(7) });
		bool dialogShown = false;
		onNextModal([&dialogShown](QWidget* dialog) {
			dialogShown = true;
			dialog->close();
		});
		chooseMenuItem(menu, QStringLiteral("Export to KML")); // a dialog it opened would run in here
		QVERIFY(!dialogShown);
	}

	// Export to KML from the map exports the shown trip from the database
	// (its 4 recorded samples, not the 2 points the map was given), under
	// the trip's file name.
	void exportToKmlFromTheMap() {
		FlightDriver sim;
		TripDataset dataset = tripToSave();
		dataset.tripId = sim.startTrip();
		sim.ticks(3);
		sim.endTrip();
		loadTrip(dataset);
		const QString path = filesDir_.filePath(QStringLiteral("map.kml"));
		QString suggested;
		rightClickMenu(QStringLiteral("Export to KML"), [&](QWidget* dialog) { suggested = saveFileDialogAs(dialog, path); });
		QCOMPARE(suggested, QStringLiteral("KJFK-KLAX_20260102100000.kml"));
		QFile file(path);
		QVERIFY(QTest::qWaitFor([&file] { return file.size() > 0 && file.open(QIODevice::ReadOnly); }, 5000));
		const QString kml = QString::fromUtf8(file.readAll());
		QVERIFY2(kml.trimmed().endsWith("</kml>"), qPrintable(kml.right(200)));
		QCOMPARE(kml.count("<when>"), 4);
	}

	// Clicking a trip's route on the overview asks for that trip.
	void clickingAnOverviewRouteAsksForThatTrip() {
		QSignalSpy clicked(widget_, &MapWidget::overviewTripClicked);
		widget_->showOverview({ overviewTrip(7) });
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "overview-endpoint"), 2, 5000);
		QTRY_VERIFY_WITH_TIMEOUT(mapFittedTo(widget_, 10, 20, 11, 21), 5000);
		QTest::mouseClick(mapInput(), Qt::LeftButton, Qt::NoModifier, midpointOnScreen(10, 20, 11, 21));
		QTRY_COMPARE_WITH_TIMEOUT(clicked.count(), 1, 5000);
		QCOMPARE(clicked.value(0).value(0).toInt(), 7);
	}

	// --- A page reload (the page is rebuilt from scratch) ---

	// The overview is drawn again on the new page.
	void aReloadedPageShowsTheOverviewAgain() {
		widget_->showOverview({ overviewTrip(7) });
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "overview-endpoint"), 2, 5000);
		reloadPage();
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "overview-endpoint"), 2, 5000);
		QCOMPARE(mapTrajectoryPointCount(widget_), 0);
	}

	// The trip is drawn again with its cursor where the user left it, not
	// back at the first sample.
	void aReloadedPageKeepsTheTripsCursor() {
		QSignalSpy loaded(widget_, &MapWidget::trajectoryLoaded);
		QSignalSpy cursor(widget_, &MapWidget::cursorIndexChanged);
		TripDataset dataset;
		dataset.points = { samplePoint(10, 20), samplePoint(11, 21), samplePoint(12, 22) };
		showTrip(dataset);
		QVERIFY(loaded.wait(10000));
		evalPageJs(widget_, QStringLiteral(
			"leafletMapInstance.eachLayer(function (l) { if (l instanceof L.Polyline) l.fire('click', { latlng: L.latLng(12, 22) }); })"));
		QTRY_COMPARE_WITH_TIMEOUT(cursor.count(), 1, 5000);
		reloadPage();
		QTRY_COMPARE_WITH_TIMEOUT(mapTrajectoryPointCount(widget_), 3, 10000);
		QTRY_COMPARE_WITH_TIMEOUT(evalPageJs(widget_, QStringLiteral(
			"var at = null; leafletMapInstance.eachLayer(function (l) { if (l instanceof L.Marker && l.dragging && l.dragging.enabled()) at = l.getLatLng(); });"
			"at ? at.lat + ',' + at.lng : ''")).toString(), QStringLiteral("12,22"), 5000);
	}
};

int main(int argc, char* argv[]) {
	QApplication::setAttribute(Qt::AA_ShareOpenGLContexts, true);
	QApplication app(argc, argv);
	TstMapWidget tc;
	return QTest::qExec(&tc, argc, argv);
}

#include "tst_map_widget.moc"
