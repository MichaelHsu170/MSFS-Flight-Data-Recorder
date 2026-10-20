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
#include "app_settings.h"
#include "engine_power.h"
#include "kml_export.h"
#include "logger.h"
#include "map_bridge.h"
#include "map_widget.h"
#include "test_support.h"
#include "trip_data_fields.h"

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
#include <QScopeGuard>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QWebEngineView>
#include <QtTest>

#include <cmath>
#include <functional>

using namespace TestSupport;

namespace {

// JS defining contactMarker(id): the map marker whose popup holds point id's
// Analyze button. Goes before the script that calls it.
const QString kContactMarkerJs = QStringLiteral(
	"function contactMarker(id) {"
	"    var marker = null;"
	"    leafletMapInstance.eachLayer(function (l) {"
	"        if (l.getPopup && l.getPopup() && String(l.getPopup().getContent()).indexOf('td-btn-' + id + '\\x22') >= 0) marker = l;"
	"    });"
	"    return marker;"
	"}\n");

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

// A touchdown's fields as map_script.cpp sends them, with its runway end's
// data, every derived condition and a short trajectory, as a JavaScript
// object literal.
QString fullContactJs() {
	return QStringLiteral(R"js({icao: 'OEMA', airportName: 'Prince Mohammad bin Abdulaziz', runway: '34C', runwayHeading: 340,
		lat: 24.5528, lng: 39.7051, airspeed: 135, verticalSpeed: -180, pitchDegrees: 3.2, bankDegrees: -1.4,
		headingDegrees: 334, distanceLength: 1250, distanceWidth: -5.6, distanceLengthPercent: 0.11, distanceWidthPercent: -0.12,
		windDirection: 299, windVelocity: 12, gForce: 1.21, actualVerticalSpeed: -95.4, touchdownRate: 262.5,
		runwayLengthFt: 10000.4, runwayWidthFt: 150.4, thresholdOffsetFt: 820.4, runwaySlope: -0.314,
		zuluTime: '2026-06-21T00:00:00.000+00:00_1', localTime: '2026-06-21T03:00:00.000+03:00_1',
		conditions: {headingTrue: 338.4, trackTrue: 340.6, magneticVariation: 4.4, crabAngle: -2.2, runwayHeadingMagnetic: 336,
			sunElevation: -3.21, temperatureC: 31.5, stationPressureHg: 29.12, altimeterHg: 29.92, altimeterMb: 1013.2,
			pressureAltitudeFt: 1949.6, densityAltitudeFt: 4210.4, visibilityM: 9999.4, fieldElevationFt: 2100.4,
			seaLevelPressureMb: 1008.6, inCloud: 1, precipitation: 'rain', precipitationRate: 2.46,
			windMinKt: 8.4, windMaxKt: 14.6, windDirectionFrom: 350.4, windDirectionTo: 20.4,
			surfaceType: 'asphalt', surfaceCondition: 'wet', totalWeightLb: 145000.4, fuelWeightLb: 21000.6},
		profile: {gates: [
				{gate: '500 ft', seconds: -42.5, heightFt: 512.4, distanceNm: 1.61, terrainFt: -20.4, airspeed: 131.4,
					verticalSpeed: -702.6, pitch: 2.04, bank: -1.26, headingTrue: 338.4, trackTrue: 340.6,
					thresholdFt: -8640.4, centerlineFt: -12.6, windDirection: 299.4, windSpeedKt: 12.4, headwindKt: 6.04,
					crosswindKt: -10.6, verticalWindFpm: -310.6, temperatureC: 31.46, inCloud: 1, precipitation: 'rain', structuralIce: 0.024},
				{gate: '1 nm', seconds: -27, heightFt: 318.4, distanceNm: 1.0, pathAngle: 3.04, windSpeedKt: 9, inCloud: 0}],
			series: [{seconds: -0.5, heightFt: 2.4, terrainFt: 30.4, groundSpeedKt: 128.6, gForce: 1.236, windSpeedKt: 9, inCloud: 0}],
			segment: {samples: 60, windMinKt: 8.4, windMaxKt: 22.6, headwindFromKt: 18.4, headwindFromHeightFt: 980.4,
				headwindToKt: 2.6, headwindToHeightFt: 120.4, maxTailwindKt: -2.6, maxCrosswindKt: -11.4, cloudKnown: true,
				lowestInCloudFt: 640.4, icingConditions: 0, maxStructuralIce: 0.024, maxPitotIce: 0.034, engineAntiIceShare: 0.25,
				gForceMin: 0.874, gForceMax: 1.236, verticalWindMinFpm: -420.4, verticalWindMinHeightFt: 410.4,
				verticalWindMaxFpm: 130.6, verticalWindMaxHeightFt: 150.6, stallMarginAlphaDeg: 9.46, stallMarginStallAlphaDeg: 14.04, stallMarginHeightFt: 60.4},
			highestTerrainFt: 612.4, highestTerrainNm: 4.2,
			indicatedVerticalSpeed: false}})js");
}

}

class TstMapWidget : public QObject {
	Q_OBJECT

	MapWidget* widget_ = nullptr;
	// Set when the first slot's page never loaded, so every later slot fails
	// at once instead of waiting out its timeouts against a page that isn't
	// there.
	bool pageNeverLoaded_ = false;
	QTemporaryDir logDir_;
	QTemporaryDir filesDir_; // files saved through the map's menu
	QString logPath_;
	TripDataset shown_; // see showTrip()

	// Runs the page's AI analysis of a liftoff with fetch() stubbed: attempt n
	// gets attempts[n] (the last one repeated), each delivered in the given
	// pieces with HTTP status statuses[n] (the last one repeated; 200 if
	// none; 0 for a request that never reaches the service, as when offline;
	// -1 for a 200 whose connection drops after its pieces; -2 for a request
	// that gets no response until it's aborted). A piece "<pause N>" delays
	// the next by N ms; "<hang>" is a read that never returns until the
	// request is aborted. The page's inactivity timeout is 1 s meanwhile.
	// Saving the report succeeds if saveOk; the trajectory loaded first is
	// profile (JS, "null" for none), and window._aiDuringLoad, if set, is
	// called while it loads. Returns what it did: calls (fetches made), saved (the
	// report saved, null if none), text (the answer shown), thinkingShown,
	// the last request's url and apiKey (its x-goog-api-key header),
	// profileRow (the row whose trajectory was asked for), statuses (the
	// result area's text while the trajectory loaded, then the status line's
	// at each request and each attempt's first read), statusShown (after),
	// statusShownAtStreamEnd (when the last attempt's stream ended),
	// thinkingPanel (the thinking panel's summary at each read while the
	// panel shows and no answer has: its pointer-events, arrow and label,
	// comma-joined, '|'-separated), thinkingSummary (the same, after),
	// console (the page's console.warn/console.error messages about Gemini,
	// in order, each after "warn: " or "error: "), info (its console.log
	// messages about Gemini, each after "log: "),
	// noticeShown (the no-trajectory notice), promptProfile (t.profile
	// when the prompt was built, as JSON), contactOnly (the prompt builder's
	// second argument: whether a contact evaluation was asked for) and
	// fetchedAt (Date.now() at the last request). Returns an empty map if the
	// analysis hasn't finished within 60 s (a recorded response read a byte at
	// a time takes over 10 s on a slow run).
	QVariantMap runAiAnalysisWith(const QList<QStringList>& attempts, bool saveOk = true, const QList<int>& statuses = {},
		const QString& profile = QStringLiteral("{gates: [], series: []}")) {
		QJsonArray json;
		for (const QStringList& pieces : attempts)
			json.append(QJsonArray::fromStringList(pieces));
		QJsonArray jsonStatuses;
		for (int status : statuses)
			jsonStatuses.append(status);
		evalPageJs(widget_, QStringLiteral(R"JS(
			(function (attempts, statuses, profile) {
			    window._ai = { calls: 0, saved: null, done: false, statuses: [], thinkingPanel: [], console: [], info: [] };
			    var realConsole = { log: console.log, warn: console.warn, error: console.error };
			    ['log', 'warn', 'error'].forEach(function (level) {
			        console[level] = function () {
			            var text = Array.prototype.join.call(arguments, ' ');
			            if (text.indexOf('Gemini') >= 0) window._ai[level === 'log' ? 'info' : 'console'].push(level + ': ' + text);
			            realConsole[level].apply(console, arguments);
			        };
			    });
			    var realIdleTimeout = AI_IDLE_TIMEOUT_MS;
			    AI_IDLE_TIMEOUT_MS = 1000;
			    function statusText() { return document.getElementById('td-status-ai').textContent; }
			    function summaryState() {
			        if (!document.getElementById('td-th-sum-ai')) return '';
			        return [document.getElementById('td-th-sum-ai').style.pointerEvents,
			                document.getElementById('td-th-arr-ai').textContent,
			                document.getElementById('td-th-lbl-ai').textContent].join(',');
			    }
			    window._ai.summaryState = summaryState;
			    function recordThinkingPanel() {
			        var det = document.getElementById('td-th-det-ai');
			        if (det && det.style.display !== 'none' && !document.getElementById('td-th-final-ai').textContent)
			            window._ai.thinkingPanel.push(summaryState());
			    }
			    var realFetch = window.fetch;
			    window.fetch = function (url, init) {
			        window._ai.url = url;
			        window._ai.fetchedAt = Date.now();
			        window._ai.apiKey = init.headers['x-goog-api-key'];
			        window._ai.statuses.push(statusText());
			        var n = window._ai.calls++;
			        var status = statuses.length ? statuses[Math.min(n, statuses.length - 1)] : 200;
			        // Never settles until the request is aborted, then rejects as the browser does.
			        function untilAborted() {
			            return new Promise(function (resolve, reject) {
			                init.signal.addEventListener('abort', function () {
			                    reject(new DOMException('signal is aborted without reason', 'AbortError'));
			                });
			            });
			        }
			        if (status === 0) return Promise.reject(new TypeError('Failed to fetch'));
			        if (status === -2) return untilAborted();
			        var drops = status === -1;
			        if (drops) status = 200;
			        var pieces = attempts[Math.min(n, attempts.length - 1)].slice();
			        var firstRead = true;
			        function serve() {
			            if (pieces[0] === '<hang>') return untilAborted();
			            var pause = /^<pause (\d+)>$/.exec(pieces[0] || '');
			            if (pause) {
			                pieces.shift();
			                return new Promise(function (resolve) { setTimeout(function () { resolve(serve()); }, Number(pause[1])); });
			            }
			            if (pieces.length) return Promise.resolve({ done: false, value: new TextEncoder().encode(pieces.shift()) });
			            return null;
			        }
			        var reader = { read: function () {
			            if (firstRead) { firstRead = false; window._ai.statuses.push(statusText()); }
			            recordThinkingPanel();
			            var served = serve();
			            if (served) return served;
			            window._ai.statusShownAtStreamEnd = document.getElementById('td-status-ai').style.display !== 'none';
			            return drops ? Promise.reject(new TypeError('network error')) : Promise.resolve({ done: true });
			        } };
			        return Promise.resolve({ ok: status < 300, status: status,
			            text: function () { return Promise.resolve(pieces.join('')); },
			            body: { getReader: function () { return reader; } } });
			    };
			    var box = document.getElementById('ai-test');
			    if (!box) { box = document.createElement('div'); box.id = 'ai-test'; document.body.appendChild(box); }
			    box.innerHTML = '<button id="td-btn-ai"></button><span id="td-spin-ai"></span><div id="td-note-ai"></div>'
			        + '<div id="td-result-ai"></div>';
			    runAiAnalysis('ai', { rowId: 5 }, 'climb-out', function (rowId, onLoaded) {
			            window._ai.profileRow = rowId;
			            window._ai.statuses.push(document.getElementById('td-result-ai').textContent);
			            if (window._aiDuringLoad) window._aiDuringLoad();
			            setTimeout(function () { onLoaded(profile); }, 0);
			        }, function (t, contactOnly) { window._ai.promptProfile = JSON.stringify(t.profile); window._ai.contactOnly = contactOnly; return 'prompt'; },
			        function (rowId, report, onSaved) { window._ai.saved = report; onSaved(%2); })
			        .then(function () {
			            window.fetch = realFetch;
			            console.log = realConsole.log;
			            console.warn = realConsole.warn;
			            console.error = realConsole.error;
			            AI_IDLE_TIMEOUT_MS = realIdleTimeout;
			            window._ai.done = true;
			        });
			})(%1, %3, %4))JS").arg(QString::fromUtf8(QJsonDocument(json).toJson(QJsonDocument::Compact)),
				saveOk ? QStringLiteral("true") : QStringLiteral("false"),
				QString::fromUtf8(QJsonDocument(jsonStatuses).toJson(QJsonDocument::Compact)), profile));
		if (!QTest::qWaitFor([this] { return evalPageJs(widget_, QStringLiteral("window._ai.done")).toBool(); }, 60000))
			return {};
		return evalPageJs(widget_, QStringLiteral(
			"({ calls: window._ai.calls, saved: window._ai.saved, url: window._ai.url, apiKey: window._ai.apiKey,"
			"   text: document.getElementById('td-th-final-ai').textContent,"
			"   thinkingShown: document.getElementById('td-th-det-ai').style.display !== 'none',"
			"   thinkingPanel: window._ai.thinkingPanel.join('|'), thinkingSummary: window._ai.summaryState(),"
			"   console: window._ai.console, info: window._ai.info,"
			"   profileRow: window._ai.profileRow, statuses: window._ai.statuses.join('|'), promptProfile: window._ai.promptProfile,"
			"   contactOnly: window._ai.contactOnly,"
			"   fetchedAt: window._ai.fetchedAt,"
			"   statusShown: document.getElementById('td-status-ai').style.display !== 'none',"
			"   statusShownAtStreamEnd: window._ai.statusShownAtStreamEnd,"
			"   noticeShown: document.getElementById('td-note-ai').style.display !== 'none' })")).toMap();
	}

	// Checks runAiAnalysisWith()'s logged failures, one per failed attempt in
	// order: retried ones as a warning saying so, the last as an error that
	// doesn't, each carrying its values (the reason, what was received).
	struct AttemptLog {
		bool retried;
		QStringList values;
	};
	// An error object the service can send mid-stream, as JSON, which is how
	// a failed attempt's log line carries it after "stream error ".
	static constexpr const char* streamError500 = R"({"code":500,"message":"Internal error encountered.","status":"INTERNAL"})";

	static void verifyAttemptLogs(const QVariantMap& r, const QList<AttemptLog>& expected) {
		const QStringList lines = r.value("console").toStringList();
		QCOMPARE(lines.size(), expected.size());
		for (int i = 0; i < lines.size(); ++i) {
			const QString& line = lines[i];
			QVERIFY2(line.startsWith(expected[i].retried ? QStringLiteral("warn: ") : QStringLiteral("error: ")), qPrintable(line));
			QVERIFY2(line.contains(QStringLiteral("retrying")) == expected[i].retried, qPrintable(line));
			for (const QString& value : expected[i].values)
				QVERIFY2(line.contains(value), qPrintable(value + QStringLiteral(" not in: ") + line));
		}
	}

	// Records each "Waiting N s for the rate limit…" status
	// runAiAnalysisWith()'s analysis shows, for rateLimitWaits().
	void watchRateLimitWaits() {
		evalPageJs(widget_, QStringLiteral(R"JS(
			window._waitsSeen = [];
			window._waitsWatch = setInterval(function () {
			    var s = document.getElementById('td-status-ai');
			    if (!s || !/Waiting \d/.test(s.textContent) || _waitsSeen.indexOf(s.textContent) >= 0) return;
			    _waitsSeen.push(s.textContent);
			}, 20);
		)JS"));
	}
	QString rateLimitWaits() { return evalPageJs(widget_, QStringLiteral("window._waitsSeen.join('|')")).toString(); }

	// Moves the real mouse, with no button pressed, to pos in the view's
	// coordinates (QTest::mouseMove() doesn't reach the page).
	void moveMouse(const QPoint& pos) {
		QMouseEvent move(QEvent::MouseMove, pos, mapInput()->mapToGlobal(pos), Qt::NoButton, Qt::NoButton, Qt::NoModifier);
		QApplication::sendEvent(mapInput(), &move);
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

	// tripToSave() with the liftoff's samples timed so that its climb-out
	// trajectory can be built (rolling at 500 ft, then 60 ft up 0.5 s after the
	// liftoff: the 35 ft gate), and the touchdown's not. The liftoff and the
	// touchdown are both row 5 (each table numbers its own rows), so a
	// touchdown's trajectory looked up as a liftoff's is the climb-out, not
	// none.
	static TripDataset tripWithClimbOut() {
		TripDataset dataset = tripToSave();
		dataset.liftoffPoints[0].rowId = 5;
		dataset.touchdowns[0].rowId = 5;
		const char* times[] = { "2026-01-02T10:00:00.000+00:00_1", "2026-01-02T10:00:01.000+00:00_1" };
		const double altitudes[] = { 500, 560 };
		for (int i = 0; i < 2; ++i) {
			TripSamplePoint& p = dataset.points[i];
			p.zuluTime = QString::fromLatin1(times[i]);
			p.rawNums.assign(TRIP_NUM_FIELD_COUNT, std::nan(""));
			p.rawNums[TRIP_NUM_plane_altitude] = altitudes[i];
			p.rawNums[TRIP_NUM_plane_alt_above_ground] = altitudes[i] - 500;
		}
		dataset.liftoffPoints[0].zuluTime = QStringLiteral("2026-01-02T10:00:00.500+00:00_1");
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

	// The lines in expected (a JS array of strings) that the touchdown prompt
	// for fullContactJs() as t, after the JS statements setup, lacks; "no
	// result" if the page gave none (it threw, or isn't loaded), so that can't
	// pass as nothing lacking.
	QStringList touchdownPromptLacks(const QString& setup, const QString& expected) {
		const QVariant lacking = evalPageJs(widget_, QStringLiteral(
			"(function () { var t = %1; %2"
			"  var lines = buildTouchdownPrompt(t).split('\\n');"
			"  return %3.filter(function (l) { return lines.indexOf(l) < 0; }); })()").arg(fullContactJs(), setup, expected));
		return lacking.isValid() ? lacking.toStringList() : QStringList({ QStringLiteral("no result") });
	}

private slots:
	// Logger::init() takes effect once per process, so it runs here.
	void initTestCase() {
		isolateFiles(); // the database and settings.ini the map's menu reads
		// Save dialogs a test can answer (see saveFileDialogAs()).
		QCoreApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
		logPath_ = logDir_.filePath(QStringLiteral("map.log"));
		Logger::init(Logger::Info, logPath_);
		// The page gets the model and evaluation choices saved before it
		// loads (checked in the first slot).
		AppSettings::instance().setAiModel(QStringLiteral("other"));
		AppSettings::instance().setAiCustomModel(QStringLiteral("saved-model"));
		AppSettings::instance().setAiProgressEvaluation(false);
		widget_ = new MapWidget;
		widget_->resize(400, 300);
		widget_->show();
	}

	void cleanupTestCase() {
		delete widget_;
	}
	void init() {
		if (pageNeverLoaded_)
			QFAIL("the map page never loaded (see the first slot)");
	}
	void cleanup() {
		cancelPendingModals(); // no dialog action outlives its test
		// reopeningAPopupMidAnalysisDoesNotRunItTwice() stubs the page's fetch
		// and report save; if it failed midway, put the real ones back.
		evalPageJs(widget_, QStringLiteral(
			"if (window._dup) { window._dup.restore(); window._dup.marker.closePopup(); window._dup = null; } true"));
		// No watchRateLimitWaits() left running.
		evalPageJs(widget_, QStringLiteral("clearInterval(window._waitsWatch); true"));
		// And with a progress evaluation chosen, the default.
		evalPageJs(widget_, QStringLiteral("window._aiProgress = true"));
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
		// qWaitFor, not QTRY_*: a failing QTRY keeps waiting to report whether
		// a longer timeout would have passed.
		if (!QTest::qWaitFor([this] { return mapTrajectoryPointCount(widget_) == 1; }, 15000)) {
			pageNeverLoaded_ = true;
			QFAIL("the map page never showed the trajectory");
		}
		QCOMPARE(spy.count(), 1);

		// The loaded page has the model and evaluation choices settings.ini
		// had (initTestCase()); later slots start from the defaults, also
		// after a page reload.
		QCOMPARE(evalPageJs(widget_, QStringLiteral(
			"var saved = [window._aiModel, window._aiCustomModel, window._aiProgress].join('|');"
			" window._aiModel = ''; window._aiCustomModel = ''; window._aiProgress = true; saved")).toString(),
			QStringLiteral("other|saved-model|false"));
		AppSettings::instance().setAiProgressEvaluation(true);
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

	// A touchdown's notice names its approach (a liftoff's, its climb-out).
	void touchdownNoticeNamesTheApproach() {
		const QString note = evalPageJs(widget_, QStringLiteral(
			"(function () { var d = document.createElement('div');"
			"  d.innerHTML = runwayContactPopupHtml('x', {icao: 'X', lat: 1, lng: 2, airspeed: 1, verticalSpeed: 0,"
			"    pitchDegrees: 0, bankDegrees: 0, headingDegrees: 0, windDirection: 0, windVelocity: 0, zuluTime: 'z'},"
			"    'Touchdown', null, 'analyzeTouchdown').html;"
			"  return d.querySelector('#td-note-x').textContent.replace(/\\s+/g, ' ').trim(); })()")).toString();
		QCOMPARE(note, QStringLiteral("The approach trajectory couldn't be built from this trip's recorded samples,"
			" so this analysis covers the touchdown itself only."));
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
	// "ICAO (Name)" and "runway (magnetic, true heading)", each part only when
	// known: the magnetic one needs the contact's sample.
	void analysisPromptNamesTheAirportAndRunway() {
		const QString js = QStringLiteral(
			"(function (name, heading, conditions) { var t = {icao: 'EGLL', airportName: name, runway: '27L',"
			"    runwayHeading: heading, lat: 1, lng: 2, airspeed: 140, verticalSpeed: 0, pitchDegrees: 0, bankDegrees: 0,"
			"    headingDegrees: 90, distanceLength: 0, distanceWidth: 0, distanceLengthPercent: 0, distanceWidthPercent: 0,"
			"    windDirection: 0, windVelocity: 0, zuluTime: 'z', gForce: 1, conditions: conditions};"
			"  return buildTouchdownPrompt(t).split('\\n').filter(function (l) {"
			"    return /^  (Airport|Runway): /.test(l); }).join('|'); })(%1)");
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("'Heathrow', 270, {runwayHeadingMagnetic: 268}"))).toString(),
			QStringLiteral("  Airport: EGLL (Heathrow)|  Runway: 27L (268° magnetic, 270° true)"));
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("'Heathrow', 270, undefined"))).toString(),
			QStringLiteral("  Airport: EGLL (Heathrow)|  Runway: 27L (270° true)"));
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("'', -1, {runwayHeadingMagnetic: 268}"))).toString(),
			QStringLiteral("  Airport: EGLL|  Runway: 27L"));
	}

	// The popup marks each heading magnetic (M) then true (T): the runway's
	// magnetic heading and the aircraft's true heading come from the
	// contact's sample, with the crab angle; without a sample, only the
	// runway's true heading, the stored magnetic heading and no crab.
	void popupHeadingsAreMarkedTrueOrMagnetic() {
		const QString js = QStringLiteral(
			"(function (conditions) { var t = {icao: '', runway: '34C', runwayHeading: 340, lat: 1, lng: 2, airspeed: 140,"
			"    verticalSpeed: 0, pitchDegrees: 0, bankDegrees: 0, headingDegrees: 334, distanceLength: 0, distanceWidth: 0,"
			"    distanceLengthPercent: 0, distanceWidthPercent: 0, windDirection: 299, windVelocity: 12, zuluTime: 'z',"
			"    conditions: conditions};"
			"  var d = document.createElement('div');"
			"  d.innerHTML = runwayContactPopupHtml('x', t, 'Touchdown', null, 'f', 'L').html;"
			"  return Array.prototype.map.call(d.querySelectorAll('.td-row'), function (r) {"
			"    return r.querySelector('.td-key').textContent + ': ' + r.querySelector('.td-val').textContent; })"
			"    .filter(function (r) { return /^(Runway|Heading|Crab|Wind):/.test(r); }).join('|'); })(%1)");
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("{headingTrue: 338.4, crabAngle: -2.6, runwayHeadingMagnetic: 336}"))).toString(),
			QStringLiteral("Runway: 34C (336° M, 340° T)|Heading: 334° M (338° T)|Crab: 3° L|Wind: 299° T / 12 kt"));
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("{headingTrue: 338.4, crabAngle: 1.6}"))).toString(),
			QStringLiteral("Runway: 34C (340° T)|Heading: 334° M (338° T)|Crab: 2° R|Wind: 299° T / 12 kt"));
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("{headingTrue: 338.4, crabAngle: 0.4}"))).toString(),
			QStringLiteral("Runway: 34C (340° T)|Heading: 334° M (338° T)|Crab: 0°|Wind: 299° T / 12 kt"));
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("undefined"))).toString(),
			QStringLiteral("Runway: 34C (340° T)|Heading: 334° M|Wind: 299° T / 12 kt"));
	}

	// The touchdown prompt carries the conditions at the contact and the
	// final trajectory (map_script.cpp's "conditions" and "profile"), one
	// line per recorded value, every heading marked true or magnetic.
	void touchdownPromptCarriesConditionsAndTrajectory() {
		QCOMPARE(touchdownPromptLacks(QString(), QStringLiteral(R"js([
				'  Vertical speed: -95 ft/min',
				'  Touchdown rate: 263 ft/min (the sink rate perpendicular to the runway surface as the wheels touched, as the simulator measured it)',
				'  Heading: 334° magnetic (338° true); ground track 341° true; crab 2.2° (nose left of the track)',
				'  Wind: 299° true at 12 kt',
				'  Runway: 10000 ft long, 150 ft wide, threshold displaced 820 ft at this end, slope 0.31° downhill in the direction of travel',
				'  Runway remaining ahead: 7930 ft (79% of the runway length)',
				'Conditions:',
				'  Time: 2026-06-21T00:00:00.000+00:00 (local 2026-06-21T03:00:00.000+03:00)',
				'  Sun: 3.2° below the horizon (civil twilight)',
				'  Position: 24.5528° N, 39.7051° E; magnetic variation 4.4° E',
				'  Ground elevation at the contact: 2100 ft',
				'  Outside air temperature: 31.5 °C',
				'  Pressure: altimeter setting 29.92 inHg (1013 mb); station pressure 29.12 inHg; actual sea-level pressure 1009 mb',
				'  Altitudes: pressure altitude 1950 ft; density altitude 4210 ft',
				'  Visibility: 9999 m',
				'  Cloud: in cloud',
				'  Precipitation: rain, rate 2.5 mm of water',
				'  Wind over the 30 s before the contact: 8-15 kt, direction 350° to 20° true (clockwise)',
				'  Surface under the aircraft: asphalt, wet',
				'  Aircraft weight: total 145000 lb; fuel 21001 lb',
				'Final trajectory (seconds from the touchdown point; height = altitude above the touchdown point, not radio height; distance = to the touchdown point; terrain = ground below relative to the touchdown point; heading/track = true; V/S = actual vertical speed; wind components = relative to the runway, not the aircraft\'s heading; threshold = along the runway from the marked threshold, negative = before it; centerline = from the extended runway centerline):',
				'  Gates:',
				'    500 ft: -42.5 s, height 512 ft, 1.61 nm, terrain -20 ft, IAS 131 kt, V/S -703 ft/min, pitch 2.0°, bank -1.3°, heading 338°, track 341°, threshold -8640 ft, centerline 13 ft left, wind 299° T 12 kt (headwind 6 kt, crosswind 11 kt from the left), vertical wind 311 ft/min down, OAT 31.5 °C, in cloud, precipitation rain, structural ice 2%',
				'    1 nm: -27.0 s, height 318 ft, 1.00 nm, path angle 3.0° from here',
				'  Samples, from 15 s before to 10 s after the touchdown (every sample, evenly thinned if too dense), then one per second to 30 s after it:',
				'    -0.5 s, height 2 ft, terrain +30 ft, GS 129 kt, 1.24 G',
				'  Approach weather and ride (from the last crossing of 1000 ft above the touchdown point down to it, or from 5 nm out, the previous contact or the start of the recording if closer):',
				'    Wind speed: 8-23 kt',
				'    Largest headwind change: headwind 18 kt at height 980 ft to headwind 3 kt at height 120 ft (a loss of 16 kt: wind shear)',
				'    Strongest tailwind: none',
				'    Strongest crosswind: 11 kt from the left',
				'    Cloud: in cloud down to height 640 ft',
				'    Icing conditions (in cloud or precipitation at -20 to +10 °C): no',
				'    Most structural ice: 2%',
				'    Most pitot ice: 3%',
				'    Engine anti-ice (carburetor heat on a piston engine): on for 25% of the samples',
				'    G-force while airborne: 0.87-1.24 G (its spread shows turbulence; a bump between samples is missed)',
				'    Vertical wind (+ up): -420 ft/min at height 410 ft to 131 ft/min at height 151 ft; a swing while the aircraft pitches or rolls fast may be the simulator\'s reading, not the air',
				'    Closest to the stall while airborne: angle of attack 9.5° vs stall 14.0° at height 60 ft',
				'  Highest terrain within 5 nm before the touchdown: +612 ft, 4.20 nm from the touchdown point (more than 500 ft above it)'
			])js")), QStringList());
	}

	// A wind whose direction holds steady (both ends rounding to 20°) names
	// that one direction.
	void touchdownPromptNamesASteadyWindsOneDirection() {
		QCOMPARE(touchdownPromptLacks(QStringLiteral(
			"Object.assign(t.conditions, {windDirectionFrom: 19.6, windDirectionTo: 20.4});"), QStringLiteral(R"js([
				'  Wind over the 30 s before the contact: 8-15 kt, direction 20° true'
			])js")), QStringList());
	}

	// An updraft at a gate reads as up (the full prompt's gate has a downdraft).
	void touchdownPromptNamesAGatesUpdraft() {
		QCOMPARE(touchdownPromptLacks(QStringLiteral(
			"Object.assign(t.profile.gates[0], {verticalWindFpm: 120.4});"), QStringLiteral(R"js([
				'    500 ft: -42.5 s, height 512 ft, 1.61 nm, terrain -20 ft, IAS 131 kt, V/S -703 ft/min, pitch 2.0°, bank -1.3°, heading 338°, track 341°, threshold -8640 ft, centerline 13 ft left, wind 299° T 12 kt (headwind 6 kt, crosswind 11 kt from the left), vertical wind 120 ft/min up, OAT 31.5 °C, in cloud, precipitation rain, structural ice 2%'
			])js")), QStringList());
	}

	// Values that round to zero read plainly: no "-0.0", a side offset of
	// 0 ft with no side, a range whose ends read the same as the one value, a
	// calm wind (0 kt) without the direction or components that are only
	// noise then, and a runway slope of 0.00° as level.
	void touchdownPromptWritesValuesRoundingToZeroPlainly() {
		QCOMPARE(touchdownPromptLacks(QStringLiteral(
			"Object.assign(t, {windVelocity: 0, bankDegrees: -0.04, distanceWidth: 0.3, runwaySlope: 0.004});"
			"Object.assign(t.conditions, {temperatureC: -0.04, windMinKt: 0.4, windMaxKt: 0.4});"
			"Object.assign(t.profile.gates[0], {windSpeedKt: 0.4, verticalWindFpm: -0.4, pitch: -0.04, centerlineFt: 0.3, terrainFt: 0.4});"
			"Object.assign(t.profile.segment, {windMinKt: 7.4, windMaxKt: 6.6, gForceMin: 1.004, gForceMax: 0.996,"
			"  verticalWindMinFpm: -0.4, verticalWindMaxFpm: 0.4});"), QStringLiteral(R"js([
				'  Pitch: 3.2°   Bank: 0.0°',
				'  Wind: calm (0 kt)',
				'  Centerline offset: 0 ft (12% of half-width)',
				'  Runway: 10000 ft long, 150 ft wide, threshold displaced 820 ft at this end, level',
				'  Outside air temperature: 0.0 °C',
				'  Wind over the 30 s before the contact: 0 kt',
				'    500 ft: -42.5 s, height 512 ft, 1.61 nm, terrain 0 ft, IAS 131 kt, V/S -703 ft/min, pitch 0.0°, bank -1.3°, heading 338°, track 341°, threshold -8640 ft, centerline 0 ft, wind calm, vertical wind 0 ft/min, OAT 31.5 °C, in cloud, precipitation rain, structural ice 2%',
				'    Wind speed: 7 kt',
				'    G-force while airborne: 1.00 G (its spread shows turbulence; a bump between samples is missed)',
				'    Vertical wind (+ up): 0 ft/min at height 410 ft; a swing while the aircraft pitches or rolls fast may be the simulator\'s reading, not the air'
			])js")), QStringList());
	}

	// A contact evaluation's prompt is the contact alone: the answer has no
	// segment or Environment section, and the data no runway dimensions,
	// Conditions, trajectory or airport instruction; its "Consider:" line
	// keeps only what the contact shows. The heading rule stays.
	void contactPromptsCarryTheContactAlone() {
		const QStringList head = {
			QStringLiteral("Respond using exactly this structure, with no preamble and no extra sections:"),
			QStringLiteral("Plain text only: no markdown and no LaTeX (e.g. write \"Vref\", not \"**Vref**\" or \"$V_{ref}$\")."),
			QString(), QStringLiteral("Grade: [A+/A/A-/B+/B/B-/C+/C/C-/D/F]"),
			QString(), QStringLiteral("Summary: [1-2 sentences overall]"),
			QString(), QStringLiteral("Strengths:"), QStringLiteral("• [one brief point per bullet]"),
			QString(), QStringLiteral("Areas to improve:"), QStringLiteral("• [one brief point per bullet]"),
			QString(), QStringLiteral("---") };
		const QStringList where = {
			QStringLiteral("  Airport: OEMA (Prince Mohammad bin Abdulaziz)"),
			QStringLiteral("  Runway: 34C (336° magnetic, 340° true)"),
			QStringLiteral("  Airspeed: 135 kt indicated") };
		const QStringList heading = {
			QStringLiteral("  Heading: 334° magnetic (338° true); ground track 341° true; crab 2.2° (nose left of the track)"),
			QStringLiteral("  Wind: 299° true at 12 kt") };
		const QString offsets = QStringLiteral(
			": 1250 ft (11% of %1) [negative = before the marked threshold, i.e. on a displaced-threshold/blast-pad area; "
			"over 100% = past the far threshold]");
		const QString centerline = QStringLiteral("  Centerline offset: 6 ft left (12% of half-width)");
		const QString rule = QStringLiteral(
			"Headings: a difference between a magnetic and a true heading is the magnetic variation, not a misalignment; "
			"judge the alignment and crab only from the aircraft's true heading and true ground track against the runway's true heading.");
		const QStringList liftoff = QStringList{ QStringLiteral(
			"You are an aviation instructor reviewing a liftoff from a Microsoft Flight Simulator 2024 flight.") }
			+ head + QStringList{ QStringLiteral("Liftoff data:") } + where + QStringList{ QStringLiteral("  Bank: -1.4°") } + heading
			+ QStringList{ QStringLiteral("  Distance from threshold at liftoff") + offsets.arg(QStringLiteral("runway")), centerline, QString(),
				QStringLiteral("Consider: crosswind component (calculate from the true wind vs the true runway heading), "
					"centerline tracking before the liftoff, use of available runway length."), rule };
		const QStringList touchdown = QStringList{ QStringLiteral(
			"You are an aviation instructor reviewing a Microsoft Flight Simulator 2024 landing.") }
			+ head + QStringList{ QStringLiteral("Landing data:") } + where + QStringList{
				QStringLiteral("  Vertical speed: -95 ft/min"),
				QStringLiteral("  Touchdown rate: 263 ft/min (the sink rate perpendicular to the runway surface as the wheels touched, "
					"as the simulator measured it)"),
				QStringLiteral("  G-force: 1.21 G"), QStringLiteral("  Pitch: 3.2°   Bank: -1.4°") } + heading
			+ QStringList{ QStringLiteral("  Distance from threshold") + offsets.arg(QStringLiteral("landing distance available")), centerline,
				QString(), QStringLiteral("Consider: touchdown rate, vertical speed & G-force comfort, crosswind component (calculate from "
					"the true wind vs the true runway heading), decrab at touchdown (crab angle), centerline tracking, touchdown zone placement."),
				rule };
		const QString js = QStringLiteral(
			"(function () { var t = %1, title = window._aircraftTitle; window._aircraftTitle = '';"
			"  var prompts = [buildLiftoffPrompt(t, true), buildTouchdownPrompt(t, true)];"
			"  window._aircraftTitle = title; return prompts; })()").arg(fullContactJs());
		const QStringList prompts = evalPageJs(widget_, js).toStringList();
		QCOMPARE(prompts.size(), 2);
		QCOMPARE(prompts[0], liftoff.join('\n'));
		QCOMPARE(prompts[1], touchdown.join('\n'));
	}

	// A contact recorded before the actual vertical speed was stored has the
	// indicated one, which lags it, and says so; the trajectory's header names
	// which one its points carry. Without a touchdown rate, no line for it.
	void promptMarksAnIndicatedVerticalSpeed() {
		QCOMPARE(touchdownPromptLacks(QStringLiteral(
			"delete t.actualVerticalSpeed; delete t.touchdownRate; t.profile.indicatedVerticalSpeed = true;"),
			QStringLiteral(R"js([
				'  Vertical speed: -180 ft/min (indicated), which lags the actual rate',
				'Final trajectory (seconds from the touchdown point; height = altitude above the touchdown point, not radio height; distance = to the touchdown point; terrain = ground below relative to the touchdown point; heading/track = true; V/S = indicated vertical speed, which lags the actual rate; wind components = relative to the runway, not the aircraft\'s heading; threshold = along the runway from the marked threshold, negative = before it; centerline = from the extended runway centerline):'
			])js")), QStringList());
		QCOMPARE(evalPageJs(widget_, QStringLiteral("(function () { var t = %1; delete t.touchdownRate;"
			"  return /Touchdown rate:/.test(buildTouchdownPrompt(t)); })()").arg(fullContactJs())).toBool(), false);
	}

	// The popup's V/S is the actual vertical speed, or for a contact recorded
	// before it was stored the indicated one, marked; a touchdown rate row
	// only when known.
	void popupVerticalSpeedIsTheActualOrMarkedIndicated() {
		const QString js = QStringLiteral(
			"(function (extra) { var t = {icao: '', runway: '', runwayHeading: -1, lat: 1, lng: 2, airspeed: 140,"
			"    verticalSpeed: -339, pitchDegrees: 0, bankDegrees: 0, headingDegrees: 334, distanceLength: 0, distanceWidth: 0,"
			"    distanceLengthPercent: 0, distanceWidthPercent: 0, windDirection: 299, windVelocity: 12, zuluTime: 'z'};"
			"  Object.assign(t, extra);"
			"  var d = document.createElement('div');"
			"  d.innerHTML = runwayContactPopupHtml('x', t, 'Touchdown', null, 'f', 'L').html;"
			"  return Array.prototype.map.call(d.querySelectorAll('.td-row'), function (r) {"
			"    return r.querySelector('.td-key').textContent + ': ' + r.querySelector('.td-val').textContent; })"
			"    .filter(function (r) { return /^(V\\/S|Touchdown rate):/.test(r); }).join('|'); })(%1)");
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("{}"))).toString(), QStringLiteral("V/S: -339 ft/min (indicated)"));
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("{actualVerticalSpeed: -412.5, touchdownRate: 509.6}"))).toString(),
			QStringLiteral("V/S: -413 ft/min|Touchdown rate: 510 ft/min"));
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("{actualVerticalSpeed: 0}"))).toString(), QStringLiteral("V/S: 0 ft/min"));
	}

	// Each point says whether the aircraft was on the ground and, when
	// recorded, each engine's throttle and power (? for an engine not
	// recorded) with the power's unit, each rounded half away from zero like
	// the other values, for up to 4 engines; the header explains only what
	// the points carry, the power by its name, and names the engines shown
	// when there are more.
	void trajectoryPointsCarryOnGroundAndEngines() {
		const QString js = QStringLiteral(
			"(function (profile) { var t = %1; t.profile = profile;"
			"  return buildTouchdownPrompt(t).split('\\n').filter(function (l) { return /^Final trajectory|^    (-?\\d|50 ft)/.test(l); }).join('|'); })")
			.arg(fullContactJs());
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("({gates: [], powerName: 'N1', powerUnit: '%', series: ["
			"{seconds: -0.5, onGround: false, heightFt: 2.4, throttle: [-4.5, null], power: [53.5, 54]},"
			"{seconds: 0, onGround: true, throttle: [30, 30]}]})")).toString(), QStringLiteral(
			"Final trajectory (seconds from the touchdown point; on ground/airborne = as the simulator reports it; height = altitude above "
			"the touchdown point, not radio height; distance = to the touchdown point; terrain = ground below relative to the touchdown point; "
			"throttle = throttle lever position per engine (engine 1 first, ? = not recorded) as the aircraft reports it, negative = reverse range; "
			"N1 = per engine):"
			"|    -0.5 s, airborne, height 2 ft, throttle -5/?%, N1 54/54%"
			"|    0.0 s, on ground, throttle 30/30%"));
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("({gates: [{gate: '50 ft', seconds: -3, power: [60]}], series: [], powerName: 'torque', "
			"powerUnit: '%'})")).toString(), QStringLiteral(
			"Final trajectory (seconds from the touchdown point; height = altitude above the touchdown point, not radio height; distance = to "
			"the touchdown point; terrain = ground below relative to the touchdown point; torque = per engine):"
			"|    50 ft: -3.0 s, torque 60%"));
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("({gates: [{gate: '50 ft', seconds: -3, power: [15.2, 14.5]}], series: [], "
			"powerName: 'MP', powerUnit: ' inHg', powerTitle: 'manifold pressure'})")).toString(), QStringLiteral(
			"Final trajectory (seconds from the touchdown point; height = altitude above the touchdown point, not radio height; distance = to "
			"the touchdown point; terrain = ground below relative to the touchdown point; MP = manifold pressure, per engine):"
			"|    50 ft: -3.0 s, MP 15/15 inHg"));
		// More than 4 engines (a B-52's 8): 4 spread from the first to the
		// last shown, the header naming them, an engine shown but not
		// recorded "?" (throttle 6 and 8); 4 shown whole without it.
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("({gates: [], powerName: 'N1', powerUnit: '%', series: ["
			"{seconds: 0, throttle: [1, 2, 3, 4], power: [11, 12, 13, 14, 15, 16, 17, 18]}]})")).toString(), QStringLiteral(
			"Final trajectory (seconds from the touchdown point; height = altitude above the touchdown point, not radio height; distance = to "
			"the touchdown point; terrain = ground below relative to the touchdown point; "
			"throttle = throttle lever position per engine (engine 1 first, ? = not recorded) as the aircraft reports it, negative = reverse range; "
			"N1 = per engine; engines 1/3/6/8 of 8 shown):"
			"|    0.0 s, throttle 1/3/?/?%, N1 11/13/16/18%"));
		// The engines counted from the throttle when it has more.
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("({gates: [], powerName: 'N1', powerUnit: '%', series: ["
			"{seconds: 0, throttle: [1, 2, 3, 4, 5, 6, 7, 8]}]})")).toString(), QStringLiteral(
			"Final trajectory (seconds from the touchdown point; height = altitude above the touchdown point, not radio height; distance = to "
			"the touchdown point; terrain = ground below relative to the touchdown point; "
			"throttle = throttle lever position per engine (engine 1 first, ? = not recorded) as the aircraft reports it, negative = reverse range; "
			"engines 1/3/6/8 of 8 shown):"
			"|    0.0 s, throttle 1/3/6/8%"));
		// 5, 6 and 7 engines: the first, the last and the two spread
		// between them.
		const QString spread = QStringLiteral(
			"(function () { return [5, 6, 7].map(function (n) { var t = %1, power = [];"
			"  for (var e = 1; e <= n; ++e) power.push(10 + e);"
			"  t.profile = {gates: [], powerName: 'N1', powerUnit: '%', series: [{seconds: 0, power: power}]};"
			"  return buildTouchdownPrompt(t).split('\\n').filter(function (l) { return /^    0\\.0 s/.test(l) || /^Final trajectory/.test(l); })"
			"    .map(function (l) { return l.replace(/^.*; (engines [^)]*)\\):$/, '$1').trim(); }).join(': '); }).join('|'); })()")
			.arg(fullContactJs());
		QCOMPARE(evalPageJs(widget_, spread).toString(), QStringLiteral(
			"engines 1/2/4/5 of 5 shown: 0.0 s, N1 11/12/14/15%"
			"|engines 1/3/4/6 of 6 shown: 0.0 s, N1 11/13/14/16%"
			"|engines 1/3/5/7 of 7 shown: 0.0 s, N1 11/13/15/17%"));
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("({gates: [], powerName: 'N1', powerUnit: '%', series: ["
			"{seconds: 0, power: [11, 12, 13, 14]}]})")).toString(), QStringLiteral(
			"Final trajectory (seconds from the touchdown point; height = altitude above the touchdown point, not radio height; distance = to "
			"the touchdown point; terrain = ground below relative to the touchdown point; N1 = per engine):"
			"|    0.0 s, N1 11/12/13/14%"));
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("({gates: [], series: [{seconds: 0, onGround: true}], powerName: 'N1'})")).toString(),
			QStringLiteral(
			"Final trajectory (seconds from the touchdown point; on ground/airborne = as the simulator reports it; height = altitude above "
			"the touchdown point, not radio height; distance = to the touchdown point; terrain = ground below relative to the touchdown point):"
			"|    0.0 s, on ground"));
	}

	// The other side of each signed value: the nose right of the track, the
	// sun above the horizon, a westerly variation, a point right of the
	// centerline, and terrain not more than 500 ft above the touchdown point.
	void touchdownPromptNamesTheOtherSides() {
		QCOMPARE(touchdownPromptLacks(QStringLiteral(
			"Object.assign(t.conditions, {crabAngle: 2.2, sunElevation: 10, magneticVariation: -4.4});"
			"t.profile.gates[0].centerlineFt = 12.6; t.profile.highestTerrainFt = 500;"
			"Object.assign(t.profile.gates[0], {headwindKt: -3.4, crosswindKt: 4.6, inCloud: 0});"
			"Object.assign(t.profile.segment, {headwindFromKt: 2, headwindToKt: 17, maxTailwindKt: 4.4, maxCrosswindKt: 0.3,"
			"  icingConditions: 1}); delete t.profile.segment.lowestInCloudFt;"), QStringLiteral(R"js([
				'  Heading: 334° magnetic (338° true); ground track 341° true; crab 2.2° (nose right of the track)',
				'  Sun: 10.0° above the horizon (day)',
				'  Position: 24.5528° N, 39.7051° E; magnetic variation 4.4° W',
				'    500 ft: -42.5 s, height 512 ft, 1.61 nm, terrain -20 ft, IAS 131 kt, V/S -703 ft/min, pitch 2.0°, bank -1.3°, heading 338°, track 341°, threshold -8640 ft, centerline 13 ft right, wind 299° T 12 kt (tailwind 3 kt, crosswind 5 kt from the right), vertical wind 311 ft/min down, OAT 31.5 °C, not in cloud, precipitation rain, structural ice 2%',
				'    Largest headwind change: headwind 2 kt at height 980 ft to headwind 17 kt at height 120 ft (a gain of 15 kt: wind shear)',
				'    Strongest tailwind: 4 kt',
				'    Strongest crosswind: 0 kt',
				'    Cloud: not in cloud',
				'    Icing conditions (in cloud or precipitation at -20 to +10 °C): yes',
				'  Highest terrain within 5 nm before the touchdown: +500 ft, 4.20 nm from the touchdown point'
			])js")), QStringList());
	}

	// Terrain at or below the touchdown point (rounded) along the approach
	// leaves the point itself as the highest, so the line says there's none
	// above it rather than giving its 0 ft at 0 nm.
	void touchdownPromptSaysNoTerrainAboveThePoint() {
		for (const char* height : { "0.4", "-30" })
			QCOMPARE(touchdownPromptLacks(QStringLiteral("t.profile.highestTerrainFt = %1; t.profile.highestTerrainNm = 0;").arg(QLatin1String(height)),
				QStringLiteral(R"js([
					'  Highest terrain within 5 nm before the touchdown: none above the touchdown point'
				])js")), QStringList());
	}

	// A terrain search that ended short of 5 nm (at a reset, or the trip's
	// start or end) says how far it reached, so "none above" isn't read as
	// covering terrain it never saw. One that reads 5.00 nm is the full range,
	// not a shorter one.
	void touchdownPromptSaysHowFarTheTerrainSearchReached() {
		QCOMPARE(touchdownPromptLacks(QStringLiteral(
			"t.profile.highestTerrainFt = -3; t.profile.highestTerrainNm = 0; t.profile.terrainRangeNm = 0.014;"), QStringLiteral(R"js([
				'  Highest terrain within 0.01 nm before the touchdown: none above the touchdown point'
			])js")), QStringList());
		QCOMPARE(touchdownPromptLacks(QStringLiteral(
			"t.profile.highestTerrainFt = -3; t.profile.highestTerrainNm = 0; t.profile.terrainRangeNm = 4.997;"), QStringLiteral(R"js([
				'  Highest terrain within 5 nm before the touchdown: none above the touchdown point'
			])js")), QStringList());
	}

	// Without the runway's heading (a contact off a matched runway), a gate's
	// wind has no headwind/crosswind components.
	void gateWindWithoutRunwayHasNoComponents() {
		QCOMPARE(touchdownPromptLacks(QStringLiteral(
			"delete t.profile.gates[0].headwindKt; delete t.profile.gates[0].crosswindKt;"), QStringLiteral(R"js([
				'    500 ft: -42.5 s, height 512 ft, 1.61 nm, terrain -20 ft, IAS 131 kt, V/S -703 ft/min, pitch 2.0°, bank -1.3°, heading 338°, track 341°, threshold -8640 ft, centerline 13 ft left, wind 299° T 12 kt, vertical wind 311 ft/min down, OAT 31.5 °C, in cloud, precipitation rain, structural ice 2%'
			])js")), QStringList());
	}

	// The trajectory's header says the wind components are relative to the
	// runway, not the aircraft's heading, whenever a height gate or the segment
	// shows one, in both prompts; without any (no runway heading), or with
	// them only on a sample or a distance gate, which don't show them, it doesn't.
	void trajectoryHeaderNamesTheWindComponentsReference() {
		const QString js = QStringLiteral(
			"(function (build, add) { var t = %1, p = t.profile;"
			"  p.gates.concat(p.series).forEach(function (q) { delete q.headwindKt; delete q.crosswindKt; });"
			"  delete p.segment.headwindFromKt; delete p.segment.maxCrosswindKt; add(p);"
			"  return build(t).indexOf(\"; wind components = relative to the runway, not the aircraft's heading\") >= 0; })")
			.arg(fullContactJs());
		for (const char* build : {"buildTouchdownPrompt", "buildLiftoffPrompt"}) {
			const QString call = js + QStringLiteral("(%1, ").arg(QLatin1String(build));
			QCOMPARE(evalPageJs(widget_, call + QStringLiteral("function () {})")).toBool(), false);
			QCOMPARE(evalPageJs(widget_, call + QStringLiteral("function (p) { p.gates[0].headwindKt = -3.4; })")).toBool(), true);
			QCOMPARE(evalPageJs(widget_, call + QStringLiteral("function (p) { p.series[0].headwindKt = -3.4; })")).toBool(), false);
			QCOMPARE(evalPageJs(widget_, call + QStringLiteral("function (p) {"
				"  p.gates.filter(function (g) { return / nm$/.test(g.gate); })[0].headwindKt = -3.4; })")).toBool(), false);
			QCOMPARE(evalPageJs(widget_, call + QStringLiteral("function (p) { p.segment.headwindFromKt = 2; })")).toBool(), true);
			QCOMPARE(evalPageJs(widget_, call + QStringLiteral("function (p) { p.segment.maxCrosswindKt = 0.3; })")).toBool(), true);
		}
	}

	// A headwind change under 15 kt isn't marked as wind shear, and one
	// rounding to 0 is a gain between headwinds (a headwind rounding to 0 is
	// one, not a tailwind); a tailwind rounding to 0 is none;
	// a segment whose cloud wasn't recorded has no cloud line, and one with
	// nothing recorded no block at all. The liftoff's block covers its climb-out.
	void segmentLinesMarkShearAndLeaveOutTheUnknown() {
		const QString js = QStringLiteral(
			"(function (build, segment) { var t = %1; t.profile.segment = segment;"
			"  return build(t).split('\\n').filter(function (l) { return /^  [\\w-]+ weather and ride|^    (Largest|Strongest tailwind|Cloud)/.test(l); }).join('|'); })")
			.arg(fullContactJs());
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("(buildTouchdownPrompt, {samples: 3, headwindFromKt: 20.4, headwindFromHeightFt: 900,"
			"headwindToKt: 6, headwindToHeightFt: 50, cloudKnown: false})")).toString(), QStringLiteral(
			"  Approach weather and ride (from the last crossing of 1000 ft above the touchdown point down to it, or from 5 nm out, the "
			"previous contact or the start of the recording if closer):"
			"|    Largest headwind change: headwind 20 kt at height 900 ft to headwind 6 kt at height 50 ft (a loss of 14 kt)"));
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("(buildLiftoffPrompt, {samples: 3, cloudKnown: true})")).toString(), QStringLiteral(
			"  Climb-out weather and ride (from the liftoff up to the first crossing of 1000 ft above the liftoff point, or to the next "
			"contact or 5 nm from the liftoff point if sooner):"
			"|    Cloud: not in cloud"));
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("(buildTouchdownPrompt, {samples: 3, headwindFromKt: 0, headwindFromHeightFt: 900,"
			"headwindToKt: -0.3, headwindToHeightFt: 50, maxTailwindKt: 0.3, cloudKnown: false})")).toString(), QStringLiteral(
			"  Approach weather and ride (from the last crossing of 1000 ft above the touchdown point down to it, or from 5 nm out, the "
			"previous contact or the start of the recording if closer):"
			"|    Largest headwind change: headwind 0 kt at height 900 ft to headwind 0 kt at height 50 ft (a gain of 0 kt)"
			"|    Strongest tailwind: none"));
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("(buildTouchdownPrompt, {samples: 3, cloudKnown: false})")).toString(), QString());
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("(buildTouchdownPrompt, undefined)")).toString(), QString());
	}

	// Each prompt has the model judge its segment against the weather: the
	// crab against the crosswind, the speed against the gusts, excursions
	// against shear, vertical wind and turbulence, and the margin to the
	// stall; and consider shear, turbulence, cloud and icing.
	void analysisPromptsJudgeTheSegmentAgainstTheWeather() {
		const QString js = QStringLiteral(
			"(function () { var t = %1;"
			"  return [buildLiftoffPrompt(t), buildTouchdownPrompt(t)].map(function (p) {"
			"    return p.split('\\n').filter(function (l) { return /^• \\[(what the samples show|stability)|^Consider:/.test(l); })"
			"      .map(function (l) { return /against the crosswind.*against the gusts.*against the wind shear, vertical wind and turbulence, and the margin to the stall/.test(l)"
			"        || /gusts, wind shear, turbulence, cloud and icing,/.test(l); }).join(); }).join('|'); })()")
			.arg(fullContactJs());
		QCOMPARE(evalPageJs(widget_, js).toString(), QStringLiteral("true,true|true,true"));
	}

	// The liftoff prompt doesn't say what kind of liftoff it was: its segment
	// section has the model tell a takeoff, a touch-and-go and a bounce apart
	// from the samples, judge the first two's rotation and climb, and a
	// bounce as part of the landing.
	void liftoffPromptHasTheModelSayWhatTheLiftoffWas() {
		const QString js = QStringLiteral(
			"(function () { return buildLiftoffPrompt(%1).split('\\n')"
			"  .filter(function (l) { return /^• \\[what the samples show/.test(l); }).join('\\n'); })()")
			.arg(fullContactJs());
		QCOMPARE(evalPageJs(widget_, js).toString(), QStringLiteral(
			"• [what the samples show this liftoff to be (a takeoff, a touch-and-go or a bounce), then for a takeoff or "
			"touch-and-go the rotation and initial climb: pitch, speed, bank and track through the gates; judge the crab "
			"after the liftoff against the crosswind, the rotation and climb speed against the gusts, any excursion against "
			"the wind shear, vertical wind and turbulence, and the margin to the stall; for a bounce, judge it as part of "
			"the landing]"));
	}

	// When the points recorded engine failures or fires, one line names
	// every engine that failed or was on fire at any of them, gates and
	// samples alike, the engines not shown too (2, 5 and 7 of 8), in
	// ascending order; either state alone gives the line; "none" when none
	// did; no line when neither was recorded.
	void trajectoryNamesEveryFailedOrBurningEngine() {
		const QString js = QStringLiteral(
			"(function (profile) { var t = %1; t.profile = profile;"
			"  return buildTouchdownPrompt(t).split('\\n').filter(function (l) { return /^  Engine failures/.test(l); }).join('|'); })")
			.arg(fullContactJs());
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("({gates: [{gate: '50 ft', seconds: -3, engineFailed: [0, 0, 0, 0, 1, 0, 0, 0],"
			"  engineOnFire: [0, 0, 0, 0, 0, 0, 0, 0]}], powerName: 'N1', powerUnit: '%', series: ["
			"{seconds: 0, power: [1, 2, 3, 4, 5, 6, 7, 8], engineFailed: [0, 1, 0, 0, 0, 0, 1, 0], engineOnFire: [0, 0, 0, 0, 0, 0, 1, null]}]})")).toString(),
			QStringLiteral("  Engine failures and fires (of all engines, at any point above): failed: engine 2, 5, 7; on fire: engine 7"));
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("({gates: [], powerName: 'N1', powerUnit: '%', series: ["
			"{seconds: 0, engineOnFire: [0, 1]}]})")).toString(),
			QStringLiteral("  Engine failures and fires (of all engines, at any point above): on fire: engine 2"));
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("({gates: [], powerName: 'N1', powerUnit: '%', series: ["
			"{seconds: 0, engineFailed: [0, 0], engineOnFire: [0, null]}]})")).toString(),
			QStringLiteral("  Engine failures and fires (of all engines, at any point above): none"));
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("({gates: [], powerName: 'N1', powerUnit: '%', series: [{seconds: 0, power: [50]}]})")).toString(),
			QString());
	}

	// Before the data, each prompt asks for its answer's sections, ending in
	// its trajectory's segment (a liftoff's "Liftoff and climb-out", a
	// touchdown's "Final segment") and "Environment"; each names its own
	// trajectory, and the liftoff's spans the liftoff and climb-out.
	void analysisPromptsAskForTheirSegmentAndEnvironment() {
		const QString js = QStringLiteral(
			"(function () { var t = %1;"
			"  return [buildLiftoffPrompt(t), buildTouchdownPrompt(t)].map(function (p) {"
			"    var lines = p.split('\\n').slice(0, p.split('\\n').indexOf('---'));"
			"    return lines.filter(function (l, i) { return /:$/.test(l) && /^• \\[/.test(lines[i + 1]); }).join()"
			"      + ',' + /\\nAirport: from your general knowledge/.test(p); })"
			"    .concat([buildLiftoffPrompt(t).indexOf('\\n  Samples, one per second from 30 s before the liftoff, then from 10 s before to 10 s after it (every sample, evenly thinned if too dense):\\n') >= 0,"
			"      buildLiftoffPrompt(t).indexOf('\\nLiftoff trajectory (seconds from the liftoff point;') >= 0,"
			"      buildTouchdownPrompt(t).indexOf('\\nFinal trajectory (seconds from the touchdown point;') >= 0]).join('|'); })()")
			.arg(fullContactJs());
		QCOMPARE(evalPageJs(widget_, js).toString(), QStringLiteral(
			"Strengths:,Areas to improve:,Liftoff and climb-out:,Environment:,true"
			"|Strengths:,Areas to improve:,Final segment:,Environment:,true|true|true|true"));
	}

	// With a runway heading, both prompts say how to compare the headings:
	// true against true, with a sample; without one (no true aircraft
	// heading), not to judge the alignment from the heading. Without a
	// runway heading there's nothing to compare, so no rule.
	void analysisPromptsSayHowToCompareHeadings() {
		const QString js = QStringLiteral(
			"(function (t) { return [buildLiftoffPrompt(t), buildTouchdownPrompt(t)].map(function (p) {"
			"  return p.split('\\n').filter(function (l) { return /^Headings:/.test(l); }).join('#') || 'none'; }).join('|'); })(%1)");
		const QString withTrue = QStringLiteral(
			"Headings: a difference between a magnetic and a true heading is the magnetic variation, not a misalignment; "
			"judge the alignment and crab only from the aircraft's true heading and true ground track against the runway's true heading.");
		QCOMPARE(evalPageJs(widget_, js.arg(fullContactJs())).toString(), withTrue + '|' + withTrue);
		const QString noSample = QStringLiteral(
			"{icao: 'EGLL', runway: '27L', runwayHeading: 270, lat: 51.47, lng: -0.45, airspeed: 140, verticalSpeed: -120,"
			" pitchDegrees: 3, bankDegrees: 0, headingDegrees: 271, distanceLength: 1000, distanceWidth: 2, distanceLengthPercent: 0.2,"
			" distanceWidthPercent: 0.04, windDirection: 250, windVelocity: 8, zuluTime: '2026-01-01T10:00:00.000+00:00_4', gForce: 1.1,"
			" profile: {gates: [], series: []}%1}");
		const QString withoutTrue = QStringLiteral(
			"Headings: a difference between a magnetic and a true heading is the magnetic variation, not a misalignment; "
			"no true aircraft heading was recorded, so don't judge the alignment from the aircraft's heading.");
		QCOMPARE(evalPageJs(widget_, js.arg(noSample.arg(QString()))).toString(), withoutTrue + '|' + withoutTrue);
		QCOMPARE(evalPageJs(widget_, js.arg(noSample.arg(QStringLiteral(", runwayHeading: -1")))).toString(), QStringLiteral("none|none"));
		QCOMPARE(evalPageJs(widget_, js.arg(noSample.arg(QStringLiteral(", runway: ''")))).toString(), QStringLiteral("none|none"));
	}

	// The segment marks what the model must not pass over: an airborne
	// G-force 0.5 G or more from 1 G (either way) as moderate turbulence and
	// 1.0 G or more as severe, a vertical wind beyond 1000 ft/min (either way)
	// as strong, and an angle of attack at or past the stall angle as a stall.
	void segmentLinesMarkTurbulenceStrongVerticalWindAndStall_data() {
		QTest::addColumn<QString>("segment");
		QTest::addColumn<QString>("lines");
		const QString g = QStringLiteral("    G-force while airborne: %1 G (its spread shows turbulence; a bump between samples is missed%2)");
		const QString moderate = QStringLiteral("; moderate turbulence or maneuvering");
		const QString severe = QStringLiteral("; severe turbulence or hard maneuvering");
		QTest::newRow("G under 0.5 from 1") << QStringLiteral("gForceMin: 0.6, gForceMax: 1.49") << g.arg(QStringLiteral("0.60-1.49"), QString());
		QTest::newRow("G 0.5 above 1") << QStringLiteral("gForceMin: 1, gForceMax: 1.5") << g.arg(QStringLiteral("1.00-1.50"), moderate);
		QTest::newRow("G 0.5 below 1") << QStringLiteral("gForceMin: 0.5, gForceMax: 1") << g.arg(QStringLiteral("0.50-1.00"), moderate);
		QTest::newRow("G under 1 above 1") << QStringLiteral("gForceMin: 1, gForceMax: 1.99") << g.arg(QStringLiteral("1.00-1.99"), moderate);
		QTest::newRow("G 1 above 1") << QStringLiteral("gForceMin: 1, gForceMax: 2") << g.arg(QStringLiteral("1.00-2.00"), severe);
		QTest::newRow("G 1 below 1") << QStringLiteral("gForceMin: 0, gForceMax: 1") << g.arg(QStringLiteral("0.00-1.00"), severe);
		const QString wind = QStringLiteral("    Vertical wind (+ up): %1 ft/min at height 300 ft to %2 ft/min at height 700 ft%3; "
			"a swing while the aircraft pitches or rolls fast may be the simulator's reading, not the air");
		const QString heights = QStringLiteral(", verticalWindMinHeightFt: 300, verticalWindMaxHeightFt: 700");
		QTest::newRow("vertical wind 1000 either way") << QStringLiteral("verticalWindMinFpm: -1000, verticalWindMaxFpm: 1000") + heights
			<< wind.arg(QStringLiteral("-1000"), QStringLiteral("1000"), QString());
		QTest::newRow("downdraft past 1000") << QStringLiteral("verticalWindMinFpm: -1001, verticalWindMaxFpm: 0") + heights
			<< wind.arg(QStringLiteral("-1001"), QStringLiteral("0"), QStringLiteral(" (strong)"));
		QTest::newRow("updraft past 1000") << QStringLiteral("verticalWindMinFpm: 0, verticalWindMaxFpm: 1001") + heights
			<< wind.arg(QStringLiteral("0"), QStringLiteral("1001"), QStringLiteral(" (strong)"));
		const QString stall = QStringLiteral("    Closest to the stall while airborne: angle of attack %1° vs stall 13.2° at height 40 ft%2");
		const QString stalled = QStringLiteral(" (stalled; where the stall began, and the G-force and vertical wind above stop there, "
			"as the aircraft may then have been out of control)");
		QTest::newRow("at the stall angle") << QStringLiteral("stallMarginAlphaDeg: 13.2, stallMarginStallAlphaDeg: 13.2, stallMarginHeightFt: 40")
			<< stall.arg(QStringLiteral("13.2"), stalled);
		QTest::newRow("below the stall angle") << QStringLiteral("stallMarginAlphaDeg: 13.1, stallMarginStallAlphaDeg: 13.2, stallMarginHeightFt: 40")
			<< stall.arg(QStringLiteral("13.1"), QString());
		QTest::newRow("with airspeed and bank") << QStringLiteral("stallMarginAlphaDeg: 16.9, stallMarginStallAlphaDeg: 13.2, stallMarginHeightFt: 40, "
			"stallMarginAirspeedKt: 42.5, stallMarginBankDeg: 43.19")
			<< stall.arg(QStringLiteral("16.9"), QStringLiteral(", IAS 43 kt, bank 43.2°") + stalled);
	}
	void segmentLinesMarkTurbulenceStrongVerticalWindAndStall() {
		QFETCH(QString, segment);
		QFETCH(QString, lines);
		QCOMPARE(evalPageJs(widget_, QStringLiteral(
			"(function () { var t = %1; t.profile.segment = {samples: 3, %2};"
			"  return buildTouchdownPrompt(t).split('\\n').filter(function (l) { return /^    (G-force|Vertical wind|Closest to the stall)/.test(l); }).join('|'); })()")
			.arg(fullContactJs(), segment)).toString(), lines);
	}

	// A crashed touchdown's prompts, both its progress and its contact
	// evaluation, say that it crashed and the simulator reset the flight, and
	// leave out the vertical speed (its sample's, after the contact stopped,
	// would contradict the touchdown rate); other touchdowns' have the
	// vertical speed and no outcome.
	void crashedTouchdownPromptsSayItCrashed() {
		const QString js = QStringLiteral(
			"(function () { var t = %1; %2"
			"  return [buildTouchdownPrompt(t), buildTouchdownPrompt(t, true)].map(function (p) {"
			"    return p.split('\\n').filter(function (l) { return /^  (Outcome|Vertical speed):/.test(l); }).join('#'); }).join('|'); })()");
		const QString outcome = QStringLiteral(
			"  Outcome: the aircraft crashed here; the simulator then reset the flight, so nothing after the crash was flown");
		QCOMPARE(evalPageJs(widget_, js.arg(fullContactJs(), QStringLiteral("t.crashed = true;"))).toString(), outcome + '|' + outcome);
		const QString verticalSpeed = QStringLiteral("  Vertical speed: -95 ft/min");
		QCOMPARE(evalPageJs(widget_, js.arg(fullContactJs(), QString())).toString(), verticalSpeed + '|' + verticalSpeed);

		// Nor does its "Consider:" line list the comfort.
		const QString consider = QStringLiteral(
			"(function () { var t = %1; %2"
			"  return [buildTouchdownPrompt(t), buildTouchdownPrompt(t, true)].map(function (p) {"
			"    return p.split('\\n').filter(function (l) { return /^Consider:/.test(l); })[0].split(', ').slice(0, 2).join(', '); }).join('|'); })()");
		const QString crashed = QStringLiteral(
			"Consider: touchdown rate, crosswind component (calculate from the true wind vs the true runway heading)");
		QCOMPARE(evalPageJs(widget_, consider.arg(fullContactJs(), QStringLiteral("t.crashed = true;"))).toString(), crashed + '|' + crashed);
		const QString landed = QStringLiteral("Consider: touchdown rate, vertical speed & G-force comfort");
		QCOMPARE(evalPageJs(widget_, consider.arg(fullContactJs(), QString())).toString(), landed + '|' + landed);
	}

	// The "Consider:" line lists what needs a runway (crosswind, centerline,
	// runway length/remaining and slope, touchdown zone) only when one was
	// matched, and then the runway's surface and terrain; without one, the
	// surface and the terrain around the contact instead. With a runway whose
	// true heading, length and slope weren't recorded (a contact from before
	// they were stored), the crosswind, runway remaining and slope, which
	// need them, are left out too. The aircraft's
	// handling ends it when the type is known; a contact-only liftoff without
	// a runway or a type has nothing to consider and no line.
	void considerLinesListRunwayItemsOnlyWithARunway_data() {
		QTest::addColumn<QString>("setup");
		QTest::addColumn<QStringList>("expected");
		const QString crosswind = QStringLiteral("crosswind component (calculate from the true wind vs the true runway heading), ");
		const QString weather = QStringLiteral("gusts, wind shear, turbulence, cloud and icing, light and visibility, ");
		QTest::newRow("runway") << QStringLiteral("window._aircraftTitle = 'Cessna C152';") << QStringList{
			QStringLiteral("Consider: ") + crosswind + QStringLiteral("centerline tracking before the liftoff, use of available runway "
				"length and the runway remaining, runway slope, liftoff and climb-out attitude and speed, density altitude, runway surface "
				"condition, ") + weather + QStringLiteral("terrain after the runway, aircraft-type-specific handling."),
			QStringLiteral("Consider: ") + crosswind + QStringLiteral("centerline tracking before the liftoff, use of available runway "
				"length, aircraft-type-specific handling."),
			QStringLiteral("Consider: touchdown rate, vertical speed & G-force comfort, ") + crosswind + QStringLiteral("decrab at touchdown "
				"(crab angle), centerline tracking, touchdown zone placement and the runway remaining, runway slope, stabilization at the "
				"gates, flare technique, density altitude, runway surface condition, ") + weather
				+ QStringLiteral("terrain before the runway, aircraft-type-specific handling."),
			QStringLiteral("Consider: touchdown rate, vertical speed & G-force comfort, ") + crosswind + QStringLiteral("decrab at touchdown "
				"(crab angle), centerline tracking, touchdown zone placement, aircraft-type-specific handling.") };
		QTest::newRow("no runway") << QStringLiteral("t.runway = ''; window._aircraftTitle = 'Cessna C152';") << QStringList{
			QStringLiteral("Consider: liftoff and climb-out attitude and speed, density altitude, surface condition, ") + weather
				+ QStringLiteral("terrain after the liftoff point, aircraft-type-specific handling."),
			QStringLiteral("Consider: aircraft-type-specific handling."),
			QStringLiteral("Consider: touchdown rate, vertical speed & G-force comfort, decrab at touchdown (crab angle), stabilization at "
				"the gates, flare technique, density altitude, surface condition, ") + weather
				+ QStringLiteral("terrain before the touchdown point, aircraft-type-specific handling."),
			QStringLiteral("Consider: touchdown rate, vertical speed & G-force comfort, decrab at touchdown (crab angle), "
				"aircraft-type-specific handling.") };
		QTest::newRow("runway without its data") << QStringLiteral("t.runwayHeading = -1; delete t.runwayLengthFt; delete t.runwaySlope;"
			" window._aircraftTitle = '';") << QStringList{
			QStringLiteral("Consider: centerline tracking before the liftoff, use of available runway length, liftoff and climb-out "
				"attitude and speed, density altitude, runway surface condition, ") + weather + QStringLiteral("terrain after the runway."),
			QStringLiteral("Consider: centerline tracking before the liftoff, use of available runway length."),
			QStringLiteral("Consider: touchdown rate, vertical speed & G-force comfort, decrab at touchdown (crab angle), centerline tracking, "
				"touchdown zone placement, stabilization at the gates, flare technique, density altitude, runway surface condition, ") + weather
				+ QStringLiteral("terrain before the runway."),
			QStringLiteral("Consider: touchdown rate, vertical speed & G-force comfort, decrab at touchdown (crab angle), centerline tracking, "
				"touchdown zone placement.") };
		QTest::newRow("no runway or type") << QStringLiteral("t.runway = ''; window._aircraftTitle = '';") << QStringList{
			QStringLiteral("Consider: liftoff and climb-out attitude and speed, density altitude, surface condition, ") + weather
				+ QStringLiteral("terrain after the liftoff point."),
			QStringLiteral("none"),
			QStringLiteral("Consider: touchdown rate, vertical speed & G-force comfort, decrab at touchdown (crab angle), stabilization at "
				"the gates, flare technique, density altitude, surface condition, ") + weather
				+ QStringLiteral("terrain before the touchdown point."),
			QStringLiteral("Consider: touchdown rate, vertical speed & G-force comfort, decrab at touchdown (crab angle).") };
	}
	void considerLinesListRunwayItemsOnlyWithARunway() {
		QFETCH(QString, setup);
		QFETCH(QStringList, expected);
		const QString js = QStringLiteral(
			"(function () { var t = %1, title = window._aircraftTitle; %2"
			"  var prompts = [buildLiftoffPrompt(t), buildLiftoffPrompt(t, true), buildTouchdownPrompt(t), buildTouchdownPrompt(t, true)];"
			"  window._aircraftTitle = title;"
			"  return prompts.map(function (p) {"
			"    return p.split('\\n').filter(function (l) { return /^Consider:/.test(l); }).join('#') || 'none'; }); })()")
			.arg(fullContactJs(), setup);
		QCOMPARE(evalPageJs(widget_, js).toStringList(), expected);
	}

	// Without a matched runway, every prompt (both contacts, both
	// evaluations) says there are no headwind or crosswind components and to
	// judge the wind against the true ground track; with one, none does.
	void promptsWithoutARunwaySayThereAreNoWindComponents() {
		const QString js = QStringLiteral(
			"(function () { var t = %1; %2"
			"  return [buildLiftoffPrompt(t), buildLiftoffPrompt(t, true), buildTouchdownPrompt(t), buildTouchdownPrompt(t, true)].map(function (p) {"
			"    return p.split('\\n').filter(function (l) { return /^  Runway: none/.test(l); }).join('#') || 'none'; }).join('|'); })()");
		const QString none = QStringLiteral(
			"  Runway: none matched (not on a known runway), so there are no headwind or crosswind components; "
			"judge the wind against the aircraft's true ground track instead");
		QCOMPARE(evalPageJs(widget_, js.arg(fullContactJs(), QStringLiteral("t.runway = '';"))).toString(),
			QStringList({ none, none, none, none }).join('|'));
		QCOMPARE(evalPageJs(widget_, js.arg(fullContactJs(), QString())).toString(), QStringLiteral("none|none|none|none"));
	}

	// A progress evaluation tells the model to cite each significant recorded
	// condition in its Environment section and to keep what the conditions
	// caused apart from the technique, naming its contact; a contact
	// evaluation has no Environment section, so no such rule.
	void progressPromptsSayHowToWeighTheWeather() {
		const QString js = QStringLiteral(
			"(function () { var t = %1;"
			"  return [buildLiftoffPrompt(t), buildLiftoffPrompt(t, true), buildTouchdownPrompt(t), buildTouchdownPrompt(t, true)].map(function (p) {"
			"    return p.split('\\n').filter(function (l) { return /^Weather:/.test(l); }).join('#') || 'none'; }).join('|'); })()")
			.arg(fullContactJs());
		const QString rule = QStringLiteral(
			"Weather: in the Environment section, cite each significant recorded condition, including those marked above, with its value "
			"and its effect on this %1; elsewhere keep what the conditions caused apart from technique, neither faulting the pilot for it "
			"nor excusing technique by it.");
		QCOMPARE(evalPageJs(widget_, js).toString(),
			QStringList({ rule.arg(QStringLiteral("liftoff")), QStringLiteral("none"), rule.arg(QStringLiteral("landing")), QStringLiteral("none") }).join('|'));
	}

	// A value that wasn't recorded gets no line (never "NaN" or
	// "undefined"): without a sample only the time and position remain, and
	// an empty trajectory adds no block.
	void promptLeavesOutWhatWasNotRecorded() {
		const QString js = QStringLiteral(
			"(function (extra) { var t = {icao: 'EGLL', runway: '27L', runwayHeading: 270, lat: 51.47, lng: -0.45,"
			"    airspeed: 140, verticalSpeed: -120, pitchDegrees: 3, bankDegrees: 0, headingDegrees: 271, distanceLength: 1000,"
			"    distanceWidth: 2, distanceLengthPercent: 0.2, distanceWidthPercent: 0.04, windDirection: 250, windVelocity: 8,"
			"    zuluTime: '2026-01-01T10:00:00.000+00:00_4', gForce: 1.1};"
			"  Object.assign(t, extra);"
			"  var p = buildTouchdownPrompt(t);"
			"  var lines = p.split('\\n');"
			"  var at = lines.indexOf('Conditions:');"
			"  return [/NaN|undefined/.test(p), /Final trajectory/.test(p),"
			"    lines.filter(function (l) { return /^  Heading:/.test(l); })[0]]"
			"    .concat(lines.slice(at + 1, lines.indexOf('', at))).join('|'); })(%1)");
		const QString withoutSample = QStringLiteral(
			"false|false|  Heading: 271° magnetic|  Time: 2026-01-01T10:00:00.000+00:00|  Position: 51.4700° N, 0.4500° W");
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("{profile: {gates: [], series: []}}"))).toString(), withoutSample);
		QCOMPARE(evalPageJs(widget_, js.arg(QStringLiteral("{conditions: {}, profile: {gates: [], series: []}}"))).toString(),
			withoutSample);
	}

	// A trajectory that couldn't be built (null, from MapBridge) isn't left
	// out silently: the prompt says so, and has the model judge the contact
	// alone and say so in its trajectory's segment section.
	void promptSaysWhenTheTrajectoryIsUnavailable() {
		const QString js = QStringLiteral(
			"(function () { var t = %1; t.profile = null;"
			"  return [buildLiftoffPrompt(t), buildTouchdownPrompt(t)].map(function (p) {"
			"    return p.split('\\n').filter(function (l) { return /^(Liftoff|Final) trajectory/.test(l); }).join('/'); }).join('|'); })()")
			.arg(fullContactJs());
		QCOMPARE(evalPageJs(widget_, js).toString(), QStringLiteral(
			"Liftoff trajectory: not available (it could not be built from the recorded samples). Judge the liftoff point from the data "
			"above alone, and in the Liftoff and climb-out section say that the trajectory was not available instead of describing it."
			"|Final trajectory: not available (it could not be built from the recorded samples). Judge the touchdown point from the data "
			"above alone, and in the Final segment section say that the trajectory was not available instead of describing it."));
	}

	// The runway's dimensions and the runway left ahead follow the contact:
	// a touchdown's distance is from the marked threshold, so its end's
	// displaced threshold counts as behind it; a liftoff's is from the
	// physical start. Each part only when stored; none without the length.
	void runwayLinesFollowTheContactType() {
		const QString js = QStringLiteral(
			"(function (build, extra) { var t = %1; Object.assign(t, extra);"
			"  var p = build(t);"
			"  return p.split('\\n').filter(function (l) { return /^  Runway(: \\d+ ft| remaining)/.test(l); })"
			"    .concat([/; threshold = along the runway from the (marked threshold|runway start)/.exec(p) || 'no threshold'].map(String)).join('|'); })")
			.arg(fullContactJs());
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("(buildLiftoffPrompt, {runwaySlope: 0.25})")).toString(),
			QStringLiteral("  Runway: 10000 ft long, 150 ft wide, threshold displaced 820 ft at this end, slope 0.25° uphill in the direction of travel"
				"|  Runway remaining ahead: 8750 ft (88% of the runway length)"
				"|; threshold = along the runway from the runway start,runway start"));
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("(buildTouchdownPrompt, {runwaySlope: 0, thresholdOffsetFt: 0, runwayWidthFt: undefined})")).toString(),
			QStringLiteral("  Runway: 10000 ft long, no displaced threshold at this end, level"
				"|  Runway remaining ahead: 8750 ft (88% of the runway length)"
				"|; threshold = along the runway from the marked threshold,marked threshold"));
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("(buildTouchdownPrompt, {runwayLengthFt: undefined, profile: {gates: [], series: [{seconds: 0}]}})")).toString(),
			QStringLiteral("no threshold"));
	}

	// Out of cloud is said, not left out; no precipitation has no rate.
	void cloudAndPrecipitationLines() {
		const QString js = QStringLiteral(
			"(function () { var t = %1; Object.assign(t.conditions, {inCloud: 0, precipitation: 'none', precipitationRate: 0});"
			"  return buildTouchdownPrompt(t).split('\\n').filter(function (l) { return /^  (Cloud|Precipitation):/.test(l); }).join('|'); })()")
			.arg(fullContactJs());
		QCOMPARE(evalPageJs(widget_, js).toString(), QStringLiteral("  Cloud: not in cloud|  Precipitation: none"));
	}

	// A crab that rounds to 0.0° names no side of the track.
	void promptCrabRoundingToZeroNamesNoSide() {
		const QString js = QStringLiteral(
			"(function () { var t = %1; t.conditions.crabAngle = -0.04;"
			"  return buildTouchdownPrompt(t).split('\\n').filter(function (l) { return /^  Heading:/.test(l); }).join('|'); })()")
			.arg(fullContactJs());
		QCOMPARE(evalPageJs(widget_, js).toString(),
			QStringLiteral("  Heading: 334° magnetic (338° true); ground track 341° true; crab 0.0°"));
	}

	// The surface line names only the parts that are known.
	void surfaceLineNamesOnlyWhatIsKnown() {
		const QString js = QStringLiteral(
			"(function (extra) { var t = %1; Object.assign(t.conditions, extra);"
			"  return buildTouchdownPrompt(t).split('\\n').filter(function (l) { return /^  Surface/.test(l); }).join('|'); })")
			.arg(fullContactJs());
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("({surfaceCondition: undefined})")).toString(),
			QStringLiteral("  Surface under the aircraft: asphalt"));
		QCOMPARE(evalPageJs(widget_, js + QStringLiteral("({surfaceType: undefined})")).toString(),
			QStringLiteral("  Surface under the aircraft: wet"));
	}

	// Each Analyze button asks MapBridge for its own contact's trajectory,
	// by its row: a liftoff's climb-out, a touchdown's approach.
	void analyzeAsksForItsOwnContactsTrajectory() {
		const QString asked = evalPageJs(widget_, QStringLiteral(R"JS(
			(function () {
			    var asked = [], realRun = runAiAnalysis;
			    var real = { lo: mapBridge.liftoffProfile, td: mapBridge.touchdownProfile };
			    mapBridge.liftoffProfile = function (rowId) { asked.push('liftoff ' + rowId); };
			    mapBridge.touchdownProfile = function (rowId) { asked.push('touchdown ' + rowId); };
			    runAiAnalysis = function (id, t, name, loadProfile) { loadProfile(t.rowId, function () {}); };
			    _liftoffStore.probe = { rowId: 3 };
			    _touchdownStore.probe = { rowId: 7 };
			    analyzeLiftoff('probe');
			    analyzeTouchdown('probe');
			    delete _liftoffStore.probe;
			    delete _touchdownStore.probe;
			    runAiAnalysis = realRun;
			    mapBridge.liftoffProfile = real.lo;
			    mapBridge.touchdownProfile = real.td;
			    return asked.join('|');
			})()
		)JS")).toString();
		QCOMPARE(asked, QStringLiteral("liftoff 3|touchdown 7"));
	}

	void sunLightBands() {
		QCOMPARE(evalPageJs(widget_, QStringLiteral("[5, -0.833, -0.9, -6, -6.1, -12, -12.1, -18, -18.1].map(sunLightLabel).join('|')")).toString(),
			QStringLiteral("day|day|civil twilight|civil twilight|nautical twilight|nautical twilight|astronomical twilight|"
				"astronomical twilight|night"));
	}

	// The prompt is bounded whatever the trip: with every value recorded and
	// every line a value can add (a crash, the aircraft weight, every segment
	// mark, every engine failed and on fire), every gate crossed with all
	// SIM_ENGINE_INDEXES engines' values (of which the prompt shows 4, naming
	// them) and the series at its most (contact_analysis.cpp's 56 every-sample
	// points and 20 one-per-second ones, each with its ground speed and
	// G-force; a path angle only on the distance gates, the only points
	// contact_analysis.cpp gives one), both prompts stay
	// within 32,000 characters: Gemini's per-minute limit of 16,000 prompt
	// tokens at 2 characters per token (Gemma's tokenizer gives each digit a
	// token of its own; these mostly numeric prompts measured 2.2-3.1), so
	// neither is ever rate-limited for good. The power is each name at its
	// widest value: a turbine's three-digit torque (N1 is
	// shorter), and a piston's manifold pressure (MP, spelled out once) under
	// 100 inHg, which no engine reaches (the most boosted run at about 80-90 inHg).
	void worstCasePromptFitsTheTokenLimit() {
		const QString js = QStringLiteral(
			"(function () { var t = %1;"
			"  window._aircraftTitle = 'Aircraft With A Very Long Livery And Model Name For The Worst Case Prompt Size';"
			"  t.crashed = true; t.conditions.totalWeightLb = 1234567.8; t.conditions.fuelWeightLb = 123456.7;"
			"  t.airportName = 'An Airport Whose Name Is As Long As Any In The Facility Database Could Be';"
			"  function point(gate) { var p = {seconds: -123.456, heightFt: -12345.678, distanceNm: 12.3456, terrainFt: -12345.678,"
			"      airspeed: 123.456, groundSpeedKt: 123.456, verticalSpeed: -12345.678, gForce: -12.345, pitch: -12.345, bank: -123.456, headingTrue: 359.6, trackTrue: 359.6,"
			"      thresholdFt: -123456.789, centerlineFt: -12345.678, pathAngle: -12.345, onGround: false,"
			"      windDirection: 359.4, windSpeedKt: 123.4, headwindKt: -123.4, crosswindKt: -123.4, verticalWindFpm: -12345.6,"
			"      temperatureC: -12.34,"
			"      inCloud: 0, precipitation: 'rain and snow', structuralIce: 1, throttle: [], power: [], engineFailed: [], engineOnFire: []};"
			"    for (var e = 0; e < %2; ++e) { p.throttle.push(-123.456); p.power.push(power); p.engineFailed.push(1); p.engineOnFire.push(1); }"
			"    if (gate) p.gate = gate; if (!/ nm$/.test(gate)) delete p.pathAngle; return p; }"
			"  var power, lengths = [];"
			"  [['torque', '%', 123.456], ['MP', ' inHg', 99.4, 'manifold pressure']].forEach(function (c) {"
			"    power = c[2];"
			"    t.profile = {gates: ['1000 ft', '500 ft', '300 ft', '200 ft', '100 ft', '50 ft', '3 nm', '2 nm', '1 nm'].map(point),"
			"      series: [], highestTerrainFt: 12345.678, highestTerrainNm: 4.999, terrainRangeNm: 4.994, powerName: c[0], powerUnit: c[1], powerTitle: c[3],"
			"      segment: {samples: 999, windMinKt: 123.4, windMaxKt: 124.4, headwindFromKt: -123.4, headwindFromHeightFt: -1234.5,"
			"        headwindToKt: -123.4, headwindToHeightFt: -1234.5, maxTailwindKt: 123.4, maxCrosswindKt: -123.4, cloudKnown: true,"
			"        lowestInCloudFt: -1234.5, icingConditions: 1, maxStructuralIce: 1, maxPitotIce: 1, engineAntiIceShare: 1,"
			"        gForceMin: -12.345, gForceMax: -11.345, verticalWindMinFpm: -12345.6, verticalWindMinHeightFt: -1234.5,"
			"        verticalWindMaxFpm: -12344.6, verticalWindMaxHeightFt: -1234.5,"
			"        stallMarginAlphaDeg: -12.34, stallMarginStallAlphaDeg: -12.34, stallMarginHeightFt: -1234.5,"
			"        stallMarginAirspeedKt: -123.4, stallMarginBankDeg: -123.45}};"
			"    for (var i = 0; i < %3; ++i) t.profile.series.push(point());"
			"    lengths.push(buildLiftoffPrompt(t).length, buildTouchdownPrompt(t).length); });"
			"  window._aircraftTitle = '';"
			"  return lengths; })()")
			.arg(fullContactJs(), QString::number(SIM_ENGINE_INDEXES), QString::number(56 + 20));
		const QVariantList lengths = evalPageJs(widget_, js).toList();
		QCOMPARE(lengths.size(), 4);
		for (const QVariant& length : lengths)
			QVERIFY2(length.toInt() > 0 && length.toInt() <= 32000, qPrintable(length.toString()));
		qInfo("worst-case prompt lengths: torque liftoff %d, touchdown %d; manifold pressure liftoff %d, touchdown %d characters",
			lengths[0].toInt(), lengths[1].toInt(), lengths[2].toInt(), lengths[3].toInt());
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
		td.actualVerticalSpeed = -182.5;  // halves again
		td.touchdownRate = 262.5;
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
		td.conditions.fromSample = true;
		td.conditions.headingTrue = 268.5;  // halves again
		td.conditions.crabAngle = -2.5;
		td.conditions.runwayHeadingMagnetic = 268;
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
		QVERIFY2(popup.contains(QStringLiteral("Runway: 27L (268° M, 270° T)")), qPrintable(popup.join('|')));
		QVERIFY2(popup.contains(QStringLiteral("Heading: 271° M (269° T)")), qPrintable(popup.join('|')));
		QVERIFY2(popup.contains(QStringLiteral("Crab: 3° L")), qPrintable(popup.join('|')));
		QVERIFY2(popup.contains(QStringLiteral("V/S: -183 ft/min")), qPrintable(popup.join('|')));
		QVERIFY2(popup.contains(QStringLiteral("Touchdown rate: 263 ft/min")), qPrintable(popup.join('|')));

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

	// Until the answer starts, a status line says what's being waited for:
	// the trajectory being built, the request on its way, then the model;
	// it's gone as soon as the answer shows.
	void aStatusLineShowsUntilTheAnswerStarts() {
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } });
		QCOMPARE(r.value("statuses").toString(),
			QStringLiteral("Building the climb-out trajectory…|Sending to gemma-4-31b-it…|Waiting for the model…"));
		QCOMPARE(r.value("statusShownAtStreamEnd").toBool(), false);
		QCOMPARE(r.value("statusShown").toBool(), false);
	}

	// The analysis first loads its contact's trajectory (by row id) into the
	// prompt's t.profile. One that can't be built (null) doesn't stop it:
	// the answer is still saved, and a notice under the button says the
	// analysis covers the contact alone. With a trajectory there's none.
	void anAnalysisWithoutATrajectoryGoesAheadWithANotice() {
		const QStringList stream = { aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) };
		QVariantMap r = runAiAnalysisWith({ stream }, true, {}, QStringLiteral("null"));
		QCOMPARE(r.value("profileRow").toInt(), 5);
		QCOMPARE(r.value("promptProfile").toString(), QStringLiteral("null"));
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Grade: A"));
		QCOMPARE(r.value("noticeShown").toBool(), true);

		r = runAiAnalysisWith({ stream }, true, {}, QStringLiteral("{gates: [], series: [{seconds: -1}]}"));
		QCOMPARE(r.value("promptProfile").toString(), QStringLiteral(R"({"gates":[],"series":[{"seconds":-1}]})"));
		QCOMPARE(r.value("noticeShown").toBool(), false);
	}

	// A progress evaluation (the default) asks for the prompt with the
	// trajectory. A contact evaluation (the popup's checkbox unchecked)
	// loads no trajectory, so it shows no "Building…" status and no notice
	// that one is missing, and asks for the contact-only prompt.
	void aContactEvaluationLoadsNoTrajectory() {
		const QStringList stream = { aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) };
		QVariantMap r = runAiAnalysisWith({ stream });
		QCOMPARE(r.value("contactOnly").toBool(), false);
		QCOMPARE(r.value("profileRow").toInt(), 5);

		evalPageJs(widget_, QStringLiteral("aiProgressChosen(false)"));
		r = runAiAnalysisWith({ stream }, true, {}, QStringLiteral("null"));
		QCOMPARE(r.value("contactOnly").toBool(), true);
		QVERIFY(!r.contains("profileRow") || r.value("profileRow").isNull());
		QCOMPARE(r.value("promptProfile").toString(), QString());
		QCOMPARE(r.value("statuses").toString(), QStringLiteral("Sending to gemma-4-31b-it…|Waiting for the model…"));
		QCOMPARE(r.value("noticeShown").toBool(), false);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Grade: A"));
	}

	// The API key travels in the x-goog-api-key header, never in the URL,
	// which the browser's console messages (copied into the log) can quote.
	void aiRequestSendsTheKeyInAHeaderNotTheUrl() {
		evalPageJs(widget_, QStringLiteral("window._geminiApiKey = 'secret-key'"));
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } });
		evalPageJs(widget_, QStringLiteral("window._geminiApiKey = ''"));
		QCOMPARE(r.value("apiKey").toString(), QStringLiteral("secret-key"));
		QCOMPARE(r.value("url").toString(),
			QStringLiteral("https://generativelanguage.googleapis.com/v1beta/models/gemma-4-31b-it:streamGenerateContent"));
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
		verifyAttemptLogs(r, {});  // nothing failed, nothing logged
	}

	// While the thinking streams, its panel reads "▾ Thinking…" and its
	// summary can't be clicked; once the answer starts, the panel closes
	// behind a clickable "▸ Show thinking".
	void theThinkingPanelIsClickableOnlyOnceTheAnswerStarts() {
		const QString thought = aiChunk({ { "Wind from the left.", true } });
		const QVariantMap r = runAiAnalysisWith({ { QLatin1Char('[') + thought + QStringLiteral(",\r\n"),
			aiChunk({ { "Grade: B", false } }, "STOP") + QLatin1Char(']') } });
		QCOMPARE(r.value("saved").toString(), QStringLiteral("<thinking>Wind from the left.</thinking>Grade: B"));
		QCOMPARE(r.value("thinkingPanel").toString(), QStringLiteral("none,▾,Thinking…"));
		QCOMPARE(r.value("thinkingSummary").toString(), QStringLiteral(",▸,Show thinking"));
	}

	// An answer stopped early (token limit), or a stream that ends with no
	// finish at all, isn't complete even after thinking: it's retried, each
	// retry logged with its finish reason and how much had arrived.
	void anAiAnswerCutOffIsRetried() {
		const QVariantMap r = runAiAnalysisWith({
			{ aiStream({ aiChunk({ { "Thought", true }, { "Grade: half", false } }, "MAX_TOKENS") }) },
			{ aiStream({ aiChunk({ { "Thought", true }, { "Grade: half", false } }) }) },
			{ aiStream({ aiChunk({ { "Grade: C", false } }, "STOP") }) } });
		QCOMPARE(r.value("calls").toInt(), 3);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Grade: C"));
		QCOMPARE(r.value("thinkingShown").toBool(), false);
		verifyAttemptLogs(r, {
			{ true, { "MAX_TOKENS", "7 characters of thinking", "11 of answer" } },
			{ true, { "none", "7 characters of thinking", "11 of answer" } } });
	}

	// An error the service sends mid-stream, after thinking arrived, in place
	// of the rest of the answer: the attempt is retried, logged with that
	// error, which the next attempt's log line doesn't repeat.
	void anErrorMidStreamIsRetriedAndLogged() {
		const QVariantMap r = runAiAnalysisWith({
			{ aiStream({ aiChunk({ { "Thought", true } }),
				QStringLiteral(R"({"error":)") + streamError500 + '}' }) },
			{ aiStream({ aiChunk({ { "Grade: half", false } }, "MAX_TOKENS") }) },
			{ aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } });
		QCOMPARE(r.value("calls").toInt(), 3);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Grade: A"));
		const QStringList lines = r.value("console").toStringList();
		verifyAttemptLogs(r, {
			{ true, { "none", "7 characters of thinking", "0 of answer", QStringLiteral("stream error ") + streamError500 } },
			{ true, { "MAX_TOKENS", "0 characters of thinking", "11 of answer" } } });
		QVERIFY2(!lines.value(1).contains(QStringLiteral("stream error")), qPrintable(lines.value(1)));  // not carried into the next attempt
	}

	// Three incomplete answers in a row: nothing is saved and the message
	// says so, naming the Analyze button, the last one logged as final; a finish with no answer text
	// counts as incomplete too.
	void threeIncompleteAiAnswersSaveNothing() {
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Thought", true } }, "STOP") }) } });
		QCOMPARE(r.value("calls").toInt(), 3);
		QVERIFY(r.value("saved").isNull());
		QCOMPARE(r.value("text").toString(), QStringLiteral("The AI didn't return a complete analysis. "
			"This can happen occasionally. Click Analyze to try again."));
		QCOMPARE(r.value("thinkingShown").toBool(), false);
		const AttemptLog thoughtOnly{ true, { "STOP", "7 characters of thinking", "0 of answer" } };
		verifyAttemptLogs(r, { thoughtOnly, thoughtOnly, { false, thoughtOnly.values } });
	}

	// The seconds after `label` (e.g. "longest silence 1.0 s") in a log line,
	// -1 if it isn't there.
	static double loggedSeconds(const QString& line, const QString& label) {
		const QRegularExpressionMatch m = QRegularExpression(QRegularExpression::escape(label) + QStringLiteral(R"( (\d+\.\d) s)")).match(line);
		return m.hasMatch() ? m.captured(1).toDouble() : -1;
	}

	// An attempt that receives nothing for the inactivity timeout, whether
	// mid-answer or waiting for the response, is abandoned and retried as an
	// incomplete answer, logged as stalled with how long it went silent.
	void aStalledAiAnswerIsRetried() {
		const QVariantMap r = runAiAnalysisWith({
			{ aiStream({ aiChunk({ { "Thought", true } }) }).chopped(1), QStringLiteral("<hang>") },
			{},
			{ aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } }, true, { 200, -2, 200 });
		QCOMPARE(r.value("calls").toInt(), 3);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Grade: A"));
		verifyAttemptLogs(r, {
			{ true, { "Gemini answer incomplete", "none", "7 characters of thinking", "stalled", "first data after" } },
			{ true, { "Gemini answer incomplete", "0 characters of thinking", "stalled", "no data" } } });
		for (const QString& line : r.value("console").toStringList()) {
			const double silence = loggedSeconds(line, QStringLiteral("longest silence"));
			QVERIFY2(silence >= 1.0 && silence < 2.0, qPrintable(line));
		}
	}

	// Three stalled attempts in a row: nothing is saved and the message says
	// the answer was incomplete, the last stall logged as final.
	void threeStalledAiAnswersSaveNothing() {
		const QVariantMap r = runAiAnalysisWith({ { QStringLiteral("<hang>") } });
		QCOMPARE(r.value("calls").toInt(), 3);
		QVERIFY(r.value("saved").isNull());
		QVERIFY(r.value("text").toString().startsWith(QStringLiteral("The AI didn't return a complete analysis.")));
		const AttemptLog stall{ true, { "Gemini answer incomplete", "stalled", "no data" } };
		verifyAttemptLogs(r, { stall, stall, { false, stall.values } });
	}

	// A stalled request is abandoned after 3 minutes of silence (README).
	void theAiInactivityTimeoutIsThreeMinutes() {
		QCOMPARE(evalPageJs(widget_, QStringLiteral("AI_IDLE_TIMEOUT_MS")).toInt(), 180000);
	}

	// An answer that finished before its connection stalled is saved, not
	// retried: the stall is no failure, so it's logged as complete.
	void anAiAnswerStalledAfterItsEndIsSaved() {
		const QVariantMap r = runAiAnalysisWith({
			{ aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }).chopped(1), QStringLiteral("<hang>") } });
		QCOMPARE(r.value("calls").toInt(), 1);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Grade: A"));
		QVERIFY(r.value("console").toStringList().isEmpty());
		const QStringList info = r.value("info").toStringList();
		QCOMPARE(info.size(), 1);
		QVERIFY2(info[0].startsWith(QStringLiteral("log: Gemini answer complete")), qPrintable(info[0]));
	}

	// A network error after a stall is still a connection failure: not marked
	// as stalled, and the last attempt's message says the service couldn't be
	// reached.
	void aNetworkErrorAfterAStallIsNotLoggedAsStalled() {
		const QVariantMap r = runAiAnalysisWith({ { QStringLiteral("<hang>") } }, true, { 200, 0 });
		QCOMPARE(r.value("calls").toInt(), 3);
		QVERIFY(r.value("text").toString().startsWith(QStringLiteral("Couldn't reach the AI service")));
		const QStringList console = r.value("console").toStringList();
		QCOMPARE(console.size(), 3);
		QVERIFY2(console[0].contains(QStringLiteral("stalled")), qPrintable(console[0]));
		for (int i = 1; i < 3; ++i) {
			QVERIFY2(console[i].contains(QStringLiteral("Gemini request failed")), qPrintable(console[i]));
			QVERIFY2(!console[i].contains(QStringLiteral("stalled")), qPrintable(console[i]));
		}
	}

	// Each attempt's timing is its own: a first attempt whose data comes
	// late and then stalls (1 s from its last piece to the retry) doesn't
	// carry its silence, send time or last piece's time into the retry's log.
	void eachAiAttemptLogsItsOwnTiming() {
		const QVariantMap r = runAiAnalysisWith({
			{ QStringLiteral("<pause 600>"), aiStream({ aiChunk({ { "Thought", true } }) }).chopped(1), QStringLiteral("<hang>") },
			{ aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } });
		QCOMPARE(r.value("calls").toInt(), 2);
		const QStringList info = r.value("info").toStringList();
		QCOMPARE(info.size(), 1);
		QVERIFY2(loggedSeconds(info[0], QStringLiteral("first data after")) < 0.4, qPrintable(info[0]));
		QVERIFY2(loggedSeconds(info[0], QStringLiteral("longest silence")) < 0.4, qPrintable(info[0]));
	}

	// An answer that keeps arriving, each piece within the timeout, isn't cut
	// off however long it takes in all, nor by the clock of an attempt
	// before it; its timing is logged when it completes.
	void aSlowlyStreamingAiAnswerIsNotCutOff() {
		const QString pause = QStringLiteral("<pause 400>");
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Grade: half", false } }, "MAX_TOKENS") }) },
			{ pause, QLatin1Char('[') + aiChunk({ { "Thought", true } }) + QStringLiteral(",\r\n"),
			pause, aiChunk({ { "Grade: ", false } }) + QStringLiteral(",\r\n"),
			pause, aiChunk({ { "B", false } }, "STOP") + QLatin1Char(']') } });
		QCOMPARE(r.value("calls").toInt(), 2);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("<thinking>Thought</thinking>Grade: B"));
		verifyAttemptLogs(r, { { true, { "MAX_TOKENS" } } });
		const QStringList info = r.value("info").toStringList();
		QCOMPARE(info.size(), 1);
		QVERIFY2(info[0].startsWith(QStringLiteral("log: Gemini answer complete")), qPrintable(info[0]));
		const double first = loggedSeconds(info[0], QStringLiteral("first data after"));
		const double silence = loggedSeconds(info[0], QStringLiteral("longest silence"));
		QVERIFY2(first >= 0.4 && first < 0.9, qPrintable(info[0]));
		QVERIFY2(silence >= 0.4 && silence < 0.9, qPrintable(info[0]));
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
	// While it runs, the evaluation checkbox, the model drop-down and the
	// Other field are disabled, in the popup it started in and the reopened
	// one, and enabled again once it's done. The reopened popup shows the
	// choices the analysis was started with (progress, the first model), not
	// the ones changed meanwhile (as from another contact's popup: contact,
	// Other with no name yet), nor their problem, which it shows, still open,
	// once the analysis is done (Analyze disabled for it). Its Copy Prompt copies the evaluation it shows (progress, with
	// the liftoff and climb-out section) while it runs.
	void reopeningAPopupMidAnalysisDoesNotRunItTwice() {
		evalPageJs(widget_, QStringLiteral("window._geminiApiKey = 'test-key'")); // the popups are built with it
		TripDataset dataset = tripToSave();
		dataset.liftoffPoints[0].rowId = 5;
		loadTrip(dataset);
		evalPageJs(widget_, kContactMarkerJs + QStringLiteral(R"JS(
			window._geminiApiKey = '';
			var id = Object.keys(_liftoffStore)[0];
			var marker = contactMarker(id);
			var realFetch = window.fetch, realSave = mapBridge.saveLiftoffAnalysisReport;
			var ai = window._dup = { id: id, marker: marker, calls: 0, saves: 0, release: null };
			window.fetch = function () {
			    ai.calls++;
			    return new Promise(function (resolve) { ai.release = resolve; });
			};
			mapBridge.saveLiftoffAnalysisReport = function (rowId, report, onSaved) { ai.saves++; onSaved(true); };
			var realCopy = mapBridge.copyText;
			mapBridge.copyText = function (text) { ai.copied = text; };
			ai.restore = function () { window.fetch = realFetch; mapBridge.saveLiftoffAnalysisReport = realSave; mapBridge.copyText = realCopy; };
			ai.choicesDisabled = function () {
			    return ['td-progress-', 'td-model-', 'td-custom-name-'].map(function (prefix) {
			        return document.getElementById(prefix + id).disabled;
			    }).join(',');
			};
			ai.choicesShown = function () {
			    return [document.getElementById('td-progress-' + id).checked, document.getElementById('td-model-' + id).value,
			            document.getElementById('td-custom-' + id).style.display, document.getElementById('td-custom-name-' + id).value,
			            document.getElementById('td-model-problem-' + id).textContent].join(',');
			};
			ai.realChoices = [window._aiProgress, window._aiModel, window._aiCustomModel];
			ai.restoreChoices = function () {
			    window._aiProgress = ai.realChoices[0]; window._aiModel = ai.realChoices[1]; window._aiCustomModel = ai.realChoices[2];
			};
			window._aiProgress = true; window._aiModel = 'gemma-4-31b-it'; window._aiCustomModel = '';
			marker.openPopup();
			ai.openedDisabled = ai.choicesDisabled();
			ai.firstButton = document.getElementById('td-btn-' + id);
			ai.firstButton.click();
		)JS"));
		// The page-wide choices the test changes are put back even when a
		// check fails, so the tests after it analyze with the usual ones.
		const auto restoreChoices = qScopeGuard([this] { evalPageJs(widget_, QStringLiteral("window._dup.restoreChoices(); true")); });
		// The request goes out once the trajectory is back from MapBridge.
		QTRY_COMPARE(evalPageJs(widget_, QStringLiteral("window._dup.calls")).toInt(), 1);

		const QString reopened = evalPageJs(widget_, QStringLiteral(R"JS(
			var ai = window._dup;
			var running = ai.choicesDisabled();
			ai.marker.closePopup();
			window._aiProgress = false; window._aiModel = 'other'; window._aiCustomModel = '';
			ai.marker.openPopup();
			var btn = document.getElementById('td-btn-' + ai.id);
			analyzeLiftoff(ai.id); // what clicking it would do were it enabled
			copyLiftoffPrompt(ai.id);
			[ai.openedDisabled, running, btn !== ai.firstButton, btn.disabled, document.getElementById('td-spin-' + ai.id).style.display,
			 ai.calls, ai.choicesDisabled(), ai.choicesShown()].join('|')
		)JS")).toString();
		QCOMPARE(reopened, QStringLiteral("false,false,false|true,true,true|true|true|inline-block|1|true,true,true|true,gemma-4-31b-it,none,,"));
		QTRY_VERIFY(evalPageJs(widget_, QStringLiteral("window._dup.copied !== undefined")).toBool());
		QVERIFY(evalPageJs(widget_, QStringLiteral("window._dup.copied")).toString().contains(QStringLiteral("Liftoff and climb-out:")));

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
			             document.getElementById('td-result-' + ai.id).textContent, ai.choicesDisabled(), ai.choicesShown()].join('|');
			ai.marker.closePopup();
			shown
		)JS")).toString();
		QCOMPARE(done, QStringLiteral("1|1|true|Grade: A|false,false,false|false,other,,,Enter a model name to analyze with it."));
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
		evalPageJs(widget_, kContactMarkerJs + QStringLiteral(R"JS(
			window._geminiApiKey = '';
			var id = Object.keys(_liftoffStore)[0];
			var marker = contactMarker(id);
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
		QTRY_VERIFY(evalPageJs(widget_, QStringLiteral("window._closed.release !== null")).toBool());  // after the trajectory
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

	// Clicking Analyze has MapBridge build that contact's trajectory from the
	// trip shown, for the prompt. A trip whose samples can't give one (no
	// times) gets the prompt's "not available" line instead, and the notice
	// under the button, which stays when the popup is reopened; before the
	// analysis there is none. A contact evaluation of it after that sends no
	// trajectory, and the notice goes.
	void analyzeGetsTheTrajectoryFromTheTrip_data() {
		QTest::addColumn<bool>("timed");
		QTest::addColumn<QString>("trajectoryLine");  // how the prompt's trajectory line starts
		QTest::addColumn<bool>("notice");
		QTest::newRow("built") << true << QStringLiteral("Liftoff trajectory (seconds from the liftoff point;") << false;
		QTest::newRow("unavailable") << false << QStringLiteral("Liftoff trajectory: not available (") << true;
	}
	void analyzeGetsTheTrajectoryFromTheTrip() {
		QFETCH(bool, timed);
		QFETCH(QString, trajectoryLine);
		QFETCH(bool, notice);
		evalPageJs(widget_, QStringLiteral("window._geminiApiKey = 'test-key'")); // the popups are built with it
		TripDataset dataset = tripToSave();
		dataset.liftoffPoints[0].rowId = 5;
		if (timed)
			dataset = tripWithClimbOut();
		loadTrip(dataset);
		evalPageJs(widget_, kContactMarkerJs + QStringLiteral(R"JS(
			window._geminiApiKey = '';
			var id = Object.keys(_liftoffStore)[0];
			var marker = contactMarker(id);
			var realFetch = window.fetch;
			var ai = window._traj = { id: id, marker: marker, prompt: null };
			window.fetch = function (url, init) {
			    window.fetch = realFetch;
			    ai.prompt = JSON.parse(init.body).contents[0].parts[0].text;
			    return Promise.resolve({ ok: false, status: 400,
			        text: function () { return Promise.resolve('[{"error":{"code":400,"message":"Stop here."}}]'); } });
			};
			marker.openPopup();
			ai.noticeBefore = document.getElementById('td-note-' + id).style.display !== 'none';
			document.getElementById('td-btn-' + id).click();
		)JS"));
		QCOMPARE(evalPageJs(widget_, QStringLiteral("window._traj.noticeBefore")), QVariant(false));  // not analyzed yet
		QTRY_VERIFY(evalPageJs(widget_, QStringLiteral("window._traj.prompt !== null && !_liftoffStore[window._traj.id].analyzing")).toBool());
		const QVariantList shown = evalPageJs(widget_, QStringLiteral(R"JS(
			var ai = window._traj;
			function noticeShown() { return document.getElementById('td-note-' + ai.id).style.display !== 'none'; }
			var first = noticeShown();
			ai.marker.closePopup();
			ai.marker.openPopup();
			var shown = [ai.prompt.split('\n').filter(function (l) { return /^(Liftoff|Final) trajectory/.test(l); }), first, noticeShown(),
			    document.getElementById('td-note-' + ai.id).textContent.replace(/\s+/g, ' ').trim()];
			ai.marker.closePopup();
			shown
		)JS")).toList();
		const QStringList lines = shown.value(0).toStringList();
		QCOMPARE(lines.size(), 1);
		QVERIFY2(lines[0].startsWith(trajectoryLine), qPrintable(lines[0]));
		QCOMPARE(shown.value(1).toBool(), notice);
		QCOMPARE(shown.value(2).toBool(), notice);  // after reopening
		QCOMPARE(shown.value(3).toString(), QStringLiteral("The climb-out trajectory couldn't be built from this trip's"
			" recorded samples, so this analysis covers the liftoff itself only."));

		// A contact evaluation of the same liftoff sends no trajectory, and
		// the earlier analysis's notice goes.
		evalPageJs(widget_, QStringLiteral(R"JS(
			var ai = window._traj;
			var realFetch = window.fetch;
			ai.prompt = null;
			window.fetch = function (url, init) {
			    window.fetch = realFetch;
			    ai.prompt = JSON.parse(init.body).contents[0].parts[0].text;
			    return Promise.resolve({ ok: false, status: 400,
			        text: function () { return Promise.resolve('[{"error":{"code":400,"message":"Stop here."}}]'); } });
			};
			window._geminiApiKey = 'test-key';
			ai.marker.openPopup();
			window._geminiApiKey = '';
			document.getElementById('td-progress-' + ai.id).click();
			document.getElementById('td-btn-' + ai.id).click();
		)JS"));
		QTRY_VERIFY(evalPageJs(widget_, QStringLiteral("window._traj.prompt !== null && !_liftoffStore[window._traj.id].analyzing")).toBool());
		const QVariantList contact = evalPageJs(widget_, QStringLiteral(R"JS(
			var ai = window._traj;
			var shown = [ai.prompt.split('\n').filter(function (l) { return /^(Liftoff|Final) trajectory/.test(l); }).length,
			    document.getElementById('td-note-' + ai.id).style.display !== 'none'];
			ai.marker.closePopup();
			shown
		)JS")).toList();
		QCOMPARE(contact.value(0).toInt(), 0);
		QCOMPARE(contact.value(1).toBool(), false);

		// Back on the overview the trip is gone, and so is its trajectory.
		widget_->showOverview({});
		evalPageJs(widget_, QStringLiteral("window._traj.afterOverview = undefined;"
			" mapBridge.liftoffProfile(5, function (p) { window._traj.afterOverview = p; })"));
		QTRY_VERIFY(evalPageJs(widget_, QStringLiteral("window._traj.afterOverview !== undefined")).toBool());
		QVERIFY(evalPageJs(widget_, QStringLiteral("window._traj.afterOverview === null")).toBool());
	}

	// Copy Prompt, there even without an API key, puts on the clipboard the
	// prompt Analyze sends for the same contact: for a progress evaluation its
	// trajectory included and looked up in its own table (the liftoff's built,
	// the touchdown's not available), for a contact evaluation (the popup's
	// checkbox unchecked) none; and says so on the button; the no-trajectory notice is left
	// to Analyze, even once the popup is reopened. The user's clipboard is put
	// back afterwards.
	void copyPromptCopiesWhatAnalyzeSends_data() {
		QTest::addColumn<QString>("kind");
		QTest::addColumn<bool>("progress");
		QTest::addColumn<QString>("trajectoryLine");  // how the prompt's trajectory line starts, empty for none
		QTest::newRow("liftoff") << QStringLiteral("liftoff") << true << QStringLiteral("Liftoff trajectory (seconds from the liftoff point;");
		QTest::newRow("touchdown") << QStringLiteral("touchdown") << true << QStringLiteral("Final trajectory: not available (");
		QTest::newRow("liftoff contact") << QStringLiteral("liftoff") << false << QString();
		QTest::newRow("touchdown contact") << QStringLiteral("touchdown") << false << QString();
	}
	void copyPromptCopiesWhatAnalyzeSends() {
		QFETCH(QString, kind);
		QFETCH(bool, progress);
		QFETCH(QString, trajectoryLine);
		evalPageJs(widget_, QStringLiteral("window._geminiApiKey = ''")); // the popups are built without one
		loadTrip(tripWithClimbOut());
		const ClipboardGuard keepUsersClipboard;
		QClipboard* clipboard = QGuiApplication::clipboard();
		clipboard->clear();
		const QVariantList before = evalPageJs(widget_, kContactMarkerJs + QStringLiteral(R"JS(
			(function (kind, progress) {
			    var store = kind === 'liftoff' ? _liftoffStore : _touchdownStore;
			    var id = Object.keys(store)[0];
			    var marker = contactMarker(id);
			    window._copy = { kind: kind, id: id, marker: marker, prompt: null, askedTrajectory: false };
			    var load = kind === 'liftoff' ? 'liftoffProfile' : 'touchdownProfile';
			    var realLoad = mapBridge[load];
			    mapBridge[load] = function (rowId, onLoaded) {
			        mapBridge[load] = realLoad;
			        window._copy.askedTrajectory = true;
			        realLoad.call(mapBridge, rowId, onLoaded);
			    };
			    window._copy.restoreLoad = function () { mapBridge[load] = realLoad; };
			    marker.openPopup();
			    var box = document.getElementById('td-progress-' + id);
			    if (box.checked !== progress) box.click();
			    var copy = document.getElementById('td-copy-' + id);
			    var shown = [document.getElementById('td-btn-' + id).disabled, copy.textContent,
			                 document.getElementById('td-model-' + id).disabled];
			    copy.click();
			    return shown;
			})('%1', %2))JS").arg(kind, progress ? QStringLiteral("true") : QStringLiteral("false"))).toList();
		QCOMPARE(before.value(0).toBool(), true);  // Analyze needs the key
		QCOMPARE(before.value(1).toString(), QStringLiteral("Copy Prompt"));
		QCOMPARE(before.value(2).toBool(), true);  // and so does its model choice
		QTRY_VERIFY_WITH_TIMEOUT(!clipboard->text().isEmpty(), 5000);
		const QString copied = clipboard->text();
		const QStringList trajectoryLines = copied.split('\n').filter(QRegularExpression(QStringLiteral("^(Liftoff|Final) trajectory")));
		QVERIFY2(trajectoryLine.isEmpty() ? trajectoryLines.isEmpty() : trajectoryLines.value(0).startsWith(trajectoryLine),
			qPrintable(copied));
		const QVariantList after = evalPageJs(widget_, QStringLiteral(R"JS(
			var c = window._copy;
			c.restoreLoad();
			[document.getElementById('td-copy-' + c.id).textContent,
			 document.getElementById('td-note-' + c.id).style.display !== 'none', c.askedTrajectory]
		)JS")).toList();
		QCOMPARE(after.value(0).toString(), QStringLiteral("✓ Copied"));
		QCOMPARE(after.value(1).toBool(), false);
		QCOMPARE(after.value(2).toBool(), progress);  // a contact evaluation loads no trajectory
		QTRY_COMPARE_WITH_TIMEOUT(evalPageJs(widget_, QStringLiteral(
			"document.getElementById('td-copy-' + window._copy.id).textContent")).toString(), QStringLiteral("Copy Prompt"), 5000);
		QCOMPARE(evalPageJs(widget_, QStringLiteral(R"JS(
			var c = window._copy;
			c.marker.closePopup();
			c.marker.openPopup();
			document.getElementById('td-note-' + c.id).style.display !== 'none'
		)JS")), QVariant(false));  // still none after reopening the popup

		// What Analyze sends.
		evalPageJs(widget_, QStringLiteral(R"JS(
			var c = window._copy;
			window._geminiApiKey = 'test-key';
			var realFetch = window.fetch;
			window.fetch = function (url, init) {
			    window.fetch = realFetch;
			    c.prompt = JSON.parse(init.body).contents[0].parts[0].text;
			    return Promise.resolve({ ok: false, status: 400,
			        text: function () { return Promise.resolve('[{"error":{"code":400,"message":"Stop here."}}]'); } });
			};
			(c.kind === 'liftoff' ? analyzeLiftoff : analyzeTouchdown)(c.id);
		)JS"));
		QTRY_VERIFY(evalPageJs(widget_, QStringLiteral(
			"var c = window._copy; c.prompt !== null && !(c.kind === 'liftoff' ? _liftoffStore : _touchdownStore)[c.id].analyzing")).toBool());
		const QString sent = evalPageJs(widget_, QStringLiteral(
			"window._geminiApiKey = ''; window._copy.marker.closePopup(); window._copy.prompt")).toString();
		QCOMPARE(copied, sent);
	}

	// Before the page's bridge is connected, Copy Prompt does nothing, without
	// an error; with its popup closed while the trajectory loads, it still
	// copies the prompt. The trajectory is held back until the closed popup
	// has left the page (Leaflet fades it out first).
	void copyPromptWithoutTheBridgeOrItsPopup() {
		evalPageJs(widget_, QStringLiteral("window._geminiApiKey = ''")); // the popups are built without one
		loadTrip(tripWithClimbOut());
		const ClipboardGuard keepUsersClipboard;
		QClipboard* clipboard = QGuiApplication::clipboard();
		clipboard->setText(QStringLiteral("before"));
		evalPageJs(widget_, QStringLiteral(R"JS(
			var g = window._guard = { id: Object.keys(_liftoffStore)[0] };
			var bridge = mapBridge;
			mapBridge = null;
			copyLiftoffPrompt(g.id).then(function () { mapBridge = bridge; g.noBridge = 'done'; },
			    function (e) { mapBridge = bridge; g.noBridge = String(e); });
		)JS"));
		QTRY_VERIFY(evalPageJs(widget_, QStringLiteral("window._guard.noBridge !== undefined")).toBool());
		QCOMPARE(evalPageJs(widget_, QStringLiteral("window._guard.noBridge")).toString(), QStringLiteral("done"));
		QCOMPARE(clipboard->text(), QStringLiteral("before"));

		evalPageJs(widget_, kContactMarkerJs + QStringLiteral(R"JS(
			var g = window._guard;
			var marker = contactMarker(g.id);
			var realLoad = loadLiftoffProfile;
			loadLiftoffProfile = function (rowId, onLoaded) {
			    realLoad(rowId, function (profile) {
			        var wait = setInterval(function () {
			            if (document.getElementById('td-copy-' + g.id)) return;
			            clearInterval(wait);
			            onLoaded(profile);
			        }, 20);
			    });
			};
			marker.openPopup();
			copyLiftoffPrompt(g.id).then(function () { loadLiftoffProfile = realLoad; g.closed = 'done'; },
			    function (e) { loadLiftoffProfile = realLoad; g.closed = String(e); });
			marker.closePopup();
		)JS"));
		QTRY_VERIFY(evalPageJs(widget_, QStringLiteral("window._guard.closed !== undefined")).toBool());
		QCOMPARE(evalPageJs(widget_, QStringLiteral("window._guard.closed")).toString(), QStringLiteral("done"));
		QTRY_VERIFY_WITH_TIMEOUT(clipboard->text().contains(QStringLiteral("Liftoff trajectory (seconds from the liftoff point;")), 5000);
	}

	// A server error is often gone a moment later: the request is retried,
	// and the answer that follows is saved.
	void aServerErrorIsRetried_data() {
		QTest::addColumn<int>("status");
		QTest::newRow("internal error") << 500;
		QTest::newRow("unavailable") << 503;
	}
	void aServerErrorIsRetried() {
		QFETCH(int, status);
		const QVariantMap r = runAiAnalysisWith({
			{ QStringLiteral(R"([{"error":{"code":%1,"message":"Try later."}}])").arg(status) },
			{ aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } }, true, { status, 200 });
		QCOMPARE(r.value("calls").toInt(), 2);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Grade: A"));
		QCOMPARE(r.value("text").toString(), QStringLiteral("Grade: A"));
		// The status line counts the retry and says why the first failed.
		QCOMPARE(r.value("statuses").toString(), QStringLiteral(
			"Building the climb-out trajectory…|Sending to gemma-4-31b-it…|Retry 2/3 (got error %1): Sending to gemma-4-31b-it…|"
			"Retry 2/3 (got error %1): Waiting for the model…").arg(status));
	}

	// A retry's status line says why the attempt before it failed, whatever
	// the failure, as each case's first retry does.
	void aRetrySaysWhyThePreviousAttemptFailed_data() {
		QTest::addColumn<int>("firstStatus");
		QTest::addColumn<QString>("firstAnswer");
		QTest::addColumn<QString>("reason");
		const QString half = aiStream({ aiChunk({ { "Grade: half", false } }, "MAX_TOKENS") });
		QTest::newRow("network error") << 0 << half << QStringLiteral("network error");
		QTest::newRow("stalled") << -2 << half << QStringLiteral("stalled");
		QTest::newRow("answer incomplete") << 200 << half << QStringLiteral("answer incomplete");
	}
	void aRetrySaysWhyThePreviousAttemptFailed() {
		QFETCH(int, firstStatus);
		QFETCH(QString, firstAnswer);
		QFETCH(QString, reason);
		const QVariantMap r = runAiAnalysisWith({ { firstAnswer }, { aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } },
			true, { firstStatus, 200 });
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Grade: A"));
		const QStringList statuses = r.value("statuses").toString().split(QLatin1Char('|'));
		QVERIFY2(statuses.size() >= 2, qPrintable(r.value("statuses").toString()));
		QCOMPARE(statuses.mid(statuses.size() - 2), (QStringList{
			QStringLiteral("Retry 2/3 (%1): Sending to gemma-4-31b-it…").arg(reason),
			QStringLiteral("Retry 2/3 (%1): Waiting for the model…").arg(reason) }));
	}

	// A rate limit is retried only after the wait the service asks for,
	// counted down in the status line; a server error after it is retried at
	// once, without that wait again, and the answer that follows is saved.
	void aRateLimitIsRetriedAfterTheWaitItAsksFor() {
		watchRateLimitWaits();
		const double startedAt = evalPageJs(widget_, QStringLiteral("Date.now()")).toDouble();
		const QVariantMap r = runAiAnalysisWith({
			{ QStringLiteral(R"([{"error":{"code":429,"message":"Quota exceeded.","details":[)"
				R"({"@type":"type.googleapis.com/google.rpc.RetryInfo","retryDelay":"2s"}]}}])") },
			{ QStringLiteral(R"([{"error":{"code":500,"message":"Internal error encountered."}}])") },
			{ aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } }, true, { 429, 500, 200 });
		QCOMPARE(rateLimitWaits(), QStringLiteral(
			"Retry 2/4 (got error 429): Waiting 2 s for the rate limit…|Retry 2/4 (got error 429): Waiting 1 s for the rate limit…"));
		QVERIFY2(r.value("fetchedAt").toDouble() >= startedAt + 2000,
			qPrintable(QString::number(r.value("fetchedAt").toDouble() - startedAt)));
		QCOMPARE(r.value("calls").toInt(), 3);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Grade: A"));
		QCOMPARE(r.value("statuses").toString(), QStringLiteral(
			"Building the climb-out trajectory…|Sending to gemma-4-31b-it…|Retry 2/4 (got error 429): Sending to gemma-4-31b-it…|"
			"Retry 3/4 (got error 500): Sending to gemma-4-31b-it…|Retry 3/4 (got error 500): Waiting for the model…"));
	}

	// A rate limit on the last of the 3 attempts (after two server errors)
	// is still waited out and retried: the wait doesn't use up an attempt.
	void aRateLimitOnTheLastAttemptIsStillRetried() {
		watchRateLimitWaits();
		const QVariantMap r = runAiAnalysisWith({
			{ QStringLiteral(R"([{"error":{"code":500,"message":"Internal error encountered."}}])") },
			{ QStringLiteral(R"([{"error":{"code":500,"message":"Internal error encountered."}}])") },
			{ QStringLiteral(R"([{"error":{"code":429,"message":"Quota exceeded.","details":[)"
				R"({"@type":"type.googleapis.com/google.rpc.RetryInfo","retryDelay":"1s"}]}}])") },
			{ aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } }, true, { 500, 500, 429, 200 });
		QCOMPARE(rateLimitWaits(), QStringLiteral("Retry 4/4 (got error 429): Waiting 1 s for the rate limit…"));
		QCOMPARE(r.value("calls").toInt(), 4);
		QCOMPARE(r.value("saved").toString(), QStringLiteral("Grade: A"));
		QCOMPARE(r.value("statuses").toString(), QStringLiteral(
			"Building the climb-out trajectory…|Sending to gemma-4-31b-it…|Retry 2/3 (got error 500): Sending to gemma-4-31b-it…|"
			"Retry 3/3 (got error 500): Sending to gemma-4-31b-it…|Retry 4/4 (got error 429): Sending to gemma-4-31b-it…|"
			"Retry 4/4 (got error 429): Waiting for the model…"));
	}

	// A rate limit that never clears is waited out 3 times: the 4th is
	// final, shown saying it's still limited, and nothing is saved. Its wait is then counted down with the run still
	// running (spinner shown, so Analyze stays disabled), after which Analyze
	// is offered again but not sent by itself.
	void aRateLimitThatNeverClearsEndsTheRun() {
		evalPageJs(widget_, QStringLiteral(R"JS(
			window._cooldownSeen = [];
			window._cooldownWatch = setInterval(function () {
			    var c = document.querySelector('#td-result-ai .td-cooldown');
			    var s = c ? c.textContent + ',' + document.getElementById('td-spin-ai').style.display : '';
			    if (c && c.textContent && _cooldownSeen.indexOf(s) < 0) _cooldownSeen.push(s);
			}, 20);
		)JS"));
		const QString body = QStringLiteral(R"([{"error":{"code":429,"message":"Quota exceeded.","details":[)"
			R"({"@type":"type.googleapis.com/google.rpc.RetryInfo","retryDelay":"1s"}]}}])");
		const QVariantMap r = runAiAnalysisWith({ { body } }, true, { 429 });
		const QString cooldown = evalPageJs(widget_, QStringLiteral(
			"clearInterval(window._cooldownWatch); window._cooldownSeen.join('|')")).toString();
		QCOMPARE(cooldown, QStringLiteral("Analyze again in 1 s.,inline-block|Click Analyze to try again.,none"));
		QCOMPARE(r.value("calls").toInt(), 4);
		QVERIFY(r.value("saved").isNull());
		QCOMPARE(r.value("text").toString(), QStringLiteral(
			"API error 429: Quota exceeded.Still rate-limited after 3 waits. Click Analyze to try again."));
		const QStringList logged{ QStringLiteral("429 ") + body };
		verifyAttemptLogs(r, { { true, logged }, { true, logged }, { true, logged }, { false, logged } });
	}

	// A rate limit's wait adds an attempt for any failure after it: a
	// network error, a server error or an incomplete answer each time after
	// one is retried until the 4th attempt, whose failure is final, its
	// message saying so.
	void aRateLimitAddsAnAttemptForAnyFailure_data() {
		QTest::addColumn<int>("status");
		QTest::addColumn<QString>("answer");
		QTest::addColumn<QString>("text");
		QTest::addColumn<QString>("logged");
		QTest::newRow("network error") << 0 << QString() << QStringLiteral(
			"Couldn't reach the AI service after 4 attempts. Check your internet connection and try again.")
			<< QStringLiteral("Failed to fetch");
		QTest::newRow("server error") << 500
			<< QStringLiteral(R"([{"error":{"code":500,"message":"Internal error encountered."}}])")
			<< QStringLiteral("API error 500: Internal error encountered.")
			<< QStringLiteral(R"(500 [{"error":{"code":500,"message":"Internal error encountered."}}])");
		QTest::newRow("answer incomplete") << 200 << aiStream({ aiChunk({ { "Grade: half", false } }, "MAX_TOKENS") })
			<< QStringLiteral("The AI didn't return a complete analysis. This can happen occasionally. Click Analyze to try again.")
			<< QStringLiteral("finish reason MAX_TOKENS");
	}
	void aRateLimitAddsAnAttemptForAnyFailure() {
		QFETCH(int, status);
		QFETCH(QString, answer);
		QFETCH(QString, text);
		QFETCH(QString, logged);
		const QString rateLimit = QStringLiteral(R"([{"error":{"code":429,"message":"Quota exceeded.","details":[)"
			R"({"@type":"type.googleapis.com/google.rpc.RetryInfo","retryDelay":"1s"}]}}])");
		const QVariantMap r = runAiAnalysisWith({ { rateLimit }, { answer } }, true, { 429, status });
		QCOMPARE(r.value("calls").toInt(), 4);
		QVERIFY(r.value("saved").isNull());
		QCOMPARE(r.value("text").toString(), text);
		verifyAttemptLogs(r, { { true, { QStringLiteral("429 ") + rateLimit } }, { true, { logged } }, { true, { logged } },
			{ false, { logged } } });
	}

	// The wait before retrying a rate limit: the retryDelay of the error's
	// RetryInfo detail, whichever detail carries it, in whole seconds
	// rounded up, with or without the streaming endpoint's array around the
	// error; a minute when the body has no such delay (no details, a delay
	// in another form, not JSON).
	void aRateLimitWaitsAsLongAsTheServiceAsks() {
		QCOMPARE(evalPageJs(widget_, QStringLiteral(R"JS(
			[rateLimitDelayMs('[{"error":{"code":429,"details":[{"@type":"type.googleapis.com/google.rpc.QuotaFailure"},'
			     + '{"@type":"type.googleapis.com/google.rpc.RetryInfo","retryDelay":"34s"}]}}]'),
			 rateLimitDelayMs('{"error":{"details":[{"retryDelay":"1.2s"}]}}'),
			 rateLimitDelayMs('{"error":{"code":429,"message":"Quota exceeded."}}'),
			 rateLimitDelayMs('{"error":{"details":[{"retryDelay":"soon"}]}}'),
			 rateLimitDelayMs('Too Many Requests')].join()
		)JS")).toString(), QStringLiteral("34000,2000,60000,60000,60000"));
	}

	// A rate limit asking for a longer wait than a minute (61 s; a per-day
	// quota's can be hours) isn't retried: it's shown at once with the wait
	// asked for, logged as final.
	void aRateLimitLongerThanAMinuteIsNotRetried() {
		const QString body = QStringLiteral(R"([{"error":{"code":429,"message":"Quota exceeded.","details":[)"
			R"({"@type":"type.googleapis.com/google.rpc.RetryInfo","retryDelay":"61s"}]}}])");
		const QVariantMap r = runAiAnalysisWith({ { body } }, true, { 429 });
		QCOMPARE(r.value("calls").toInt(), 1);
		QVERIFY(r.value("saved").isNull());
		QCOMPARE(r.value("text").toString(), QStringLiteral(
			"API error 429: Quota exceeded.The service asks to wait 61 s before the next request (a daily quota used up, maybe). Try again later."));
		verifyAttemptLogs(r, { { false, { QStringLiteral("429 ") + body } } });
	}

	// Three server errors in a row: the last one's message is shown and
	// nothing is saved; each is logged, the last as final.
	void threeServerErrorsShowTheLastOne() {
		const QVariantMap r = runAiAnalysisWith({ { QStringLiteral(R"([{"error":{"code":500,"message":"Internal error encountered."}}])") } },
			true, { 500 });
		QCOMPARE(r.value("calls").toInt(), 3);
		QVERIFY(r.value("saved").isNull());
		QCOMPARE(r.value("text").toString(), QStringLiteral("API error 500: Internal error encountered."));
		const QStringList body{ R"(500 [{"error":{"code":500,"message":"Internal error encountered."}}])" };
		verifyAttemptLogs(r, { { true, body }, { true, body }, { false, body } });
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
		QCOMPARE(r.value("statusShown").toBool(), false);  // no answer came, but it's over
		verifyAttemptLogs(r, { { false, { R"(400 [{"error":{"code":400,"message":"API key not valid."}}])" } } });
	}

	// A model the service doesn't have (a mistyped Other name) won't appear
	// on retry: it's shown at once, naming the model asked for.
	void anUnknownModelIsNotRetried() {
		evalPageJs(widget_, QStringLiteral("window._aiModel = 'other'; window._aiCustomModel = 'gemma-typo'"));
		const QVariantMap r = runAiAnalysisWith({ { QStringLiteral(R"([{"error":{"code":404,"message":"models/gemma-typo is not found."}}])") } },
			true, { 404 });
		evalPageJs(widget_, QStringLiteral("window._aiModel = ''; window._aiCustomModel = ''"));
		QCOMPARE(r.value("calls").toInt(), 1);
		QCOMPARE(r.value("text").toString(), QStringLiteral(
			"API error 404: models/gemma-typo is not found.Check the model name: gemma-typo wasn't found."));
	}

	// The request asks the model chosen beside the Analyze button: a listed
	// one, or the name entered for Other, trimmed and encoded for the URL; a
	// choice settings.ini can't give (unset, edited by hand) asks the default.
	void theAnalysisAsksTheChosenModel_data() {
		QTest::addColumn<QString>("model");
		QTest::addColumn<QString>("customModel");
		QTest::addColumn<QString>("asked");  // in the URL
		QTest::newRow("listed") << QStringLiteral("gemma-4-26b-a4b-it") << QStringLiteral("gemini-3-pro") << QStringLiteral("gemma-4-26b-a4b-it");
		QTest::newRow("other") << QStringLiteral("other") << QStringLiteral(" gemini-3-pro ") << QStringLiteral("gemini-3-pro");
		QTest::newRow("encoded") << QStringLiteral("other") << QStringLiteral("a b/c?") << QStringLiteral("a%20b%2Fc%3F");
		QTest::newRow("unset") << QString() << QString() << QStringLiteral("gemma-4-31b-it");
		QTest::newRow("unknown") << QStringLiteral("gemini-3-pro") << QString() << QStringLiteral("gemma-4-31b-it");
	}
	void theAnalysisAsksTheChosenModel() {
		QFETCH(QString, model);
		QFETCH(QString, customModel);
		QFETCH(QString, asked);
		evalPageJs(widget_, QStringLiteral("window._aiModel = %1[0]; window._aiCustomModel = %1[1]").arg(
			QString::fromUtf8(QJsonDocument(QJsonArray{ model, customModel }).toJson(QJsonDocument::Compact))));
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } });
		evalPageJs(widget_, QStringLiteral("window._aiModel = ''; window._aiCustomModel = ''"));
		QCOMPARE(r.value("url").toString(),
			QStringLiteral("https://generativelanguage.googleapis.com/v1beta/models/%1:streamGenerateContent").arg(asked));
	}

	// The request asks the model the analysis was started with (Other, with
	// its name), even when the choice and the Other name change (from another
	// contact's popup) while its trajectory loads.
	void theAnalysisAsksTheModelItStartedWith() {
		evalPageJs(widget_, QStringLiteral("window._aiModel = 'other'; window._aiCustomModel = 'gemini-3-pro';"
			" window._aiDuringLoad = function () { window._aiModel = 'gemma-4-26b-a4b-it'; window._aiCustomModel = 'gemma-typo'; }"));
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } });
		evalPageJs(widget_, QStringLiteral("window._aiModel = ''; window._aiCustomModel = ''; delete window._aiDuringLoad"));
		QCOMPARE(r.value("url").toString(),
			QStringLiteral("https://generativelanguage.googleapis.com/v1beta/models/gemini-3-pro:streamGenerateContent"));
	}

	// The Analyze button, the evaluation checkbox over the model drop-down,
	// and Copy Prompt fit side by side in both popups, with the spinner shown
	// too, and none of them moves or shrinks when it shows. Each button is as
	// tall as the checkbox and the drop-down together.
	void theAnalysisButtonsFitBesideTheModelDropDown() {
		evalPageJs(widget_, QStringLiteral("window._geminiApiKey = 'test-key'")); // the popups are built with it
		loadTrip(tripWithClimbOut());
		const QStringList shown = evalPageJs(widget_, kContactMarkerJs + QStringLiteral(R"JS(
			window._geminiApiKey = '';
			[_liftoffStore, _touchdownStore].map(function (store) {
			    var id = Object.keys(store)[0];
			    var marker = contactMarker(id);
			    marker.openPopup();
			    var btn = document.getElementById('td-btn-' + id);
			    var row = btn.parentElement;
			    // Each shown control's left and right edge, in order, and
			    // whether each starts after the one before ends, all inside the row.
			    function layout() {
			        var edges = [];
			        Array.prototype.forEach.call(row.children, function (c) {
			            if (c.style.display === 'none') return;
			            var b = c.getBoundingClientRect();
			            edges.push([b.left, b.right]);
			        });
			        var r = row.getBoundingClientRect();
			        var fits = edges[0][0] >= r.left && edges[edges.length - 1][1] <= r.right;
			        for (var i = 1; i < edges.length; i++) fits = fits && edges[i][0] >= edges[i - 1][1];
			        return { fits: fits, edges: edges.map(function (e) { return e.join(','); }) };
			    }
			    var idle = layout();
			    var choices = row.children[1].getBoundingClientRect();
			    var box = document.getElementById('td-progress-' + id).parentElement.getBoundingClientRect();
			    var select = document.getElementById('td-model-' + id).getBoundingClientRect();
			    var copy = document.getElementById('td-copy-' + id).getBoundingClientRect();
			    var spans = function(r) { return r.top <= box.top && r.bottom >= select.bottom; };
			    var tall = btn.getBoundingClientRect().height === choices.height && copy.height === choices.height
			        && box.bottom <= select.top && spans(btn.getBoundingClientRect()) && spans(copy);
			    showAnalysisRunning(id, true);
			    var running = layout();
			    showAnalysisRunning(id, false);
			    marker.closePopup();
			    var spinnerAt = 2; // after the button and the drop-down
			    var unmoved = running.edges.filter(function (e, i) { return i !== spinnerAt; }).join('|') === idle.edges.join('|');
			    return [idle.edges.length, idle.fits, running.edges.length, running.fits, unmoved, tall].join('|');
			})
		)JS")).toStringList();
		QCOMPARE(shown, (QStringList{ QStringLiteral("3|true|4|true|true|true"), QStringLiteral("3|true|4|true|true|true") }));
	}

	// Between the Analyze and Copy Prompt buttons, over the model drop-down, a
	// checkbox chooses a progress evaluation (checked, the default) or a
	// contact evaluation, its tooltip saying what each covers and why to
	// choose it, for each popup's own contact and trajectory. It can be changed
	// without an API key (Copy Prompt needs none); each change is saved to
	// settings.ini, and every popup opened after shows it.
	void theProgressCheckboxChoosesTheEvaluation() {
		evalPageJs(widget_, QStringLiteral("window._geminiApiKey = ''"));
		loadTrip(tripWithClimbOut());
		const QStringList shown = evalPageJs(widget_, kContactMarkerJs + QStringLiteral(R"JS(
			function state(store) {
			    var id = Object.keys(store)[0];
			    var marker = contactMarker(id);
			    marker.openPopup();
			    var box = document.getElementById('td-progress-' + id);
			    var option = box.parentElement;
			    var between = option.parentElement.previousElementSibling === document.getElementById('td-btn-' + id)
			        && option.parentElement.nextElementSibling.nextElementSibling === document.getElementById('td-copy-' + id)
			        && option.nextElementSibling === document.getElementById('td-model-' + id);
			    var result = [String(between), String(box.disabled), String(box.checked), box.nextSibling.textContent.trim()].join('|');
			    // The tooltip's text, its line breaks as new lines.
			    var tip = document.getElementById('td-progress-tip-' + id).innerHTML.replace(/<br>/g, '\n');
			    return { id: id, marker: marker, box: box, result: result, tip: tip };
			}
			var liftoff = state(_liftoffStore);
			var shown = [liftoff.result, liftoff.tip];
			liftoff.box.click();
			liftoff.marker.closePopup();
			var touchdown = state(_touchdownStore);
			shown.push(touchdown.result, touchdown.tip);
			touchdown.marker.closePopup();
			shown
		)JS")).toStringList();
		QCOMPARE(shown, (QStringList{
			QStringLiteral("true|false|true|Progress evaluation"),
			QStringLiteral("Checked, a progress evaluation: grades the liftoff together with the climb-out. The prompt adds "
				"the conditions, the runway data and the climb-out trajectory (gates, samples, weather and ride), so the model "
				"can judge how the climb-out was flown and how the conditions affected it, and the answer adds a section for "
				"each. The prompt is longer: it uses more of the per-minute token limit, and the answer takes longer.\n\n"
				"Unchecked, a contact evaluation: grades the liftoff alone, from the values in this popup. The prompt is short "
				"and the answer quick, for when only the liftoff itself matters."),
			QStringLiteral("true|false|false|Progress evaluation"),
			QStringLiteral("Checked, a progress evaluation: grades the touchdown together with the approach. The prompt adds "
				"the conditions, the runway data and the approach trajectory (gates, samples, weather and ride), so the model "
				"can judge how the approach was flown and how the conditions affected it, and the answer adds a section for "
				"each. The prompt is longer: it uses more of the per-minute token limit, and the answer takes longer.\n\n"
				"Unchecked, a contact evaluation: grades the touchdown alone, from the values in this popup. The prompt is short "
				"and the answer quick, for when only the touchdown itself matters.") }));
		AppSettings& settings = AppSettings::instance();
		QTRY_COMPARE(settings.aiProgressEvaluation(), false);
		const QString checked = evalPageJs(widget_, kContactMarkerJs + QStringLiteral(R"JS(
			var id = Object.keys(_liftoffStore)[0];
			var marker = contactMarker(id);
			marker.openPopup();
			var box = document.getElementById('td-progress-' + id);
			box.click();
			marker.closePopup();
			String(box.checked)
		)JS")).toString();
		QCOMPARE(checked, QStringLiteral("true"));
		QTRY_COMPARE(settings.aiProgressEvaluation(), true);
	}

	// The checkbox's tooltip is the page's own, shown while the real mouse is
	// over the checkbox and its label and gone once it leaves, not the
	// browser's title tooltip, which shows only after a delay.
	void theProgressTooltipShowsWhileTheMouseIsOverIt() {
		loadTrip(tripWithClimbOut());
		QCOMPARE(evalPageJs(widget_, kContactMarkerJs + QStringLiteral(R"JS(
			var id = window._tipId = Object.keys(_liftoffStore)[0];
			window._tipMarker = contactMarker(id);
			_tipMarker.openPopup();
			document.getElementById('td-progress-' + id).parentElement.hasAttribute('title')
		)JS")).toBool(), false);
		const auto tipDisplay = [this] {
			return evalPageJs(widget_, QStringLiteral("getComputedStyle(document.getElementById('td-progress-tip-' + _tipId)).display")).toString();
		};
		QCOMPARE(tipDisplay(), QStringLiteral("none"));
		// The popup is taller than the test's view, and the map can still move
		// after it opens (the popup's own panning), so each poll brings the
		// checkbox's label to the view's middle and moves the mouse there, and
		// the tooltip is read only once a poll finds both where they were, as
		// the page applies the hover after the move, not during the poll.
		QPoint mouse;
		const auto hoverOption = [this, &tipDisplay, &mouse] {
			const QVariantList xy = evalPageJs(widget_, QStringLiteral(R"JS(
				var option = document.getElementById('td-progress-' + _tipId).parentElement;
				var r = option.getBoundingClientRect();
				var dx = r.left + r.width / 2 - innerWidth / 2, dy = r.top + r.height / 2 - innerHeight / 2;
				if (Math.abs(dx) >= 1 || Math.abs(dy) >= 1) leafletMapInstance.panBy([dx, dy], { animate: false });
				r = option.getBoundingClientRect();
				var x = r.left + r.width / 2, y = r.top + r.height / 2;
				option.contains(document.elementFromPoint(x, y)) ? [x, y] : []
			)JS")).toList();
			if (xy.size() != 2)
				return QString();
			const QPoint at(qRound(xy.value(0).toDouble()), qRound(xy.value(1).toDouble()));
			if (at != mouse) {
				mouse = at;
				moveMouse(at);
				return QString();
			}
			const QString display = tipDisplay();
			if (display != QStringLiteral("block"))
				mouse = QPoint();  // moved again on the next poll
			return display;
		};
		QTRY_COMPARE_WITH_TIMEOUT(hoverOption(), QStringLiteral("block"), 15000);
		moveMouse(QPoint(5, 5));
		QTRY_COMPARE(tipDisplay(), QStringLiteral("none"));
		evalPageJs(widget_, QStringLiteral("_tipMarker.closePopup(); true"));
	}

	// The drop-down beside the Analyze button, under the evaluation
	// checkbox, lists the two models, then Other, which shows a field for the
	// model's name, with a hint to paste it from Google AI Studio; Analyze
	// waits for a name that isn't one of the listed two. Each change is saved to settings.ini, the
	// name as it's typed (its field's change event, which a popup closed with
	// Escape never fires, isn't needed), and a popup opened after shows it
	// and analyzes with it.
	void theModelDropDownChoosesAndSavesTheModel() {
		evalPageJs(widget_, QStringLiteral("window._geminiApiKey = 'test-key'")); // the popups are built with it
		loadTrip(tripWithClimbOut());
		const QStringList shown = evalPageJs(widget_, kContactMarkerJs + QStringLiteral(R"JS(
			window._geminiApiKey = '';
			window._aiModel = '';
			window._aiCustomModel = '';
			var id = Object.keys(_liftoffStore)[0];
			var marker = contactMarker(id);
			marker.openPopup();
			var btn = document.getElementById('td-btn-' + id);
			var select = document.getElementById('td-model-' + id);
			var field = document.getElementById('td-custom-name-' + id);
			function state() {
			    return [select.value, document.getElementById('td-custom-' + id).style.display !== 'none',
			            document.getElementById('td-model-problem-' + id).textContent, btn.disabled].join('|');
			}
			function type(text) { field.value = text; field.dispatchEvent(new Event('input')); }
			var shown = [String(btn.nextElementSibling === select.parentElement),
			    Array.prototype.map.call(select.options, function (o) { return o.value + '=' + o.textContent; }).join(','),
			    state()];
			select.value = 'other';
			select.dispatchEvent(new Event('change'));
			shown.push(state(), document.querySelector('#td-custom-' + id + ' .td-hint').textContent);
			type('gemma-4-26b-a4b-it');
			shown.push(state());
			type(' gemini-3-pro ');
			shown.push(state());
			marker.closePopup();
			shown
		)JS")).toStringList();
		QCOMPARE(shown, (QStringList{ QStringLiteral("true"),
			QString::fromUtf8("gemma-4-31b-it=gemma-4-31b-it,gemma-4-26b-a4b-it=gemma-4-26b-a4b-it,other=Other…"),
			QStringLiteral("gemma-4-31b-it|false||false"),
			QStringLiteral("other|true|Enter a model name to analyze with it.|true"),
			QStringLiteral("To avoid typos, copy the name with the Copy to clipboard button in"
				" Google AI Studio's model selection dialog and paste it here."),
			QStringLiteral("other|true|gemma-4-26b-a4b-it is in the list: choose it there.|true"),
			QStringLiteral("other|true||false") }));
		AppSettings& settings = AppSettings::instance();
		QTRY_COMPARE(settings.aiCustomModel(), QStringLiteral("gemini-3-pro"));
		QCOMPARE(settings.aiModel(), QStringLiteral("other"));

		// Another popup shows the choice, and analyzes with it.
		evalPageJs(widget_, kContactMarkerJs + QStringLiteral(R"JS(
			var id = Object.keys(_touchdownStore)[0];
			var m = window._model = { id: id, marker: contactMarker(id), url: null };
			m.marker.openPopup();
			m.shown = [document.getElementById('td-model-' + id).value,
			    document.getElementById('td-custom-' + id).style.display !== 'none',
			    document.getElementById('td-custom-name-' + id).value].join('|');
			window._geminiApiKey = 'test-key';
			var realFetch = window.fetch;
			window.fetch = function (url) {
			    window.fetch = realFetch;
			    m.url = url;
			    return Promise.resolve({ ok: false, status: 400,
			        text: function () { return Promise.resolve('[{"error":{"code":400,"message":"Stop here."}}]'); } });
			};
			document.getElementById('td-btn-' + id).click();
		)JS"));
		QTRY_VERIFY(evalPageJs(widget_, QStringLiteral(
			"window._model.url !== null && !_touchdownStore[window._model.id].analyzing")).toBool());
		const QStringList reopened = evalPageJs(widget_, QStringLiteral(R"JS(
			var m = window._model;
			window._geminiApiKey = '';
			var select = document.getElementById('td-model-' + m.id);
			select.value = 'gemma-4-26b-a4b-it';
			select.dispatchEvent(new Event('change'));
			var shown = [m.shown, m.url, select.value, document.getElementById('td-custom-' + m.id).style.display !== 'none',
			             document.getElementById('td-btn-' + m.id).disabled];
			m.marker.closePopup();
			window._aiModel = '';
			window._aiCustomModel = '';
			shown.map(String)
		)JS")).toStringList();
		QCOMPARE(reopened, (QStringList{ QStringLiteral("other|true|gemini-3-pro"),
			QStringLiteral("https://generativelanguage.googleapis.com/v1beta/models/gemini-3-pro:streamGenerateContent"),
			QStringLiteral("gemma-4-26b-a4b-it"), QStringLiteral("false"), QStringLiteral("false") }));
		// The Other name is kept for when Other is chosen again.
		QTRY_COMPARE(settings.aiModel(), QStringLiteral("gemma-4-26b-a4b-it"));
		QCOMPARE(settings.aiCustomModel(), QStringLiteral("gemini-3-pro"));
	}

	// A service that can't be reached (offline) is tried three times, then
	// the message says so and nothing is saved; each try is logged with its
	// error, the last as final.
	void anUnreachableServiceIsTriedThreeTimesThenSaysSo() {
		const QVariantMap r = runAiAnalysisWith({ { QString() } }, true, { 0 });
		QCOMPARE(r.value("calls").toInt(), 3);
		QVERIFY(r.value("saved").isNull());
		QCOMPARE(r.value("text").toString(), QStringLiteral(
			"Couldn't reach the AI service after 3 attempts. Check your internet connection and try again."));
		QCOMPARE(r.value("thinkingShown").toBool(), false);
		const QStringList values{ "Failed to fetch", "0 characters of thinking", "0 of answer" };
		verifyAttemptLogs(r, { { true, values }, { true, values }, { false, values } });
	}

	// A connection that drops mid-answer, after thinking arrived, is retried
	// the same way, logged with how much had arrived; after the third, the
	// message replaces the thinking panel, whose "Thinking…" would otherwise
	// never finish.
	void aConnectionDroppedAfterThinkingHidesTheThinkingPanel() {
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Thought", true } }) }) } }, true, { -1 });
		QCOMPARE(r.value("calls").toInt(), 3);
		QVERIFY(r.value("saved").isNull());
		QCOMPARE(r.value("text").toString(), QStringLiteral(
			"Couldn't reach the AI service after 3 attempts. Check your internet connection and try again."));
		QCOMPARE(r.value("thinkingShown").toBool(), false);
		const QStringList values{ "network error", "7 characters of thinking", "0 of answer" };
		verifyAttemptLogs(r, { { true, values }, { true, values }, { false, values } });
	}

	// A connection that drops after the service sent an error mid-stream:
	// each attempt's log line carries both the connection error and the
	// service's error.
	void aConnectionDroppedAfterAStreamErrorLogsBoth() {
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Thought", true } }),
			QStringLiteral(R"({"error":)") + streamError500 + '}' }) } }, true, { -1 });
		QCOMPARE(r.value("calls").toInt(), 3);
		const QStringList values{ "network error", "7 characters of thinking", "0 of answer",
			QStringLiteral("stream error ") + streamError500 };
		verifyAttemptLogs(r, { { true, values }, { true, values }, { false, values } });
	}

	// A touchdown analyzed before shows its saved report when its popup
	// opens, with or without an API key: the answer, and the thinking behind
	// a closed "▸ Show thinking" panel only if the report has some.
	void aSavedReportIsShownWhenThePopupOpens_data() {
		QTest::addColumn<QString>("report");
		QTest::addColumn<QString>("shown");
		QTest::newRow("with thinking") << QStringLiteral("<thinking>Wind from the left.</thinking>\nGrade: B")
			<< QStringLiteral("Grade: B|Wind from the left.|closed|▸|Show thinking");
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
			                 det ? (det.open ? 'open' : 'closed') : 'none'];
			    if (det)
			        shown.push(document.getElementById('td-th-arr-' + id).textContent,
			                   document.getElementById('td-th-lbl-' + id).textContent.trim());
			    shown = shown.join('|');
			    marker.closePopup();
			    return shown;
			})())JS")).toString();
		QCOMPARE(popup, shown);
	}

	// A crashed touchdown is marked with a nose-down airplane (still a touchdown
	// marker, with crash-icon added) and its popup is titled "Crash"; any
	// other touchdown keeps the landing mark and "Touchdown". Either way its
	// button is just "Analyze", which names no landing a crash didn't make.
	void aCrashedTouchdownHasACrashMarker_data() {
		QTest::addColumn<bool>("crashed");
		QTest::addColumn<QString>("shown");
		QTest::newRow("crashed") << true << QStringLiteral("touchdown-icon crash-icon|crash mark|Crash|✦ Analyze");
		QTest::newRow("landed") << false << QStringLiteral("touchdown-icon|landing mark|Touchdown|✦ Analyze");
	}
	void aCrashedTouchdownHasACrashMarker() {
		QFETCH(bool, crashed);
		QFETCH(QString, shown);
		TripDataset dataset = tripToSave();
		dataset.touchdowns[0].crashed = crashed;
		loadTrip(dataset);
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "touchdown-icon"), 1, 5000);
		QCOMPARE(evalPageJs(widget_, QStringLiteral(R"JS(
			(function () {
			    var marker = null;
			    leafletMapInstance.eachLayer(function (l) {
			        if (l instanceof L.Marker && /^touchdown-icon/.test(l.options.icon.options.className)) marker = l;
			    });
			    var html = marker.options.icon.options.html;
			    marker.openPopup();
			    var popup = marker.getPopup().getElement();
			    var title = /^(Crash|Touchdown)Airport/.exec(popup.querySelector('.leaflet-popup-content').textContent);
			    var button = popup.querySelector('.td-analyze-btn').textContent.trim();
			    marker.closePopup();
			    return [marker.options.icon.options.className,
			            html.indexOf('\u2708') >= 0 && html.indexOf('\u{1F6EC}') < 0 ? 'crash mark' : html.indexOf('\u{1F6EC}') >= 0 ? 'landing mark' : 'no mark',
			            title ? title[1] : 'no title', button].join('|');
			})())JS")).toString(), shown);
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
