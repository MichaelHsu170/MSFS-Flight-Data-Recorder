// Map widget (map_widget.cpp): the QWebEngineView wrapper around the
// Leaflet/OSM trajectory map -- setDataset()/showOverview()/
// resetZoom()/setEventsVisible() driving the real page, and the page's cursor
// and range forwarded only for the current trajectory; not the pure JS
// string-building math (already covered standalone in tst_map_script.cpp).
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
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtTest>

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
	QString logPath_;

	// Runs the page's AI analysis of a liftoff with fetch() stubbed: attempt n
	// gets attempts[n] (the last one repeated), each delivered in the given
	// pieces; saving the report succeeds if saveOk. Returns what it did: calls
	// (fetches made), saved (the report saved, null if none), text (the answer
	// shown), thinkingShown.
	QVariantMap runAiAnalysisWith(const QList<QStringList>& attempts, bool saveOk = true) {
		QJsonArray json;
		for (const QStringList& pieces : attempts)
			json.append(QJsonArray::fromStringList(pieces));
		evalPageJs(widget_, QStringLiteral(R"JS(
			(function (attempts) {
			    window._ai = { calls: 0, saved: null, done: false };
			    var realFetch = window.fetch;
			    window.fetch = function () {
			        var pieces = attempts[Math.min(window._ai.calls++, attempts.length - 1)].slice();
			        var reader = { read: function () {
			            return Promise.resolve(pieces.length ? { done: false, value: new TextEncoder().encode(pieces.shift()) } : { done: true });
			        } };
			        return Promise.resolve({ ok: true, body: { getReader: function () { return reader; } } });
			    };
			    var box = document.getElementById('ai-test');
			    if (!box) { box = document.createElement('div'); box.id = 'ai-test'; document.body.appendChild(box); }
			    box.innerHTML = '<button id="td-btn-ai"></button><span id="td-spin-ai"></span><div id="td-result-ai"></div>';
			    runAiAnalysis('ai', { rowId: 5 }, function () { return 'prompt'; },
			        function (rowId, report, onSaved) { window._ai.saved = report; onSaved(%2); }, 'Analyze Liftoff')
			        .then(function () { window.fetch = realFetch; window._ai.done = true; });
			})(%1))JS").arg(QString::fromUtf8(QJsonDocument(json).toJson(QJsonDocument::Compact)),
				saveOk ? QStringLiteral("true") : QStringLiteral("false")));
		if (!QTest::qWaitFor([this] { return evalPageJs(widget_, QStringLiteral("window._ai.done")).toBool(); }, 5000))
			return {};
		return evalPageJs(widget_, QStringLiteral(
			"({ calls: window._ai.calls, saved: window._ai.saved,"
			"   text: document.getElementById('td-th-final-ai').textContent,"
			"   thinkingShown: document.getElementById('td-th-det-ai').style.display !== 'none' })")).toMap();
	}

private slots:
	// Logger::init() takes effect once per process, so it runs here.
	void initTestCase() {
		logPath_ = logDir_.filePath(QStringLiteral("map.log"));
		Logger::init(Logger::Info, logPath_);
		widget_ = new MapWidget;
		widget_->resize(400, 300);
		widget_->show();
	}

	void cleanupTestCase() {
		delete widget_;
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
		widget_->setDataset(dataset);
		QCOMPARE(spy.count(), 1); // page not ready: setDataset's immediate-emit path

		// Let the page actually finish loading and refreshProvider() re-push the
		// same dataset; suppressNextTrajectoryLoaded_ should swallow that second,
		// deferred emit so callers never see the same load reported twice. Also
		// leaves the shared widget_ in the "page ready" state for every slot below.
		QTest::qWait(8000);
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
		widget_->setDataset(dataset);

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
		widget_->setDataset(first);
		widget_->setDataset(second); // supersedes the first before its background compute can finish

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
		widget_->setDataset(dataset);

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

	void aCursorIndexIsForwardedOnlyForTheCurrentTrajectory() {
		MapBridge* bridge = widget_->findChild<MapBridge*>();
		QVERIFY(bridge);
		QSignalSpy loaded(widget_, &MapWidget::trajectoryLoaded);
		QSignalSpy pageCursor(bridge, &MapBridge::cursorIndexChanged);
		QSignalSpy forwarded(widget_, &MapWidget::cursorIndexChanged);
		TripDataset dataset;
		dataset.points = { samplePoint(10, 20), samplePoint(11, 21), samplePoint(12, 22) };
		widget_->setDataset(dataset);
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
		widget_->setDataset(dataset);
		QCOMPARE(widget_->defaultMapImageFileName(), QStringLiteral("KJFK-KLAX_20240315103000.png"));

		widget_->showOverview({});
		QCOMPARE(widget_->defaultMapImageFileName(), QStringLiteral("trips.png"));
	}

	void resetZoomRefitsTheMapToTheTrajectory() {
		TripDataset dataset;
		dataset.points = { samplePoint(10, 20), samplePoint(11, 21) };
		widget_->setDataset(dataset);
		// Waits for this trip on the page, not for trajectoryLoaded: the
		// previous slot's loads, never waited for, can still emit that late.
		QTRY_COMPARE_WITH_TIMEOUT(mapTrajectoryPointCount(widget_), 2, 10000);
		QTest::qWait(1500); // let the animated fit (0.75 s) settle
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
		QTest::qWait(1500); // let both fits settle
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
		widget_->setDataset(withPoints);
		QVERIFY(spy.wait(10000));
		QTRY_COMPARE_WITH_TIMEOUT(mapElementCount(widget_, "leaflet-marker-draggable"), 1, 5000);

		TripDataset empty;
		empty.tripId = 9;
		widget_->setDataset(empty);
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
		widget_->setDataset(dataset);

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

	// A report the database didn't take stays shown, with a note under it
	// that it wasn't saved.
	void anAiAnswerThatCouldNotBeSavedSaysSo() {
		const QVariantMap r = runAiAnalysisWith({ { aiStream({ aiChunk({ { "Grade: A", false } }, "STOP") }) } }, false);
		QCOMPARE(r.value("calls").toInt(), 1);
		const QString text = r.value("text").toString();
		QVERIFY2(text.startsWith(QStringLiteral("Grade: A")), qPrintable(text));
		QVERIFY2(text.contains(QStringLiteral("Couldn't save this analysis")), qPrintable(text));
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
};

int main(int argc, char* argv[]) {
	QApplication::setAttribute(Qt::AA_ShareOpenGLContexts, true);
	QApplication app(argc, argv);
	TstMapWidget tc;
	return QTest::qExec(&tc, argc, argv);
}

#include "tst_map_widget.moc"
