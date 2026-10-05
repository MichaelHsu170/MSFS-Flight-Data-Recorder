// Charts panel widget (charts_panel.cpp): the QQuickWidget wrapper around
// the QML chart surface -- setDataset()/setVisibleRange()/setCursorIndex()/
// valueAt() driving the real QML series objects, not the pure chart-data math
// (already covered standalone in tst_chart_data.cpp).
#include "charts_panel.h"
#include "local_time_zone.h"

#include <QDateTimeAxis>
#include <QJSValue>
#include <QLineSeries>
#include <QQuickWidget>
#include <QQuickItem>
#include <QSignalSpy>
#include <QTimeZone>
#include <QValueAxis>
#include <QtTest>

#include <algorithm>
#include <functional>
#include <optional>

namespace {

// Shows the panel and waits for its QML root to load; nullptr if it doesn't.
QQuickItem* shownRoot(ChartsPanel& panel, QSize size = QSize(400, 300)) {
	panel.resize(size);
	panel.show();
	QQuickWidget* view = panel.findChild<QQuickWidget*>();
	return QTest::qWaitFor([view]() { return view->rootObject() != nullptr; }, 5000) ? view->rootObject() : nullptr;
}

TripSamplePoint samplePoint(double base, const QString& zulu) {
	TripSamplePoint p;
	p.zuluTime = zulu;
	const float power = (float)base;
	p.engine = { 1, 2, { power, power }, { power, power } };
	p.verticalSpeed = base; p.airspeed = base; p.groundSpeed = base; p.altitude = base;
	p.fuelTotalQuantityWeight = base; p.pitchDegrees = base; p.bankDegrees = base;
	return p;
}

// Sample `second`'s zulu time in a trip recorded from hour:00:00 on 2026-03-05.
QString zuluAt(int hour, int second) {
	return QStringLiteral("2026-03-05T%1:00:%2.000+00:00_4").arg(hour, 2, 10, QLatin1Char('0')).arg(second, 2, 10, QLatin1Char('0'));
}

// Where zuluAt() lands on the time axis: that UTC instant in epoch ms.
qint64 utcMsAt(int hour, int second) {
	return QDateTime(QDate(2026, 3, 5), QTime(hour, 0, second), QTimeZone::UTC).toMSecsSinceEpoch();
}

// count samples one second apart from hour:00:00, sample i with every value i.
TripDataset tripAt(int hour, int count = 10) {
	TripDataset dataset;
	for (int i = 0; i < count; ++i)
		dataset.points.push_back(samplePoint(i, zuluAt(hour, i)));
	return dataset;
}

// The engine power chart's lines that are drawn, e.g. { "engSpeed1", "engLoad1" }.
QStringList visibleEngineLines(QObject* root) {
	QStringList names;
	for (const QString& quantity : { QStringLiteral("engSpeed"), QStringLiteral("engLoad") })
		for (int i = 1; i <= 4; ++i) {
			const QString name = quantity + QString::number(i);
			QLineSeries* line = root->findChild<QLineSeries*>(name + QStringLiteral("Series"));
			if (line && line->isVisible())
				names << name;
		}
	return names;
}

// The engine power chart's legend: each shown series' label.
QStringList engineLegend(QObject* root) {
	QVariant shown = root->findChild<QObject*>(QStringLiteral("engineBlock"))->property("shownSeries");
	if (shown.metaType() == QMetaType::fromType<QJSValue>())
		shown = shown.value<QJSValue>().toVariant();
	QStringList labels;
	for (const QVariant& series : shown.toList())
		labels << series.toMap().value(QStringLiteral("label")).toString();
	return labels;
}

QString engineEmptyText(QObject* root) {
	return root->findChild<QObject*>(QStringLiteral("engineBlock"))->property("shownEmptyText").toString();
}

// The message drawn over each chart's plot, in chart order ("" when none).
QStringList chartMessages(QObject* root) {
	QStringList texts;
	for (QQuickItem* text : root->findChildren<QQuickItem*>(QStringLiteral("emptyText")))
		texts << (text->isVisible() ? text->property("text").toString() : QString());
	return texts;
}

// How many of the charts' axes draw any part: labels, line, title or grid.
// sharedXAxis only holds the time range for the drawn X axes.
int shownAxes(QObject* root) {
	int shown = 0;
	for (QAbstractAxis* axis : root->findChildren<QAbstractAxis*>())
		if (axis->objectName() != QStringLiteral("sharedXAxis")
			&& (axis->isVisible() || axis->isLineVisible() || axis->isTitleVisible()
				|| axis->isGridVisible() || axis->isSubGridVisible()))
			++shown;
	return shown;
}

// The texts drawn by the chart (GraphsView) whose axisY is axis, other than
// its title, that sit where `at` (given the text's and the plot's scene
// rects) accepts, ordered by the key `at` returns for them. Renders a frame
// first, since Qt Graphs lays the labels out when it draws.
QStringList chartLabels(ChartsPanel& panel, QValueAxis* axis,
		const std::function<std::optional<double>(const QRectF& text, const QRectF& plot)>& at) {
	panel.findChild<QQuickWidget*>()->grabFramebuffer();
	QQuickItem* root = panel.findChild<QQuickWidget*>()->rootObject();
	for (QQuickItem* view : root->findChildren<QQuickItem*>()) {
		if (!view->property("plotArea").isValid() || view->property("axisY").value<QObject*>() != axis)
			continue;
		const QRectF plot = view->mapRectToScene(view->property("plotArea").toRectF());
		QList<std::pair<double, QString>> labels;
		for (QQuickItem* item : view->findChildren<QQuickItem*>()) {
			if (!item->isVisible() || !item->inherits("QQuickText"))
				continue;
			const QRectF r = item->mapRectToScene(QRectF(0, 0, item->width(), item->height()));
			const QString text = item->property("text").toString();
			const std::optional<double> key = at(r, plot);
			if (key && !text.isEmpty() && text != axis->titleText())
				labels.append({ *key, text });
		}
		std::sort(labels.begin(), labels.end());
		QStringList texts;
		for (const auto& label : labels)
			texts << label.second;
		return texts;
	}
	return {};
}

// The tick labels drawn along the left-hand Y axis of the chart whose axisY
// is axis, top to bottom.
QStringList leftAxisLabels(ChartsPanel& panel, QValueAxis* axis) {
	return chartLabels(panel, axis, [](const QRectF& r, const QRectF& plot) -> std::optional<double> {
		if (r.right() <= plot.left() && r.center().y() >= plot.top() - 1 && r.center().y() <= plot.bottom() + 1)
			return r.center().y();
		return std::nullopt;
	});
}

// The time labels drawn under the chart whose axisY is axis, left to right.
QStringList timeAxisLabels(ChartsPanel& panel, QValueAxis* axis) {
	return chartLabels(panel, axis, [](const QRectF& r, const QRectF& plot) -> std::optional<double> {
		if (r.top() >= plot.bottom() - 1 && r.center().x() >= plot.left() - 1 && r.center().x() <= plot.right() + 1)
			return r.center().x();
		return std::nullopt;
	});
}

// How many charts show their legend.
int shownLegends(QObject* root) {
	int shown = 0;
	for (QQuickItem* legend : root->findChildren<QQuickItem*>(QStringLiteral("legend")))
		shown += legend->isVisible();
	return shown;
}

// How many points all the chart lines hold together.
int pointsInLines(QObject* root) {
	int points = 0;
	for (QLineSeries* line : root->findChildren<QLineSeries*>())
		points += line->count();
	return points;
}

}

class TstChartsPanel : public QObject {
	Q_OBJECT

private slots:
	// Every test runs in a local time zone with DST changes, so none can rely
	// on local time matching zulu time or on a day without a skipped hour.
	void initTestCase() { TestSupport::usePacificLocalTime(); }

	// US Pacific clocks jump from 02:00 PST to 03:00 PDT at 10:00Z on
	// 2026-03-08. A trip across that change still reads in zulu time along
	// the time axis, evenly spaced, with no skipped hour.
	void timeAxisLabelsAreZuluAcrossTheLocalDstChange() {
		QVERIFY(QDateTime(QDate(2026, 3, 8), QTime(2, 30)).time() != QTime(2, 30)); // Pacific time is in effect
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel, QSize(900, 400));
		QVERIFY(root);
		TripDataset dataset;
		for (int i = 0; i < 20; ++i)
			dataset.points.push_back(samplePoint(i, QStringLiteral("2026-03-08T%1.000+00:00_0")
				.arg(QTime(9, 59, 50).addSecs(i).toString(QStringLiteral("HH:mm:ss")))));
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(dataset);
		QVERIFY(spy.wait(5000));
		const QStringList zulu = {
			QStringLiteral("2026-03-08\n09:59:50.000"), QStringLiteral("2026-03-08\n09:59:52.000"),
			QStringLiteral("2026-03-08\n09:59:54.000"), QStringLiteral("2026-03-08\n09:59:56.000"),
			QStringLiteral("2026-03-08\n09:59:58.000"), QStringLiteral("2026-03-08\n10:00:00.000"),
			QStringLiteral("2026-03-08\n10:00:02.000"), QStringLiteral("2026-03-08\n10:00:04.000"),
			QStringLiteral("2026-03-08\n10:00:06.000"), QStringLiteral("2026-03-08\n10:00:08.000"),
		};
		QCOMPARE(timeAxisLabels(panel, root->findChild<QValueAxis*>(QStringLiteral("vsYAxis"))), zulu);
	}

	// Engine load series 2-4 re-add series 1's right-hand axis, and Qt Graphs
	// warns once for each (main.cpp filters these from the log). Any further
	// one would be an axis wired to a second chart by mistake.
	void constructsAndLoadsAQmlRoot() {
		const QRegularExpression sharedAxis(QStringLiteral("axis already associated with"));
		for (int series = 2; series <= 4; ++series)
			QTest::ignoreMessage(QtWarningMsg, sharedAxis);
		QTest::failOnWarning(sharedAxis);
		ChartsPanel panel;
		QVERIFY(shownRoot(panel));
	}

	void setDatasetEmitsSeriesLoaded() {
		ChartsPanel panel;
		QVERIFY(shownRoot(panel));

		TripDataset dataset;
		dataset.points = {
			samplePoint(1, QStringLiteral("2026-03-04T10:00:00.000+00:00_3")),
			samplePoint(2, QStringLiteral("2026-03-04T10:00:01.000+00:00_3")),
			samplePoint(3, QStringLiteral("2026-03-04T10:00:02.000+00:00_3")),
		};
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(dataset);
		QVERIFY(spy.wait(5000));
	}

	// No trip (at startup, and after a deselect): no lines, axes, grid or
	// legend, and every chart says so -- then "Loading…" until a trip loads.
	void withNoTripEveryChartIsBlankAndSaysNoTripSelected() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);
		const QStringList noTrip(9, QStringLiteral("No trip selected"));
		QCOMPARE(chartMessages(root), noTrip);
		QCOMPARE(shownAxes(root), 0);
		QCOMPARE(shownLegends(root), 0);
		QVERIFY(!root->findChild<QQuickItem*>(QStringLiteral("endOfTrajectoryLine"))->isVisible());

		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		TripDataset dataset;
		dataset.points = { samplePoint(1, QStringLiteral("2026-03-04T09:00:00.000+00:00_3")),
			samplePoint(2, QStringLiteral("2026-03-04T09:00:01.000+00:00_3")) };
		panel.setDataset(dataset);
		// Its lines are computed off the GUI thread, so not in yet.
		QCOMPARE(chartMessages(root), QStringList(9, QStringLiteral("Loading…")));
		QCOMPARE(shownAxes(root), 0);
		QVERIFY(spy.wait(5000));
		QCOMPARE(chartMessages(root), QStringList(9, QString()));
		QCOMPARE(shownAxes(root), 19);  // each chart's X and Y, and engine load
		QCOMPARE(shownLegends(root), 9);
		QVERIFY(root->findChild<QQuickItem*>(QStringLiteral("endOfTrajectoryLine"))->isVisible());
		// Both points in each of the 25 lines, hidden engines' lines included.
		QCOMPARE(pointsInLines(root), 2 * 25);

		// The deselect finishes before setDataset() returns.
		panel.setDataset(TripDataset());
		QCOMPARE(spy.count(), 2);
		QCOMPARE(pointsInLines(root), 0);
		QCOMPARE(chartMessages(root), noTrip);
		QCOMPARE(shownAxes(root), 0);
		QCOMPARE(shownLegends(root), 0);
		QVERIFY(!root->findChild<QQuickItem*>(QStringLiteral("endOfTrajectoryLine"))->isVisible());
	}

	// A trip that recorded no point is blank like no trip, but says it has no
	// data.
	void aTripWithNoPointSaysNoDataRecorded() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);

		TripDataset empty;
		empty.tripId = 7;
		panel.setDataset(empty);
		QCOMPARE(chartMessages(root), QStringList(9, QStringLiteral("No data recorded")));
		QCOMPARE(shownAxes(root), 0);
		QCOMPARE(shownLegends(root), 0);

		panel.setDataset(TripDataset());
		QCOMPARE(chartMessages(root), QStringList(9, QStringLiteral("No trip selected")));
	}

	void valueAtIsEmptyWithNoDataset() {
		ChartsPanel panel;
		QVERIFY(panel.valueAt(0).isEmpty());
	}

	// A second trip replaces the first's lines and values entirely.
	void loadingASecondDatasetReplacesTheFirst() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);
		QDateTimeAxis* timeAxis = root->findChild<QDateTimeAxis*>(QStringLiteral("sharedXAxis"));
		QVERIFY(timeAxis);

		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		TripDataset first;
		first.points = { samplePoint(1, QStringLiteral("2026-03-05T11:00:00.000+00:00_4")),
			samplePoint(1, QStringLiteral("2026-03-05T11:00:01.000+00:00_4")) };
		panel.setDataset(first);
		QVERIFY(spy.wait(5000));

		TripDataset second;
		second.points = { samplePoint(2, QStringLiteral("2026-03-05T12:00:00.000+00:00_4")) };
		panel.setDataset(second);
		QVERIFY(spy.wait(5000));
		const qint64 secondMs = QDateTime(QDate(2026, 3, 5), QTime(12, 0, 0), QTimeZone::UTC).toMSecsSinceEpoch();
		QCOMPARE(pointsInLines(root), 25);
		// A one-sample trip still gets a 1 s wide axis (chart_data.h).
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), secondMs);
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), secondMs + 1000);
		QCOMPARE(panel.valueAt(secondMs)[QStringLiteral("engSpeed1")].toDouble(), 2.0);
	}

	void aSupersededDatasetLoadIsDiscardedWithoutEmittingSeriesLoaded() {
		ChartsPanel panel;
		QVERIFY(shownRoot(panel));

		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		TripDataset first;
		first.points = { samplePoint(1, QStringLiteral("2026-03-05T13:00:00.000+00:00_4")) };
		TripDataset second;
		second.points = { samplePoint(2, QStringLiteral("2026-03-05T14:00:00.000+00:00_4")) };
		panel.setDataset(first);
		panel.setDataset(second); // supersedes the first before its background compute can finish
		QVERIFY(spy.wait(5000));
		QTest::qWait(300); // give the superseded watcher a chance to finish too
		QCOMPARE(spy.count(), 1); // the superseded load never emits
	}

	void setCursorIndexSetsOrClearsTheQmlCursorTimeProperty() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);

		const QString t0 = QStringLiteral("2026-03-05T15:00:00.000+00:00_4");
		const QString t1 = QStringLiteral("2026-03-05T15:00:01.000+00:00_4");
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		TripDataset dataset;
		dataset.points = { samplePoint(1, t0), samplePoint(2, t1) };
		panel.setDataset(dataset);
		QVERIFY(spy.wait(5000));

		// t1 on the time axis: its UTC instant.
		const double t1Ms = (double)QDateTime(QDate(2026, 3, 5), QTime(15, 0, 1), QTimeZone::UTC).toMSecsSinceEpoch();
		panel.setCursorIndex(1);
		QCOMPARE(root->property("cursorTime").toDouble(), t1Ms);

		panel.setCursorIndex(99); // out of range
		QCOMPARE(root->property("cursorTime").toDouble(), -1.0);
	}

	void setDatasetClearsTheCursorOfThePreviousDataset() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);

		const QString t0 = QStringLiteral("2026-03-05T15:30:00.000+00:00_4");
		const QString t1 = QStringLiteral("2026-03-05T15:30:01.000+00:00_4");
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		TripDataset dataset;
		dataset.points = { samplePoint(1, t0), samplePoint(2, t1) };
		panel.setDataset(dataset);
		QVERIFY(spy.wait(5000));
		const double t1Ms = (double)QDateTime(QDate(2026, 3, 5), QTime(15, 30, 1), QTimeZone::UTC).toMSecsSinceEpoch();

		// Reloading the same trip: the old cursor time lies inside the new X
		// axis range, so a stale value would stay drawn.
		panel.setCursorIndex(1);
		QCOMPARE(root->property("cursorTime").toDouble(), t1Ms);
		panel.setDataset(dataset);
		QCOMPARE(root->property("cursorTime").toDouble(), -1.0);
		QVERIFY(spy.wait(5000));

		// Deselect (empty dataset) clears it too.
		panel.setCursorIndex(1);
		QCOMPARE(root->property("cursorTime").toDouble(), t1Ms);
		panel.setDataset(TripDataset());
		QCOMPARE(root->property("cursorTime").toDouble(), -1.0);
	}

	// The map can finish drawing a trip, and send its cursor, before the
	// charts do: the index is the new trip's, so it's drawn once that loads.
	void setCursorIndexWaitsForALoadingTrip() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(tripAt(10, 3));
		QVERIFY(spy.wait(5000));

		panel.setDataset(tripAt(11, 3));
		panel.setCursorIndex(1);
		QCOMPARE(root->property("cursorTime").toDouble(), -1.0); // not the first trip's sample 1
		QVERIFY(spy.wait(5000));
		QCOMPARE(root->property("cursorTime").toDouble(), (double)utcMsAt(11, 1));

		// An index for a trip replaced before it loaded isn't applied to the next.
		panel.setDataset(tripAt(12, 3));
		panel.setCursorIndex(2);
		panel.setDataset(tripAt(13, 3));
		QVERIFY(spy.wait(5000));
		QCOMPARE(root->property("cursorTime").toDouble(), -1.0);
	}

	void theEnginePowerChartIsLabeledByTheDatasetsEngine() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);
		QValueAxis* speedAxis = root->findChild<QValueAxis*>(QStringLiteral("engSpeedYAxis"));
		QValueAxis* loadAxis = root->findChild<QValueAxis*>(QStringLiteral("engLoadYAxis"));
		QVERIFY(speedAxis && loadAxis);
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);

		// The first point recorded no power; the chart uses the second's.
		TripDataset piston;
		piston.points = { samplePoint(1, QStringLiteral("2026-03-05T20:00:00.000+00:00_4")),
			samplePoint(2, QStringLiteral("2026-03-05T20:00:01.000+00:00_4")) };
		piston.points[0].engine = { 0, 0 };
		piston.points[1].engine = { 0, 1, { 2400 }, { 24.5f } };
		panel.setDataset(piston);
		QVERIFY(spy.wait(5000));
		QVariantMap spec = root->property("engineSpec").toMap();
		QCOMPARE(spec.value("count").toInt(), 1);
		QCOMPARE(spec.value("speedAxisTitle").toString(), QStringLiteral("RPM"));
		// Sized to the data (niceAxisMax()), unlike a jet's fixed 110 %.
		QCOMPARE(speedAxis->max(), 3000.0);
		QCOMPARE(loadAxis->max(), 40.0);
		// The QML chart: one engine's lines, legend and both axis titles.
		QCOMPARE(visibleEngineLines(root), (QStringList{ "engSpeed1", "engLoad1" }));
		QCOMPARE(engineLegend(root), (QStringList{ "RPM #1", "MP #1" }));
		QCOMPARE(speedAxis->titleText(), QStringLiteral("RPM"));
		QVERIFY(loadAxis->isVisible());
		QCOMPARE(loadAxis->titleText(), QStringLiteral("Manifold Pressure (inHg)"));
		QCOMPARE(engineEmptyText(root), QString());

		TripDataset jet;
		jet.points = { samplePoint(50, QStringLiteral("2026-03-05T21:00:00.000+00:00_4")) };
		panel.setDataset(jet);
		QVERIFY(spy.wait(5000));
		QCOMPARE(root->property("engineSpec").toMap().value("speedLabel").toString(), QStringLiteral("N1"));
		QCOMPARE(speedAxis->max(), 110.0);
		QCOMPARE(loadAxis->max(), 110.0);
		QCOMPARE(visibleEngineLines(root), (QStringList{ "engSpeed1", "engSpeed2", "engLoad1", "engLoad2" }));
		QCOMPARE(engineLegend(root), (QStringList{ "N1 #1", "N1 #2", "N2 #1", "N2 #2" }));
		QCOMPARE(speedAxis->titleText(), QStringLiteral("N1 (%)"));
		QCOMPARE(loadAxis->titleText(), QStringLiteral("N2 (%)"));

		// An N1 overspeed past the fixed 110 sizes that axis to the data
		// (115 * 1.25 = 143.75, in steps of 50); N2 at exactly 110 still fits.
		TripDataset overspeed = jet;
		overspeed.points[0].engine = { 1, 2, { 115, 50 }, { 110, 50 } };
		panel.setDataset(overspeed);
		QVERIFY(spy.wait(5000));
		QCOMPARE(speedAxis->max(), 150.0);
		QCOMPARE(loadAxis->max(), 110.0);

		// An old trip with no engine power recorded: the no-data message.
		TripDataset none;
		none.points = { samplePoint(1, QStringLiteral("2026-03-05T22:00:00.000+00:00_4")) };
		none.points[0].engine = {};
		panel.setDataset(none);
		QVERIFY(spy.wait(5000));
		QCOMPARE(root->property("engineSpec").toMap(), (QVariantMap{ { "count", 0 } }));
		QCOMPARE(engineEmptyText(root), QStringLiteral("No engine power data recorded"));
		QCOMPARE(visibleEngineLines(root), QStringList());
		QCOMPARE(engineLegend(root), QStringList());
		QCOMPARE(speedAxis->titleText(), QStringLiteral("Engine Power"));
		QVERIFY(!loadAxis->isVisible());
		// Not the jet's 110 left over: niceAxisMax(0) like any empty axis.
		QCOMPARE(speedAxis->max(), 1.0);

		// Deselect after a jet: no trip, so no engine lines or labels, and the
		// no-trip message instead of the no-power one.
		panel.setDataset(jet);
		QVERIFY(spy.wait(5000));
		panel.setDataset(TripDataset());
		QVERIFY(!root->property("engineSpec").isValid());
		QCOMPARE(engineEmptyText(root), QStringLiteral("No trip selected"));
		QCOMPARE(visibleEngineLines(root), QStringList());
		QCOMPARE(engineLegend(root), QStringList());
		QVERIFY(!loadAxis->isVisible());
	}

	// The "%.0f" axes label a short range (a level or empty trip) once per
	// whole unit instead of repeating rounded labels like "1 1 1 0 0 0" or
	// "-0"; a range of 10 or more keeps Qt Graphs' automatic ticks.
	void wholeNumberAxesNeverRepeatALabel() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel, QSize(800, 1600));
		QVERIFY(root);
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		struct Case { double value; QStringList speed, vs; };
		const Case cases[] = {
			{ 0, { "1", "0" }, { "2", "1", "0", "-1", "-2" } },
			{ 3, { "4", "3", "2", "1", "0" }, { "4", "3", "2", "1", "0", "-1" } },
			{ 9, { "14", "12", "10", "8", "6", "4", "2", "0" }, { "14", "12", "10", "8", "6", "4", "2", "0", "-2", "-4" } },
			{ 50, { "80", "72", "64", "56", "48", "40", "32", "24", "16", "8", "0" },
				{ "80", "70", "60", "50", "40", "30", "20", "10", "0", "-10", "-20" } },
		};
		for (const Case& c : cases) {
			TripDataset dataset;
			dataset.points = { samplePoint(0, zuluAt(10, 0)), samplePoint(c.value, zuluAt(10, 1)) };
			panel.setDataset(dataset);
			QVERIFY(spy.wait(5000));
			// samplePoint puts the value in every chart: speed, altitude and
			// fuel share one range, as do V/S, pitch and bank.
			for (const char* name : { "speedYAxis", "altYAxis", "fuelYAxis" })
				QCOMPARE(leftAxisLabels(panel, root->findChild<QValueAxis*>(QLatin1String(name))), c.speed);
			for (const char* name : { "vsYAxis", "pitchYAxis", "bankYAxis" })
				QCOMPARE(leftAxisLabels(panel, root->findChild<QValueAxis*>(QLatin1String(name))), c.vs);
		}
		// The engine axes go through the same rule: samplePoint's jet has a
		// fixed 0-110 % scale, which keeps its automatic ticks.
		QCOMPARE(leftAxisLabels(panel, root->findChild<QValueAxis*>(QStringLiteral("engSpeedYAxis"))),
			QStringList({ "100", "80", "60", "40", "20", "0" }));
	}

	// The engine power chart's right-hand axis narrows its plot; the other
	// charts match it so one time lines up across all of them.
	void everyChartsPlotEndsAtTheSameX() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel, QSize(800, 600));
		QVERIFY(root);
		// A trip with engine power: with no trip, the axes are all hidden.
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		TripDataset dataset;
		dataset.points = { samplePoint(1, QStringLiteral("2026-03-05T19:00:00.000+00:00_4")) };
		panel.setDataset(dataset);
		QVERIFY(spy.wait(5000));
		QList<QQuickItem*> views;
		for (QQuickItem* item : root->findChildren<QQuickItem*>())
			if (item->property("plotArea").isValid())
				views.append(item);
		QCOMPARE(views.size(), 9);
		auto plotRight = [](QQuickItem* view) {
			const QRectF plot = view->property("plotArea").toRectF();
			return view->mapToScene(QPointF(plot.x() + plot.width(), 0)).x();
		};
		QVERIFY(QTest::qWaitFor([&]() {
			return std::all_of(views.begin(), views.end(), [&](QQuickItem* v) { return qFuzzyCompare(plotRight(v), plotRight(views.first())); });
		}, 2000));
		QVERIFY(plotRight(views.first()) < root->width() - 40); // the right-hand axis is there
	}

	void valueAtUsesTheFullResolutionDataAfterADatasetLoad() {
		ChartsPanel panel;
		QVERIFY(shownRoot(panel));

		const QString t0 = QStringLiteral("2026-03-05T17:00:00.000+00:00_4");
		const QString t1 = QStringLiteral("2026-03-05T17:00:01.000+00:00_4");
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		TripDataset dataset;
		dataset.points = { samplePoint(3, t0), samplePoint(9, t1) };
		panel.setDataset(dataset);
		QVERIFY(spy.wait(5000));

		QVariantMap v = panel.valueAt(chartTimeMs(t1));
		QCOMPARE(v[QStringLiteral("engSpeed1")].toDouble(), 9.0);
	}

	// The hover readout while a second trip loads: the first trip's lines are
	// still shown, zoomed to a slice, and it reads their sample under the
	// pointer -- not one of the second trip's.
	void valueAtWhileATripLoadsReadsTheShownTrip() {
		ChartsPanel panel;
		QVERIFY(shownRoot(panel));

		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(tripAt(16));
		QVERIFY(spy.wait(5000));
		panel.setVisibleRange(2, 5);

		TripDataset second;
		for (int i = 0; i < 10; ++i)
			second.points.push_back(samplePoint(100 + i, zuluAt(17, i)));
		panel.setDataset(second);
		QCOMPARE(spy.count(), 1);  // still loading
		QCOMPARE(panel.valueAt(chartTimeMs(zuluAt(16, 4)))[QStringLiteral("alt")].toDouble(), 4.0);
		// Past the zoomed slice's 4 shown samples.
		QCOMPARE(panel.valueAt(chartTimeMs(zuluAt(16, 8)))[QStringLiteral("alt")].toDouble(), 8.0);

		QVERIFY(spy.wait(5000));
		QCOMPARE(panel.valueAt(chartTimeMs(zuluAt(17, 8)))[QStringLiteral("alt")].toDouble(), 108.0);
	}

	void setVisibleRangeWithNoTripLeavesTheTimeAxisAlone() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);
		QDateTimeAxis* timeAxis = root->findChild<QDateTimeAxis*>(QStringLiteral("sharedXAxis"));
		QVERIFY(timeAxis);
		const QDateTime min = timeAxis->min();
		const QDateTime max = timeAxis->max();

		// No trip has been loaded: zooming all the way out has no time range
		// to show, so the hidden axis keeps the one it started with.
		panel.setVisibleRange(-1, -1);
		QCOMPARE(timeAxis->min(), min);
		QCOMPARE(timeAxis->max(), max);
	}

	void setVisibleRangeWithNegativeIndexAfterALoadReloadsTheFullResolutionView() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);
		QDateTimeAxis* timeAxis = root->findChild<QDateTimeAxis*>(QStringLiteral("sharedXAxis"));
		QValueAxis* altAxis = root->findChild<QValueAxis*>(QStringLiteral("altYAxis"));
		QVERIFY(timeAxis && altAxis);

		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(tripAt(18));
		QVERIFY(spy.wait(5000));
		const double fullAltMax = altAxis->max();

		panel.setVisibleRange(2, 5);
		QCOMPARE(pointsInLines(root), 4 * 25);
		panel.setVisibleRange(-1, -1); // zoomed all the way out: the whole trip again
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), utcMsAt(18, 0));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), utcMsAt(18, 9));
		QCOMPARE(pointsInLines(root), 10 * 25);
		QCOMPARE(altAxis->max(), fullAltMax);
		QVERIFY(root->property("isFullRangeVisible").toBool());
	}

	void setVisibleRangeZoomsToASliceAndIgnoresADuplicateRange() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);
		QDateTimeAxis* timeAxis = root->findChild<QDateTimeAxis*>(QStringLiteral("sharedXAxis"));
		QValueAxis* altAxis = root->findChild<QValueAxis*>(QStringLiteral("altYAxis"));
		QVERIFY(timeAxis && altAxis);

		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(tripAt(19));
		QVERIFY(spy.wait(5000));
		const double fullAltMax = altAxis->max();

		// Zoomed slice: samples 2..5 only, with the Y axes fitted to them
		// (altitude 5 at most instead of 9).
		QLineSeries* line = root->findChildren<QLineSeries*>().value(0);
		QVERIFY(line);
		QSignalSpy replaced(line, &QXYSeries::pointsReplaced);
		panel.setVisibleRange(2, 5);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), utcMsAt(19, 2));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), utcMsAt(19, 5));
		QCOMPARE(pointsInLines(root), 4 * 25);
		QVERIFY(altAxis->max() < fullAltMax);
		QVERIFY(!root->property("isFullRangeVisible").toBool());
		QCOMPARE(replaced.count(), 1);

		panel.setVisibleRange(2, 5); // Leaflet's zoomend+moveend duplicate: not drawn again
		QCOMPARE(replaced.count(), 1);

		// A single-sample slice is widened by a second so the axis isn't empty.
		panel.setVisibleRange(3, 3);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), utcMsAt(19, 3));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), utcMsAt(19, 4));
		QCOMPARE(pointsInLines(root), 25);
	}

	// While a trip loads, the old trip's lines stay: a range (which indexes
	// the new trip) leaves their axes alone, and the last one applies once
	// loaded.
	void setVisibleRangeWaitsForALoadingTrip() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);
		QDateTimeAxis* timeAxis = root->findChild<QDateTimeAxis*>(QStringLiteral("sharedXAxis"));
		QValueAxis* speedAxis = root->findChild<QValueAxis*>(QStringLiteral("engSpeedYAxis"));
		QVERIFY(timeAxis && speedAxis);

		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(tripAt(20));
		QVERIFY(spy.wait(5000));
		panel.setVisibleRange(2, 5);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), utcMsAt(20, 2));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), utcMsAt(20, 5));
		QCOMPARE(speedAxis->max(), 110.0);  // the jet's fixed N1 axis

		panel.setDataset(tripAt(21));
		panel.setVisibleRange(-1, -1);
		panel.setVisibleRange(3, 4);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), utcMsAt(20, 2));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), utcMsAt(20, 5));
		QCOMPARE(speedAxis->max(), 110.0);

		QVERIFY(spy.wait(5000));
		// The last range sent while loading.
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), utcMsAt(21, 3));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), utcMsAt(21, 4));

		// A range for a trip replaced before it loaded isn't applied to the next.
		panel.setDataset(tripAt(22));
		panel.setVisibleRange(3, 4);
		panel.setDataset(tripAt(23));
		QVERIFY(spy.wait(5000));
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), utcMsAt(23, 0));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), utcMsAt(23, 9));

		// The full range, sent while loading or after (Leaflet's second
		// event), is what loads: not drawn again.
		QLineSeries* line = root->findChildren<QLineSeries*>().value(0);
		QVERIFY(line);
		QSignalSpy replaced(line, &QXYSeries::pointsReplaced);
		panel.setDataset(tripAt(19));
		panel.setVisibleRange(-1, -1);
		QVERIFY(spy.wait(5000));
		panel.setVisibleRange(-1, -1);
		QCOMPARE(replaced.count(), 1);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), utcMsAt(19, 0));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), utcMsAt(19, 9));
	}
};

QTEST_MAIN(TstChartsPanel)
#include "tst_charts_panel.moc"
