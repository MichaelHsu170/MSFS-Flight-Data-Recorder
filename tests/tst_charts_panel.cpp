// Charts panel widget (charts_panel.cpp): the QQuickWidget wrapper around
// the QML chart surface -- setDataset()/setVisibleRange()/valueAt() driving the real QML series objects, not the pure chart-data math
// (already covered standalone in tst_chart_data.cpp).
#include "charts_panel.h"

#include <QDateTimeAxis>
#include <QJSValue>
#include <QLineSeries>
#include <QQuickWidget>
#include <QQuickItem>
#include <QSignalSpy>
#include <QValueAxis>
#include <QtTest>

#include <algorithm>

namespace {

TripSamplePoint samplePoint(double base, const QString& zulu) {
	TripSamplePoint p;
	p.zuluTime = zulu;
	const float power = (float)base;
	p.engine = { 1, 2, { power, power }, { power, power } };
	p.verticalSpeed = base; p.airspeed = base; p.groundSpeed = base; p.altitude = base;
	p.fuelTotalQuantityWeight = base; p.pitchDegrees = base; p.bankDegrees = base;
	return p;
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
	// Engine load series 2-4 re-add series 1's right-hand axis, and Qt Graphs
	// warns once for each (main.cpp filters these from the log). Any further
	// one would be an axis wired to a second chart by mistake.
	void constructsAndLoadsAQmlRoot() {
		const QRegularExpression sharedAxis(QStringLiteral("axis already associated with"));
		for (int series = 2; series <= 4; ++series)
			QTest::ignoreMessage(QtWarningMsg, sharedAxis);
		QTest::failOnWarning(sharedAxis);
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));
	}

	void setDatasetEmitsSeriesLoaded() {
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));

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
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));
		QQuickItem* root = panel.findChild<QQuickWidget*>()->rootObject();
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
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));
		QQuickItem* root = panel.findChild<QQuickWidget*>()->rootObject();

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
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));
		QQuickItem* root = panel.findChild<QQuickWidget*>()->rootObject();
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
		const qint64 secondMs = QDateTime(QDate(2026, 3, 5), QTime(12, 0, 0)).toMSecsSinceEpoch();
		QCOMPARE(pointsInLines(root), 25);
		// A one-sample trip still gets a 1 s wide axis (chart_data.h).
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), secondMs);
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), secondMs + 1000);
		QCOMPARE(panel.valueAt(secondMs)[QStringLiteral("engSpeed1")].toDouble(), 2.0);
	}

	void aSupersededDatasetLoadIsDiscardedWithoutEmittingSeriesLoaded() {
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));

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
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));

		const QString t0 = QStringLiteral("2026-03-05T15:00:00.000+00:00_4");
		const QString t1 = QStringLiteral("2026-03-05T15:00:01.000+00:00_4");
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		TripDataset dataset;
		dataset.points = { samplePoint(1, t0), samplePoint(2, t1) };
		panel.setDataset(dataset);
		QVERIFY(spy.wait(5000));

		QQuickItem* root = panel.findChild<QQuickWidget*>()->rootObject();
		// t1 on the time axis: its zulu time read as local time.
		const double t1Ms = (double)QDateTime(QDate(2026, 3, 5), QTime(15, 0, 1)).toMSecsSinceEpoch();
		panel.setCursorIndex(1);
		QCOMPARE(root->property("cursorTime").toDouble(), t1Ms);

		panel.setCursorIndex(99); // out of range
		QCOMPARE(root->property("cursorTime").toDouble(), -1.0);
	}

	void setDatasetClearsTheCursorOfThePreviousDataset() {
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));

		const QString t0 = QStringLiteral("2026-03-05T15:30:00.000+00:00_4");
		const QString t1 = QStringLiteral("2026-03-05T15:30:01.000+00:00_4");
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		TripDataset dataset;
		dataset.points = { samplePoint(1, t0), samplePoint(2, t1) };
		panel.setDataset(dataset);
		QVERIFY(spy.wait(5000));
		QQuickItem* root = panel.findChild<QQuickWidget*>()->rootObject();
		const double t1Ms = (double)QDateTime(QDate(2026, 3, 5), QTime(15, 30, 1)).toMSecsSinceEpoch();

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

	void theEnginePowerChartIsLabeledByTheDatasetsEngine() {
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));
		QQuickItem* root = panel.findChild<QQuickWidget*>()->rootObject();
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

	// The engine power chart's right-hand axis narrows its plot; the other
	// charts match it so one time lines up across all of them.
	void everyChartsPlotEndsAtTheSameX() {
		ChartsPanel panel;
		panel.resize(800, 600);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));
		QQuickItem* root = panel.findChild<QQuickWidget*>()->rootObject();
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
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));

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
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));

		const auto at = [](int hour, int second) {
			return QStringLiteral("2026-03-05T%1:00:%2.000+00:00_4").arg(hour).arg(second, 2, 10, QLatin1Char('0'));
		};
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		TripDataset first;
		for (int i = 0; i < 10; ++i)
			first.points.push_back(samplePoint(i, at(16, i)));
		panel.setDataset(first);
		QVERIFY(spy.wait(5000));
		panel.setVisibleRange(2, 5);

		TripDataset second;
		for (int i = 0; i < 10; ++i)
			second.points.push_back(samplePoint(100 + i, at(17, i)));
		panel.setDataset(second);
		QCOMPARE(spy.count(), 1);  // still loading
		QCOMPARE(panel.valueAt(chartTimeMs(at(16, 4)))[QStringLiteral("alt")].toDouble(), 4.0);
		// Past the zoomed slice's 4 shown samples.
		QCOMPARE(panel.valueAt(chartTimeMs(at(16, 8)))[QStringLiteral("alt")].toDouble(), 8.0);

		QVERIFY(spy.wait(5000));
		QCOMPARE(panel.valueAt(chartTimeMs(at(17, 8)))[QStringLiteral("alt")].toDouble(), 108.0);
	}

	void setVisibleRangeWithNoTripLeavesTheTimeAxisAlone() {
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));
		QDateTimeAxis* timeAxis = panel.findChild<QQuickWidget*>()->rootObject()->findChild<QDateTimeAxis*>(QStringLiteral("sharedXAxis"));
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
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));

		QQuickItem* root = panel.findChild<QQuickWidget*>()->rootObject();
		QDateTimeAxis* timeAxis = root->findChild<QDateTimeAxis*>(QStringLiteral("sharedXAxis"));
		QValueAxis* altAxis = root->findChild<QValueAxis*>(QStringLiteral("altYAxis"));
		QVERIFY(timeAxis && altAxis);
		// Where each sample lands on the time axis: its zulu time read as local time.
		const auto localMs = [](int second) { return QDateTime(QDate(2026, 3, 5), QTime(18, 0, second)).toMSecsSinceEpoch(); };

		TripDataset dataset;
		for (int i = 0; i < 10; ++i)
			dataset.points.push_back(samplePoint(i, QStringLiteral("2026-03-05T18:00:%1.000+00:00_4").arg(i, 2, 10, QLatin1Char('0'))));
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(dataset);
		QVERIFY(spy.wait(5000));
		const double fullAltMax = altAxis->max();

		panel.setVisibleRange(2, 5);
		QCOMPARE(pointsInLines(root), 4 * 25);
		panel.setVisibleRange(-1, -1); // zoomed all the way out: the whole trip again
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), localMs(0));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), localMs(9));
		QCOMPARE(pointsInLines(root), 10 * 25);
		QCOMPARE(altAxis->max(), fullAltMax);
		QVERIFY(root->property("isFullRangeVisible").toBool());
	}

	void setVisibleRangeZoomsToASliceAndIgnoresADuplicateRange() {
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));
		QQuickItem* root = panel.findChild<QQuickWidget*>()->rootObject();
		QDateTimeAxis* timeAxis = root->findChild<QDateTimeAxis*>(QStringLiteral("sharedXAxis"));
		QValueAxis* altAxis = root->findChild<QValueAxis*>(QStringLiteral("altYAxis"));
		QVERIFY(timeAxis && altAxis);
		const auto localMs = [](int second) { return QDateTime(QDate(2026, 3, 5), QTime(19, 0, second)).toMSecsSinceEpoch(); };

		TripDataset dataset;
		for (int i = 0; i < 10; ++i)
			dataset.points.push_back(samplePoint(i, QStringLiteral("2026-03-05T19:00:%1.000+00:00_4").arg(i, 2, 10, QLatin1Char('0'))));
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(dataset);
		QVERIFY(spy.wait(5000));
		const double fullAltMax = altAxis->max();

		// Zoomed slice: samples 2..5 only, with the Y axes fitted to them
		// (altitude 5 at most instead of 9).
		QLineSeries* line = root->findChildren<QLineSeries*>().value(0);
		QVERIFY(line);
		QSignalSpy replaced(line, &QXYSeries::pointsReplaced);
		panel.setVisibleRange(2, 5);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), localMs(2));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), localMs(5));
		QCOMPARE(pointsInLines(root), 4 * 25);
		QVERIFY(altAxis->max() < fullAltMax);
		QVERIFY(!root->property("isFullRangeVisible").toBool());
		QCOMPARE(replaced.count(), 1);

		panel.setVisibleRange(2, 5); // Leaflet's zoomend+moveend duplicate: not drawn again
		QCOMPARE(replaced.count(), 1);

		// A single-sample slice is widened by a second so the axis isn't empty.
		panel.setVisibleRange(3, 3);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), localMs(3));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), localMs(4));
		QCOMPARE(pointsInLines(root), 25);
	}

	// While a trip loads, the old trip's lines stay: a range (which indexes
	// the new trip) leaves their axes alone, and the last one applies once
	// loaded.
	void setVisibleRangeWaitsForALoadingTrip() {
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));
		QQuickItem* root = panel.findChild<QQuickWidget*>()->rootObject();
		QDateTimeAxis* timeAxis = root->findChild<QDateTimeAxis*>(QStringLiteral("sharedXAxis"));
		QValueAxis* speedAxis = root->findChild<QValueAxis*>(QStringLiteral("engSpeedYAxis"));
		QVERIFY(timeAxis && speedAxis);
		const auto at = [](int hour, int second) {
			return QStringLiteral("2026-03-05T%1:00:%2.000+00:00_4").arg(hour).arg(second, 2, 10, QLatin1Char('0'));
		};
		// Where at() lands on the time axis: its zulu time read as local time.
		const auto localMs = [](int hour, int second) {
			return QDateTime(QDate(2026, 3, 5), QTime(hour, 0, second)).toMSecsSinceEpoch();
		};
		const auto trip = [&at](int hour) {
			TripDataset dataset;
			for (int i = 0; i < 10; ++i)
				dataset.points.push_back(samplePoint(i, at(hour, i)));
			return dataset;
		};

		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(trip(20));
		QVERIFY(spy.wait(5000));
		panel.setVisibleRange(2, 5);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), localMs(20, 2));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), localMs(20, 5));
		QCOMPARE(speedAxis->max(), 110.0);  // the jet's fixed N1 axis

		panel.setDataset(trip(21));
		panel.setVisibleRange(-1, -1);
		panel.setVisibleRange(3, 4);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), localMs(20, 2));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), localMs(20, 5));
		QCOMPARE(speedAxis->max(), 110.0);

		QVERIFY(spy.wait(5000));
		// The last range sent while loading.
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), localMs(21, 3));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), localMs(21, 4));

		// A range for a trip replaced before it loaded isn't applied to the next.
		panel.setDataset(trip(22));
		panel.setVisibleRange(3, 4);
		panel.setDataset(trip(23));
		QVERIFY(spy.wait(5000));
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), localMs(23, 0));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), localMs(23, 9));

		// The full range, sent while loading or after (Leaflet's second
		// event), is what loads: not drawn again.
		QLineSeries* line = root->findChildren<QLineSeries*>().value(0);
		QVERIFY(line);
		QSignalSpy replaced(line, &QXYSeries::pointsReplaced);
		panel.setDataset(trip(19));
		panel.setVisibleRange(-1, -1);
		QVERIFY(spy.wait(5000));
		panel.setVisibleRange(-1, -1);
		QCOMPARE(replaced.count(), 1);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), localMs(19, 0));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), localMs(19, 9));
	}
};

QTEST_MAIN(TstChartsPanel)
#include "tst_charts_panel.moc"
