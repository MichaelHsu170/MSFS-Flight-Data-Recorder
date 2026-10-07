// Charts panel widget (charts_panel.cpp): the QQuickWidget wrapper around
// the QML chart surface -- setDataset()/setVisibleRange()/setCursorIndex()/
// valueAt() driving the real QML series objects, not the pure chart-data math
// (already covered standalone in tst_chart_data.cpp).
#include "charts_panel.h"
#include "local_time_zone.h"

#include <QDateTimeAxis>
#include <QJSValue>
#include <QLineSeries>
#include <QQmlListReference>
#include <QQuickWidget>
#include <QQuickItem>
#include <QSignalSpy>
#include <QTimeZone>
#include <QValueAxis>
#include <QtTest>

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace {

// Shows the panel and waits for its QML root to load; nullptr if it doesn't.
QQuickItem* shownRoot(ChartsPanel& panel, QSize size = QSize(400, 300)) {
	panel.resize(size);
	panel.show();
	QQuickWidget* view = panel.findChild<QQuickWidget*>();
	return QTest::qWaitFor([view]() { return view->rootObject() != nullptr; }, 5000) ? view->rootObject() : nullptr;
}

// Makes p an aircraft of engineType whose engine i records values[i-1] as
// fieldA and fieldB, its other engine values not recorded (NaN).
void setPower(TripSamplePoint& p, int engineType, TripEngineField fieldA, TripEngineField fieldB,
	const std::vector<std::pair<double, double>>& values) {
	p.engineType = engineType;
	p.engineValues.assign(values.size() * TRIP_ENGINE_FIELD_COUNT, std::nan(""));
	for (size_t i = 0; i < values.size(); ++i) {
		p.engineValues[i * TRIP_ENGINE_FIELD_COUNT + fieldA] = values[i].first;
		p.engineValues[i * TRIP_ENGINE_FIELD_COUNT + fieldB] = values[i].second;
	}
}

// A twin jet with N1 and N2 of base.
TripSamplePoint samplePoint(double base, const QString& zulu) {
	TripSamplePoint p;
	p.zuluTime = zulu;
	setPower(p, 1, TRIP_ENGINE_turb_eng_n1, TRIP_ENGINE_turb_eng_n2, { { base, base }, { base, base } });
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

// A one-sample trip at hour:00:00 of a jet with count engines, each at N1
// and N2 of 50.
TripDataset jetTrip(int hour, int count) {
	TripDataset dataset;
	dataset.points = { samplePoint(50, zuluAt(hour, 0)) };
	setPower(dataset.points[0], 1, TRIP_ENGINE_turb_eng_n1, TRIP_ENGINE_turb_eng_n2,
		std::vector<std::pair<double, double>>(count, { 50, 50 }));
	return dataset;
}

// The engine power chart's lines that are drawn, e.g. { "engN1_1", "engN2_1" }.
QStringList visibleEngineLines(QObject* root) {
	QStringList names;
	for (const QString& prefix : { QStringLiteral("engN1_"), QStringLiteral("engN2_") })
		for (int i = 1; i <= SIM_ENGINE_INDEXES; ++i) {
			const QString name = prefix + QString::number(i);
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

	// Qt Graphs warns when a series re-adds an axis to a chart it already
	// belongs to: an axis wired to a second chart by mistake.
	void constructsAndLoadsAQmlRoot() {
		QTest::failOnWarning(QRegularExpression(QStringLiteral("axis already associated with")));
		ChartsPanel panel;
		QVERIFY(shownRoot(panel));
	}

	// Every series ChartsPanel drives is a line in the loaded QML, and the
	// engine power chart's lines are in the order its legend lists them
	// (engineSeries() in charts_panel.qml): all N1 lines by engine, then N2.
	void everyChartSeriesIsALineInTheQml() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);
		for (const ChartSeriesDef& def : CHART_SERIES)
			QVERIFY2(root->findChild<QLineSeries*>(def.objectName), qPrintable(def.objectName));
		QQmlListReference lines(root->findChild<QObject*>(QStringLiteral("engineBlock")), "lineSeries");
		QVERIFY(lines.isValid());
		QStringList names;
		for (qsizetype i = 0; i < lines.count(); ++i)
			names << lines.at(i)->objectName();
		QStringList expected;
		for (const char* line : { "engN1_", "engN2_" })
			for (int i = 1; i <= SIM_ENGINE_INDEXES; ++i)
				expected << QLatin1String(line) + QString::number(i) + QStringLiteral("Series");
		QCOMPARE(names, expected);
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
		QCOMPARE(shownAxes(root), 18);  // each chart's X and Y
		QCOMPARE(shownLegends(root), 9);
		QVERIFY(root->findChild<QQuickItem*>(QStringLiteral("endOfTrajectoryLine"))->isVisible());
		// Both points in each of the 21 lines: the twin's 4 engine lines and
		// the 17 others; the other engines' lines are empty.
		QCOMPARE(pointsInLines(root), 2 * 21);

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
		QCOMPARE(pointsInLines(root), 21);
		// A one-sample trip still gets a 1 s wide axis (chart_data.h).
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), secondMs);
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), secondMs + 1000);
		QCOMPARE(panel.valueAt(secondMs)[QStringLiteral("engN1_1")].toDouble(), 2.0);
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
		QValueAxis* axis = root->findChild<QValueAxis*>(QStringLiteral("engineYAxis"));
		QVERIFY(axis);
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);

		// The first point recorded no power; the chart uses the second's. A
		// piston's N1/N2 equivalents: crankshaft RPM and prop RPM.
		TripDataset piston;
		piston.points = { samplePoint(1, QStringLiteral("2026-03-05T20:00:00.000+00:00_4")),
			samplePoint(2, QStringLiteral("2026-03-05T20:00:01.000+00:00_4")) };
		setPower(piston.points[0], 0, TRIP_ENGINE_general_eng_rpm, TRIP_ENGINE_prop_rpm, {});
		setPower(piston.points[1], 0, TRIP_ENGINE_general_eng_rpm, TRIP_ENGINE_prop_rpm, { { 2400, 2350 } });
		panel.setDataset(piston);
		QVERIFY(spy.wait(5000));
		QCOMPARE(root->property("engineSpec").toMap().value("count").toInt(), 1);
		// Sized to the data (niceAxisMax()), unlike a jet's fixed 110 %.
		QCOMPARE(axis->max(), 3000.0);
		QCOMPARE(visibleEngineLines(root), (QStringList{ "engN1_1", "engN2_1" }));
		QCOMPARE(engineLegend(root), (QStringList{ "RPM #1", "Prop RPM #1" }));
		QCOMPARE(axis->titleText(), QStringLiteral("RPM"));
		QCOMPARE(engineEmptyText(root), QString());
		QCOMPARE(panel.valueAt(chartTimeMs(piston.points[1].zuluTime))[QStringLiteral("engN2_1")].toDouble(), 2350.0);

		// A piston trip from before prop RPM was recorded: its RPM line only.
		TripDataset oldPiston = piston;
		oldPiston.points[1].engineValues[TRIP_ENGINE_prop_rpm] = std::nan("");
		panel.setDataset(oldPiston);
		QVERIFY(spy.wait(5000));
		QCOMPARE(visibleEngineLines(root), QStringList{ "engN1_1" });
		QCOMPARE(engineLegend(root), QStringList{ "RPM #1" });

		const TripDataset jet = jetTrip(21, 2);
		panel.setDataset(jet);
		QVERIFY(spy.wait(5000));
		QCOMPARE(axis->max(), 110.0);
		QCOMPARE(visibleEngineLines(root), (QStringList{ "engN1_1", "engN1_2", "engN2_1", "engN2_2" }));
		QCOMPARE(engineLegend(root), (QStringList{ "N1 #1", "N1 #2", "N2 #1", "N2 #2" }));
		QCOMPARE(axis->titleText(), QStringLiteral("N1 / N2 (%)"));

		// An overspeed past the fixed 110, of N1 or of N2 (one shared axis),
		// sizes the axis to the data (115 * 1.25 = 143.75, in steps of 50); at
		// exactly 110 it still fits.
		for (const auto& [n1, n2, max] : { std::tuple{ 115.0, 50.0, 150.0 }, std::tuple{ 50.0, 115.0, 150.0 },
				std::tuple{ 110.0, 110.0, 110.0 } }) {
			TripDataset overspeed = jet;
			setPower(overspeed.points[0], 1, TRIP_ENGINE_turb_eng_n1, TRIP_ENGINE_turb_eng_n2, { { 50, 50 }, { n1, n2 } });
			panel.setDataset(overspeed);
			QVERIFY(spy.wait(5000));
			QCOMPARE(axis->max(), max);
		}

		// An old turboprop trip recorded prop RPM and torque only, neither of
		// them an N1/N2: the no-data message.
		TripDataset oldTurboprop;
		oldTurboprop.points = { samplePoint(1, QStringLiteral("2026-03-05T21:30:00.000+00:00_4")) };
		setPower(oldTurboprop.points[0], 5, TRIP_ENGINE_prop_rpm, TRIP_ENGINE_turb_eng_max_torque_percent, { { 1700, 80 } });
		panel.setDataset(oldTurboprop);
		QVERIFY(spy.wait(5000));
		QCOMPARE(root->property("engineSpec").toMap(), (QVariantMap{ { "count", 0 } }));
		QCOMPARE(engineEmptyText(root), QStringLiteral("No engine power data recorded"));

		// No engine type recorded: the no-data message too.
		TripDataset none;
		none.points = { samplePoint(1, QStringLiteral("2026-03-05T22:00:00.000+00:00_4")) };
		none.points[0].engineType = -1;
		none.points[0].engineValues.clear();
		panel.setDataset(none);
		QVERIFY(spy.wait(5000));
		QCOMPARE(root->property("engineSpec").toMap(), (QVariantMap{ { "count", 0 } }));
		QCOMPARE(engineEmptyText(root), QStringLiteral("No engine power data recorded"));
		QCOMPARE(visibleEngineLines(root), QStringList());
		QCOMPARE(engineLegend(root), QStringList());
		QCOMPARE(axis->titleText(), QStringLiteral("Engine Power"));
		// Not the jet's 110 left over: niceAxisMax(0) like any empty axis.
		QCOMPARE(axis->max(), 1.0);

		// Deselect after a jet: no trip, so no engine lines or labels, and the
		// no-trip message instead of the no-power one.
		panel.setDataset(jet);
		QVERIFY(spy.wait(5000));
		panel.setDataset(TripDataset());
		QVERIFY(!root->property("engineSpec").isValid());
		QCOMPARE(engineEmptyText(root), QStringLiteral("No trip selected"));
		QCOMPARE(visibleEngineLines(root), QStringList());
		QCOMPARE(engineLegend(root), QStringList());
	}

	// Every engine the trip has gets its N1 and N2 lines, up to the 16 engine
	// indexes MSFS reports.
	void theEnginePowerChartShowsEveryEngine() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		for (int count : { 6, SIM_ENGINE_INDEXES }) {
			panel.setDataset(jetTrip(22, count));
			QVERIFY(spy.wait(5000));
			QStringList lines, legend;
			for (const char* line : { "N1", "N2" })
				for (int i = 1; i <= count; ++i) {
					lines << QStringLiteral("eng%1_%2").arg(QLatin1String(line)).arg(i);
					legend << QStringLiteral("%1 #%2").arg(QLatin1String(line)).arg(i);
				}
			QCOMPARE(visibleEngineLines(root), lines);
			QCOMPARE(engineLegend(root), legend);
			QCOMPARE(panel.valueAt(chartTimeMs(zuluAt(22, 0)))[QStringLiteral("engN2_%1").arg(count)].toDouble(), 50.0);
		}
	}

	// A trip with fewer engines than the last one leaves none of the extra
	// engines' points behind.
	void aTripWithFewerEnginesClearsTheExtraEnginesLines() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(jetTrip(23, 4));
		QVERIFY(spy.wait(5000));
		QCOMPARE(pointsInLines(root), 8 + 17);
		panel.setDataset(jetTrip(23, 2));
		QVERIFY(spy.wait(5000));
		QCOMPARE(pointsInLines(root), 4 + 17);
		QCOMPARE(root->findChild<QLineSeries*>(QStringLiteral("engN1_3Series"))->count(), 0);
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
		// The engine axis goes through the same rule: samplePoint's jet has a
		// fixed 0-110 % scale, which keeps its automatic ticks.
		QCOMPARE(leftAxisLabels(panel, root->findChild<QValueAxis*>(QStringLiteral("engineYAxis"))),
			QStringList({ "100", "80", "60", "40", "20", "0" }));
	}

	// One time lines up across all the charts.
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
		QCOMPARE(v[QStringLiteral("engN1_1")].toDouble(), 9.0);
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
		QCOMPARE(pointsInLines(root), 4 * 21);
		panel.setVisibleRange(-1, -1); // zoomed all the way out: the whole trip again
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), utcMsAt(18, 0));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), utcMsAt(18, 9));
		QCOMPARE(pointsInLines(root), 10 * 21);
		QCOMPARE(altAxis->max(), fullAltMax);
		QVERIFY(root->property("isFullRangeVisible").toBool());
	}

	void zoomingOutOfAOneSampleTripKeepsTheTimeAxisOneSecondWide() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);
		QDateTimeAxis* timeAxis = root->findChild<QDateTimeAxis*>(QStringLiteral("sharedXAxis"));
		QVERIFY(timeAxis);

		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		TripDataset trip;
		trip.points = { samplePoint(1, QStringLiteral("2026-03-05T12:00:00.000+00:00_4")) };
		panel.setDataset(trip);
		QVERIFY(spy.wait(5000));
		const qint64 sampleMs = QDateTime(QDate(2026, 3, 5), QTime(12, 0, 0), QTimeZone::UTC).toMSecsSinceEpoch();

		panel.setVisibleRange(0, 0);
		panel.setVisibleRange(-1, -1);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), sampleMs);
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), sampleMs + 1000);
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
		QLineSeries* line = root->findChild<QLineSeries*>(QStringLiteral("altitudeSeries"));
		QVERIFY(line);
		QSignalSpy replaced(line, &QXYSeries::pointsReplaced);
		panel.setVisibleRange(2, 5);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), utcMsAt(19, 2));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), utcMsAt(19, 5));
		QCOMPARE(pointsInLines(root), 4 * 21);
		QVERIFY(altAxis->max() < fullAltMax);
		QVERIFY(!root->property("isFullRangeVisible").toBool());
		QCOMPARE(replaced.count(), 1);

		panel.setVisibleRange(2, 5); // Leaflet's zoomend+moveend duplicate: not drawn again
		QCOMPARE(replaced.count(), 1);

		// A single-sample slice is widened by a second so the axis isn't empty.
		panel.setVisibleRange(3, 3);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), utcMsAt(19, 3));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), utcMsAt(19, 4));
		QCOMPARE(pointsInLines(root), 21);
	}

	// While a trip loads, the old trip's lines stay: a range (which indexes
	// the new trip) leaves their axes alone, and the last one applies once
	// loaded.
	void setVisibleRangeWaitsForALoadingTrip() {
		ChartsPanel panel;
		QQuickItem* root = shownRoot(panel);
		QVERIFY(root);
		QDateTimeAxis* timeAxis = root->findChild<QDateTimeAxis*>(QStringLiteral("sharedXAxis"));
		QValueAxis* engineAxis = root->findChild<QValueAxis*>(QStringLiteral("engineYAxis"));
		QVERIFY(timeAxis && engineAxis);

		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(tripAt(20));
		QVERIFY(spy.wait(5000));
		panel.setVisibleRange(2, 5);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), utcMsAt(20, 2));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), utcMsAt(20, 5));
		QCOMPARE(engineAxis->max(), 110.0);  // the jet's fixed N1/N2 axis

		panel.setDataset(tripAt(21));
		panel.setVisibleRange(-1, -1);
		panel.setVisibleRange(3, 4);
		QCOMPARE(timeAxis->min().toMSecsSinceEpoch(), utcMsAt(20, 2));
		QCOMPARE(timeAxis->max().toMSecsSinceEpoch(), utcMsAt(20, 5));
		QCOMPARE(engineAxis->max(), 110.0);

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
		QLineSeries* line = root->findChild<QLineSeries*>(QStringLiteral("altitudeSeries"));
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
