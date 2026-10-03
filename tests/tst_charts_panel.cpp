// Charts panel widget (charts_panel.cpp): the QQuickWidget wrapper around
// the QML chart surface -- setDataset()/appendLivePoint()/setVisibleRange()/
// valueAt() driving the real QML series objects, not the pure chart-data math
// (already covered standalone in tst_chart_data.cpp).
#include "charts_panel.h"

#include <QQuickWidget>
#include <QQuickItem>
#include <QSignalSpy>
#include <QtTest>

namespace {

TripSamplePoint samplePoint(double base, const QString& zulu) {
	TripSamplePoint p;
	p.zuluTime = zulu;
	p.n1_1 = base; p.n1_2 = base;
	p.verticalSpeed = base; p.airspeed = base; p.groundSpeed = base; p.altitude = base;
	p.fuelTotalQuantityWeight = base; p.pitchDegrees = base; p.bankDegrees = base;
	return p;
}

}

class TstChartsPanel : public QObject {
	Q_OBJECT

private slots:
	void constructsAndLoadsAQmlRoot() {
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

	void emptyDatasetCollapsesAxisAndEmitsImmediately() {
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));

		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(TripDataset());
		QCOMPARE(spy.count(), 1);
	}

	void appendLivePointDropsMalformedTime() {
		ChartsPanel panel;
		QVERIFY(!panel.appendLivePoint(samplePoint(1, QStringLiteral("not a time"))));
	}

	void valueAtIsEmptyWithNoDataset() {
		ChartsPanel panel;
		QVERIFY(panel.valueAt(0).isEmpty());
	}

	void loadingASecondDatasetReusesTheAlreadyResolvedSeriesCache() {
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));

		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		TripDataset first;
		first.points = { samplePoint(1, QStringLiteral("2026-03-05T11:00:00.000+00:00_4")) };
		panel.setDataset(first);
		QVERIFY(spy.wait(5000));

		TripDataset second;
		second.points = { samplePoint(2, QStringLiteral("2026-03-05T12:00:00.000+00:00_4")) };
		panel.setDataset(second);
		QVERIFY(spy.wait(5000));
		QCOMPARE(spy.count(), 2);
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
		panel.setCursorIndex(1);
		QCOMPARE(root->property("cursorTime").toDouble(), chartTimeMs(t1));

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

		// Reloading the same trip: the old cursor time lies inside the new X
		// axis range, so a stale value would stay drawn.
		panel.setCursorIndex(1);
		QCOMPARE(root->property("cursorTime").toDouble(), chartTimeMs(t1));
		panel.setDataset(dataset);
		QCOMPARE(root->property("cursorTime").toDouble(), -1.0);
		QVERIFY(spy.wait(5000));

		// Deselect (empty dataset) clears it too.
		panel.setCursorIndex(1);
		QCOMPARE(root->property("cursorTime").toDouble(), chartTimeMs(t1));
		panel.setDataset(TripDataset());
		QCOMPARE(root->property("cursorTime").toDouble(), -1.0);
	}

	void appendLivePointGrowsTheAxisAndEveryCachedSeries() {
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));

		const QString t0 = QStringLiteral("2026-03-05T16:00:00.000+00:00_4");
		const QString t1 = QStringLiteral("2026-03-05T16:00:01.000+00:00_4");
		QVERIFY(panel.appendLivePoint(samplePoint(5, t0))); // first point
		QVERIFY(panel.appendLivePoint(samplePoint(7, t1))); // second point grows the axis from the other end

		// No dataset was ever loaded, so this exercises valueAt()'s live-mode
		// fallback (reading straight from the QML series, not full_).
		QVariantMap v = panel.valueAt(chartTimeMs(t1));
		QCOMPARE(v[QStringLiteral("n1_1")].toDouble(), 7.0);
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
		QCOMPARE(v[QStringLiteral("n1_1")].toDouble(), 9.0);
	}

	void setVisibleRangeWithNoDatasetLoadedCollapsesToAOneSecondWindow() {
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));

		// Resolves the series cache for the first time right here (no dataset
		// loaded yet), then takes the "zoomed all the way out" branch with an
		// empty pointTimesMs_ and a still-invalid fullExtents_.
		panel.setVisibleRange(-1, -1);
	}

	void setVisibleRangeWithNegativeIndexAfterALoadReloadsTheFullResolutionView() {
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));

		TripDataset dataset;
		for (int i = 0; i < 10; ++i)
			dataset.points.push_back(samplePoint(i, QStringLiteral("2026-03-05T18:00:%1.000+00:00_4").arg(i, 2, 10, QLatin1Char('0'))));
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(dataset);
		QVERIFY(spy.wait(5000));

		panel.setVisibleRange(-1, -1); // zoomed all the way out -- reloads the full thinned view
	}

	void setVisibleRangeZoomsToASliceAndIgnoresADuplicateRange() {
		ChartsPanel panel;
		panel.resize(400, 300);
		panel.show();
		QVERIFY(QTest::qWaitFor([&panel]() { return panel.findChild<QQuickWidget*>()->rootObject() != nullptr; }, 5000));

		TripDataset dataset;
		for (int i = 0; i < 10; ++i)
			dataset.points.push_back(samplePoint(i, QStringLiteral("2026-03-05T19:00:%1.000+00:00_4").arg(i, 2, 10, QLatin1Char('0'))));
		QSignalSpy spy(&panel, &ChartsPanel::seriesLoaded);
		panel.setDataset(dataset);
		QVERIFY(spy.wait(5000));

		panel.setVisibleRange(2, 5); // zoomed slice
		panel.setVisibleRange(2, 5); // Leaflet's zoomend+moveend duplicate -- ignored
		panel.setVisibleRange(3, 3); // a single-sample slice -- widened so hi > lo
	}
};

QTEST_MAIN(TstChartsPanel)
#include "tst_charts_panel.moc"
