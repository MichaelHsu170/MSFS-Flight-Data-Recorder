// Map widget (map_widget.cpp): the QWebEngineView wrapper around the
// Leaflet/OSM trajectory map -- setDataset()/appendLivePoint()/showOverview()/
// resetZoom()/setEventsVisible() driving the real page, not the pure JS
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
#include "map_widget.h"

#include <QApplication>
#include <QSignalSpy>
#include <QTimer>
#include <QtTest>

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

}

class TstMapWidget : public QObject {
	Q_OBJECT

	MapWidget* widget_ = nullptr;
	QTimer* liveTimer_ = nullptr;

private slots:
	void initTestCase() {
		widget_ = new MapWidget;
		widget_->resize(400, 300);
		widget_->show();
		liveTimer_ = widget_->findChild<QTimer*>();
		QVERIFY(liveTimer_);
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
		QVERIFY(!liveTimer_->isActive());
		widget_->appendLivePoint(samplePoint(1, 2));
		QVERIFY(!liveTimer_->isActive()); // no dataset pushed yet: nothing to flush

		widget_->resetZoom();              // early-return branch; must not crash
		widget_->setEventsVisible(false);  // early-return branch; must not crash

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

	void afterPageReadyIsReadySetDatasetPushesEverythingAndEmitsOnce() {
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

	void appendLivePointAfterPageReadyStartsTheTimerAndFlushStopsItWhenDrained() {
		QVERIFY(!liveTimer_->isActive());
		widget_->appendLivePoint(samplePoint(5, 6));
		QVERIFY(liveTimer_->isActive());
		// flushLivePoints() fires every 250ms and stops the timer once the
		// pending-point queue is drained.
		QVERIFY(QTest::qWaitFor([this]() { return !liveTimer_->isActive(); }, 2000));
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

	void resetZoomAndSetEventsVisibleAfterPageReadyRunWithoutCrashing() {
		widget_->resetZoom();
		widget_->setEventsVisible(true);
	}
};

int main(int argc, char* argv[]) {
	QApplication::setAttribute(Qt::AA_ShareOpenGLContexts, true);
	QApplication app(argc, argv);
	TstMapWidget tc;
	return QTest::qExec(&tc, argc, argv);
}

#include "tst_map_widget.moc"
