// Trajectory view (trajectory_view.cpp): the composite map+charts+data-table
// widget -- setDataset()/clearAndShowOverview()'s fan-out to the three
// sub-panels, the pendingRenders_ bookkeeping that turns two async subview
// loads into one renderingFinished(), the map's cursor, visible range and
// overview clicks passed on, and the two splitter-width wires (map/table and
// map+table/charts).
//
// Needs a custom main() and no QT_QPA_PLATFORM=offscreen, exactly like
// tst_map_widget.cpp: this widget owns a MapWidget (QWebEngineView)
// internally. See that file's header comment for why. Also like that file,
// everything below shares ONE TrajectoryView (built in initTestCase(),
// reused by every slot in declaration order) rather than one per test --
// in this sandboxed environment only the first QWebEngineView-backed widget
// built in a process reliably finishes loading.
#include "trajectory_view.h"
#include "app_settings.h"
#include "charts_panel.h"
#include "data_table_panel.h"
#include "map_widget.h"
#include "test_support.h"

#include <QApplication>
#include <QQuickItem>
#include <QQuickWidget>
#include <QSignalSpy>
#include <QSplitter>
#include <QTableWidget>
#include <QtTest>

using namespace TestSupport;

namespace {

TripSamplePoint samplePoint(double lat, double lon, const QString& zulu) {
	TripSamplePoint p;
	p.latitude = lat;
	p.longitude = lon;
	p.zuluTime = zulu;
	p.engine = { 1, 1, { (float)lat } };
	return p;
}

QTableWidget* dataTable(TrajectoryView& view) {
	return view.findChild<DataTablePanel*>()->findChild<QTableWidget*>();
}

QString zuluShown(TrajectoryView& view) {
	QTableWidget* t = dataTable(view);
	for (int r = 0; r < t->rowCount(); ++r)
		if (t->item(r, 0)->text() == QStringLiteral("Time (Zulu)"))
			return t->item(r, 1)->text();
	return QStringLiteral("<no Time (Zulu) row>");
}

// The two QSplitters are told apart by orientation: mapTableSplitter_ (map +
// data table side by side) is horizontal, the outer one (that row + charts
// stacked) is vertical.
QSplitter* splitterWithOrientation(TrajectoryView& view, Qt::Orientation orientation) {
	for (QSplitter* s : view.findChildren<QSplitter*>())
		if (s->orientation() == orientation)
			return s;
	return nullptr;
}

}

class TstTrajectoryView : public QObject {
	Q_OBJECT

	TrajectoryView* view_ = nullptr;

private slots:
	void initTestCase() {
		isolateFiles();
		view_ = new TrajectoryView;
		view_->resize(1000, 700); // clears both splitters' minimum sizes
		view_->show();
	}

	void cleanupTestCase() {
		delete view_;
	}

	void setDatasetPushesToEveryPanelAndEmitsRenderingFinishedOnceBothSubviewsLoad() {
		auto dataset = std::make_shared<TripDataset>();
		dataset->tripId = 7;
		dataset->points = {
			samplePoint(1, 2, QStringLiteral("2026-04-01T10:00:00.000+00:00_7")),
			samplePoint(3, 4, QStringLiteral("2026-04-01T10:00:01.000+00:00_7")),
		};

		QSignalSpy spy(view_, &TrajectoryView::renderingFinished);
		view_->setDataset(dataset);

		QVERIFY(spy.wait(15000)); // both chartsPanel_ and mapWidget_ must finish before this fires
		QCOMPARE(spy.count(), 1);
		QCOMPARE(zuluShown(*view_), QStringLiteral("2026-04-01T10:00:01.000+00:00_7"));
	}

	// Runs right after the load above: the null dataset changes nothing and
	// starts no render.
	void setDatasetWithANullPointerKeepsTheShownTrip() {
		QSignalSpy spy(view_, &TrajectoryView::renderingFinished);
		view_->setDataset(nullptr);
		QTest::qWait(300);
		QCOMPARE(spy.count(), 0);
		QCOMPARE(zuluShown(*view_), QStringLiteral("2026-04-01T10:00:01.000+00:00_7"));
	}

	void clearAndShowOverviewClearsTheTripAndDoesNotSpuriouslyEmitRenderingFinished() {
		// chartsPanel_->setDataset(emptyDataset) takes its synchronous
		// early-return path and emits seriesLoaded() immediately while
		// pendingRenders_ is already 0 (set just above it) -- onSubviewLoaded()'s
		// "pendingRenders_ <= 0" guard must swallow that, or this would
		// spuriously fire renderingFinished() on every overview reset.
		QSignalSpy spy(view_, &TrajectoryView::renderingFinished);
		view_->clearAndShowOverview({});
		QTest::qWait(300); // give mapWidget_'s background trajectory-clear a chance to fire trajectoryLoaded too
		QCOMPARE(spy.count(), 0);
		QCOMPARE(zuluShown(*view_), QString());
	}

	void resetZoomRefitsTheMapAndShowsTheChartsFullRange() {
		auto dataset = std::make_shared<TripDataset>();
		dataset->tripId = 8;
		dataset->points = {
			samplePoint(10, 20, QStringLiteral("2026-04-01T10:30:00.000+00:00_8")),
			samplePoint(10.5, 20.5, QStringLiteral("2026-04-01T10:30:01.000+00:00_8")),
			samplePoint(11, 21, QStringLiteral("2026-04-01T10:30:02.000+00:00_8")),
		};
		QSignalSpy spy(view_, &TrajectoryView::renderingFinished);
		view_->setDataset(dataset);
		QVERIFY(spy.wait(15000));
		QTest::qWait(1500); // let the map's animated fit (0.75 s) settle
		const int fitted = mapZoom(view_);
		QVERIFY(fitted > 2);

		// Zoom both out of their full view: the map directly on the page (its
		// range report reaches the charts), then the charts to a slice once
		// that report is in, so it can't land afterwards and undo the slice.
		QSignalSpy rangeReported(view_->findChild<MapWidget*>(), &MapWidget::visibleRangeChanged);
		evalPageJs(view_, QStringLiteral("leafletMapInstance.setZoom(2, {animate: false})"));
		// One report each for zoomend and moveend; they may already be in, as
		// evalPageJs() runs the event loop.
		QTRY_COMPARE_WITH_TIMEOUT(rangeReported.count(), 2, 5000);
		ChartsPanel* charts = view_->findChild<ChartsPanel*>();
		QQuickItem* chartsRoot = charts->findChild<QQuickWidget*>()->rootObject();
		charts->setVisibleRange(0, 1);
		QCOMPARE(chartsRoot->property("isFullRangeVisible").toBool(), false);

		view_->resetZoom();
		QCOMPARE(chartsRoot->property("isFullRangeVisible").toBool(), true);
		QTRY_COMPARE_WITH_TIMEOUT(mapZoom(view_), fitted, 5000);
	}

	// Runs on the trip loaded just above. The map's cursor (a click on the
	// line, or the marker dragged) moves the charts' cursor and the data
	// table's sample; the map's visible range zooms the charts. Emitted
	// straight from MapWidget: tst_map_widget covers the page producing them.
	void theMapsCursorAndRangeReachTheChartsAndTheDataTable() {
		MapWidget* map = view_->findChild<MapWidget*>();
		QQuickItem* chartsRoot = view_->findChild<ChartsPanel*>()->findChild<QQuickWidget*>()->rootObject();

		emit map->cursorIndexChanged(1);
		QCOMPARE(zuluShown(*view_), QStringLiteral("2026-04-01T10:30:01.000+00:00_8"));
		QCOMPARE(chartsRoot->property("cursorTime").toDouble(), 1775039401000.0); // 2026-04-01 10:30:01 UTC

		emit map->visibleRangeChanged(0, 1);
		QCOMPARE(chartsRoot->property("isFullRangeVisible").toBool(), false);
		emit map->visibleRangeChanged(-1, -1);
		QCOMPARE(chartsRoot->property("isFullRangeVisible").toBool(), true);
	}

	// A click on a trip's route on the overview map is passed on, for Trip
	// History to open that trip (MainWindow connects the two).
	void anOverviewRouteClickIsPassedOn() {
		QSignalSpy clicked(view_, &TrajectoryView::overviewTripClicked);
		emit view_->findChild<MapWidget*>()->overviewTripClicked(7);
		QCOMPARE(clicked.count(), 1);
		QCOMPARE(clicked.at(0).at(0).toInt(), 7);
	}

	void setRightPanelWidthResizesTheMapTableSplitter() {
		QSplitter* splitter = splitterWithOrientation(*view_, Qt::Horizontal);
		QVERIFY(splitter);
		view_->setRightPanelWidth(300);
		QCOMPARE(splitter->sizes().last(), 300);
	}

	// moveSplitter() itself is protected (QSplitter's own concern, not
	// TrajectoryView's); splitterMoved is a public signal, so it's emitted
	// directly here to drive TrajectoryView's lambda the same way a real drag
	// would, without needing a window-system-level synthetic drag.
	void draggingTheMapTableSplitterHandleEmitsRightPanelWidthChangedAndPersistsOnRelease() {
		QSplitter* splitter = splitterWithOrientation(*view_, Qt::Horizontal);
		QVERIFY(splitter);
		splitter->setSizes({ splitter->width() - 220, 220 });
		const int rightWidth = splitter->sizes().last(); // setSizes() can clamp/round; read back what actually landed
		const int savedWidth = AppSettings::instance().rightPanelWidth();
		QVERIFY(savedWidth != rightWidth);
		QSignalSpy spy(view_, &TrajectoryView::rightPanelWidthChanged);
		emit splitter->splitterMoved(rightWidth, 1);
		QVERIFY(spy.count() >= 1);
		QCOMPARE(spy.last().at(0).toInt(), rightWidth);
		QCOMPARE(AppSettings::instance().rightPanelWidth(), savedWidth); // not saved mid-drag

		sendLeftButton(splitter->handle(1), QEvent::MouseButtonRelease, QPoint(5, 5));
		QCOMPARE(AppSettings::instance().rightPanelWidth(), rightWidth);
	}

	void draggingTheMainSplitterHandlePersistsTheChartsPanelHeightOnRelease() {
		QSplitter* splitter = splitterWithOrientation(*view_, Qt::Vertical);
		QVERIFY(splitter);
		splitter->setSizes({ splitter->height() - 250, 250 });
		const int chartsHeight = splitter->sizes().last(); // setSizes() can clamp/round; read back what actually landed

		sendLeftButton(splitter->handle(1), QEvent::MouseButtonRelease, QPoint(5, 5));
		QCOMPARE(AppSettings::instance().chartsPanelHeight(), chartsHeight);
	}
};

int main(int argc, char* argv[]) {
	QApplication::setAttribute(Qt::AA_ShareOpenGLContexts, true);
	QApplication app(argc, argv);
	TstTrajectoryView tc;
	return QTest::qExec(&tc, argc, argv);
}

#include "tst_trajectory_view.moc"
