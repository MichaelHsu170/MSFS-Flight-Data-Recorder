// Trajectory view (trajectory_view.cpp): the composite map+charts+data-table
// widget -- setDataset()/clearAndShowOverview()'s fan-out to the three
// sub-panels, the pendingRenders_ bookkeeping that turns two async subview
// loads into one renderingFinished(), live-point buffering while unpinned,
// and the two splitter-width wires (map/table and map+table/charts).
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
#include "data_table_panel.h"

#include <QApplication>
#include <QCoreApplication>
#include <QMouseEvent>
#include <QSignalSpy>
#include <QSplitter>
#include <QTableWidget>
#include <QtTest>

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
		view_ = new TrajectoryView;
		view_->resize(1000, 700); // clears both splitters' minimum sizes
		view_->show();
	}

	void cleanupTestCase() {
		delete view_;
	}

	void startsWithNoTripSelected() {
		QCOMPARE(view_->currentTripId(), -1);
	}

	void setDatasetWithANullPointerIsIgnored() {
		view_->setDataset(nullptr);
		QCOMPARE(view_->currentTripId(), -1); // ignored, not "loaded a trip with id -1"
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
		QCOMPARE(view_->currentTripId(), 7);

		QVERIFY(spy.wait(15000)); // both chartsPanel_ and mapWidget_ must finish before this fires
		QCOMPARE(spy.count(), 1);
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
		QCOMPARE(view_->currentTripId(), -1);
		QTest::qWait(300); // give mapWidget_'s background trajectory-clear a chance to fire trajectoryLoaded too
		QCOMPARE(spy.count(), 0);
		QCOMPARE(zuluShown(*view_), QString());
	}

	void resetZoomRunsAgainstTheMapAndChartsWithoutCrashing() {
		view_->resetZoom();
	}

	void appendLivePointInFollowModeAppendsToTheDatasetAndEveryPanel() {
		auto dataset = std::make_shared<TripDataset>();
		dataset->tripId = 9;
		dataset->points = { samplePoint(1, 2, QStringLiteral("2026-04-01T11:00:00.000+00:00_9")) };
		QSignalSpy spy(view_, &TrajectoryView::renderingFinished);
		view_->setDataset(dataset);
		QVERIFY(spy.wait(15000));

		view_->appendLivePoint(samplePoint(5, 6, QStringLiteral("2026-04-01T11:00:01.000+00:00_9")));
		QCOMPARE(dataset->points.size(), size_t(2)); // TrajectoryView shares this shared_ptr, not a copy
		QCOMPARE(zuluShown(*view_), QStringLiteral("2026-04-01T11:00:01.000+00:00_9"));
	}

	void appendLivePointDroppedByChartsPanelIsSkippedEverywhere() {
		auto dataset = std::make_shared<TripDataset>();
		dataset->tripId = 10;
		dataset->points = { samplePoint(1, 2, QStringLiteral("2026-04-01T12:00:00.000+00:00_10")) };
		QSignalSpy spy(view_, &TrajectoryView::renderingFinished);
		view_->setDataset(dataset);
		QVERIFY(spy.wait(15000));

		// A malformed zuluTime makes ChartsPanel::appendLivePoint() return
		// false; TrajectoryView must then skip the dataset/map/table too, or
		// their point counts would desync from the charts' by one.
		view_->appendLivePoint(samplePoint(9, 9, QStringLiteral("not a time")));
		QCOMPARE(dataset->points.size(), size_t(1));
		QCOMPARE(zuluShown(*view_), QStringLiteral("2026-04-01T12:00:00.000+00:00_10"));
	}

	void appendLivePointWhileNotFollowingIsBufferedThenFlushedBySetLiveFollowTrue() {
		auto dataset = std::make_shared<TripDataset>();
		dataset->tripId = 11;
		dataset->points = { samplePoint(1, 2, QStringLiteral("2026-04-01T13:00:00.000+00:00_11")) };
		QSignalSpy spy(view_, &TrajectoryView::renderingFinished);
		view_->setDataset(dataset);
		QVERIFY(spy.wait(15000));

		view_->setLiveFollow(false);
		view_->appendLivePoint(samplePoint(5, 6, QStringLiteral("2026-04-01T13:00:01.000+00:00_11")));
		QCOMPARE(dataset->points.size(), size_t(1)); // buffered, not yet applied
		QCOMPARE(zuluShown(*view_), QStringLiteral("2026-04-01T13:00:00.000+00:00_11"));

		view_->setLiveFollow(true); // flushes the buffered point into the dataset/panels
		QCOMPARE(dataset->points.size(), size_t(2));
		QCOMPARE(zuluShown(*view_), QStringLiteral("2026-04-01T13:00:01.000+00:00_11"));
	}

	void aBufferedPointDroppedByChartsPanelIsSkippedWhenFlushedBySetLiveFollowTrue() {
		auto dataset = std::make_shared<TripDataset>();
		dataset->tripId = 12;
		dataset->points = { samplePoint(1, 2, QStringLiteral("2026-04-01T14:00:00.000+00:00_12")) };
		QSignalSpy spy(view_, &TrajectoryView::renderingFinished);
		view_->setDataset(dataset);
		QVERIFY(spy.wait(15000));

		view_->setLiveFollow(false);
		// Malformed zuluTime: buffered now, and must still be dropped (not
		// applied to the dataset/map/table) once the flush loop below reaches it.
		view_->appendLivePoint(samplePoint(9, 9, QStringLiteral("not a time")));
		view_->setLiveFollow(true);
		QCOMPARE(dataset->points.size(), size_t(1));
		QCOMPARE(zuluShown(*view_), QStringLiteral("2026-04-01T14:00:00.000+00:00_12"));
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
		QSignalSpy spy(view_, &TrajectoryView::rightPanelWidthChanged);
		emit splitter->splitterMoved(rightWidth, 1);
		QVERIFY(spy.count() >= 1);
		QCOMPARE(spy.last().at(0).toInt(), rightWidth);

		const QMouseEvent release(QEvent::MouseButtonRelease, QPointF(5, 5), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
		QCoreApplication::sendEvent(splitter->handle(1), const_cast<QMouseEvent*>(&release));
		QCOMPARE(AppSettings::instance().rightPanelWidth(), rightWidth);
	}

	void draggingTheMainSplitterHandlePersistsTheChartsPanelHeightOnRelease() {
		QSplitter* splitter = splitterWithOrientation(*view_, Qt::Vertical);
		QVERIFY(splitter);
		splitter->setSizes({ splitter->height() - 250, 250 });
		const int chartsHeight = splitter->sizes().last(); // setSizes() can clamp/round; read back what actually landed

		const QMouseEvent release(QEvent::MouseButtonRelease, QPointF(5, 5), Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
		QCoreApplication::sendEvent(splitter->handle(1), const_cast<QMouseEvent*>(&release));
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
