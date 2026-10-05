#include "map_widget.h"
#include "map_bridge.h"
#include "app_settings.h"
#include "kml_export.h"
#include "map_script.h"
#include "version.h"

#include <QWebEngineView>
#include <QWebEnginePage>
#include <QWebEngineProfile>
#include <QWebEngineContextMenuRequest>
#include <QWebChannel>
#include <QVBoxLayout>
#include <QToolButton>
#include <QMenu>
#include <QContextMenuEvent>
#include <QResizeEvent>
#include <QFileDialog>
#include <QClipboard>
#include <QGuiApplication>
#include <QPixmap>
#include <QMessageBox>
#include <QUrl>
#include <QElapsedTimer>
#include <QFutureWatcher>
#include "logger.h"
#include <QtConcurrent/QtConcurrentRun>

#include <functional>
#include <vector>

namespace {

// Routes the page's JS console (including Leaflet's tileerror/tileload and
// our own console.log/error calls in map.html) to the debug log -- errors and
// warnings as WARNING, everything else as INFO -- so failures are visible
// without opening Chromium DevTools by hand.
class LoggingPage : public QWebEnginePage {
public:
	explicit LoggingPage(QObject* parent = nullptr) : QWebEnginePage(parent) {}

protected:
	void javaScriptConsoleMessage(JavaScriptConsoleMessageLevel level, const QString& message, int lineNumber, const QString& sourceID) override {
		Logger::Level logLevel = (level == ErrorMessageLevel || level == WarningMessageLevel)
		    ? Logger::Warning : Logger::Info;
		Logger::log(logLevel, "MapJS",
		    QStringLiteral("%1:%2 %3").arg(sourceID).arg(lineNumber).arg(message));
	}
};

// Trims Chromium's default context menu down to just what's useful on a map
// tile view -- drops Back/Forward (no in-page navigation happens here), Save
// Page, and View Page Source, replaces Reload with Reset Zoom (reloading
// would blank the Leaflet page rather than just refit the view), and
// replaces the stock per-tile Save/Copy image (which only captured the one
// right-clicked tile) with whole-view versions that grab the composited map
// -- trajectory, markers, and underlying tiles together -- as a single image.
// Copy is added back in when text is selected (e.g. in a liftoff/touchdown
// popup) since the trimmed-down menu would otherwise drop it entirely.
class FilteredWebEngineView : public QWebEngineView {
public:
	// exportKmlAvailable gates whether "Export to KML" is even shown -- there's
	// no single trip's trajectory to export while the map is showing the
	// departure->destination overview of every trip.
	FilteredWebEngineView(std::function<void()> resetZoom, std::function<QString()> defaultFileName,
		std::function<void()> exportKml, std::function<bool()> exportKmlAvailable, QWidget* parent)
		: QWebEngineView(parent), resetZoomHandler_(std::move(resetZoom)), defaultFileNameHandler_(std::move(defaultFileName)),
		  exportKmlHandler_(std::move(exportKml)), exportKmlAvailableHandler_(std::move(exportKmlAvailable)) {}

protected:
	void contextMenuEvent(QContextMenuEvent* event) override {
		QWebEngineContextMenuRequest* request = lastContextMenuRequest();
		if (!request)
			return;

		auto* menu = new QMenu(this);
		menu->setAttribute(Qt::WA_DeleteOnClose);

		if (!request->selectedText().isEmpty()) {
			QAction* copyAction = menu->addAction(QStringLiteral("Copy"));
			connect(copyAction, &QAction::triggered, this, [this]() {
				// Read the selection and copy it via Qt (rather than triggering the
				// page's own async Copy action) so the clear below is guaranteed to
				// run only after the text has actually been captured -- chaining
				// through runJavaScript's result callback, instead of firing two
				// independent commands back-to-back, is what makes the ordering safe.
				QWebEnginePage* webPage = page();
				webPage->runJavaScript(QStringLiteral("window.getSelection().toString()"), [webPage](const QVariant& result) {
					QGuiApplication::clipboard()->setText(result.toString());
					webPage->runJavaScript(QStringLiteral("window.getSelection().removeAllRanges();"));
				});
			});
			menu->addSeparator();
		}

		QAction* resetZoomAction = menu->addAction(QStringLiteral("Reset Zoom"));
		connect(resetZoomAction, &QAction::triggered, this, [this]() { resetZoomHandler_(); });

		menu->addSeparator();
		QAction* saveImageAction = menu->addAction(QStringLiteral("Save Image"));
		connect(saveImageAction, &QAction::triggered, this, [this]() { saveMapImage(); });
		QAction* copyImageAction = menu->addAction(QStringLiteral("Copy Image"));
		connect(copyImageAction, &QAction::triggered, this, [this]() { copyMapImage(); });
		if (exportKmlAvailableHandler_()) {
			QAction* exportKmlAction = menu->addAction(QStringLiteral("Export to KML"));
			connect(exportKmlAction, &QAction::triggered, this, [this]() { exportKmlHandler_(); });
		}

		// CopyLinkToClipboard isn't kept enabled/disabled in sync with the
		// context menu request the way navigation/edit actions are -- its
		// enabled bit is only ever flipped by Qt's own default context-menu
		// builder, which this override replaces, so isEnabled() on it stays
		// false here. Gate on the request's own linkUrl instead.
		if (!request->linkUrl().isEmpty()) {
			menu->addSeparator();
			menu->addAction(pageAction(QWebEnginePage::CopyLinkToClipboard));
		}

		menu->popup(event->globalPos());
	}

private:
	void saveMapImage() {
		const QString fileName = QFileDialog::getSaveFileName(
		    this, QStringLiteral("Save Map Image"), defaultFileNameHandler_(), QStringLiteral("PNG Image (*.png)"));
		if (!fileName.isEmpty() && !grab().save(fileName, "PNG"))
			QMessageBox::critical(this, QStringLiteral("Error"), QStringLiteral("Failed to save the map image to %1.").arg(fileName));
	}

	void copyMapImage() {
		QGuiApplication::clipboard()->setPixmap(grab());
	}

	std::function<void()> resetZoomHandler_;
	std::function<QString()> defaultFileNameHandler_;
	std::function<void()> exportKmlHandler_;
	std::function<bool()> exportKmlAvailableHandler_;
};

}

MapWidget::MapWidget(QWidget* parent) : QWidget(parent) {
	// fdr_core is a static library: Qt only auto-registers a .qrc's compiled
	// resource data when it's linked into an executable/shared library
	// directly, so map.qrc needs this explicit init here (ran from both
	// the app and any test linking fdr_core) or qrc:/map/... 404s.
	Q_INIT_RESOURCE(map);
	// OSM's tile usage policy requires a valid User-Agent identifying the
	// application -- QtWebEngine's default UA is a generic Chromium string
	// that doesn't, which tile servers can reject.
	QWebEngineProfile::defaultProfile()->setHttpUserAgent(QStringLiteral("MSFS-Flight-Data-Recorder v" APP_VERSION));

	view_ = new FilteredWebEngineView([this]() { resetZoom(); }, [this]() { return defaultMapImageFileName(); },
		[this]() { exportKml(); }, [this]() { return dataset_ != nullptr; }, this);
	view_->setPage(new LoggingPage(view_));
	channel_ = new QWebChannel(this);
	bridge_ = new MapBridge(this);

	channel_->registerObject(QStringLiteral("mapBridge"), bridge_);
	view_->page()->setWebChannel(channel_);
	// A cursor or range measured on a trajectory replaced since (the page sent
	// it before running the newer setTrajectory()) indexes another trip.
	connect(bridge_, &MapBridge::cursorIndexChanged, this, [this](int index, int version) {
		if (version != datasetVersion_) {
			Logger::logf(Logger::Trace, "Map", "cursor on superseded trajectory v%d (now v%d); ignoring", version, datasetVersion_);
			return;
		}
		lastCursorIndex_ = index;
		emit cursorIndexChanged(index);
	});
	connect(bridge_, &MapBridge::visibleRangeChanged, this, [this](int startIndex, int endIndex, int version) {
		if (version != datasetVersion_) {
			Logger::logf(Logger::Trace, "Map", "visible range of superseded trajectory v%d (now v%d); ignoring", version, datasetVersion_);
			return;
		}
		emit visibleRangeChanged(startIndex, endIndex);
	});
	connect(bridge_, &MapBridge::overviewTripClicked, this, &MapWidget::overviewTripClicked);
	connect(view_, &QWebEngineView::loadFinished, this, &MapWidget::onLoadFinished);

	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(view_);

	// Floats free over the map (not part of the layout above), so it takes no
	// row of vertical space of its own.
	eventsToggle_ = new QToolButton(this);
	eventsToggle_->setText(QStringLiteral("⚑"));
	eventsToggle_->setCheckable(true);
	eventsToggle_->setChecked(true);
	eventsToggle_->setToolTip(QStringLiteral("Show/hide cockpit event markers (gear, flaps, spoilers, etc.) on the map"));
	eventsToggle_->setFixedSize(28, 28);
	eventsToggle_->setStyleSheet(QStringLiteral(
		"QToolButton { background: rgba(255,255,255,200); border: 1px solid #888888; border-radius: 4px; font-size: 14px; }"
		"QToolButton:checked { background: rgba(120,170,255,220); border: 1px solid #3366cc; }"
		"QToolButton:hover { border: 1px solid #3366cc; }"));
	eventsToggle_->raise();
	connect(eventsToggle_, &QToolButton::toggled, this, &MapWidget::setEventsVisible);

	view_->load(QUrl(QStringLiteral("qrc:/map/map.html")));
}

void MapWidget::resizeEvent(QResizeEvent* event) {
	QWidget::resizeEvent(event);
	const int margin = 8;
	eventsToggle_->move(width() - eventsToggle_->width() - margin, margin);
}

void MapWidget::setDataset(const TripDataset& dataset) {
	++datasetVersion_;
	// A newer load supersedes any earlier one this flag might still refer to.
	suppressNextTrajectoryLoaded_ = false;
	inOverviewMode_ = false;
	lastCursorIndex_ = -1;
	// TrajectoryView::dataset_ (shared_ptr) outlives this pointer exactly like
	// DataTablePanel's own raw dataset_ pointer -- see trajectory_view.cpp's
	// setDataset() for the destruction-ordering guarantee.
	dataset_ = &dataset;
	trajCoords_.clear();
	trajCoords_.reserve(dataset.points.size());
	QElapsedTimer copyTimer; copyTimer.start();
	for (const TripSamplePoint& p : dataset.points)
		trajCoords_.emplace_back(p.latitude, p.longitude);
	Logger::logf(Logger::Profile, "Map", "coord copy: %lld µs  (%zu pts)", copyTimer.nsecsElapsed() / 1000, trajCoords_.size());
	liftoffPoints_ = dataset.liftoffPoints;
	touchdowns_ = dataset.touchdowns;
	events_ = dataset.events;
	aircraftTitle_ = dataset.aircraftTitle;
	if (pageReady_) {
		Logger::logf(Logger::Trace, "Map", "setDataset: page ready, pushing dataset v%d immediately (%zu pts, %zu touchdowns, %zu events)",
		             datasetVersion_, trajCoords_.size(), touchdowns_.size(), events_.size());
		// Inject the aircraft title before pushing liftoff points/touchdowns so
		// liftoffPopupHtml()/touchdownPopupHtml() see the correct value when they
		// run setLiftoffs/setTouchdowns.
		runJs(mapSetStringJs(QStringLiteral("window._aircraftTitle"), aircraftTitle_));
		pushTrip();
	} else {
		Logger::logf(Logger::Trace, "Map", "setDataset: page not ready yet; deferring push of dataset v%d until refreshProvider()", datasetVersion_);
		// Page still loading; trajectory will be pushed in refreshProvider() when
		// ready. Signal immediately so TrajectoryView's pending counter doesn't
		// stall, but remember to swallow that deferred push's own emit (in
		// pushTrip() below) so this dataset load doesn't count twice.
		suppressNextTrajectoryLoaded_ = true;
		emit trajectoryLoaded();
	}
}

void MapWidget::showOverview(const std::vector<TripSummary>& trips) {
	QElapsedTimer t; t.start();
	++datasetVersion_;
	inOverviewMode_ = true;
	overviewTrips_ = trips;
	lastCursorIndex_ = -1;
	dataset_ = nullptr;
	trajCoords_.clear();
	liftoffPoints_.clear();
	touchdowns_.clear();
	events_.clear();
	Logger::logf(Logger::Profile, "Map", "showOverview: cleared vectors: %lld µs", t.nsecsElapsed() / 1000);
	if (!pageReady_) {
		Logger::log(Logger::Trace, "Map", QStringLiteral("showOverview: page not ready yet; overview will be pushed once the page loads"));
		return;
	}
	const QString js = mapSetOverviewJs(trips);
	Logger::logf(Logger::Profile, "Map", "showOverview: JSON built: %lld µs", t.nsecsElapsed() / 1000);
	runJs(js);
	Logger::logf(Logger::Profile, "Map", "showOverview: runJs done: %lld µs", t.nsecsElapsed() / 1000);
}

void MapWidget::resetZoom() {
	if (pageReady_)
		runJs(QStringLiteral("resetZoom();"));
}

QString MapWidget::defaultMapImageFileName() const {
	if (inOverviewMode_)
		return QStringLiteral("trips.png");
	return defaultBaseFileName() + QStringLiteral(".png");
}

QString MapWidget::defaultBaseFileName() const {
	const QString base = airportPairName(liftoffPoints_, touchdowns_, QStringLiteral("trip"));

	// Uses the trip's actual departure time (same source TripHistoryPanel's
	// row context menu uses), not the first liftoff's zuluTime -- a trip with
	// no detected liftoff would otherwise get no timestamp suffix at all.
	// Falls back to the trajectory's first sample if departureZuluTime itself
	// is somehow unset.
	const QString departureZulu = dataset_ && !dataset_->departureZuluTime.isEmpty() ? dataset_->departureZuluTime
		: (dataset_ && !dataset_->points.empty() ? dataset_->points.front().zuluTime : QString());
	return appendDepartureTimestamp(base, departureZulu);
}

void MapWidget::exportKml() {
	// The menu only offers this with a trip loaded, but it's a non-modal
	// popup: the map can switch to the overview while it's still open.
	if (!dataset_) return;
	const QString fileName = QFileDialog::getSaveFileName(this, QStringLiteral("Export to KML"),
		defaultBaseFileName() + QStringLiteral(".kml"), QStringLiteral("KML File (*.kml)"));
	if (fileName.isEmpty()) return;
	QString error;
	if (!exportTripDatasetToKmlFile(*dataset_, fileName, &error))
		QMessageBox::critical(this, QStringLiteral("Error"), QStringLiteral("Failed to export KML to %1.\n%2").arg(fileName, error));
}

void MapWidget::onLoadFinished(bool ok) {
	if (!ok) {
		Logger::log(Logger::Warning, "Map", QStringLiteral("map.html failed to load"));
		return;
	}
	pageReady_ = true;

	// Deliver the Gemini API key and current aircraft title as JS globals so
	// the in-popup streaming analysis can reach them without round-tripping
	// through QWebChannel.
	runJs(mapSetStringJs(QStringLiteral("window._geminiApiKey"), AppSettings::instance().geminiApiKey()));
	runJs(mapSetStringJs(QStringLiteral("window._aircraftTitle"), aircraftTitle_));

	refreshProvider();
}

void MapWidget::refreshProvider() {
	runJs(QStringLiteral("initProvider();"));
	// initProvider() rebuilds the JS-side map state, which would otherwise
	// silently reset event-marker visibility to its default -- re-apply
	// whatever the toggle button last requested.
	runJs(mapSetEventsVisibleJs(eventsVisible_));
	if (inOverviewMode_) {
		Logger::log(Logger::Trace, "Map", QStringLiteral("refreshProvider: overview mode active; re-pushing overview"));
		showOverview(overviewTrips_);
	} else {
		Logger::log(Logger::Trace, "Map", QStringLiteral("refreshProvider: detail mode active; re-pushing trajectory/liftoff points/touchdowns/events"));
		pushTrip();
	}
}

void MapWidget::setEventsVisible(bool visible) {
	eventsVisible_ = visible;
	if (!pageReady_)
		return;
	runJs(mapSetEventsVisibleJs(visible));
}

void MapWidget::pushTrip() {
	// Built off the main thread: a long flight can have 60k+ sample points
	// (see mapSetTrajectoryJs() for how they're thinned).
	runJsBuiltInBackground("trajectory", trajCoords_.size(),
		[coords = trajCoords_, version = datasetVersion_]() { return mapSetTrajectoryJs(coords, version); },
		[this]() {
			// setTrajectory() (just run) always snaps the marker to the first
			// point -- if a cursor was pinned before this rebuild (a
			// refreshProvider() reload re-pushing the same dataset, not a
			// genuinely new one; see lastCursorIndex_ in map_widget.h), restore
			// it now instead of leaving the marker stuck at the trip start.
			if (lastCursorIndex_ != -1)
				runJs(QStringLiteral("setCursorIndex(%1);").arg(lastCursorIndex_));
			if (suppressNextTrajectoryLoaded_)
				suppressNextTrajectoryLoaded_ = false;
			else
				emit trajectoryLoaded();
		});
	runJsBuiltInBackground("liftoffs", liftoffPoints_.size(),
		[liftoffPoints = liftoffPoints_]() { return mapSetLiftoffsJs(liftoffPoints); });
	runJsBuiltInBackground("touchdowns", touchdowns_.size(),
		[touchdowns = touchdowns_]() { return mapSetTouchdownsJs(touchdowns); });
	runJsBuiltInBackground("events", events_.size(),
		[events = events_]() { return mapSetEventsJs(events); });
}

void MapWidget::runJsBuiltInBackground(const char* what, size_t count, std::function<QString()> build,
	std::function<void()> afterRun) {
	const int ver = datasetVersion_;
	auto* watcher = new QFutureWatcher<QString>(this);
	connect(watcher, &QFutureWatcher<QString>::finished, this, [this, watcher, ver, what, afterRun]() {
		watcher->deleteLater();
		if (ver != datasetVersion_) {  // a newer setDataset()/showOverview() superseded this one
			Logger::logf(Logger::Trace, "Map", "%s JS superseded (dataset v%d -> v%d); discarding it", what, ver, datasetVersion_);
			return;
		}
		QElapsedTimer t; t.start();
		runJs(watcher->result());
		Logger::logf(Logger::Profile, "Map", "runJs %s: %lld µs", what, t.nsecsElapsed() / 1000);
		if (afterRun)
			afterRun();
	});
	watcher->setFuture(QtConcurrent::run([what, count, build = std::move(build)]() {
		QElapsedTimer t; t.start();
		QString js = build();
		Logger::logf(Logger::Profile, "Map", "%s JSON (bg): %lld µs  (%zu items)", what, t.nsecsElapsed() / 1000, count);
		return js;
	}));
}

void MapWidget::runJs(const QString& script) {
	view_->page()->runJavaScript(script);
}
