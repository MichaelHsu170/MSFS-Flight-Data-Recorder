# Tests

Automated tests that pin the app's current behavior so refactoring can be
checked without flying in MSFS. Each `tst_*.cpp` is a Qt Test executable run
by CTest.

## Running

Pick **Test** from the Ctrl+Shift+B menu (builds Debug, then runs every
test), or from a terminal:

```powershell
.\.vscode\scripts\build.bat Debug test
.\.vscode\scripts\build.bat Release test
```

`ctest.exe` itself ships with CMake next to `cmake.exe`
(`...\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\`),
which isn't on PATH by default; `build.bat` finds it there.

A single test program can also be run directly, e.g.
`build\Debug\tests\tst_airport_lookup.exe`, optionally with one test
function name as an argument. The whole suite took about 14 s on the
verification machine.

## How they work

- **fdr_core** (root `CMakeLists.txt`) is all app code except `main.cpp`,
  shared by the app and the tests.
- **fake_simconnect.cpp** replaces `SimConnect.lib`: it implements every
  `SimConnect_*` function the app calls, records the calls, and delivers
  made-up packets to the app's dispatch callback.
- **test_support.cpp** builds those packets and provides `FlightDriver`,
  which runs a real `RecorderBridge` against the fake: set fields on
  `record` (position, on-ground, engines, ...), call `tick()` to send a
  sample. `tick()` also moves the event flood filter's clock, so its
  0.5 s / 5 s windows pass without real waiting. `FlightDriver::airports` is
  a made-up world that answers the recorder's airport/runway lookups
  (`serviceLookups()`).
- Each test program works in its own temporary directory (Debug builds put
  `flight_data.db` and `settings.ini` in the working directory). Release
  builds put them next to the test executables in `build/<config>/tests/`,
  never next to the app, which is why tests don't run in parallel.

## Coverage

| Area | Test | Scenarios |
|---|---|---|
| Geometry, formatting, runway codes, write queues (`types.h`) | `tst_types` | distance/bearing/destination/intersection, DMS and timestamp text, every runway designator and compass code, AIRPORT copy/clear, queue ordering and shutdown |
| SimConnect connection and registration | `tst_recording` | registrations, data definition size = copied sample size, every mapped event in the notification group, open/quit/dispatch failure, reconnect retry, connect while connected, quit while disconnected, unhandled packet ids logged, notifications without a GUI context reach no bridge |
| Trip start and stop, samples | `tst_recording` | start conditions (sim running, not paused, loaded flight, on ground, either engine, recording enabled), trip row contents, sample interval (default and from settings), midnight rollover, pause, pitch/bank sign, live signals, stop on engine shutdown / leaving the flight / sim quit / app close, consecutive trips, failed trip insert (no recording, retried next sample), failed destination-time write (trip still ends), setting recording-enabled to its current value |
| Nearest airport candidates (`add_nearest_airports()` in `airport_lookup.cpp`) on their own | `tst_airport_candidates` | empty list, nearest five in order with distances, only 4-letter idents, region kept, accumulation across chunks, farther than a full list ignored, south as near as north |
| Runway matching module (`runway_match.cpp`) on its own | `tst_runway_match` | strict hit in both directions, stored runway ends, margin-only hits (past the end, beside), no hit (far, short of the margin), no runways, crossing runways, north = 360 (incl. a primary end stored as 0), displaced threshold for touchdowns only, thresholds longer than the runway, disabled threshold data, trace lines |
| Liftoff, touchdown, airport and runway matching | `tst_airport_lookup` | departure and touchdown rows, runway match in both directions, designators, crossing runways, magnetic variation, centerline offset sign, displaced thresholds (incl. touchdown before threshold, disabled threshold data), approach track vs heading, stale approach position, touch-and-go markers, lookups queued behind a pending one, facility definition registered once, every fallback (no airports, margin hit, within/beyond 5 km, farther candidate, multi-packet list, non-airport idents), rejected request, unrelated exception, stale responses, deferred departure, reconnect reset, departure logged with its liftoff time, touchdown whose row was never inserted, liftoff marker whose insert fails, departure's own insert failure retried on next liftoff, off-runway touchdown whose row was never inserted, failed landing-destination write, failed lookup write caught by dispatch, malformed facility data (negative runway count, runway index beyond the count, orphan and extra pavement records), facility data rejected midway, facility data for an ended trip not landing on the next trip's touchdown |
| Event flood filter (`event_filter.cpp`) on its own | `tst_event_filter` | quiet-period hold (carried trip and timestamps), commit on next occurrence, two quick repeats, fast burst suppressed / kept suppressed / ends with or without a flush, flap bypass, independent names, slow flood retracted + suppressed + recovers (with or without a flush), repeats 2.5 s apart, double + single = slow flood, shutdown flush, unique seqs |
| Cockpit events and flood protection | `tst_events` | quiet-period recording, every mapped event name, no trip, flap whitelist, below/at burst threshold, burst recovery, slow flood retraction + suppression + recovery, event resolved after trip end, flush on shutdown, deleted trip, failed event write logged and retracted from the UI, failed retraction logged, crash message, unknown event |
| Database | `tst_database` | missing database, schema and indexes, repeatable migration, column upgrade of old databases, group-name uniqueness, every trip_data field written and read back identically on the live and stored paths, recorder write API (trip insert, destination time/position, trip airport with/without runway, clearing the destination, liftoff/touchdown rows and their airport with/without runway, liftoff-only clamp of negative threshold distance, failed write throws and rolls back, failed sample write logged by the writer thread which keeps draining), UI connections (missing database, move, read-only), AI analysis reports (save, replace, invalid/unknown row), trip list (order, status, group), liftoff/touchdown/event reads, event positions, trip deletion |
| Trip groups | `tst_groups` | create (trim, order, blank, duplicates incl. non-ASCII case), name-exists check (Unicode case, excluded group), rename, assign/unassign, trip counts, delete ungroups trips (and rolls back if it fails partway), reorder, name tie-break |
| Shared helpers | `tst_trip_dataset` | timestamp parsing, file-name pieces, decimation (within budget, stride, last sample kept, slices), field labels, field lists unique, bool bits unique |
| Chart data (`chart_data.cpp`) | `tst_chart_data` | series table matches `charts_panel.qml` (series names, hover keys, count), sample fields to series, zulu time on the axis, malformed times, nice axis max / signed range, extents (whole trip and slices), series build incl. malformed-time fill, one-sample and no-valid-time axes, thinning, nearest sample, hover values |
| Map scripts (`map_script.cpp`) | `tst_map_script` | string globals escaped, trajectory whole / thinned (indices, ends) / empty, live points, liftoff and touchdown popup fields, events, empty lists, events toggle, overview segments and "Ungrouped" |
| KML export | `tst_kml` | header/name, path and track in meters, liftoff/touchdown descriptions, no-airport and no-runway rows, event grouping, XML escaping, empty trip, unparseable times, write failure |
| settings.ini | `tst_settings` | default file, defaults for missing/invalid values, values from file, in-place edits keep comments and other sections, new key/section, section header with a trailing comment, unreadable and read-only files left alone, hidden fields, column widths, recording toggle, log level |
| Logging | `tst_logger` | `.old` rotation, header, level filter, line format, C shim (incl. levels beyond Profile and an out-of-range level), crash logging and its level filter, second init keeps the file but takes the new level, level names |
| Data Table panel | `tst_data_table_panel` | rows for every field, value formatting (numbers, DMS, Yes/No), which point is shown, cursor, clearing, hidden fields, Visible Fields dialog OK/Cancel, column width, right-click Copy on a value cell only |
| Trip History | `tst_trip_history` | durations and totals, column text, status colors, selectability, group filter, newest-first list, overview signal, loading a trip (samples, liftoffs, touchdowns, events), select by id (incl. reentrant-load guard), unknown group id falls back to Ungrouped, live trips, delete with confirm/cancel (incl. clearing the selection when the deleted trip was selected), Set Group menu, Deselect/Reset Zoom menu, selection surviving a bridge-triggered refresh, context menu suppressed while loading, idempotent load-finished, column widths |
| Live Status panel | `tst_live_status_panel` | version, connection indicator, log lines, recording messages, 500-line cap, recording indicator states, toggle click (and drag-off), toggle while recording, event lines and retraction (incl. an event line already pruned by the cap), blank event text, hover tooltip on the toggle, stale trip end, snapshot line |
| Manage Groups dialog | `tst_manage_groups_dialog` | list and trip counts, add, duplicate message, cancel, rename, rename collision message, database that can't be opened, blank rename, delete confirm/cancel, delete with no selection, failed delete, reorder by drop, failed reorder, delete/rename/reorder without a writable database |
| Map page bridge | `tst_map_bridge` | cursor/range/overview forwarding, saving AI reports, invalid row ids, no database file |
| SQLite statement helpers (`db_query.cpp`) on their own | `tst_db_query` | text column read (value and NULL), prepare success and malformed-SQL failure, row iteration (every row, early stop), a genuine `SQLITE_BUSY` step failure against a second locking connection, UTF-8 text binding, exec success/malformed-SQL/constraint-violation failure, transaction commit, rollback on a failed body, failure when already inside a transaction |
| Splitter handle-release helper (`splitter_utils.h`) | `tst_splitter_utils` | callback fires once per release (not on press), fires again on a second release, doesn't consume the event (passes it on to filters installed after it), destroyed along with the handle |
| Charts panel widget (`charts_panel.cpp`) | `tst_charts_panel` | QML root loads, `setDataset()` emits `seriesLoaded`, an empty dataset collapses the axis and emits immediately, malformed-time live points are dropped, `valueAt()` is empty with no dataset loaded, a second dataset reuses the resolved series cache, a superseded in-flight load is discarded without emitting, `setCursorIndex()` sets/clears the QML `cursorTime` property (incl. out of range), `appendLivePoint()` grows the axis and every cached series (and `valueAt()`'s live-mode fallback reads it), `valueAt()` reads full-resolution data after a load, `setVisibleRange()`'s zoomed-out/zoomed-slice/duplicate-range/degenerate-single-sample branches |
| Map widget (`map_widget.cpp`) | `tst_map_widget` | page loads and `setDataset()` before the page is ready emits `trajectoryLoaded` immediately then suppresses the later, deferred re-emit; once ready, `setDataset()` pushes trajectory/liftoffs/touchdowns/events and emits once; a superseded `setDataset()` call discards the first load's background JS build without a second emit; `appendLivePoint()` only arms the flush timer once the page is ready, and the timer stops itself once drained; `defaultMapImageFileName()` tracks overview vs. a loaded trip's airport pair and departure timestamp; `resetZoom()`/`setEventsVisible()` before and after the page is ready |
| Trajectory view (`trajectory_view.cpp`) | `tst_trajectory_view` | a null `setDataset()` is ignored; a real dataset fans out to the map/charts/data table and `renderingFinished()` fires once both async subviews finish; `clearAndShowOverview()` resets the current trip without spuriously emitting `renderingFinished()` from the synchronous empty-chart reset; `resetZoom()`; live points apply to the dataset/map/charts/table while following, are dropped everywhere when `ChartsPanel` rejects a malformed time (both the direct and the buffered-then-flushed path), and are buffered and then flushed in order when `setLiveFollow(false)`/`(true)`; `setRightPanelWidth()`; both splitters' drag (`rightPanelWidthChanged`) and release-persists-to-settings wiring |

## Not covered (check by hand)

- **Charts panel's QML** (`charts_panel.qml` / Qt Graphs): that the series
  and axes visually show what `chart_data` computes, zoom slices, the hover
  tooltip -- `tst_charts_panel` drives the real QML root through the C++
  wrapper but doesn't inspect rendered pixels.
- **`ChartsPanel`'s "QML object missing" guards** (`charts_panel.cpp`): two
  related families, both unreachable with the real, working
  `charts_panel.qml` compiled into the binary.
  - `root == nullptr` checks (`buildSeriesCache()`, both in `setDataset()`,
    `setCursorIndex()`, `setVisibleRange()`). `view_->setSource()` is called
    exactly once, synchronously, from the constructor, against a QML file
    compiled into the binary's own `.qrc` (no network fetch to go async
    over), and nothing else ever re-sources or tears down `view_`
    independently of `ChartsPanel` itself. So `rootObject()` is already
    non-null by the time the constructor returns and stays that way for the
    object's whole life. (An earlier version of a test tried to exercise
    this path by constructing a `ChartsPanel` without calling `show()`; it
    turned out `setSource()` doesn't need a shown window to finish loading,
    so the root was already ready, `setDataset()` took the async
    background-compute path instead of the intended sync early-return, and
    the test's premise didn't hold -- removed rather than kept as a
    flaky/misleading pass.)
  - A specific named child missing from an otherwise-ready root
    (`setAxisRange()`'s `!axis`, `loadFullSlice()`'s `!series`,
    `appendLivePoint()`'s `!cache_.valid` after a `buildSeriesCache()`
    retry): every series and axis object name `buildSeriesCache()` looks up
    is always present in the real `charts_panel.qml`, so these guard a QML
    typo/renumbering that would also fail loudly elsewhere, not a state any
    test can reach standalone.
- **Map rendering** (`map.html` in QtWebEngine): that the page actually
  draws what `map_script` sends (trajectory, markers and popups, overview
  routes with group colors/legend), the right-click menu's Save/Copy Image
  and `FilteredWebEngineView::contextMenuEvent()`'s building of it, and
  cursor/zoom sync with the charts -- `tst_map_widget` drives the real page
  through `MapWidget`'s public API and checks that `trajectoryLoaded`/timer
  behavior is correct, but never inspects rendered pixels or opens the
  context menu.
- **`MapWidget` gaps that need a real desktop, file dialog, or an
  unreachable timing window** (`map_widget.cpp`): `exportKml()` and
  `FilteredWebEngineView::saveMapImage()` (both block on
  `QFileDialog::getSaveFileName`); `onLoadFinished()`'s `!ok` branch (would
  need `map.html`'s own load to genuinely fail); `refreshProvider()`'s
  `inOverviewMode_` re-push-overview branch (never hit because
  `tst_map_widget` keeps its one shared `MapWidget` in detail mode whenever
  the page reloads); `pushTrajectory()`'s `lastCursorIndex_ != -1`
  cursor-restore branch (needs a `refreshProvider()` reload after
  `MapBridge::cursorIndexChanged` has already pinned a cursor, which no
  test drives since that signal only ever fires from JS running inside the
  page). `tst_map_widget` also can't run under `QT_QPA_PLATFORM=offscreen`
  (QtWebEngine's GPU process hits a fatal DCHECK sharing a GL context with
  an offscreen window) and, in this sandboxed environment, only the first
  `QWebEngineView`-backed widget built in a process reliably finishes
  loading -- see the comments at the top of `tst_map_widget.cpp`.
- **AI analysis** (Gemini streaming, retries, stored report shown again).
- **Window layout**: `MainWindow` itself (not exercised by any test) and the
  `overviewTripClicked` signal's three-hop forwarding chain (`MapBridge` ->
  `MapWidget` -> `TrajectoryView`), which would need a real click inside the
  WebEngine page's overview route to trigger end to end -- `tst_map_bridge`
  and `tst_trajectory_view` each cover one end of it (`MapBridge` emitting
  it, `TrajectoryView` re-emitting it) but not a click driving it from the
  page. `TrajectoryView`'s own splitter-size-persisted-on-release wiring is
  covered by `tst_trajectory_view`.
- **Startup** (`main.cpp`): single-instance lock, crash/terminate logging.
- **Trip History**: Export to KML from the row menu (file dialog), the
  "still saving" delete block, opening Manage Groups from the panel, a
  failed/missing read or write database connection (`ensureHistoryConnection`,
  `refreshTrips`, `setTripGroupFromUi`), `onRowActivated`'s `loading_`/Live
  guards (already unreachable in practice -- every caller filters those
  cases before calling it), `tryFinishLoad`'s "not all watchers done yet"
  branch (timing-dependent on QtConcurrent scheduling, not reproducible
  without injecting an artificial delay), and the hover/right-click paths in
  `eventFilter()` (simulating a real `QEvent::MouseMove` or right-click via
  `QTest` crashes the offscreen platform plugin in this environment).
- **The real SimConnect**: the fake follows the SDK's documented behavior;
  its function signatures are checked against the real `SimConnect.h` only
  when building on Windows.
- **`queryAllTrips`'s triple-schema-failure guard** (`db_history.cpp`):
  returns an empty list if even the legacy (no-name, no-region) `SELECT`
  fails to prepare, which needs a `trips` table missing core columns that
  no real migration produces.
- **`db.cpp` failure paths below a failing statement**: `db_bind()`'s and
  `sqlite3_reset()`/`COMMIT`/`BEGIN`'s error branches and the
  finalize-after-commit log in `db_insert_update_table()` (need SQLite to
  fail at that exact step, not at prepare); the writer threads'
  `catch (...)` blocks (every write throws only `db_exception`);
  `db_delete_events()`'s empty-list return (the flood filter never retracts
  zero events); `migrate_db()`/`create_schema()`/`create_db_indexes()`/
  `migrate_table_columns()` failure logging (needs a corrupt or locked
  database file); and `connect_db()`'s open failure, which calls `exit(1)`.
- **Allocation failures**: the `malloc`/`calloc` NULL branches in
  `flight_phase.cpp` (contact records, sample record, `last_sample` cache)
  and `airport_lookup.cpp` (runway buffer). No test can make the allocator
  fail without replacing it.
- **`recorder.cpp`'s `catch (...)` in the dispatch callback**: everything it
  calls throws only `db_exception`, which the preceding handler catches.
- **Logger I/O failures** (`logger.cpp`): `CreateFile` failing in `init()`
  and `WriteFile` failing in `writeLine()`; also `writeLine()`'s "no file
  open" return, which `formatLine()` already rules out before calling it.
- **settings.ini default-file creation failure** (`app_settings.cpp`): the
  file is created when it is missing, and the test can't make the working
  directory unwritable while still letting the process run there.
- **KML short write** (`kml_export.cpp`): `QFile::write()` returning fewer
  bytes than asked needs a full disk.
- **Branches with no observable effect or no way in**:
  - `airport_lookup.cpp`'s wrap of a 0° bearing to 360°. Every use of the
    bearing in `runway_match.cpp` takes differences modulo 360, so only a
    trace log line shows it. A due-north test passed with the wrap removed
    and was dropped.
  - `lookup_on_facility_data()`'s stale-trip check: `lookup_on_facility_data_end()`
    makes the same check and drops the response, so removing only the first
    changes nothing a test can see (the second is covered).
  - `db_groups.cpp`'s prepare-failure returns in `queryAllGroups()`,
    `groupNameExists()` and `reorderGroups()`: SQLite treats stepping or
    finalizing a null statement as a harmless no-op, so the results are the
    same with or without them.
  - `runway_match.cpp`'s "intersection calc failed" skip: per suspected bug
    2 below, the intersection never reports failure.
  - `ManageGroupsDialog::onListReordered()`'s `updating_` guard: `reload()`
    never emits `reordered`.
  - `LiveStatusPanel::eventFilter()`'s non-toggle branch: the filter is only
    installed on the toggle.
  - The closing brace after `break` in `flight_phase.cpp`'s `RUNWAY`/`AIRPORT`
    case, which the coverage tool attributes a line to.

## Suspected bugs these tests exposed

Recorded here rather than fixed, so the tests describe today's behavior.

1. **Distance on the exact runway centerline.** When a liftoff/touchdown
   point lies exactly on the centerline, `COORDINATE::intersectionCoordinate()`
   degenerates and the threshold distance can come out as ~65.7 million ft
   (the far side of the globe), depending on floating-point rounding. Not
   pinned by a test because the result is compiler-dependent; the runway
   tests place points 3 m off the centerline instead.
2. **Coincident courses aren't detected.** `intersectionCoordinate()` checks
   `sin(...) == 0`, which rounding never hits, so two points on the same
   course return a point instead of the (360,360) "no intersection" marker
   the runway matching relies on (`tst_types::intersectionOfCoincidentCoursesReturnsAPoint`).
3. **Live trip display is unreachable.** `RecorderBridge::liveDataPoint` is
   emitted but not connected to anything, `TrajectoryView::setLiveFollow()`
   is never called, and live trips can't be selected in Trip History, so
   `TrajectoryView::appendLivePoint()` and the panels' `appendLivePoint()`
   never run in the app.
4. **A failed coordinate-only write leaves the lookup stuck.** When the
   airport list has no candidates (`lookup_on_airport_list()` in
   `airport_lookup.cpp`), `on_lookup_resolved(NO_AIRPORT)` writes the trip
   row first. If that write throws, `lookup.pending = FALSE` and the request
   for the next queued touchdown are skipped. Every later airport/runway
   lookup then waits until the next reconnect. Pinned with `QEXPECT_FAIL` in
   `tst_airport_lookup::failedTripWriteOnACoordinateOnlyTouchdownStillFinishesTheLookup`.
5. **A failing trip-airport write re-requests the same lookup forever.** In
   `lookup_on_facility_data_end()`, if the trips `UPDATE` throws before the
   liftoff/touchdown record is marked resolved, `FacilityLookupCleanup`'s
   destructor calls `request_next_touchdown_facility_lookup()`, which
   immediately asks for the same record again. While the database error
   persists, this is an endless loop of SimConnect requests. Not pinned by a
   test: `serviceLookups()` would loop forever. The failure tests stop the
   forced error before answering lookups instead.
