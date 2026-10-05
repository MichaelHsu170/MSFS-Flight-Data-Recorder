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

`ctest.exe` itself ships with CMake next to `cmake.exe`, which needn't be
on PATH: `build.bat` uses the `cmake.exe` on PATH, else Visual Studio's
bundled copy (`<VS install>\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\`),
and runs the `ctest.exe` beside it.

A single test program can also be run directly, e.g.
`build\Debug\tests\tst_airport_lookup.exe`, optionally with one test
function name as an argument. The whole suite took about 220 s (Debug)
on the verification machine.

Don't run two test runs at once (e.g. `build.bat … test` and
`coverage.ps1`): the map tests show real windows on the desktop (see
`fdr_add_webengine_test()` in `CMakeLists.txt`), and one run's windows can
make the other's map test time out.

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
| Geometry, formatting, runway codes, write queues (`types.h`) | `tst_types` | an unset coordinate is invalid, distance/bearing/destination, bearing wrap and difference, DMS and timestamp text (UTC offset sign per SimConnect's UTC-minus-local convention; a time under half a millisecond before the next second carried into it, never second 60, but not past the end of the day), every runway designator and compass code, `copy_cstr()` cuts to fit and zero-fills, AIRPORT copy/clear, queue ordering and shutdown |
| SimConnect connection and registration | `tst_recording` | registrations, data definition size = copied sample size, every mapped event in the notification group, open/quit/dispatch failure, no connecting until `start()`, reconnect retry, connect while connected, quit while disconnected, unhandled packet ids logged, sample data for an unknown request ignored, notifications without a GUI context reach no bridge, engine power SimVars registered for engines 1-4 in record order with their units, ENG COMBUSTION 1-4 landing in their record fields |
| Trip start and stop, samples | `tst_recording` | start conditions (sim running, not paused, loaded flight, on ground, any of the aircraft's engines 1-4 running but none past its engine count, recording enabled), the recording toggle ignored while recording, trip row contents, sample interval (default and from settings), midnight rollover, pause, pitch/bank sign, each sample signalled with its values in the current data (kept up to date without a trip too), stop on engine shutdown (a quad only once all four are off) / leaving the flight / sim quit / app close (the stopping sample not recorded), consecutive trips, failed trip insert (no recording, retried next sample), failed destination-time write (trip still ends), setting recording-enabled to its current value |
| Nearest airport candidates (`add_nearest_airports()` in `airport_lookup.cpp`) on their own | `tst_airport_candidates` | empty list, nearest five in order with distances, only 4-letter idents, region kept, accumulation across chunks, farther than a full list ignored, south as near as north |
| Runway matching module (`runway_match.cpp`) on its own | `tst_runway_match` | strict hit in both directions, a point exactly on the centerline, stored runway ends, margin-only hits (past the end, beside), no hit (far, short of the margin), no runways, crossing runways, north = 360 (incl. a primary end stored as 0), displaced threshold for touchdowns only (from either end), thresholds longer than the runway, disabled threshold data, trace lines |
| Liftoff, touchdown, airport and runway matching | `tst_airport_lookup` | departure and touchdown rows, runway match in both directions, designators, crossing runways, magnetic variation (incl. a true heading past north), centerline offset sign, displaced thresholds (incl. touchdown before threshold, disabled threshold data), approach track vs heading, stale approach position, touch-and-go markers, lookups queued behind a pending one, facility definition registered once, every fallback (no airports, margin hit, within/beyond 5 km, farther candidate, multi-packet list, non-airport idents), rejected request, unrelated exception, stale responses, deferred departure, reconnect reset, departure logged with its liftoff time, touchdown whose row was never inserted, liftoff marker whose insert fails, departure's own insert failure retried on next liftoff, off-runway touchdown whose row was never inserted, failed landing-destination write, failed lookup write caught by dispatch, a trip write that keeps failing (with or without an airport) ending the lookup once, malformed facility data (negative runway count, runway index beyond the count, orphan and extra pavement records), facility data rejected midway, facility data for an ended trip not landing on the next trip's touchdown, the runway of a touchdown whose trip write failed not reused by the next touchdown |
| Event flood filter (`event_filter.cpp`) on its own | `tst_event_filter` | quiet-period hold (carried trip and timestamps), commit on next occurrence, two quick repeats, fast burst suppressed / kept suppressed / ends with or without a flush, flap bypass, independent names, slow flood retracted + suppressed + recovers (with or without a flush), repeats 2.5 s apart, double + single = slow flood, shutdown flush, unique seqs |
| Cockpit events and flood protection | `tst_events` | quiet-period recording, every mapped event name, no trip, flap whitelist, below/at burst threshold, burst recovery, slow flood retraction + suppression + recovery, event resolved after trip end, flush on shutdown, deleted trip, failed event write logged and retracted from the UI, failed retraction logged, crash message, unknown event |
| Database | `tst_database` | missing database, migration failing when the database can't be opened, schema and indexes, repeatable migration, column upgrade of old databases, migration failing when a missing column can't be added (its ALTER failing to prepare or to run) and adding it next time, a table that can't be created failing it (the others still created), an index that can't be created logged as a warning without failing it (the others still created), legacy N1/N2 moved into `engine_speed`/`engine_load` (jet rows only, per engine count, old columns dropped, rerun is a no-op; the rebuild keeps rowids across a gap and columns the current schema doesn't name, keeps NOT NULL only where the old column had it, recreates the index, and reports progress per batch of rows copied (to 65%), then after the drop (75%), the commit (90%) and the indexes (100%), each step weighted by roughly its share of the time, passing on only a rising percentage (a 305-row copy whose batches round to 0% or repeat); rows at the smallest and largest possible rowid are copied too; a migration failing after the rows were copied reports failure, changes nothing, still lets the indexes be created, and is redone next start; one cancelled after the first batch or just before committing is rolled back the same way; a trip_data whose columns can't be read fails it, logged), local times stored by older builds with the UTC offset's sign reversed corrected once in all six local-time columns ("+00:00" and other text left alone, a new database marked as needing none, a failure rolling back every column, logged, and redone next start), group-name uniqueness, every trip_data field written and read back identically, a sample with no engine power stored as NULL rather than an empty BLOB, recorder write API (trip insert, destination time/position, trip airport with/without runway, clearing the destination, liftoff/touchdown rows and their airport with/without runway, liftoff-only clamp of negative threshold distance, failed write throws and rolls back, failed sample write logged by the writer thread which keeps draining), UI connections (missing database, move, read-only), AI analysis reports (save, replace, invalid/unknown row), trip list (order, status, group; empty and logged when it can't be read), liftoff/touchdown/event reads, event positions, a trip's dataset put together (samples named after the trip, with or without a connection; liftoffs, touchdowns and events added, events placed), trip deletion |
| Trip groups | `tst_groups` | create (trim, order, blank, duplicates incl. non-ASCII case, a failed placement query logged), name-exists check (Unicode case, excluded group), rename, assign/unassign, trip counts, delete ungroups trips (and rolls back if it fails partway), reorder (and rolls back, logged, if it fails partway), name tie-break |
| Shared helpers | `tst_trip_dataset` | timestamp parsing, airport labels ("ICAO (Name)"), file-name pieces, decimation (within budget, stride, last sample kept, slices), field labels, the engine a field belongs to (per-engine prefixes with an engine suffix only), field lists unique, bool bits unique, bool packing into its groups |
| Engine power (`engine_power.cpp`) | `tst_engine_power` | speed/load SimVars and labels per engine type (piston, jet, helo turbine, turboprop), fixed vs data-sized axes, other engine types record nothing, an engine type no int holds (NaN, ±1e300) reads as unknown, engine count clamped to 0-4 (an unreadable one reads as 0), engine combustion counted only for the aircraft's engines, BLOB packing (byte order, count clamped, none = NULL) and round trip, BLOB unpacking (partial, oversized, NULL, empty; engines past the count cleared) |
| Chart data (`chart_data.cpp`) | `tst_chart_data` | series table matches `charts_panel.qml` (series names, hover keys, count), sample fields to series (each recorded engine's speed/load), engine extents, the trip's engine (first point with power), engine labels by type, each zulu time as its UTC instant on the axis (hover time in UTC too, across a local DST change; every test runs in US Pacific time), malformed times, nice axis max / signed range, extents (whole trip and slices), one sample's values read back from the series, series build incl. malformed-time fill, one-sample and no-valid-time axes, thinning, nearest sample, hover values |
| Map scripts (`map_script.cpp`) | `tst_map_script` | string globals escaped, trajectory whole (with its version) / thinned (indices, ends) / empty, liftoff and touchdown popup fields, events, empty lists, events toggle, overview segments and "Ungrouped" |
| KML export | `tst_kml` | header/name, path and track in meters, liftoff/touchdown descriptions, no-airport and no-runway rows, event grouping, XML escaping of names, description values shown literally (escaped, so a "]]>" in one can't end the CDATA), empty trip, unparseable times, write failure, the export-failed error box names the file and the reason |
| settings.ini | `tst_settings` | default file (app-written sections labeled "Auto-managed"), defaults for missing/invalid values, values from file, in-place edits keep comments and other sections, new key, new section under the default file's header, section header with a trailing comment, unreadable and read-only files left alone, hidden fields, column widths, recording toggle, log level |
| Logging | `tst_logger` | `.old` rotation, header, level filter, line format, crash logging and its level filter, second init keeps the file but takes the new level, level names |
| Data Table panel | `tst_data_table_panel` | rows for every field, value formatting (numbers, DMS, Yes/No, engine speed/load by engine type, fields of engines past the engine count left blank), which point is shown, cursor, clearing, hidden fields, Visible Fields dialog OK/Cancel, column width, right-click Copy on a value cell only |
| Trip History | `tst_trip_history` | durations and totals, column text (airports as "ICAO (Name)"), nothing for an invalid or out-of-range index, status colors, selectability, group filter, newest-first list, overview signal, loading a trip (samples, liftoffs, touchdowns, events), select by id (incl. reentrant-load guard), unknown group id falls back to Ungrouped, live trips, delete with confirm/cancel (incl. the confirmation naming the airports as "ICAO (Name)" or "-", and clearing the selection when the deleted trip was selected), a failed delete or group change saying so and changing nothing, a trip still saving refused deletion (when chosen and once confirmed), Export to KML from the row menu (the save dialog suggesting the trip's file name, the file holding the samples recorded, a failed export's error box naming the file, a trip with no samples saying so and writing no file), Set Group menu, a hovered row tinted until the mouse leaves, hovered cells painted like unhovered ones, a right-click on another row keeping the selection, deleting the filtered-by group through Manage Groups resetting the filter to All Trips, Deselect/Reset Zoom menu, selection surviving a bridge-triggered refresh, context menu suppressed while loading, idempotent load-finished, column widths |
| Live Status panel | `tst_live_status_panel` | version, connection indicator (green or red dot), log lines, recording messages, 500-line cap, the newest line scrolled into view, recording indicator states (red, green while recording, grey when disabled; a pointing-hand cursor except while recording), toggle click (and drag-off; a right click does nothing), toggle while recording and while a stopped trip is still being saved (the dot still showing it recording until the trip has ended), event lines and retraction (incl. an event line already pruned by the cap), blank event text, hover tooltip on the toggle, stale trip end, snapshot line |
| Manage Groups dialog | `tst_manage_groups_dialog` | list and trip counts, a database it can't read (said so in the dialog, logged, listed again once readable), add, duplicate message, failed add, cancel, rename (the renamed group staying selected), rename collision message, failed rename, database that can't be opened, blank rename, delete confirm/cancel, delete with no selection, failed delete, reorder by drop, failed reorder, delete/rename/reorder without a writable database, the Close button |
| Map page bridge | `tst_map_bridge` | cursor and range (each with its trajectory version)/overview forwarding, saving AI reports and saying whether each was saved (not with an invalid row id or no database file) |
| SQLite statement helpers (`db_query.cpp`) on their own | `tst_db_query` | text column read (value and NULL), prepare success and malformed-SQL failure, row iteration (every row, early stop without a logged error), a genuine `SQLITE_BUSY` step failure against a second locking connection, every statement finalized (the connection closes cleanly), UTF-8 text binding, exec success/malformed-SQL/constraint-violation failure, transaction commit, rollback on a failed body, rollback on a failed commit (a deferred foreign key), failure when already inside a transaction; each failure logs its context, and a failed prepare also logs the statement |
| Splitter helpers (`splitter_utils.h`) | `tst_splitter_utils` | handle-release callback fires once per release (not on press), fires again on a second release, doesn't consume the event (passes it on to filters installed after it), destroyed along with the handle; `setSecondSectionSize()` sets the second section and gives the first the difference, keeping the total |
| Charts panel widget (`charts_panel.cpp`) | `tst_charts_panel` | QML root loads, `setDataset()` emits `seriesLoaded`, with no trip (at startup, and after a deselect, which empties every line and emits before `setDataset()` returns) every chart hides its axes (labels, line, title, grid), legend and end-of-trip line and shows "No trip selected", then "Loading…" until a trip's lines are in, all back once it loads, a trip with no point blank the same way but saying "No data recorded", `valueAt()` is empty with no dataset loaded, the time axis labeled in zulu time, evenly spaced across a local DST change (every test runs in US Pacific time), a second dataset replaces the first's lines, time axis and values, a superseded in-flight load is discarded without emitting, `setCursorIndex()` sets/clears the QML `cursorTime` property (incl. out of range, cleared by a new dataset or a deselect; while a trip loads, the last index applied once loaded and dropped if another trip replaces it first), `valueAt()` reads full-resolution data after a load (and the shown trip's while another one loads), the engine power chart labeled by the dataset's engine (data-sized vs fixed axes, a fixed one sized to the data once it goes past it; in the QML, only the recorded engines' lines and legend entries shown, both axis titles, the load axis hidden with no engines, the "Engine Power" fallback title and no-data message, with the axis re-sized to the empty data rather than left at the previous trip's scale; cleared on deselect, where "No trip selected" replaces the no-data message), every chart's plot ending at the same x, whole-number axes labeled once per unit on a range under 10 (no repeated or "-0" labels) and with automatic ticks otherwise, `setVisibleRange()`'s no-trip (time axis left alone)/while-loading (the old trip's axes left alone, the last range applied once loaded -- a full range, sent while loading or right after, is already what loads and isn't drawn again -- and dropped if another trip replaces it first)/zoomed-out (a one-sample trip kept 1 s wide)/zoomed-slice/duplicate-range/degenerate-single-sample branches |
| Map widget (`map_widget.cpp`) | `tst_map_widget` | page loads and `setDataset()` before the page is ready emits `trajectoryLoaded` immediately then suppresses the later, deferred re-emit; once ready, `setDataset()` draws the trajectory line and the liftoff/touchdown markers on the page and emits once; event markers stay hidden when `setEventsVisible(false)` was set before the page loaded, and the toggle shows/hides them afterwards; a superseded `setDataset()` call discards the first load's background JS build without a second emit; the page's cursor index (from a click on the line) and visible range, each tagged with the trajectory version it was measured on, are forwarded for the current trajectory and dropped for an earlier one; `defaultMapImageFileName()` tracks overview vs. a loaded trip's airport pair and departure timestamp; `resetZoom()` refits a zoomed-out map to the trajectory; a trip loaded while the previous fit is still zooming is fitted once that zoom ends; a trip with no samples removes the previous trip's cursor marker; showing the overview removes the loaded trip's line, cursor and liftoff/touchdown/event markers along with their popup data; markup in the liftoff/touchdown and event popup values shows as text, not elements; the threshold and centerline distances show in the popup and the AI prompt only with a matched runway, a negative threshold distance rounding away from zero (-2.5 ft as -3 ft); a touchdown's popup lists the same fields and values as its KML export placemark description (airport as "ICAO (Name)"), plus its coordinate; the AI prompt names the airport and runway the same way ("ICAO (Name)", "27L (270°)", each without the part in brackets when it's unknown); AI analysis with `fetch()` stubbed: a finished answer is saved with or without thinking (the thinking panel shown only with it), streamed in pieces split mid-object, with braces and quotes in its text kept whole; a real response recorded from the service (`data/ai_stream_response.json`), fed in pieces from 1 byte to whole, is saved with all its thinking and its answer (an unmatched brace, quotes and a backslash) intact; one cut off (`MAX_TOKENS` or no finish) is retried, and three incomplete ones (incl. a finish with no answer) save nothing and say so; a finished answer the database couldn't save stays shown with a note saying so; a server error (500, 503) or rate limit (429) is retried and the answer after it saved, three in a row show the last one's message (read from the array the streaming endpoint wraps it in) and save nothing, and a rejected request (400) isn't retried and points at `gemini_api_key`; a liftoff popup closed and reopened mid-analysis shows it still running (button disabled, spinner), doesn't start a second one, and shows the answer once it's in; the page's console messages go to the log under MapJS, errors and warnings as WARN and the rest as INFO; with the real mouse: the right-click menu (Export to KML offered only with a trip shown), Save Image (suggesting the trip's file name, the saved PNG the whole view with the trajectory's line in its pixels; one that can't be saved says so), Copy Image (the same picture on the clipboard, which is put back afterwards), Export to KML (the trip's recorded samples, under its file name; chosen after the map switched to the overview, nothing), Copy for text selected on the page (the text on the clipboard, the selection cleared), Copy Link on a link (its address on the clipboard), and a click on an overview route emitting `overviewTripClicked` with its trip id; a page reload drawing the overview again, or the trip with its cursor where the user left it |
| Trajectory view (`trajectory_view.cpp`) | `tst_trajectory_view` | a null `setDataset()` is ignored and keeps the shown trip; a real dataset fans out to the map/charts/data table and `renderingFinished()` fires once both async subviews finish; `clearAndShowOverview()` resets the current trip without spuriously emitting `renderingFinished()` from the synchronous empty-chart reset; `resetZoom()` refits the map to the trajectory and shows the charts' full range; `setRightPanelWidth()`; both splitters' drag (`rightPanelWidthChanged`, the setting left alone mid-drag) and release-persists-to-settings wiring |
| Main window startup (`main_window.cpp`) | `tst_main_window` | the window opens with a "Checking the database" notice in Trip History's place, which a quick migration replaces with Trip History without ever showing a percentage; a failed one says so (word-wrapped) and keeps it, with neither Trip History nor the simulator connection started; a slow one keeps it while Trip History and the simulator connection wait, then shows "Updating the database for this version… N%" once the trip_data rebuild reports progress; a quick rebuild, whose steps all finish within Qt's 40 ms progress throttling, still reports its last 100%; once it finishes Trip History replaces the notice without the window being recreated, the simulator connection starts and the database is migrated; closing the window mid-rebuild (while the migration waits on a locked database, so not racing the copy) cancels it, leaving the legacy columns for the next start and neither Trip History nor the simulator connection started; closing it after the worker returned but before its finished signal is handled starts neither; closing it after the migration finished has nothing to cancel |

## Not covered (check by hand)

- **Charts panel's QML** (`charts_panel.qml` / Qt Graphs): that the series
  and axes visually show what `chart_data` computes, zoom slices, the hover
  tooltip -- `tst_charts_panel` drives the real QML root through the C++
  wrapper but doesn't inspect rendered pixels.
- **`ChartsPanel`'s "QML object missing" guards** (`charts_panel.cpp`): two
  related families, both unreachable with the real, working
  `charts_panel.qml` compiled into the binary.
  - `root == nullptr` checks (`buildSeriesCache()`, `setDataset()`,
    `setCursorIndex()`, `setAllXAxisRange()`, `setEngine()`). `view_->setSource()`
    is called exactly once, synchronously, from the constructor, against a
    QML file compiled into the binary's own `.qrc` (no network fetch to go
    async over), and nothing else ever re-sources or tears down `view_`
    independently of `ChartsPanel` itself. So `rootObject()` is null only if
    that QML fails to load (e.g. Qt Graphs missing from a deployment), and
    then for the object's whole life; the code after a root was once found
    (the background load's apply step, `setVisibleRange()` past its cache
    check) doesn't check again.
  - A specific named child missing from an otherwise-ready root
    (`setAxisRange()`'s `!axis`, `setYAxes()`'s axis checks,
    `setVisibleRange()`'s `!cache_.valid || !cache_.xAxis`, and the
    `series` checks in `loadFullSlice()` and `setDataset()`'s clearing
    loop): every series and axis object name `buildSeriesCache()` looks up
    is always present in the real `charts_panel.qml`, so these guard a QML
    typo/renumbering that would also fail loudly elsewhere, not a state any
    test can reach standalone.
- **Map rendering** (`map.html` in QtWebEngine): that the page draws the
  markers, popups and overview group colors/legend the way `map_script`
  sends them, and cursor/zoom sync with the charts as seen on screen --
  `tst_map_widget` reads back the page's own state (the trajectory line's
  points, marker counts, the zoom level) and checks pixels only for the
  trajectory line in a saved or copied map image.
- **`MapWidget::onLoadFinished()`'s `!ok` branch** (`map_widget.cpp`): it
  would need `map.html`'s own load, from the binary's resources, to
  genuinely fail. `tst_map_widget` also can't run under `QT_QPA_PLATFORM=offscreen`
  (QtWebEngine's GPU process hits a fatal DCHECK sharing a GL context with
  an offscreen window) and, in this sandboxed environment, only the first
  `QWebEngineView`-backed widget built in a process reliably finishes
  loading -- see the comments at the top of `tst_map_widget.cpp`.
- **AI analysis against the real service**: `tst_map_widget` checks the page
  against a stubbed `fetch()`, one of them replaying a real Gemma stream
  recorded on 2026-10-05 (a finished answer ends with `finishReason: "STOP"`).
  That a server error comes back as HTTP 500 with its error wrapped in an
  array was checked by hand once, not by a test. An unreachable service and a stored report shown
  again have no test.
- **Window layout**: `MainWindow`'s splitter sizes and signal wiring
  (`tst_main_window` covers only startup around the database migration).
  The `overviewTripClicked` chain is covered in two pieces: a real click on
  the page through `MapBridge` to `MapWidget` by `tst_map_widget`, and
  `TrajectoryView` re-emitting it by `tst_trajectory_view`.
  `TrajectoryView`'s own splitter-size-persisted-on-release wiring is
  covered by `tst_trajectory_view`.
- **Startup** (`main.cpp`): single-instance lock, crash/terminate logging,
  the log filter that drops Qt Graphs' "axis already associated" warning
  (`main.cpp` isn't part of the test library; `tst_charts_panel` checks
  that the charts raise only the three expected ones).
- **The native Windows save dialog**: the tests answer the Qt-drawn one
  (`Qt::AA_DontUseNativeDialogs`), which a test can fill in; the app shows
  the native one. Both return the chosen path the same way through
  `QFileDialog::getSaveFileName`.
- **Trip History**: a failed/missing read or write database connection
  (`ensureHistoryConnection`, `refreshTrips`, `setTripGroupFromUi`),
  `onRowActivated`'s `loading_`/Live guards (already unreachable in
  practice -- every caller filters those cases before calling it), and
  `tryFinishLoad`'s "not all watchers done yet" branch (timing-dependent on
  QtConcurrent scheduling, not reproducible without injecting an
  artificial delay).
- **The real SimConnect**: the fake follows the SDK's documented behavior;
  its function signatures are checked against the real `SimConnect.h` only
  when building on Windows.
- **`db.cpp` failure paths below a failing statement**: `check_bind()`'s
  (every `db_bind*()` helper's) and
  `sqlite3_reset()`/`COMMIT`/`BEGIN`'s error branches and the
  finalize-after-commit log in `db_insert_update_table()` (need SQLite to
  fail at that exact step, not at prepare); the "unknown exception"
  branch of `current_exception_message()`, which the writer threads use
  (every write throws only `db_exception`);
  `db_delete_events()`'s empty-list return (the flood filter never retracts
  zero events); `create_schema()` failing the migration when a `CREATE TABLE`
  fails (`aTableThatCantBeCreatedFailsTheMigration` reaches it, but the
  table is then missing, so `migrate_table_columns()` fails it just the
  same and no test can tell); `migrate_table_columns()`'s early return when it reads no
  columns (`migrateFailsWhenTripDatasColumnsCantBeRead` reaches it, but
  without it the `ALTER`s fail and fail the migration just the same, as
  does the legacy engine rebuild, so no test can tell);
  `copy_rows_in_batches()`'s prepare, row-count and mid-copy step
  failures (the count runs after this transaction's `CREATE` took the write
  lock, so nothing can lock it out) and the `engine_pack` registration failure (the tested legacy
  migration failure happens after the copy, at the rename);
  `migrate_legacy_engine_columns()`'s `BEGIN`, `CREATE TABLE trip_data_new`
  and `COMMIT` failures, which share the tested rollback path: `BEGIN` fails
  only inside an open transaction, which nothing leaves; a `CREATE` made to
  fail by an existing `trip_data_new` still fails at the copy if its check
  is broken, so no test sees it; and failing `COMMIT` needs another
  connection's read lock, which makes the column additions before the
  rebuild fail first; `migrate_local_time_offsets()`'s `PRAGMA user_version`
  read failures (SQLite would have to fail a pragma on a database it
  opened) and its `ROLLBACK`, which no test can see (`migrate_db()` closes
  the connection right after, and closing rolls back the open transaction
  just the same); and `connect_db()`'s open failure and failed
  `create_schema()`, which call `exit(1)`/`exit(2)` -- the app starts the
  bridge only after `migrate_db()` succeeded, so they need the database to
  break in between.
- **Allocation failures**: the `malloc`/`calloc` NULL branches in
  `flight_phase.cpp` (contact records, sample record, `last_sample` cache),
  `airport_lookup.cpp` (runway buffer) and `types.h` (`AIRPORT::copy()`'s
  runway buffer). No test can make the allocator
  fail without replacing it.
- **`recorder.cpp`'s `catch (...)` in the dispatch callback**: everything it
  calls throws only `db_exception`, which the preceding handler catches.
- **Logger I/O failures** (`logger.cpp`): `CreateFile` failing in `init()`
  and `WriteFile` failing in `writeLine()`; also `writeLine()`'s "no file
  open" return, which `formatLine()` already rules out before calling it;
  and `levelTag()`'s `"?    "` tag for a value outside `Logger::Level`,
  which every caller passes by name.
- **settings.ini default-file creation failure** (`app_settings.cpp`): the
  file is created when it is missing, and the test can't make the working
  directory unwritable while still letting the process run there.
- **KML short write and failed flush** (`kml_export.cpp`): `QFile::write()`
  returning fewer bytes than asked, or the flush of a file small enough to
  sit in QFile's buffer failing, needs a full or failing disk.
- **Branches with no observable effect or no way in**:
  - `airport_lookup.cpp`'s wrap of a 0° bearing to 360°. Every use of the
    bearing in `runway_match.cpp` takes differences modulo 360, so only a
    trace log line shows it. A due-north test passed with the wrap removed
    and was dropped.
  - `lookup_on_facility_data()`'s stale-trip check: `lookup_on_facility_data_end()`
    makes the same check and drops the response, so removing only the first
    changes nothing a test can see (the second is covered).
  - `db_groups.cpp`'s prepare-failure returns in `queryAllGroups()` and
    `groupNameExists()`, and `db_history.cpp`'s in every query: SQLite treats stepping or finalizing a null
    statement as a harmless no-op, so the results are the same with or
    without them.
  - `queryTripData()`'s engine count as the smaller of the
    `engine_speed`/`engine_load` BLOBs' counts: the app always writes both
    with the same count, so they differ only in a hand-edited database.
  - The same function's checks for a column missing from `trip_data`
    (`colDouble`/`colInt`/`colText`/`colEngineValues` reading nothing for
    an index of -1): `migrate_db()` adds every column before anything
    reads the table, so a column is missing only in a hand-edited database.
  - The same function reading a BLOB's pointer before its size, the order
    SQLite documents: either order gives the same result for a BLOB value,
    since neither call converts it.
  - `enginePowerFromRecord()`'s guard on a NaN or out-of-range NUMBER OF
    ENGINES: MSVC's unguarded cast gives `INT_MIN` for those, which the clamp
    turns into the same 0 count. A test with NaN and 1e300 passed with the
    guard removed and was dropped (the same guard on ENGINE TYPE is covered).
  - `TripHistoryPanel::onTableContextMenu()`'s return when the menu has
    nothing in it (a right-click below the last row, or on a trip still
    recording): Qt shows no popup for an empty `QMenu::exec()` on the test
    platform either, so a test passed with the return removed and was
    dropped. It stays so no other platform shows an empty popup.
  - `AIRPORT::copy()` freeing a runway buffer the target already holds:
    it only keeps that buffer from leaking, which no test can observe.
  - `LiveStatusPanel::eventFilter()`'s non-toggle branch: the filter is only
    installed on the toggle.
  - `DataTablePanel::showPoint()`'s range check, `rawNum()`'s check that a
    number field's index is within the sample's `rawNums` (with the
    resulting NaN checks of the GPS row and the number rows). Every sample
    the app shows is loaded
    by `queryTripData()`, which fills one value per number field, and those
    columns are always in that field list, so none of these checks can fail.
    `tst_data_table_panel` passed with each one removed.
  - `tripFieldEngine()`'s lower bound on the last character (`< '1'`): field
    names are C identifiers, whose only character below `'1'` is `'0'`, and
    an engine "0" gives 0 either way.
  - `MapWidget::resetZoom()`'s `pageReady_` guard: without it the call runs
    `resetZoom();` on a page still loading, where the function and its map
    don't exist yet, so the call does nothing either way.
  - The closing brace after `break` in `flight_phase.cpp`'s `RUNWAY`/`AIRPORT`
    case, which the coverage tool attributes a line to.

## Open questions (check with real sim data)

Left as they are until a recorded trip shows whether they matter.

1. **Negative torque on the engine chart.** For turboprops and helicopter
   turbines, the engine chart's load axis plots TURB ENG MAX TORQUE PERCENT.
   Like the speed, altitude and fuel axes, it only sets a maximum, so its
   minimum stays 0 (`ChartsPanel::setYAxes()`). If MSFS reports negative
   torque (e.g. a windmilling propeller in a descent), that part of the
   line is clipped at 0; the Data Table still shows the value. Check a
   turboprop trip's `engine_load` for negative values; if there are any,
   track the lowest load in `ChartExtents` and use `niceSignedAxisRange()`
   as the vertical speed axis does, with a negative-torque test.
