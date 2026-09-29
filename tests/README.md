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
| SimConnect connection and registration | `tst_recording` | registrations, data definition size = copied sample size, every mapped event in the notification group, open/quit/dispatch failure, reconnect retry |
| Trip start and stop, samples | `tst_recording` | start conditions (sim running, not paused, loaded flight, on ground, either engine, recording enabled), trip row contents, sample interval (default and from settings), midnight rollover, pause, pitch/bank sign, live signals, stop on engine shutdown / leaving the flight / sim quit / app close, consecutive trips |
| Runway matching module (`runway_match.cpp`) on its own | `tst_runway_match` | strict hit in both directions, stored runway ends, margin-only hits (past the end, beside), no hit (far, short of the margin), no runways, crossing runways, north = 360, displaced threshold for touchdowns only, disabled threshold data, trace lines |
| Liftoff, touchdown, airport and runway matching | `tst_airport_lookup` | departure and touchdown rows, runway match in both directions, designators, crossing runways, magnetic variation, centerline offset sign, displaced thresholds (incl. touchdown before threshold, disabled threshold data), approach track vs heading, stale approach position, touch-and-go markers, lookups queued behind a pending one, facility definition registered once, every fallback (no airports, margin hit, within/beyond 5 km, farther candidate, multi-packet list, non-airport idents), rejected request, unrelated exception, stale responses, deferred departure, reconnect reset |
| Event flood filter (`event_filter.cpp`) on its own | `tst_event_filter` | quiet-period hold (carried trip and timestamps), commit on next occurrence, two quick repeats, fast burst suppressed / kept suppressed / ends with or without a flush, flap bypass, independent names, slow flood retracted + suppressed + recovers (with or without a flush), repeats 2.5 s apart, double + single = slow flood, shutdown flush, unique seqs |
| Cockpit events and flood protection | `tst_events` | quiet-period recording, every mapped event name, no trip, flap whitelist, below/at burst threshold, burst recovery, slow flood retraction + suppression + recovery, event resolved after trip end, flush on shutdown, deleted trip, crash message, unknown event |
| Database | `tst_database` | missing database, schema and indexes, repeatable migration, column upgrade of old databases, group-name uniqueness, every trip_data field written and read back identically on the live and stored paths, recorder write API (trip insert, destination time/position, trip airport with/without runway, clearing the destination, liftoff/touchdown rows and their airport with/without runway, liftoff-only clamp of negative threshold distance, failed write throws and rolls back), UI connections (missing database, move, read-only), AI analysis reports (save, replace, invalid/unknown row), trip list (order, status, group), liftoff/touchdown/event reads, event positions, trip deletion |
| Trip groups | `tst_groups` | create (trim, order, blank, duplicates incl. non-ASCII case), name-exists check (Unicode case, excluded group), rename, assign/unassign, trip counts, delete ungroups trips, reorder, name tie-break |
| Shared helpers | `tst_trip_dataset` | timestamp parsing, file-name pieces, decimation (within budget, stride, last sample kept, slices), field labels, field lists unique, bool bits unique |
| Chart data (`chart_data.cpp`) | `tst_chart_data` | series table matches `charts_panel.qml` (series names, hover keys, count), sample fields to series, zulu time on the axis, malformed times, nice axis max / signed range, extents (whole trip and slices), series build incl. malformed-time fill, one-sample and no-valid-time axes, thinning, nearest sample, hover values |
| Map scripts (`map_script.cpp`) | `tst_map_script` | string globals escaped, trajectory whole / thinned (indices, ends) / empty, live points, liftoff and touchdown popup fields, events, empty lists, events toggle, overview segments and "Ungrouped" |
| KML export | `tst_kml` | header/name, path and track in meters, liftoff/touchdown descriptions, no-runway rows, event grouping, XML escaping, empty trip, unparseable times, write failure |
| settings.ini | `tst_settings` | default file, defaults for missing/invalid values, values from file, in-place edits keep comments and other sections, new key/section, hidden fields, column widths, recording toggle |
| Logging | `tst_logger` | `.old` rotation, header, level filter, line format, C shim, crash logging, single init, level names |
| Data Table panel | `tst_data_table_panel` | rows for every field, value formatting (numbers, DMS, Yes/No), which point is shown, cursor, clearing, hidden fields, Visible Fields dialog OK/Cancel, column width |
| Trip History | `tst_trip_history` | durations and totals, column text, status colors, selectability, group filter, newest-first list, overview signal, loading a trip (samples, liftoffs, touchdowns, events), select by id, live trips, delete with confirm/cancel, Set Group menu, Deselect/Reset Zoom menu, column widths |
| Live Status panel | `tst_live_status_panel` | version, connection indicator, log lines, recording messages, 500-line cap, recording indicator states, toggle click (and drag-off), toggle while recording, event lines and retraction, stale trip end, snapshot line |
| Manage Groups dialog | `tst_manage_groups_dialog` | list and trip counts, add, duplicate message, cancel, rename, rename collision message, database that can't be opened, blank rename, delete confirm/cancel, reorder |
| Map page bridge | `tst_map_bridge` | cursor/range/overview forwarding, saving AI reports, invalid row ids |

## Not covered (check by hand)

- **Charts panel** (QML / Qt Graphs): that the series and axes show what
  `chart_data` computes, zoom slices, the hover tooltip.
- **Map** (`map.html` in QtWebEngine): that the page draws what
  `map_script` sends (trajectory, markers and popups, overview routes with
  group colors/legend), right-click menu (Save/Copy Image, Export to KML),
  cursor and zoom sync with the charts.
- **AI analysis** (Gemini streaming, retries, stored report shown again).
- **Window layout**: `TrajectoryView`, `MainWindow`, splitter sizes saved
  on release.
- **Startup** (`main.cpp`): single-instance lock, crash/terminate logging.
- **Trip History**: Export to KML from the row menu (file dialog), the
  "still saving" delete block, opening Manage Groups from the panel.
- **The real SimConnect**: the fake follows the SDK's documented behavior;
  its function signatures are checked against the real `SimConnect.h` only
  when building on Windows.

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
