# Tests

Automated tests that pin the app's current behavior so refactoring can be
checked without flying in MSFS. Each `tst_*.cpp` is a Qt Test executable run
by CTest. They sit in subfolders mirroring the app's source folders (e.g.
`db/tst_database.cpp` tests `db/db.cpp`); the shared helpers
(`fake_simconnect`, `test_support`, `local_time_zone.h`) and the test data
(`data/`) stay at the top of `tests/`.

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

`build.bat Debug only tst_a tst_b` builds and runs only the named test
programs. A single test program can also be run directly, e.g.
`build\Debug\tests\tst_airport_lookup.exe`, optionally with one test
function name as an argument. The whole suite took between about 140 s
and 230 s (Debug) on the verification machine, varying from run to run.
If `map.html` never loads (a script error, say), `tst_map_widget`'s first
test fails after 15 s and every later one fails at once, rather than each
waiting out its timeouts.

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
| SimConnect connection and registration | `tst_recording` | registrations, data definition size = copied sample size, every mapped event in the notification group, open/quit/dispatch failure, no connecting until `start()`, reconnect retry, connect while connected, quit while disconnected, unhandled packet ids logged, sample data for an unknown request ignored, a short sample packet dropped with one warning per connection, notifications without a GUI context reach no bridge, per-engine SimVars registered with their units for engine indexes 1-16 (not 17), each landing in its engine's place in its record array |
| Trip start and stop, samples | `tst_recording` | start conditions (sim running, not paused, loaded flight, on ground, any of the aircraft's engines running (engine 6 of six, engine 16 of sixteen) but none past its engine count, recording enabled), the recording toggle ignored while recording, trip row contents, sample interval (default and from settings), midnight rollover, pause, pitch/bank sign, each sample signalled with its values in the current data (kept up to date without a trip too), stop on engine shutdown (a quad only once all four are off) / leaving the flight / sim quit / app close (the stopping sample not recorded), consecutive trips, failed trip insert (no recording, retried next sample), failed destination-time write (trip still ends), setting recording-enabled to its current value |
| Nearest airport candidates (`add_nearest_airports()` in `airport_lookup.cpp`) on their own | `tst_airport_candidates` | empty list, nearest five in order with distances, only 4-letter idents, region kept, accumulation across chunks, farther than a full list ignored, south as near as north |
| Runway matching module (`runway_match.cpp`) on its own | `tst_runway_match` | strict hit in both directions, a point exactly on the centerline, stored runway ends, margin-only hits (past the end, beside), the side edge half the width from the centerline (2.5 m inside it a hit, 2.5 m outside only a margin hit), no hit (far, short of the margin), no runways, crossing runways, north = 360 (incl. a primary end stored as 0), displaced threshold for touchdowns only (from either end), thresholds longer than the runway, disabled threshold data (at either end), along/across offsets of a point from either end, the used end's length, width, start, true heading, displaced threshold and slope (the secondary end's slope reversed), trace lines |
| Liftoff, touchdown, airport and runway matching | `tst_airport_lookup` | departure and touchdown rows (the actual vertical speed stored with them), runway match in both directions, designators, crossing runways, the true heading matched rather than the magnetic one (incl. past north; the magnetic one still stored), centerline offset sign, displaced thresholds (incl. touchdown before threshold, disabled threshold data), approach track vs heading, stale approach position (forgotten from 100 ft, the band's top), touch-and-go markers, a liftoff counted once airborne 0.5 s or risen 0.5 ft, not at 0.45 of either (a shorter, level flicker of the on-ground flag recording neither a liftoff nor the touchdown after it), recorded with the frame it left the ground in (not the confirming frame's, nor an earlier ignored flicker's), its 0.5 s counted across midnight, an unconfirmed liftoff not carried into the next trip, lookups queued behind a pending one, facility definition registered once, its runway fields in `RUNWAY`'s wire order (through `SLOPE`), the used end's runway length, width, displaced threshold, start, true heading and slope stored with the contact (the secondary end's own start, threshold and reversed slope; none without a runway), every fallback (no airports, margin hit, within/beyond 5 km, farther candidate, multi-packet list, non-airport idents), rejected request, unrelated exception, stale responses, deferred departure, reconnect reset, departure logged with its liftoff time, touchdown whose row was never inserted, liftoff marker whose insert fails, departure's own insert failure retried on next liftoff, off-runway touchdown whose row was never inserted, failed landing-destination write, failed lookup write caught by dispatch, a trip write that keeps failing (with or without an airport) ending the lookup once, malformed facility data (negative runway count, runway index beyond the count, orphan and extra pavement records), facility data rejected midway, facility data for an ended trip not landing on the next trip's touchdown, the runway of a touchdown whose trip write failed not reused by the next touchdown |
| Event flood filter (`event_filter.cpp`) on its own | `tst_event_filter` | quiet-period hold (carried trip and timestamps), commit on next occurrence, two quick repeats, fast burst suppressed / kept suppressed / ends with or without a flush, flap bypass, independent names, slow flood retracted + suppressed + recovers (with or without a flush), repeats 2.5 s apart, double + single = slow flood, shutdown flush, unique seqs, seqs continuing after a given one |
| Cockpit events and flood protection | `tst_events` | quiet-period recording, every mapped event name, no trip, flap whitelist, below/at burst threshold, burst recovery, slow flood retraction + suppression + recovery, a slow flood keeping an earlier run's events, event resolved after trip end, flush on shutdown, deleted trip, failed event write logged and retracted from the UI, failed retraction logged, crash message, unknown event |
| Database | `tst_database` | missing database, migration failing when the database can't be opened, schema and indexes, repeatable migration, column upgrade of old databases, migration failing when a missing column can't be added (its ALTER failing to prepare or to run) and adding it next time, a table that can't be created failing it (the others still created), an index that can't be created logged as a warning without failing it (the others still created), older layouts' per-engine values moved into `trip_engine_data`, one row per engine of the sample's engine count (the oldest layout's N1/N2 columns for every engine type; from the layout before `trip_engine_data`, each engine type's `engine_speed`/`engine_load` into the value it meant, engines 3-4 only where those hold them, and engines 1-2's other values and on/off bits, those bits cleared and the rest kept; values not recorded read back as not recorded; old columns dropped, rerun is a no-op); the rebuild keeps rowids as the sample ids across a gap and columns the current schema doesn't name, keeps NOT NULL only where the old column had it, recreates the index, and reports progress per batch of rows copied (to 65%), then after the drop (75%), the commit (90%) and the indexes (100%), each step weighted by roughly its share of the time, passing on only a rising percentage (a 305-row copy whose batches round to 0% or repeat); rows at the smallest and largest possible rowid are copied too, with their engine rows; a migration failing after the rows were copied reports failure, changes nothing, still lets the indexes be created, and is redone next start; one cancelled after the first batch or just before committing is rolled back the same way; a trip_data whose columns can't be read fails it, logged), local times stored by older builds with the UTC offset's sign reversed corrected once in all six local-time columns ("+00:00" and other text left alone, a new database marked as needing none, a failure rolling back every column, logged, and redone next start), group-name uniqueness, every trip_data field and every engine value of a 4-engine sample written and read back identically (the engine indexes past its engine count not stored), one engine row per engine whatever the engine type, keyed by the sample's id and trip, a sample whose engine rows can't be written (their statement can't be prepared, or an insert fails) not written either, nor any of its engine rows, engine values still loading after an earlier trip is deleted and the database vacuumed, engine rows of an engine outside 1-16 or of no loaded sample skipped on load, recorder write API (trip insert, destination time/position, trip airport with/without runway, clearing the destination, liftoff/touchdown rows and their airport with/without runway (the runway's length, width, threshold, start, heading and slope stored, NULL without one, read back with NULL as unknown; the actual vertical speed stored and read back the same way; older contact tables gain these columns), liftoff-only clamp of negative threshold distance, failed write throws and rolls back, as does one refused at its commit by another connection's read, failed sample write logged by the writer thread which keeps draining), UI connections (missing database, move, read-only), AI analysis reports (save, replace, invalid/unknown row), trip list (order, status, group; empty and logged when it can't be read), liftoff/touchdown/event reads, event positions, a trip's dataset put together (samples named after the trip, with or without a connection; liftoffs, touchdowns and events added, events placed, a touchdown's conditions derived from its samples), a NULL sample value (a column added after the row was written) read back as not recorded, trip deletion (its engine rows too) |
| Trip groups | `tst_groups` | create (trim, order, blank, duplicates incl. non-ASCII case, a failed placement query logged), name-exists check (Unicode case, excluded group), rename, assign/unassign, trip counts, delete ungroups trips (and rolls back if it fails partway), reorder (and rolls back, logged, if it fails partway), name tie-break |
| Liftoff/touchdown conditions and trajectory (`contact_analysis.cpp`) | `tst_contact_analysis` | density altitude, sun elevation by latitude, longitude and time (the solstice's highest and lowest sun, the equation of time in November), every surface type/condition name in the SDK's order (none for 255, -1, a fraction or NaN), precipitation names (none, rain, snow, both; none for 0 or NaN), precipitation, sea-level pressure and in/out of cloud from the contact's sample (cloud unknown on a sample without the precipitation state), conditions from the contact's sample (magnetic variation and crab across north, the runway's magnetic heading from its true one and the variation, rounded, across north and 360 rather than 0, unknown without either, station pressure and altimeter, pressure and density altitude, ground elevation, surface), the sun and each point's distance at the contact's own position (latitude and longitude not swapped), each point's airspeed, vertical speed, pitch, bank, true heading and track from its sample, the wind's range over only the last 30 s (30 s before the contact in, 31 s out) across north (its end past north too), touchdown gates at each height's last crossing (a go-around's descent not counted, gates never reached left out, a sample after the touchdown never one), the window (its highest terrain too) starting at the previous contact and a liftoff's ending at the next in time (liftoffs listed before touchdowns), liftoff gates at the first crossing after the liftoff measured from the last on-ground sample (a downhill roll's higher runway behind not counted, a sample at a gate's height crossing it; the field elevation the liftoff point's), the touchdown series from 15 s before to 10 s after (both ends included, cut short where the recording ends), a touchdown after the trip's last sample measured from that sample, the liftoff series' 10 s each side (both ends included) always including its on-ground sample, each series reaching past the neighboring contacts while the gates and terrain stop at them (a bounce showing in both touchdowns' series by its on-ground state and height, touchdowns closer than a sample sharing theirs), a bounce within one sample having no gates (no terrain), each point's on-ground state and each engine's throttle and power with its unit (N1 in % for a jet, torque in % for a turboprop or helicopter turbine, manifold pressure in inHg for a piston, none for another type; an engine not recorded unknown), the series capped at 60 keeping both ends and, mid-series, the contact's own sample and its on-ground one, gates by height above the touchdown rather than radio height over a cliff (terrain and highest terrain reported), a liftoff's highest terrain under its climb-out (measured from its on-ground sample), highest terrain only within 5 nm (4.9 nm in, stopping at the first sample beyond, 5.5 nm), the path angle of a 3° approach at each distance gate down to the touchdown point (its sample rolled past it), gates in time order, each point's distance along the runway from the marked threshold (touchdown) or the runway start (liftoff) and across the centerline, no trajectory when a sample's time inside its window (a gate's or the series') or the contact's own can't be read (the conditions still derived, without the sun and with the wind range from the contact's sample alone) while one outside it is ignored, the wind range stopping at a sample whose time can't be read, no samples or values not recorded staying unknown, each point's vertical speed the actual one when recorded (the trajectory marked indicated otherwise), a touchdown's rate the first new touchdown normal velocity from its sample on (not the previous touchdown's still latched, a sample without it skipped; unknown with no sample before it or no new value; never one past the next liftoff or touchdown), each point's wind with its headwind/tailwind and crosswind components against the runway's true heading (from the right positive, the wind and runway on either side of north; unknown without the runway heading), outside air temperature, cloud, precipitation and structural ice, the approach's weather and ride from the last crossing of 1000 ft (or the previous contact) down to the touchdown and the climb-out's from the liftoff up to the first crossing of 1000 ft or the next contact (950 ft not yet the top; every sample before that contact kept, none when it comes before the liftoff's next sample), each only within 5 nm of the contact (stopping at the first sample beyond, though the 1000 ft crossing is further back) (wind range, the largest headwind change in the order flown with its heights (where the wind held at an extreme, the two closest samples in time, the first of equally close pairs; a steady headwind its first sample at both ends), strongest tailwind and crosswind by size, lowest height in cloud, icing conditions when any sample was in cloud or precipitation (snow out of cloud too) at -20 to +10 °C (both ends included; unknown without the temperature), most structural and pitot ice, the share of samples with engine anti-ice on in any engine (2 of 3), the G-force range while airborne only, the vertical wind range, the airborne sample with the smallest margin below the stall angle of attack rather than the highest angle, skipping samples without the stall angle) |
| Shared helpers | `tst_trip_dataset` | timestamp parsing, airport labels ("ICAO (Name)"), file-name pieces, decimation (within budget, stride, last sample kept, slices), field labels, each engine's values read per engine (not recorded past the sample's engines), field lists unique, bool bits unique, bool packing into its groups |
| Engine power (`engine_power.cpp`) | `tst_engine_power` | the N1/N2-equivalent values and labels each engine type shows (piston crankshaft RPM / prop RPM; jet, helo turbine and turboprop N1 / N2), fixed vs data-sized axis, other engine types show nothing, engine count clamped to 0-16 (an unreadable one reads as 0), engine combustion counted only for the aircraft's engines (up to engine 16) |
| Chart data (`chart_data.cpp`) | `tst_chart_data` | series table matches `charts_panel.qml` (series names, hover keys, count; engine series named by engine), sample fields to series (each engine's N1/N2 equivalents its engine type shows, for all 16 engine indexes; a value not recorded as 0), engine extents (any engine's N1 or N2), the trip's engine (first point with engine 1's N1 or N2, which of the two were recorded, an old turboprop trip with neither, up to 16 engines), engine labels by type, engine series built only for the trip's engines (0 read back for the others), each zulu time as its UTC instant on the axis (hover time in UTC too, across a local DST change; every test runs in US Pacific time), malformed times, nice axis max / signed range, extents (whole trip and slices), one sample's values read back from the series (vertical speed the actual one, or the indicated one for a sample recorded without it, 0 with neither), series build incl. malformed-time fill, one-sample and no-valid-time axes, thinning, nearest sample, hover values |
| Map scripts (`map_script.cpp`) | `tst_map_script` | string globals escaped, trajectory whole (with its version) / thinned (indices, ends) / empty, liftoff and touchdown popup fields, a contact's conditions with only the values known, every condition and trajectory value under the name `map.html` reads, its runway data when known, a contact's actual vertical speed (0 sent) and a touchdown's rate when known, a final trajectory (for MapBridge) saying whether its vertical speed is the indicated one, a final trajectory (for MapBridge) with only the values known and each point's threshold/centerline distances when known (out of cloud sent as known), each point's on-ground state and its throttle and power per engine (absent when no engine's is known, an unknown engine's null) with the power's name and unit, each point's weather and the segment's weather, ride and stall margin under the names `map.html` reads (no segment without samples), events, empty lists, events toggle, overview segments and "Ungrouped" |
| KML export | `tst_kml` | header/name, path and track in meters, liftoff/touchdown descriptions (the aircraft heading magnetic with the true one in brackets, the runway's magnetic, then true, given a sample), the crab row's side (left, right, 0°, none without a sample), the V/S row's actual vertical speed or the indicated one marked "(indicated)", a touchdown rate row only on a touchdown with one, placemark child order per the KML 2.2 schema, no-airport and no-runway rows, event grouping, XML escaping of names, description values shown literally (escaped, so a "]]>" in one can't end the CDATA), empty trip, unparseable times, write failure, a failed export leaving the earlier file at that path as it was, the export-failed error box names the file and the reason, a cancelled save dialog exporting nothing, a database that can't be opened saying so and writing no file |
| settings.ini | `tst_settings` | default file (app-written sections labeled "Auto-managed"), an existing file kept as it is at startup, defaults for missing/invalid values, values from file, in-place edits keep comments and other sections, new key, new section under the default file's header, section header with a trailing comment, unreadable and read-only files left alone, hidden fields, column widths, recording toggle, log level |
| Logging | `tst_logger` | `.old` rotation, header, level filter, line format, crash logging and its level filter, second init keeps the file but takes the new level, level names |
| Data Table panel | `tst_data_table_panel` | rows for every field, value formatting (numbers, DMS, Yes/No, a value not recorded blank, the GPS row too), one row per engine value for each engine of the trip ("Eng Oil Pressure 3"), on/off ones as Yes/No and values not recorded left blank, which point is shown, cursor, clearing, hidden fields (incl. an engine row), Visible Fields dialog OK/Cancel (a hidden row the trip lacks kept hidden), column width, right-click Copy on a value cell only |
| Trip History | `tst_trip_history` | durations and totals, column text (airports as "ICAO (Name)"), nothing for an invalid or out-of-range index, status colors, selectability, group filter, newest-first list, overview signal, loading a trip (samples, liftoffs, touchdowns, events), select by id (incl. reentrant-load guard), unknown group id falls back to Ungrouped, live trips, delete with confirm/cancel (incl. the confirmation naming the airports as "ICAO (Name)" or "-", and clearing the selection when the deleted trip was selected), a failed delete or group change saying so and changing nothing, a trip still saving refused deletion (when chosen and once confirmed), opening (from a row or the map, the shown trip staying highlighted) and Export to KML, Export to KML from the row menu (the save dialog suggesting the trip's file name, the file holding the samples recorded, a failed export's error box naming the file, a trip with no samples saying so and writing no file), Set Group menu, its "Manage Groups…" opening the groups dialog (a group deleted there gone from the trip's row), a hovered row tinted until the mouse leaves, hovered cells painted like unhovered ones, a right-click on another row keeping the selection, deleting the filtered-by group through Manage Groups resetting the filter to All Trips, Deselect/Reset Zoom menu, selection surviving a bridge-triggered refresh, context menu suppressed while loading, idempotent load-finished, column widths |
| Live Status panel | `tst_live_status_panel` | version, connection indicator (green or red dot), log lines, recording messages, 500-line cap, the newest line scrolled into view, recording indicator states (red, green while recording, grey when disabled; a pointing-hand cursor except while recording), toggle click (and drag-off; a right click does nothing), toggle while recording and while a stopped trip is still being saved (the dot still showing it recording until the trip has ended), event lines and retraction (incl. an event line already pruned by the cap), blank event text, hover tooltip on the toggle, stale trip end, snapshot line (the actual vertical speed, not the indicated one; reset when the trip ends, kept on a stale trip end) |
| Manage Groups dialog | `tst_manage_groups_dialog` | list and trip counts, a database it can't read (said so in the dialog, logged, listed again once readable), add, duplicate message, failed add, cancel, rename (the renamed group staying selected), rename collision message, failed rename, database that can't be opened, blank rename, delete confirm/cancel, delete with no selection, failed delete, reorder by drop, failed reorder, delete/rename/reorder without a writable database, the Close button |
| Map page bridge | `tst_map_bridge` | cursor and range (each with its trajectory version)/overview forwarding, saving AI reports and saying whether each was saved (not with an invalid row id or no database file), each liftoff's and touchdown's trajectory built from the dataset set, by row id (none for another table's row, an unknown row, a trip without samples, or no dataset) |
| SQLite statement helpers (`db_query.cpp`) on their own | `tst_db_query` | text column read (value and NULL), prepare success and malformed-SQL failure, row iteration (every row, early stop without a logged error), a genuine `SQLITE_BUSY` step failure against a second locking connection, every statement finalized (the connection closes cleanly), UTF-8 text binding, exec success/malformed-SQL/constraint-violation failure, transaction commit, rollback on a failed body, rollback on a failed commit (a deferred foreign key), failure when already inside a transaction; each failure logs its context, and a failed prepare also logs the statement |
| Splitter helpers (`splitter_utils.h`) | `tst_splitter_utils` | handle-release callback fires once per release (not on press), fires again on a second release, doesn't consume the event (passes it on to filters installed after it), destroyed along with the handle; `setSecondSectionSize()` sets the second section and gives the first the difference, keeping the total |
| Charts panel widget (`charts_panel.cpp`) | `tst_charts_panel` | QML root loads without Qt Graphs axis warnings, every series ChartsPanel drives is a QML line (engine lines in legend order), `setDataset()` emits `seriesLoaded`, with no trip (at startup, and after a deselect, which empties every line, leaves none drawn and emits before `setDataset()` returns) every chart hides its axes (labels, line, title, grid), legend and end-of-trip line and shows "No trip selected", then "Loading…" until a trip's lines are in, all back once it loads, a trip with no point blank the same way but saying "No data recorded", `valueAt()` is empty with no dataset loaded, the time axis labeled in zulu time, evenly spaced across a local DST change (every test runs in US Pacific time), a second dataset replaces the first's lines, time axis and values, a superseded in-flight load is discarded without emitting, `setCursorIndex()` sets/clears the QML `cursorTime` property (incl. out of range, cleared by a new dataset or a deselect; while a trip loads, the last index applied once loaded and dropped if another trip replaces it first), `valueAt()` reads full-resolution data after a load (and the shown trip's while another one loads), the engine power chart labeled by the dataset's engine (data-sized vs fixed axis, the fixed one sized to the data once N1 or N2 goes past it; in the QML, only the recorded engines' lines and legend entries shown -- a piston's RPM and prop RPM, an old piston trip's RPM only, a jet's N1 and N2, all 6 or 16 engines of a bigger aircraft --, the axis title, an old turboprop trip with neither as no data, the "Engine Power" fallback title and no-data message, with the axis re-sized to the empty data rather than left at the previous trip's scale; cleared on deselect, where "No trip selected" replaces the no-data message), a trip with fewer engines leaving none of the last trip's extra engine lines (none shown, and none still drawn by Qt Graphs: each drawn line is a shown engine's), every chart's plot ending at the same x, whole-number axes labeled once per unit on a range under 10 (no repeated or "-0" labels) and with automatic ticks otherwise, `setVisibleRange()`'s no-trip (time axis left alone)/while-loading (the old trip's axes left alone, the last range applied once loaded -- a full range, sent while loading or right after, is already what loads and isn't drawn again -- and dropped if another trip replaces it first)/zoomed-out (a one-sample trip kept 1 s wide)/zoomed-slice/duplicate-range/degenerate-single-sample branches |
| Map widget (`map_widget.cpp`) | `tst_map_widget` | page loads and `setDataset()` before the page is ready emits `trajectoryLoaded` immediately then suppresses the later, deferred re-emit; once ready, `setDataset()` draws the trajectory line and the liftoff/touchdown markers on the page and emits once; event markers stay hidden when `setEventsVisible(false)` was set before the page loaded, and the toggle shows/hides them afterwards; a superseded `setDataset()` call discards the first load's background JS build without a second emit; the page's cursor index (from a click on the line) and visible range, each tagged with the trajectory version it was measured on, are forwarded for the current trajectory and dropped for an earlier one; the visible range is the samples in view plus one on each side, (-1, -1) for the whole trip, and, zoomed in between samples, the one nearest the view's middle plus one on each side; `defaultMapImageFileName()` tracks overview vs. a loaded trip's airport pair and departure timestamp; `resetZoom()` refits a zoomed-out map to the trajectory; a trip loaded while the previous fit is still zooming is fitted once that zoom ends; a trip with no samples removes the previous trip's cursor marker; showing the overview removes the loaded trip's line, cursor and liftoff/touchdown/event markers along with their popup data; markup in the liftoff/touchdown and event popup values shows as text, not elements; the threshold and centerline distances show in the popup and the AI prompt only with a matched runway, a negative threshold distance rounding away from zero (-2.5 ft as -3 ft); a touchdown's popup lists the same fields and values as its KML export placemark description (airport as "ICAO (Name)", headings and crab from a sample, the actual vertical speed and the touchdown rate, halves rounded alike), plus its coordinate; the popup's V/S is the actual vertical speed (0 shown), or the indicated one marked "(indicated)" without it, with a touchdown rate row only when known; the touchdown prompt's vertical speed is the actual one, or the indicated one said to lag it, with a touchdown rate line only when known, and its trajectory header names which vertical speed the points carry; the AI prompt names the airport and runway the same way ("ICAO (Name)", "27L (268° magnetic, 270° true)", the magnetic one only with a sample, each without the part in brackets when it's unknown); the popup marks each heading magnetic then true (the runway's magnetic and true headings, the aircraft's magnetic heading with its true one in brackets, and the crab angle, right/left/0; wind true; only the runway's true heading, the aircraft's magnetic heading and no crab without a sample); the touchdown prompt carries the conditions and final trajectory (gates with path angle, samples, highest terrain), one line per recorded value; the weather on the height gates only (wind with its headwind/tailwind and crosswind, 0 kt naming no side, and no components off a matched runway; outside air, cloud, precipitation, structural ice), not on a distance gate or a sample; the approach's or climb-out's weather and ride lines (wind shear marked from a 15 kt change, a gain or a loss, one rounding to 0 a gain between headwinds, a headwind rounding to 0 not a tailwind; no tailwind, or one rounding to 0, as "none"; in or not in cloud; icing yes/no; structural and pitot ice as percentages; anti-ice; the G-force range with its sampling caveat; the vertical wind range; the angle of attack closest to the stall), each left out when not recorded and the block left out when none is; both prompts telling the model to judge the crab against the crosswind, the speed against the gusts and excursions against the shear and turbulence, and to consider wind shear, turbulence, cloud and icing; each trajectory point's on-ground/airborne state and each engine's throttle and power ("?" for an engine not recorded) with the power's unit, the header explaining only what the points carry and naming the power (N1, torque, manifold pressure); each signed value naming its side (nose left or right, sun above or below the horizon, variation east or west, left or right of the centerline, terrain flagged only when more than 500 ft above); each prompt asks for its answer's sections, ending in its trajectory's segment (a liftoff's "Rotation and climb-out", a touchdown's "Final segment") and "Environment", and names its own trajectory (a liftoff's "Takeoff trajectory", spanning its rotation and climb-out); a value not recorded gets no line (no "NaN" or "undefined"; without a sample only time and position; an empty trajectory no block); a trajectory that couldn't be built gets a line saying so in both prompts; the sun's light bands; the cloud line (in or not in cloud) and the precipitation line (its rate only with precipitation); a crab that rounds to 0.0° naming no side; the surface line naming only its known parts; the runway lines (length, width, displaced threshold or none, slope uphill/downhill/level, the runway remaining in ft and %, measured from the marked threshold for a touchdown and the runway start for a liftoff) and each point's threshold and centerline distances; the worst-case prompt (every value, every gate, all 16 engines with three-digit torque or a manifold pressure under 100 inHg, 60 samples) within 32,000 characters, the per-minute token limit at 2 characters per token; AI analysis with `fetch()` stubbed: the trajectory loaded first, by row id, into the prompt, and a status line until the answer starts ("Building the climb-out trajectory…", "Sending to LLM…", "Waiting for the model…", a retry counted as "Retrying (2/3): …"), hidden once it does; one without a trajectory still saved, with the notice under the button (none with one, nor before an analysis; a touchdown's naming its approach); clicking Analyze on the page getting the trajectory from the trip shown through MapBridge (or the "not available" line and the notice, still shown after reopening the popup, for a trip whose samples can't give one), each Analyze button asking for its own contact's by row, and none once the overview is shown; Copy Prompt (shown without an API key too) putting on the clipboard the same prompt Analyze sends for that liftoff or touchdown, its trajectory included, showing "✓ Copied" for a moment and leaving the no-trajectory notice to Analyze (none after reopening the popup either), each contact's trajectory from its own table; Copy Prompt doing nothing, without an error, before the page's bridge is connected, and still copying when its popup is closed while the trajectory loads; the API key sent in the `x-goog-api-key` header, not the URL; a finished answer is saved with or without thinking (the thinking panel shown only with it, reading "▾ Thinking…" and not clickable while the thinking streams, then closed behind a clickable "▸ Show thinking" once the answer starts), streamed in pieces split mid-object, with braces and quotes in its text kept whole; a real response recorded from the service (`data/ai_stream_response.json`), fed in pieces from 1 byte to whole, is saved with all its thinking and its answer (an unmatched brace, quotes and a backslash) intact; one cut off (`MAX_TOKENS` or no finish), or carrying an error object mid-stream after its thinking, is retried, and three incomplete ones (incl. a finish with no answer) save nothing and say so; each failed attempt logged through `console.warn` (retried) or `console.error` (the last) with its reason (the HTTP status and body, the connection error, or the finish reason), any error object sent mid-stream (also when the connection then drops) and the characters of thinking and answer received, a successful one logging nothing; a finished answer the database couldn't save stays shown with a note saying so; a server error (500, 503) is retried at once and a rate limit (429) after waiting the minute out ("Retrying (2/3): Waiting 60 s for the per-minute token limit…"), the answer after it saved; the per-minute token limit's wait (none while the minute's prompts and this one fit in 16,000 tokens, exactly too; otherwise until enough of the oldest are a minute old; one sent exactly 60 s ago not counted; one over the limit alone waiting for an empty minute) and its estimate (2 characters per token, rounded up); an analysis that doesn't fit waiting, counting down, and sent once the earlier prompts are a minute old; the service's token count (from the answer) replacing the estimate for the prompt sent and its retry; three server errors in a row show the last one's message (read from the array the streaming endpoint wraps it in) and save nothing, and a rejected request (400) isn't retried and points at `gemini_api_key`; an unreachable service is tried three times, then says so and saves nothing, the same as a connection that drops after thinking arrived (its thinking panel hidden again); a touchdown's saved report shows when its popup opens, its thinking behind a closed "▸ Show thinking" panel only if it has some; a liftoff popup closed and reopened mid-analysis shows it still running (button disabled, spinner), doesn't start a second one, and shows the answer once it's in; one whose popup was closed while an answer cut off after its thinking was being retried still saves the next answer, which the reopened popup shows with its button enabled again; the page's console messages go to the log under MapJS, errors and warnings as WARN and the rest as INFO; with the real mouse: the right-click menu (Export to KML offered only with a trip shown), Save Image (suggesting the trip's file name, the saved PNG the whole view with the trajectory's line in its pixels; one that can't be saved says so), Copy Image (the same picture on the clipboard, which is put back afterwards), Export to KML (the trip's recorded samples, under its file name; chosen after the map switched to the overview, nothing), Copy for text selected on the page (the text on the clipboard, the selection cleared), Copy Link on a link (its address on the clipboard), and a click on an overview route emitting `overviewTripClicked` with its trip id; a page reload drawing the overview again, or the trip with its cursor where the user left it |
| Trajectory view (`trajectory_view.cpp`) | `tst_trajectory_view` | a null `setDataset()` is ignored and keeps the shown trip; a real dataset fans out to the map/charts/data table and `renderingFinished()` fires once both async subviews finish; `clearAndShowOverview()` resets the current trip without spuriously emitting `renderingFinished()` from the synchronous empty-chart reset; `resetZoom()` refits the map to the trajectory and shows the charts' full range; the map's cursor moves the charts' cursor and the data table's sample, its visible range zooms the charts, and an overview route click is re-emitted as `overviewTripClicked`; `setRightPanelWidth()`; both splitters' drag (`rightPanelWidthChanged`, the setting left alone mid-drag) and release-persists-to-settings wiring |
| Main window (`main_window.cpp`) | `tst_main_window` | growing the window keeps Live Status and the Data Table at the width chosen; dragging either one's handle resizes the other, saving the width only on release; a trip clicked on the map loaded by Trip History and shown by the trajectory view, its rendering finishing unlocking Trip History, and Trip History's Reset Zoom and Deselect reaching the view; the window opens with a "Checking the database" notice in Trip History's place, which a quick migration replaces with Trip History without ever showing a percentage; a failed one says so (word-wrapped) and keeps it, with neither Trip History nor the simulator connection started; a slow one keeps it while Trip History and the simulator connection wait, then shows "Updating the database for this version… N%" once the trip_data rebuild reports progress; a quick rebuild, whose steps all finish within Qt's 40 ms progress throttling, still reports its last 100%; once it finishes Trip History replaces the notice without the window being recreated, the simulator connection starts and the database is migrated; closing the window mid-rebuild (while the migration waits on a locked database, so not racing the copy) cancels it, leaving the legacy columns for the next start and neither Trip History nor the simulator connection started; closing it after the worker returned but before its finished signal is handled starts neither; closing it after the migration finished has nothing to cancel |

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
- **The map menu's no-request guard** (`FilteredWebEngineView::contextMenuEvent()`'s
  `!request` return, `map_widget.cpp`): a context-menu event sent to the
  view itself, with no menu request from the page, never reaches
  `contextMenuEvent()` -- with the guard removed, such a test still
  passed -- so no test can get there without one.
- **AI analysis against the real service**: `tst_map_widget` checks the page
  against a stubbed `fetch()`, one of them replaying a real Gemma stream
  recorded on 2026-10-05 (a finished answer ends with `finishReason: "STOP"`).
  That a server error comes back as HTTP 500 with its error wrapped in an
  array was checked by hand once, not by a test.
- **Which way the runway slope points**: the code takes the facility
  data's SLOPE as rising from the primary end toward the secondary, and
  negates it for the secondary end (`runway_match.cpp`). The SDK gives
  only "degrees", and the tests feed made-up values, so they check the
  negation but not the simulator's convention. To check: land in each
  direction on a clearly sloped runway and compare the stored
  `runway_slope` with the published gradient.
- **The precipitation, sea-level pressure, in-cloud and actual vertical
  speed SimVars'** names and units (`AMBIENT PRECIP STATE`, `AMBIENT
  PRECIP RATE`, `SEA LEVEL PRESSURE`, `AMBIENT IN CLOUD`, `VELOCITY WORLD
  Y` in `simconnect_defs.h`): the SimConnect fake delivers made-up samples
  whatever the definition asks for, so a wrong name or unit passes every
  test. To check: record a flight in rain or cloud and compare the stored
  values with the simulator's weather, and compare the stored
  `velocity_world_y` with the altitude change between samples.
- **The structural ice, wind component, angle of attack and G-force
  SimVars'** names and units (`STRUCTURAL ICE PCT` in percent, `AMBIENT
  WIND X`/`Z` in knots, `AMBIENT WIND Y` in feet per minute, `INCIDENCE
  ALPHA`/`BETA` and `STALL ALPHA` in degrees, `MAX G FORCE`/`MIN G FORCE`
  in G, in `simconnect_defs.h`): the same fake, so a wrong name or unit
  passes every test, and whether the simulator resets MAX/MIN G FORCE
  per flight isn't documented. To check: fly through icing, turbulence
  and close to the stall, compare the stored values with the simulator's
  own readouts (ice percentage, the vertical wind against the vertical
  speed change, the angle of attack against the stall warning), check
  `ambient_wind_velocity` against the hypotenuse of the X and Z
  components, and see whether the max/min G restart after a new flight.
- **How `PLANE TOUCHDOWN NORMAL VELOCITY` behaves** (`touchdownRate()` in
  `contact_analysis.cpp`): the code assumes the simulator keeps each
  touchdown's value until the next one, may set it up to a sample after
  the on-ground flag, and reports it positive into the surface. The tests
  feed made-up values, so they check the reading of those values, not the
  simulator's behavior. To check: land at a known sink rate, then bounce,
  and compare the stored per-sample values around each touchdown.
- **Startup** (`main.cpp`): single-instance lock, crash/terminate logging,
  the log filter (`main.cpp` isn't part of the test library).
- **The native Windows save dialog**: the tests answer the Qt-drawn one
  (`Qt::AA_DontUseNativeDialogs`), which a test can fill in; the app shows
  the native one. Both return the chosen path the same way through
  `QFileDialog::getSaveFileName`.
- **Trip History**: a failed/missing read or write database connection
  (`ensureHistoryConnection`, `refreshTrips`, `setTripGroupFromUi`, the
  row menu's Delete Trip) -- making the database unopenable means removing
  its file, which Windows refuses while the simulator fake and the panel
  each hold a connection to it,
  `onRowActivated`'s `loading_`/Live guards (already unreachable in
  practice -- every caller filters those cases before calling it), and
  `tryFinishLoad`'s "not all watchers done yet" branch (timing-dependent on
  QtConcurrent scheduling, not reproducible without injecting an
  artificial delay).
- **The "Event ignored (no active trip)" log limit** (`commit_event()`'s
  `EVENT_NO_TRIP_LOG_COOLDOWN`, `recorder.cpp`): at most one TRACE line
  per event name every 5 s. It changes nothing but how many debug log lines
  are written (the event is ignored either way), so a test could only count
  log lines -- an incidental detail, not a behavior anything relies on.
- **The real SimConnect**: the fake follows the SDK's documented behavior;
  its function signatures are checked against the real `SimConnect.h` only
  when building on Windows.
- **`db.cpp` failure paths below a failing statement**: `check_bind()`'s
  (every `db_bind*()` helper's) error branch (every index bound is a fixed,
  valid one); `db_insert_update_table()`'s `BEGIN` error branch (`BEGIN`
  fails only inside an open transaction, and every failure path rolls
  back), and its finalize-after-commit log (it returns the last step's
  result, which is "done" by then) --
  its `COMMIT` error branch is tested, with another connection's read
  lock (`writeRefusedAtCommitThrowsAndRollsBack`); the "unknown exception"
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
  lock, so nothing can lock it out) and the `engine_value` registration failure (the tested legacy
  migration failure happens after the copy, at the rename);
  `migrate_trip_engine_data()`'s `CREATE TABLE trip_data_new` failure (a
  `CREATE` made to fail by an existing `trip_data_new` still fails at the
  copy if its check is broken, so no test sees it);
  `in_migration_transaction()`, which both migrations run in (its
  transaction is `inTransaction()`'s, tested in `tst_db_query`): a failed
  `BEGIN` or `COMMIT` there (`BEGIN` fails only inside an open transaction,
  which nothing leaves, and failing `COMMIT` needs another connection's read
  lock, which makes the column additions before the rebuild fail first), and
  its saving of the body's SQLite error, which only fills in the reason on
  the log line; `migrate_local_time_offsets()`'s `PRAGMA user_version`
  read failures (SQLite would have to fail a pragma on a database it
  opened), its failed `PRAGMA user_version = 1` (it runs after
  `create_schema()` and the `UPDATE`s in the same transaction succeeded,
  so the database can be written), and the number of corrected values it
  logs (only the log line shows it); and `connect_db()`'s open failure, failed
  `create_schema()` and failed read of the last `event_seq`, which call
  `exit(1)`/`exit(2)` -- the app starts the
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
- **KML failed write or flush** (`kml_export.cpp`): a write or flush that
  fails partway through the temporary file needs a full or failing disk.
- **Branches with no observable effect or no way in**:
  - `lookup_on_facility_data()`'s stale-trip check: `lookup_on_facility_data_end()`
    makes the same check and drops the response, so removing only the first
    changes nothing a test can see (the second is covered).
  - `db_groups.cpp`'s prepare-failure returns in `queryAllGroups()` and
    `groupNameExists()`, and `db_history.cpp`'s in every query: SQLite treats stepping or finalizing a null
    statement as a harmless no-op, so the results are the same with or
    without them.
  - `queryTripData()`'s checks for a column missing from `trip_data`
    (`colDouble`/`colInt`/`colText` reading nothing, and a number field
    reading NaN, for an index of -1):
    `migrate_db()` adds every column before anything reads the table, so a
    column is missing only in a hand-edited database.
  - `engineCount()`'s guard (`simVarInt()`) on a NaN or out-of-range NUMBER
    OF ENGINES: MSVC's unguarded cast gives `INT_MIN` for those, which the
    clamp turns into the same 0 count. `tst_engine_power` checks that 0, but
    passes with the guard removed; the guard stays because the cast is
    undefined behavior.
  - `contact_analysis.cpp`'s guards against undefined behavior on an
    unknown value: `enumName()`'s `value >= 0` (a negative SURFACE TYPE or
    CONDITION would read before its name table) and `precipitationName()`'s
    NaN check (a NaN cast to `int`). `tst_contact_analysis` checks that -1
    and NaN give no name, but passes with either guard removed (MSVC read an
    empty name there, and casts NaN to `INT_MIN`, which has none of the
    precipitation bits); they stay because the read and the cast are
    undefined behavior.
  - `contact_analysis.cpp`'s wind range skipping a sample's NaN speed or
    direction: `std::min`/`std::max` return their first argument when the
    second is NaN, so the range comes out the same without either check.
    They stay so the loop doesn't depend on that argument order.
  - `sunElevationDegrees()`'s smallest equation-of-time term
    (`1.25 * eccentricity * eccentricity * sin(2m)`): it moves the sun by
    at most about 0.02°, below the 0.1° the prompt shows it to and the
    tests check it against, so a test passes without it. It stays because
    it is part of NOAA's equations.
  - `RUNWAY::clear()`'s `slope = 0` and `RUNWAY_OPERATION::true_heading`'s
    -1 default (`types.h`): a runway is filled from the facility data and
    an operation's heading set by every match (`match_runways()`) before
    `db_set_contact_airport()` stores them, so neither default reaches a
    stored contact.
  - `map.html`'s `hasValue()` checking that a value is a number, not just
    present: `map_script.cpp`'s `setKnown()` leaves out an unknown (NaN)
    value, so every key the page checks is a number or absent, and a test
    passes with the check reduced to presence.
  - `TripHistoryPanel::onTableContextMenu()`'s return when the menu has
    nothing in it (a right-click below the last row, or on a trip still
    recording): Qt shows no popup for an empty `QMenu::exec()` on the test
    platform either, so a test passed with the return removed and was
    dropped. It stays so no other platform shows an empty popup.
  - `AIRPORT::copy()` freeing a runway buffer the target already holds:
    it only keeps that buffer from leaking, which no test can observe.
  - `LiveStatusPanel::eventFilter()`'s non-toggle branch: the filter is only
    installed on the toggle.
  - `MapWidget::resetZoom()`'s `pageReady_` guard: without it the call runs
    `resetZoom();` on a page still loading, where the function and its map
    don't exist yet, so the call does nothing either way.
  - Lines the coverage tool attributes to a closing brace no path ends at:
    after `break` in `flight_phase.cpp`'s `RUNWAY`/`AIRPORT` case, after the
    `throw` in `db.cpp`'s `db_error()` and `db_insert_update_table()`'s
    `catch`, after `current_exception_message()`'s `return e.message`, and
    after `TripHistoryModel::data()`'s column `switch`, whose every case
    returns.
