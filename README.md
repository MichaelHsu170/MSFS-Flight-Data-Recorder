# MSFS Flight Data Recorder

A Qt desktop application for Microsoft Flight Simulator 2024 that records telemetry, cockpit events, and liftoff/landing data to a local SQLite database and visualises them on an interactive map with synchronised timeline charts. When no trip is selected the map shows all recorded routes as departure-to-destination line segments colored by trip group, and each chart is blank with a "No trip selected" message ("No data recorded" for a trip with no data, such as one just starting to record). Hovering over any chart shows a tooltip with the values of all curves in that chart at the cursor's time position.

![Overview map: recorded routes across Europe and Asia shown as color-coded departure-to-destination line segments, grouped by trip group (GlobalTravel, FlightTraining) with a legend, on a zoomed-out world map when no trip is selected](imgs/Screenshot%202026-08-22%20155717.png)

![Trajectory view: Fenix A320 circuit around Toulouse (LFBO) with the full-flight engine power (N1/N2 for this jet) and vertical-speed charts below and a hover tooltip showing engine values at the cursor position](imgs/Screenshot%202026-07-05%20195501.png)

![Touchdown AI analysis](imgs/Screen%20Recording%202026-07-04%20094211.gif)

## Prerequisites

| Dependency | Version / Path |
|---|---|
| Visual Studio or its Build Tools | 18 (2026), any edition, with the C++ x64 tools (`build.bat` finds it through the Visual Studio Installer) |
| CMake | 3.22 or newer (the tests set `ENVIRONMENT_MODIFICATION`), on PATH or Visual Studio's bundled copy ("C++ CMake tools for Windows") |
| Qt (MSVC 2022 64-bit) | 6.11.1 at `C:\Qt\6.11.1\msvc2022_64` |
| MSFS 2024 SimConnect SDK | at `C:\MSFS 2024 SDK\SimConnect SDK` |

Qt modules required: `Widgets`, `Graphs`, `Concurrent`, `Quick`, `QuickWidgets`, `QuickControls2`, `WebEngineWidgets`, `WebChannel`, `CoreTools`.

SQLite3 is bundled under `third_party/sqlite3/`, so no separate install is needed. The map's Leaflet 1.9.4 (BSD 2-Clause, its `LICENSE` alongside) is bundled under `third_party/leaflet/` and built into the app, so it isn't fetched at run time; the map tiles still come from the internet.

## Build with VS Code

Press **Ctrl+Shift+B** and pick **Debug**, **Release**, **Test**, **Coverage**, or **Clean** from the menu. This works whether or not the CMake Tools extension is installed: the menu is plain VS Code tasks driving `cmake` directly, not the extension.

- **Debug** / **Release**: configure (first time only) and build into `build/Debug` or `build/Release`. On repeat builds, configure is skipped and only `cmake --build` runs, which keeps the lead time low; CMake's own `ZERO_CHECK` step still reconfigures automatically if `CMakeLists.txt` changes.
- **Test**: builds Debug, then runs the automated tests (see [Tests](#tests)).
- **Coverage**: runs the tests of an up-to-date Debug build under OpenCppCoverage and writes `coverage_report/index.html` and `coverage.xml` (not committed).
- **Clean**: removes `build/Debug`, `build/Release`, and the `build/` folder itself.

Each build automatically:
1. Runs CMake configure the first time (generates a Visual Studio solution under `build/Debug` or `build/Release`)
2. Compiles with MSBuild via `build.bat` (which sources `vcvarsall.bat` once)
3. Runs `windeployqt` to copy Qt DLLs, QML modules, and WebEngine resources next to the exe
4. Copies `sqlite3.dll` next to the exe
5. Copies `SimConnect.dll` next to the exe (both Debug and Release use dynamic linking)

Press **F5** to debug (choose the **Debug** or **Release** launch configuration) or **Ctrl+F5** to run without debugging.

## Tests

Automated tests run the app's code against a fake SimConnect with made-up flight data, so no simulator is needed. Pick **Test** from the Ctrl+Shift+B menu, or run:

```powershell
.\.vscode\scripts\build.bat Debug test
```

Add `rebuild` as the third argument (`build.bat Debug test rebuild`, or `build.bat Debug notest rebuild` to skip the tests) to recompile everything first, so every file's compiler warnings show. `build.bat Debug only tst_map_widget tst_map_script` builds and runs just the named test programs. Any other extra argument is an error. The app and the tests compile at warning level 4 (`/W4`); warnings from Qt's and the SimConnect SDK's headers are turned off.

See [tests/README.md](tests/README.md) for what is covered, what still needs a manual check, and open questions for real sim data.

## Manual Build (PowerShell)

Configure and build Release in one step:
```powershell
.\.vscode\scripts\build.bat Release
```

Or drive CMake yourself:
```powershell
cmake -S . -B build/Release -G "Visual Studio 18 2026" -A x64 `
  -DSIMCONNECT_DIR="C:\MSFS 2024 SDK\SimConnect SDK" `
  -DCMAKE_PREFIX_PATH="C:\Qt\6.11.1\msvc2022_64"
cmake --build build/Release --config Release
```
(Run from a Developer PowerShell/Command Prompt for VS 2026, or call `build.bat` above which sets that environment up for you.)

## Output Locations

The binary is placed in a `bin/` subdirectory of the build tree:

| Config | Executable |
|---|---|
| Debug | `build/Debug/bin/MSFS-Flight-Data-Recorder.exe` |
| Release | `build/Release/bin/MSFS-Flight-Data-Recorder.exe` |

The test executables go to `build/Debug/tests/` and `build/Release/tests/`.

## Runtime Files

All runtime files follow the same rule: **Debug** builds use the **current working directory** (the project root when launched from VS Code); **Release** builds use the **directory containing the exe**.

| File | Purpose |
|---|---|
| `flight_data.db` | SQLite database; created on first run, grows as flights are recorded |
| `settings.ini` | User preferences (panel sizes, table column widths, hidden data-table fields, log level, sample interval, automatic-recording toggle, Gemini API key); created on first launch |
| `msfs_fdr_debug.log` | Unified log, started fresh on each launch, with the previous run's log kept as `msfs_fdr_debug.log.old`; level-filtered output from all modules (Qt, SimConnect, DB, map, charts) |

## settings.ini Reference

The file is a standard Windows INI edited automatically by the app as the user resizes panels or changes preferences. All values are human-readable integers or comma-separated strings and can be edited by hand while the app is not running.

```ini
[ai]
; Gemini API key for the AI liftoff/landing analysis feature.
; Obtain a free key from Google AI Studio (aistudio.google.com), then paste it
; here and restart the app. The app never writes this value.
; Without a key the Analyze Liftoff and Analyze Landing buttons are disabled.
gemini_api_key=

[recording]
; Maximum time between telemetry samples written to trip_data, in milliseconds.
; Lower values produce finer trajectory and chart resolution at the cost of
; a larger database and slower trip load times. Must be a positive integer.
; Default: 500  (0.5 s -- adequate for all aircraft types including fast jets
; at subsonic speeds; go lower only for supersonic recording needs).
sample_interval_ms=500

; Auto-managed by the app. Whether automatic recording is allowed to start,
; toggled via the Recording indicator in the Live Status panel. Disabling it
; only prevents a new trip from starting; it doesn't stop one already in
; progress. Default: true.
enabled=true

[logging]
; Maximum log level written to msfs_fdr_debug.log.
; Levels (inclusive -- each includes all levels above it):
;   FATAL    -- unrecoverable errors only
;   WARNING  -- unexpected conditions that don't abort the app
;   INFO     -- operational events (connect, recording start/stop, liftoff, touchdown)
;   TRACE    -- fine-grained diagnostic detail, e.g. raw Qt debug output (high-volume)
;   PROFILE  -- performance timing for all subsystems (highest volume; for profiling only)
; Default: INFO
verbose=INFO

; Auto-managed by the app.
[layout]
; Width in pixels of the Live Status panel (top-right) and Data Table panel
; (bottom-right). Both columns share one value so they stay aligned when
; either splitter is dragged. Default: 260.
right_panel_width=260

; Height in pixels of the Charts panel (below the map). The map takes the
; remaining vertical space. Default: 400.
charts_panel_height=400

; Auto-managed by the app.
[data_table]
; Comma-separated list of field labels hidden in the Data Table panel via the
; Visible Fields dialog. Absent or empty means all fields are visible.
hidden_fields=

; Auto-managed by the app. Persisted column widths for the tables in the
; UI that support user resizing.
[table_column_width]
; Width in pixels of the Field column in the Data Table panel. The Value
; column always stretches to fill the rest. Default: 140.
data_table_field_column_width=140

; Column widths in pixels for the Trip History table, as comma-separated
; key=value pairs keyed by TripHistoryModel::Column enum member name (e.g.
; TitleColumn=120). Columns using Stretch sizing are never stored. Unknown
; or missing keys fall back to that column's coded default.
trip_history_column_widths=
```

## Database

`flight_data.db` is a SQLite database. At every launch, in the background once the window has opened, `migrate_db()` creates any missing tables and indexes and adds any columns present in the current code but missing from the on-disk table (`ALTER TABLE ADD COLUMN`), so databases created by older builds are automatically upgraded without data loss. The trip list and the simulator connection wait for it to finish, with a "Checking the database" notice in the trip list's place meanwhile. The longest upgrade, moving the per-engine values of a database from before `trip_engine_data` into that table, rebuilds `trip_data` by copying its rows into a new table in batches (one pass, where dropping the columns in place would rewrite the table once per column), and the notice then shows the percentage of that rebuild done: copying the rows, dropping the old table, committing and recreating the indexes, each weighted by roughly its share of the time. Older builds recorded fewer engine values, and the upgrade moves what they did record: every per-engine value of engines 1 and 2, and for engines 3 and 4 only the engine power the chart showed (piston RPM / manifold pressure, jet N1 / N2, turboprop prop RPM / torque, helicopter turbine N1 / torque). A value an older build didn't record stays empty and shows blank. Databases from builds that stored local times with the UTC offset's sign reversed (UTC+2 as `-02:00`) are corrected once: every stored local time is rewritten in one transaction, with no percentage shown, and closing the window doesn't stop it (the app waits for it to finish before exiting). If a table or column can't be added or that rebuild or correction fails (the error is in `msfs_fdr_debug.log`), the notice says so and stays, the simulator isn't connected, and the next launch tries again. `connect_db()` repeats the same check when the simulator connects.

| Table | Contents |
|---|---|
| `trips` | One row per flight session: departure/destination airport ICAO, runway, times, ATC callsign |
| `trip_data` | Telemetry sampled at a configurable interval (default 0.5 s) while recording, one row per sample keyed by `id`: 114 numeric variables (position, altitude, airspeed, gear/flaps/spoilers, fuel, weather, …) plus 71 on/off states (autopilot modes, switches, warnings, …) bit-packed into three `bool_group_*` columns; the full list is in `trip_data_fields.h` |
| `trip_engine_data` | Each sample's per-engine values, one row per engine the aircraft has (its NUMBER OF ENGINES, up to the 16 engines MSFS reports), keyed by the sample's `trip_data.id` and the engine number: 32 values (N1, N2, RPM, prop RPM, manifold pressure, torque, oil, fuel flow, EGT, …, and on/off states such as combustion, fire and starter), as MSFS sends them; the full list is `TRIP_ENGINE_FIELDS` in `trip_data_fields.h` |
| `trip_events` | Discrete cockpit events (gear up/down, flaps, spoilers, parking brake, anti-ice, etc.) with zulu and local timestamps |
| `trip_liftoffs` | One row per liftoff (the trip's departure, plus any touch-and-go): airport, runway (plus its real facility heading, usually a few degrees off the runway number), airspeed, vertical speed (indicated, and the actual one: NULL for rows recorded before it), pitch/bank/heading, wind direction/speed, lateral/longitudinal distance from the runway threshold and centreline, the matched runway's length, width, displaced threshold at the end used, start position, exact true heading and slope (NULL for rows recorded before these columns, or without a matched runway), and the stored AI analysis report |
| `trip_touchdowns` | One row per touchdown: airport, runway (plus its real facility heading, usually a few degrees off the runway number), airspeed, vertical speed (indicated, and the actual one: NULL for rows recorded before it), g-force, pitch/bank/heading, wind direction/speed, lateral/longitudinal distance from the runway threshold and centreline, the matched runway's length, width, displaced threshold at the end used, start position, exact true heading and slope (NULL for rows recorded before these columns, or without a matched runway), and the stored AI analysis report |
| `trip_groups` | User-defined trip groups: name and list order (`trips.group_id` references a row here; NULL = ungrouped) |

Recording starts automatically when any of the aircraft's engines is running on the ground (unless automatic recording is disabled via the **Recording** indicator in the Live Status panel) and stops when all of them are shut down on the ground or the simulator leaves the flight. A trip that ends abnormally (simulator crash or process kill before engine shutdown) is marked as **Open** in the UI.

A liftoff is recorded when the simulator's on-ground flag clears and the aircraft then stays airborne for at least 0.5 s, or rises at least 0.5 ft above the height it left the ground at, whichever comes first. The row has the values of the moment it left the ground, not of when it was confirmed. Back on the ground before then, it counts as never having left: no liftoff and no touchdown are recorded. This drops the flicker of the flag when the gear skims the runway (touching down fast, with the wings still carrying most of the weight), which would otherwise record a burst of liftoffs and touchdowns a fraction of a second apart. A real bounce, where the aircraft rises off the runway, is still recorded as a liftoff and a second touchdown. A very short, shallow one (under 0.5 s, rising less than 0.5 ft) counts as part of the same touchdown. Trips recorded before this filter keep the contacts they were recorded with.

## SimConnect

The app connects to MSFS 2024 via SimConnect and retries every 2 seconds until the simulator accepts the connection. The application name sent to MSFS is `"Flight Data Recorder"`.

Both Debug and Release link dynamically against `SimConnect.dll`. The DLL is copied next to the exe by the CMake post-build step and must be present at runtime for the live recording feature to work.

## AI Liftoff & Landing Analysis

Clicking a liftoff or touchdown marker on the map opens a popup with two panels:

- **Left**: raw telemetry for that liftoff or landing: airport (ICAO code, with its name in parentheses), runway (with its real facility heading in parentheses, when known), coordinate, airspeed, vertical speed, touchdown rate (landings only, when known), G-force (landings only), pitch/bank, heading, crab angle, threshold distance and centreline offset (when a runway was matched), wind, and Zulu/local time. The KML export's placemark descriptions list the same fields, without the coordinate.
  - **Vertical speed** is the actual one (the simulator's `VELOCITY WORLD Y`). The cockpit's indicated vertical speed (`VERTICAL SPEED`) lags it, so at a liftoff it can still read negative. A liftoff or touchdown recorded before the actual one was stored shows the indicated one, marked "(indicated)". The charts and the AI prompt's trajectory use the same: the actual vertical speed, or for a trip recorded before it, the indicated one (the prompt says which). The Live Status panel shows the actual one too, and the Data Table both, as recorded: **Vertical Speed** (indicated) and **Velocity World Y** (actual, blank for samples recorded before it).
  - **Touchdown rate** is the sink rate perpendicular to the runway surface as the wheels touched, as the simulator measures it (`PLANE TOUCHDOWN NORMAL VELOCITY`, ft/min, positive into the surface). The simulator keeps the last touchdown's value until the next one and can update it a sample after the on-ground flag sets, so the rate is the first new value from the touchdown's sample up to the next liftoff or touchdown. Without one (no sample before the touchdown, or the value never changed), the row is left out. Trips recorded before this version have it too, since every sample recorded the value.
  - Every heading is marked magnetic (**M**) or true (**T**), magnetic first. The runway shows both, e.g. "08 (74° M, 62° T)": the runway number comes from the magnetic heading, and the true one compares with the wind's (true). The magnetic one is the runway's true heading converted with the magnetic variation from the trip's sample at that moment. The aircraft's heading shows the recorded magnetic heading, then in brackets the true one from the same sample (e.g. "334° M (338° T)"), so each compares directly with the runway's. The difference that remains is the **crab** angle: heading minus ground track, "2° R" when the nose points right of the track. A liftoff or touchdown without a matching sample shows only the runway's true heading, the aircraft's magnetic heading, and no crab. The KML export and the AI prompt (as "magnetic"/"true") use the same order.
- **Right**: an **Analyze Liftoff** / **Analyze Landing** button that streams a graded analysis from the Gemini AI model (`gemma-4-31b-it` via the Google Generative Language API), and a **Copy Prompt** button that copies the prompt Analyze would send, final trajectory included, to the clipboard, to use with another model. Copy Prompt works without an API key.

The prompt sends the popup's values, every heading in true, including the runway's real heading (not just its two-digit number, which can be off by up to ~10°), so the model can calculate an accurate crosswind component. It also sends what the trip's samples show around the liftoff or touchdown, derived from them (`db/contact_analysis.cpp`), so trips recorded before this feature get it too: the conditions when the trip loads, the final trajectory when its Analyze or Copy Prompt button is clicked.

- **Conditions**: time, the sun's elevation and the light (day, civil/nautical/astronomical twilight, night; computed from the position and time, since at high latitude the same clock time can be midnight sun or polar night), position and magnetic variation, ground elevation, outside air temperature, altimeter setting, station pressure and the actual sea-level pressure (so a wrong altimeter setting shows), pressure and density altitude, visibility, whether the aircraft was in cloud, precipitation and its rate, the wind's range over the 30 s before, and the surface type and condition. Sea-level pressure, cloud and precipitation are recorded only by this version onward, so older trips don't have them.
- **Runway**, when one was matched: its length, width, displaced threshold at the end used and slope (uphill or downhill in the direction of travel), and the runway remaining ahead of the touchdown or liftoff in ft and % of its length. These come from the simulator's facility data when the contact is recorded, so liftoffs and touchdowns recorded before this feature don't have them.
- **Final trajectory** (for a liftoff, called the takeoff trajectory):
  - **Gates**: the last crossing before a touchdown of 1000, 500, 300, 200, 100 and 50 ft and of 3, 2 and 1 nm, each distance gate with its path angle; or the first crossing after a liftoff of 35, 100, 200, 500 and 1000 ft. A gate that wasn't crossed is left out.
  - **Samples**: every sample from 15 s before to 10 s after a touchdown, or from 10 s before to 10 s after a liftoff, at most 60 (evenly thinned when there are more).
  - Each gate and sample also says whether the simulator has the aircraft on the ground or airborne, and, per engine, the throttle lever position as the aircraft reports it (negative in the reverse range) and its power: N1 (%) for a jet, torque (%) for a turboprop or helicopter turbine, and manifold pressure (inHg) for a piston, which shows its power whether the propeller is fixed-pitch or constant-speed (where the governor holds the RPM). Another engine type gets the throttle only. With these the model can tell a bounce (the throttle at idle) from a touch-and-go or go-around (the throttle forward).
  - **Highest terrain** under the 5 nm before the touchdown or after the liftoff.
  - With the runway data, each gate and sample also gives its distance along the runway from the marked threshold (touchdown) or the runway start (liftoff), negative before it, and its distance right or left of the extended centerline.
  - **Heights are measured from the touchdown or liftoff point, not radio height.** Radio height follows the terrain under the aircraft (a valley, cliff or the sea before the runway), so each point also gives the terrain below it.
  - The gates and highest terrain stop at the trip's previous or next liftoff/touchdown, so a touch-and-go's circuit isn't counted twice. The samples don't: they cover their fixed seconds whatever happened in them, so a bounce, go-around or touch-and-go shows in them for the model to judge. Each touchdown of a bounced landing gets its own trajectory.
  - **When it can't be built**, because the trip has no samples, or a time in its window (the contact's or a sample's) can't be read, the analysis covers the liftoff or touchdown itself only. The prompt tells the model so, and a notice under the button tells you.

Until the answer starts, a status line in the result area says what it's waiting for: "Building the approach trajectory…" (or climb-out), "Waiting N s for the per-minute token limit…" when it has to (below), "Sending to LLM…", then "Waiting for the model…", counting retries ("Retrying (2/3): …").

**Per-minute token limit**: the API key's free tier takes at most 16,000 prompt tokens per minute, so the app never sends more than that. Before each request, retries included, it adds up the prompts sent in the last 60 s, across all analyses, and if this one doesn't fit, it waits, counting down, until enough of them are a minute old.
- A prompt's tokens are estimated at 2 characters each before it's sent. The prompts measured 2.2-3.1 characters per token, since Gemma's tokenizer gives each digit its own token.
- Once the answer starts, the service's own count replaces the estimate, for that prompt and its retries.
- A rate-limit error (another app using the same key, say) counts as the whole minute used, so its retry waits a minute instead of failing again at once.
- Only the prompts count: the limit is on input tokens. The count starts over when the map page is reloaded or the app restarts.

A value that wasn't recorded is left out, never sent as 0. The model is also told to describe the airport's terrain, procedures and hazards from its general knowledge, labeled as such. It must check each one against the runway end used and the recorded trajectory before applying it, and say when it doesn't know the airport. The prompt is bounded by construction: at most 32,000 characters with every value present for all 16 engines MSFS reports, the 16,000 tokens of the per-minute limit at the estimate's 2 characters per token, so a prompt always fits it.

The analysis is returned in a fixed structure:

```
Grade: A+ … F
Summary: 1-2 sentence overall impression
Strengths:  • …
Areas to improve:  • …
Final segment:  • stability at the gates, and the flare, decrab, touchdown and
                  rollout, including any bounce (a landing's), or
Rotation and climb-out:  • the rotation and initial climb (a liftoff's)
Environment:  • how the conditions, airport, runway and terrain affected it
```

Reports saved before this change keep their old format; analyzing again gives the new one.

While the model is reasoning the toggle label reads **Thinking…** and is non-interactive. When the reasoning phase ends it collapses into a **Show thinking** / **Hide thinking** toggle so the final report is always the first thing visible. A model that answers without reasoning shows just the report, with no toggle. A response counts as complete when the model says it finished and the report isn't empty; if it's cut off or empty, the AI service can't be reached, or it reports a server error or rate limit (retried after waiting out the minute, above), the request is made up to three times in all before a plain-language error message is shown. A rejected request (e.g. an invalid API key) is not retried.

### Setup

1. Get a free API key at [aistudio.google.com](https://aistudio.google.com).
2. Open `settings.ini` in a text editor while the app is not running.
3. Set `gemini_api_key=YOUR_KEY` under `[ai]`.
4. Restart the app. The button becomes active on the next liftoff or touchdown popup.

## Trip Groups

Trips can be sorted into user-defined groups. **Manage Groups…** (above the trip table) adds, renames (double-click), deletes and reorders (drag) groups, and says so if it can't open the database to list them; right-clicking a trip row offers **Set Group**. The **Group** filter above the table limits the table and the overview map to one group, and the overview map colors each group's routes differently, with a legend.

## Export and Images

- **Export to KML**: available from a trip row's right-click menu and from the map's right-click menu while a trip is shown. The KML file contains the 3D flight path, a time-animated track, and liftoff, touchdown and event placemarks, for viewing in Google Earth.
- **Save Image** / **Copy Image**: the map's right-click menu saves or copies the whole visible map (trajectory, markers and tiles) as one image.

## Project Structure

```
MSFS-Flight-Data-Recorder/
├── MSFS-Flight-Data-Recorder/    C++ source, one folder per feature (every folder is on the include path)
│   ├── app/                      Application startup and shared services
│   │   ├── main.cpp              Entry point: log file, Qt style, window setup
│   │   ├── logger.h / .cpp       Unified logger: level-filtered (Fatal/Warning/Info/Trace/Profile), module-tagged output to msfs_fdr_debug.log
│   │   ├── app_paths.h / .cpp    Where settings.ini, flight_data.db and the log live (working dir in Debug, exe folder in Release)
│   │   ├── app_settings.h / .cpp QSettings wrapper for settings.ini
│   │   ├── version.h.in          Template for the generated version.h (APP_VERSION from CMakeLists.txt)
│   │   └── app.qrc / app.rc / app_icon.ico  Application icon and Windows version resource
│   ├── recording/                Recording a flight from SimConnect
│   │   ├── types.h               Core C structs shared across all modules
│   │   ├── simconnect_defs.h     SimConnect enums, the cockpit event list (COCKPIT_EVENTS) and the flight data SimVars (FLIGHT_DATA_FIELDS), which declare FLIGHT_DATA_RECORD
│   │   ├── recorder.h / .cpp     SimConnect dispatch callback (routes each message), cockpit event commit, DB writer shutdown
│   │   ├── sim_link.h / .cpp     What is asked of SimConnect: flight data definition, cockpit event registration and names, sample decoding
│   │   ├── flight_phase.h / .cpp Trip start/stop, liftoff/touchdown detection, applying airport lookup results
│   │   ├── airport_lookup.h / .cpp  Which airport a departure/liftoff/touchdown was at: SimConnect facility requests, nearest candidates, fallbacks
│   │   ├── runway_match.h / .cpp Which runway a liftoff/touchdown point is on, and its threshold/centerline distances
│   │   ├── event_filter.h / .cpp Flood protection for cockpit events (fast bursts, slow repeats)
│   │   ├── engine_power.h / .cpp Engine count, combustion, and each engine type's charted N1/N2 equivalents, labels and units
│   │   ├── trip_data_fields.h    X-macro list of all trip_data columns (keeps the schema, writes, reads and data table in sync)
│   │   ├── recorder_bridge.h / .cpp  Qt wrapper: QTimer-driven dispatch, connection retry, Qt signals
│   │   └── gui_notify.h          Free functions called by recorder.cpp, flight_phase.cpp, airport_lookup.cpp and db.cpp to report state changes
│   ├── db/                       The SQLite database
│   │   ├── db.h / .cpp           SQLite write path: schema creation, the recorder's writes, buffered telemetry flush
│   │   ├── db_connection.h / .cpp  Read-only/read-write connections for the UI and background queries
│   │   ├── db_query.h / .cpp     Statement helpers shared by the UI-side queries: row loop, single statements, transactions (also the schema migrations')
│   │   ├── db_history.h / .cpp   Read-only queries: trip list, telemetry, events, liftoff points, touchdowns; trip deletion, AI reports
│   │   ├── contact_analysis.h / .cpp  Each liftoff's/touchdown's conditions (on load) and final trajectory (on analysis), derived from the trip's samples
│   │   ├── db_groups.h / .cpp    Trip-group queries: list, add, rename, delete, reorder, assign a trip
│   │   └── trip_dataset.h        Shared data structs (TripSamplePoint, TripEvent, TripDataset, etc.) and helpers
│   ├── ui/                       The main window and its panels
│   │   ├── main_window.h / .cpp  Top-level QMainWindow shell, startup database migration notice, cross-feature signal wiring
│   │   ├── live_status_panel.h/.cpp  Connection/recording status widget and scrolling log
│   │   ├── trip_history_panel.h/.cpp Trip list table with background dataset loading, group filter, trip deletion
│   │   ├── manage_groups_dialog.h/.cpp Dialog to add, rename, delete and reorder trip groups
│   │   ├── trajectory_view.h / .cpp  Composite view: owns map, data table, and charts; cursor-sync wiring
│   │   ├── data_table_panel.h / .cpp Per-sample field/value table with Visible Fields dialog
│   │   ├── splitter_utils.h      Splitter helpers: act once a drag ends; size the second section
│   │   ├── charts/               Timeline charts
│   │   │   ├── chart_data.h / .cpp   Chart series, axis ranges and hover values, built from trip data
│   │   │   ├── charts_panel.h / .cpp QQuickWidget hosting charts_panel.qml (timeline charts with hover tooltip)
│   │   │   ├── charts_panel.qml  QML layout for stacked timeline charts (engine power, speed, altitude, gear, etc.)
│   │   │   └── charts.qrc        Qt resource file bundling charts_panel.qml
│   │   └── map/                  Trajectory map
│   │       ├── map_widget.h / .cpp   QWebEngineView hosting map.html (Leaflet/OSM trajectory map)
│   │       ├── map_bridge.h / .cpp   QWebChannel QObject bridging JS ↔ C++ for the map
│   │       ├── map_script.h / .cpp   The JavaScript calls sent to map.html, built from trip data
│   │       ├── map.html          Leaflet map: trajectory polyline, liftoff/touchdown markers with AI analysis popup, event markers
│   │       └── map.qrc           Qt resource file bundling map.html and the Leaflet library (third_party/leaflet/)
│   └── export/                   KML export
│       ├── kml_export.h / .cpp   Builds and writes a trip's KML file
│       └── kml_export_dialog.h/.cpp  Export to KML for both menus: save dialog, background export, error box
├── tests/                        Automated tests (Qt Test + CTest) -- see tests/README.md
│   ├── fake_simconnect.h / .cpp  Stand-in for SimConnect.lib driven by made-up packets
│   ├── test_support.h / .cpp     Packet builders, FlightDriver, fake airport world, DB/dialog/mouse helpers
│   ├── local_time_zone.h         Runs a test program in US Pacific time (for the zulu/local time tests)
│   ├── data/ai_stream_response.json  A real streamed AI response, recorded once from the service
│   └── <app source folder>/tst_*.cpp  One test program per feature area, in the folder (app/, db/, export/, recording/, ui/, ui/map/, ui/charts/) of the code it tests
├── third_party/sqlite3/          Bundled SQLite3 (sqlite3.h, sqlite3.lib, sqlite3.dll)
├── third_party/leaflet/          Bundled Leaflet 1.9.4 (leaflet.js, leaflet.css, images, LICENSE), built into the app
├── .github/
│   ├── copilot-instructions.md   Code quality rules (single source of truth) and review entry point for Copilot
│   ├── diff-review.md            Canonical code-change review process
│   └── prompts/diff-review.prompt.md  Copilot /diff-review entry point
├── .claude/skills/diff-review/   Claude Code /diff-review entry point
├── CLAUDE.md                     Claude Code instructions (imports .github/copilot-instructions.md)
├── .vscode/
│   ├── tasks.json                Debug, Release, Test, Coverage, Clean tasks (Ctrl+Shift+B)
│   ├── launch.json               Debug and Release launch configurations
│   ├── settings.json             Keeps the CMake Tools extension, if installed, from auto-configuring
│   ├── c_cpp_properties.json     IntelliSense include paths (incl. the SimConnect SDK), defines and compiler
│   └── scripts/
│       ├── build.bat             Sources vcvarsall.bat once, configures (if needed), builds, optionally runs the tests
│       ├── clean.ps1             Removes the build/ directory
│       ├── coverage.ps1          Runs the tests under OpenCppCoverage and writes a line-coverage report
│       └── review_copy.ps1       Syncs build/review-copy, the copy the diff review breaks code in to check coverage
└── CMakeLists.txt                fdr_core library (all app code but main.cpp), the app, and tests/
```
