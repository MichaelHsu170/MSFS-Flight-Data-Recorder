# MSFS Flight Data Recorder

A Qt desktop application for Microsoft Flight Simulator 2024 that records telemetry, cockpit events, and liftoff/landing data to a local SQLite database and visualises them on an interactive map with synchronised timeline charts. When no trip is selected the map shows all recorded routes as blue departure-to-destination line segments. Hovering over any chart shows a tooltip with the values of all curves in that chart at the cursor's time position.

![Overview map — recorded routes across Europe and Asia shown as color-coded departure-to-destination line segments, grouped by trip group (GlobalTravel, FlightTraining) with a legend, on a zoomed-out world map when no trip is selected](imgs/Screenshot%202026-08-22%20155717.png)

![Trajectory view — Fenix A320 circuit around Toulouse (LFBO) with the full-flight N1/N2 and vertical-speed charts below and a hover tooltip showing engine values at the cursor position](imgs/Screenshot%202026-07-05%20195501.png)

![Touchdown AI analysis](imgs/Screen%20Recording%202026-07-04%20094211.gif)

## Prerequisites

| Dependency | Version / Path |
|---|---|
| Visual Studio Build Tools | 18 (2026) at `C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools` |
| Qt (MSVC 2022 64-bit) | 6.11.1 at `C:\Qt\6.11.1\msvc2022_64` |
| MSFS 2024 SimConnect SDK | at `C:\MSFS 2024 SDK\SimConnect SDK` |

Qt modules required: `Widgets`, `Graphs`, `GraphsWidgets`, `Concurrent`, `Quick`, `QuickWidgets`, `QuickControls2`, `WebEngineWidgets`, `WebChannel`, `CoreTools`.

SQLite3 is bundled under `third_party/sqlite3/` — no separate install needed.

## Build with VS Code

Press **Ctrl+Shift+B** and pick **Debug**, **Release**, **Test**, or **Clean** from the menu. This works whether or not the CMake Tools extension is installed — the menu is plain VS Code tasks driving `cmake` directly, not the extension.

- **Debug** / **Release** — configure (first time only) and build into `build/Debug` or `build/Release`. On repeat builds, configure is skipped and only `cmake --build` runs, which keeps the lead time low; CMake's own `ZERO_CHECK` step still reconfigures automatically if `CMakeLists.txt` changes.
- **Test** — builds Debug, then runs the automated tests (see [Tests](#tests)).
- **Clean** — removes `build/Debug`, `build/Release`, and the `build/` folder itself.

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

See [tests/README.md](tests/README.md) for what is covered, what still needs a manual check, and suspected bugs the tests exposed.

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
| `flight_data.db` | SQLite database — created on first run, grows as flights are recorded |
| `settings.ini` | User preferences (panel sizes, table column widths, hidden data-table fields, log level, sample interval, automatic-recording toggle, Gemini API key) — created on first launch |
| `msfs_fdr_debug.log` | Unified log — started fresh on each launch, with the previous run's log kept as `msfs_fdr_debug.log.old`; level-filtered output from all modules (Qt, SimConnect, DB, map, charts) |

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
; Default: 500  (0.5 s — adequate for all aircraft types including fast jets
; at subsonic speeds; go lower only for supersonic recording needs).
sample_interval_ms=500

; Auto-managed by the app. Whether automatic recording is allowed to start,
; toggled via the Recording indicator in the Live Status panel.
enabled=true

[logging]
; Maximum log level written to msfs_fdr_debug.log.
; Levels (inclusive — each includes all levels above it):
;   FATAL    — unrecoverable errors only
;   WARNING  — unexpected conditions that don't abort the app
;   INFO     — operational events (connect, recording start/stop, liftoff, touchdown)
;   TRACE    — fine-grained diagnostic detail, e.g. raw Qt debug output (high-volume)
;   PROFILE  — performance timing for all subsystems (highest volume; for profiling only)
; Default: INFO
verbose=INFO

[layout]
; Width in pixels of the Live Status panel (top-right) and Data Table panel
; (bottom-right). Both columns share one value so they stay aligned when
; either splitter is dragged. Default: 260.
right_panel_width=260

; Height in pixels of the Charts panel (below the map). The map takes the
; remaining vertical space. Default: 400.
charts_panel_height=400

[data_table]
; Comma-separated list of field labels hidden in the Data Table panel via the
; Fields dialog. Absent or empty means all fields are visible.
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

`flight_data.db` is a SQLite database. At every launch `migrate_db()` creates any missing tables and indexes and adds any columns present in the current code but missing from the on-disk table (`ALTER TABLE ADD COLUMN`), so databases created by older builds are automatically upgraded without data loss. `connect_db()` repeats the same check when the simulator connects.

| Table | Contents |
|---|---|
| `trips` | One row per flight session: departure/destination airport ICAO, runway, times, ATC callsign |
| `trip_data` | Telemetry sampled at a configurable interval (default 0.5 s) while recording: 140 numeric variables (position, altitude, airspeed, engine N1/N2, gear/flaps/spoilers, fuel, …) plus 96 on/off states (autopilot modes, switches, warnings, …) bit-packed into three `bool_group_*` columns — the full list is in `trip_data_fields.h` |
| `trip_events` | Discrete cockpit events (gear up/down, flaps, spoilers, parking brake, anti-ice, etc.) with zulu and local timestamps |
| `trip_liftoffs` | One row per liftoff (the trip's departure, plus any touch-and-go): airport, runway (plus its real facility heading — usually a few degrees off the runway number), airspeed, vertical speed, pitch/bank/heading, wind direction/speed, lateral/longitudinal distance from the runway threshold and centreline, and the stored AI analysis report |
| `trip_touchdowns` | One row per touchdown: airport, runway (plus its real facility heading — usually a few degrees off the runway number), airspeed, vertical speed, g-force, pitch/bank/heading, wind direction/speed, lateral/longitudinal distance from the runway threshold and centreline, and the stored AI analysis report |
| `trip_groups` | User-defined trip groups: name and list order (`trips.group_id` references a row here; NULL = ungrouped) |

Recording starts automatically when an engine is running on the ground (unless automatic recording is disabled via the **Recording** indicator in the Live Status panel) and stops when all engines shut down on the ground or the simulator leaves the flight. A trip that ends abnormally (simulator crash or process kill before engine shutdown) is marked as **Open** in the UI.

## SimConnect

The app connects to MSFS 2024 via SimConnect and retries every 2 seconds until the simulator accepts the connection. The application name sent to MSFS is `"Flight Data Recorder"`.

Both Debug and Release link dynamically against `SimConnect.dll`. The DLL is copied next to the exe by the CMake post-build step and must be present at runtime for the live recording feature to work.

## AI Liftoff & Landing Analysis

Clicking a liftoff or touchdown marker on the map opens a popup with two panels:

- **Left** — raw telemetry for that liftoff or landing: airport, runway (with its real facility heading in parentheses, when known), airspeed, vertical speed, G-force (landings only), pitch/bank, heading, wind, threshold distance, and centreline offset.
- **Right** — an **Analyze Liftoff** / **Analyze Landing** button that streams a graded analysis from the Gemini AI model (`gemma-4-31b-it` via the Google Generative Language API). The prompt includes the runway's real heading (not just its two-digit number, which can be off by up to ~10°) so the model can calculate an accurate crosswind component.

The analysis is returned in a fixed structure:

```
Grade: A+ … F
Summary: 1–2 sentence overall impression
Strengths:  • …
Areas to improve:  • …
```

While the model is reasoning the toggle label reads **Thinking…** and is non-interactive. When the reasoning phase ends it collapses into a **Show thinking** / **Hide thinking** toggle so the final report is always the first thing visible. If the model returns an incomplete response the request is retried automatically up to three times before a plain-language error message is shown.

### Setup

1. Get a free API key at [aistudio.google.com](https://aistudio.google.com).
2. Open `settings.ini` in a text editor while the app is not running.
3. Set `gemini_api_key=YOUR_KEY` under `[ai]`.
4. Restart the app — the button becomes active on the next liftoff or touchdown popup.

## Trip Groups

Trips can be sorted into user-defined groups. **Manage Groups…** (above the trip table) adds, renames (double-click), deletes and reorders (drag) groups; right-clicking a trip row offers **Set Group**. The **Group** filter above the table limits the table and the overview map to one group, and the overview map colors each group's routes differently, with a legend.

## Export and Images

- **Export to KML** — available from a trip row's right-click menu and from the map's right-click menu while a trip is shown. The KML file contains the 3D flight path, a time-animated track, and liftoff, touchdown and event placemarks, for viewing in Google Earth.
- **Save Image** / **Copy Image** — the map's right-click menu saves or copies the whole visible map (trajectory, markers and tiles) as one image.

## Project Structure

```
MSFS-Flight-Data-Recorder/
├── MSFS-Flight-Data-Recorder/    C++ source
│   ├── main.cpp                  Entry point: log file, Qt style, window setup
│   ├── types.h                   Core C structs shared across all modules
│   ├── simconnect_defs.h         SimConnect enums, the cockpit event list (COCKPIT_EVENTS) and FLIGHT_DATA_RECORD
│   ├── recorder.h / .cpp         SimConnect dispatch callback (routes each message), cockpit event commit, DB writer shutdown
│   ├── sim_link.h / .cpp         What is asked of SimConnect: flight data definition, cockpit event registration and names, sample decoding
│   ├── flight_phase.h / .cpp     Trip start/stop, liftoff/touchdown detection, applying airport lookup results
│   ├── airport_lookup.h / .cpp   Which airport a departure/liftoff/touchdown was at: SimConnect facility requests, nearest candidates, fallbacks
│   ├── runway_match.h / .cpp     Which runway a liftoff/touchdown point is on, and its threshold/centerline distances
│   ├── event_filter.h / .cpp     Flood protection for cockpit events (fast bursts, slow repeats)
│   ├── recorder_bridge.h / .cpp  Qt wrapper: QTimer-driven dispatch, connection retry, Qt signals
│   ├── gui_notify.h              Free functions called by recorder.cpp, flight_phase.cpp and db.cpp to report state changes
│   ├── db.h / .cpp               SQLite write path: schema creation, the recorder's writes, buffered telemetry flush
│   ├── db_connection.h / .cpp    Read-only/read-write connections for the UI and background queries
│   ├── db_history.h / .cpp       Read-only queries: trip list, telemetry, events, liftoff points, touchdowns; trip deletion, AI reports
│   ├── db_groups.h / .cpp        Trip-group queries: list, add, rename, delete, reorder, assign a trip
│   ├── logger.h / .cpp           Unified logger: level-filtered (Fatal/Warning/Info/Trace/Profile), module-tagged output to msfs_fdr_debug.log
│   ├── logger_c.h                C-compatible shim (log_c / log_cf) for Qt-free translation units (db.cpp)
│   ├── app_settings.h / .cpp     QSettings wrapper for settings.ini
│   ├── trip_dataset.h            Shared data structs (TripSamplePoint, TripEvent, TripDataset, etc.) and helpers
│   ├── trip_data_fields.h        X-macro list of all trip_data columns (keeps live and historical paths in sync)
│   ├── version.h.in              Template for the generated version.h (APP_VERSION from CMakeLists.txt)
│   ├── main_window.h / .cpp      Top-level QMainWindow shell and cross-feature signal wiring
│   ├── live_status_panel.h/.cpp  Connection/recording status widget and scrolling log
│   ├── trip_history_panel.h/.cpp Trip list table with background dataset loading, group filter, trip deletion
│   ├── manage_groups_dialog.h/.cpp Dialog to add, rename, delete and reorder trip groups
│   ├── kml_export.h / .cpp       Builds and writes a trip's KML file
│   ├── splitter_utils.h          Helper to persist splitter sizes only when a drag ends
│   ├── trajectory_view.h / .cpp  Composite view: owns map, data table, and charts; cursor-sync wiring
│   ├── map_script.h / .cpp       The JavaScript calls sent to map.html, built from trip data
│   ├── map_widget.h / .cpp       QWebEngineView hosting map.html (Leaflet/OSM trajectory map)
│   ├── map_bridge.h / .cpp       QWebChannel QObject bridging JS ↔ C++ for the map
│   ├── chart_data.h / .cpp       Chart series, axis ranges and hover values, built from trip data
│   ├── charts_panel.h / .cpp     QQuickWidget hosting charts_panel.qml (timeline charts with hover tooltip)
│   ├── data_table_panel.h / .cpp Per-sample field/value table with hide-field dialog
│   └── resources/
│       ├── app.qrc / app.rc / app_icon.ico  Application icon and Windows version resource
│       ├── charts.qrc / map.qrc  Qt resource files bundling the QML and HTML below
│       ├── charts_panel.qml      QML layout for stacked timeline charts (N1/N2, speed, altitude, gear, etc.)
│       └── map.html              Leaflet map: trajectory polyline, liftoff/touchdown markers with AI analysis popup, event markers
├── tests/                        Automated tests (Qt Test + CTest) -- see tests/README.md
│   ├── fake_simconnect.h / .cpp  Stand-in for SimConnect.lib driven by made-up packets
│   ├── test_support.h / .cpp     Packet builders, FlightDriver, fake airport world, DB/dialog helpers
│   └── tst_*.cpp                 One test program per feature area
├── third_party/sqlite3/          Bundled SQLite3 (sqlite3.h, sqlite3.lib, sqlite3.dll)
├── .github/
│   ├── copilot-instructions.md   Code quality rules (single source of truth) and review entry point for Copilot
│   ├── diff-review.md            Canonical code-change review process
│   └── prompts/diff-review.prompt.md  Copilot /diff-review entry point
├── .claude/skills/diff-review/   Claude Code /diff-review entry point
├── CLAUDE.md                     Claude Code instructions (imports .github/copilot-instructions.md)
├── .vscode/
│   ├── tasks.json                Debug, Release, Test, Clean tasks (Ctrl+Shift+B)
│   ├── launch.json               Debug and Release launch configurations
│   ├── settings.json             Keeps the CMake Tools extension, if installed, from auto-configuring
│   └── scripts/
│       ├── build.bat             Sources vcvarsall.bat once, configures (if needed), builds, optionally runs the tests
│       └── clean.ps1             Removes the build/ directory
└── CMakeLists.txt                fdr_core library (all app code but main.cpp), the app, and tests/
```
