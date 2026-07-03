# MSFS Flight Data Recorder

A Qt desktop application for Microsoft Flight Simulator 2024 that records telemetry, cockpit events, and landing data to a local SQLite database and visualises them on an interactive map with synchronised timeline charts.

![Trajectory view](imgs/Screenshot%202026-07-04%20062158.jpg)

![Touchdown data](imgs/Screenshot%202026-07-04%20062220.jpg)

## Prerequisites

| Dependency | Version / Path |
|---|---|
| Visual Studio Build Tools | 18 (2026) at `C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools` |
| Qt (MSVC 2022 64-bit) | 6.11.1 at `C:\Qt\6.11.1\msvc2022_64` |
| MSFS 2024 SimConnect SDK | at `C:\MSFS 2024 SDK\SimConnect SDK` |

Qt modules required: `Widgets`, `Graphs`, `GraphsWidgets`, `Concurrent`, `Quick`, `QuickWidgets`, `QuickControls2`, `WebEngineWidgets`, `WebChannel`, `CoreTools`.

SQLite3 is bundled under `third_party/sqlite3/` — no separate install needed.

## Build with VS Code

Press **Ctrl+Shift+B** and select **Debug** or **Release** when prompted.

The build task automatically:
1. Runs CMake configure (generates a Visual Studio solution under `build/Debug` or `build/Release`)
2. Compiles with MSBuild via `with_vs_env.bat` (which sources `vcvarsall.bat` first)
3. Runs `windeployqt` to copy Qt DLLs, QML modules, and WebEngine resources next to the exe
4. Copies `sqlite3.dll` next to the exe
5. Copies `SimConnect.dll` next to the exe (both Debug and Release use dynamic linking)

Press **F5** to debug or **Ctrl+F5** to run without debugging.

### Clean

**Ctrl+Shift+P → Tasks: Run Task → CMake: Clean** — removes the entire `build/` directory.

## Manual Build (PowerShell)

Configure:
```powershell
.\.vscode\scripts\with_vs_env.bat `
  "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" `
  -S . -B build/Release -G "Visual Studio 18 2026" -A x64 `
  -DSIMCONNECT_DIR="C:\MSFS 2024 SDK\SimConnect SDK" `
  -DCMAKE_PREFIX_PATH="C:\Qt\6.11.1\msvc2022_64"
```

Build:
```powershell
.\.vscode\scripts\with_vs_env.bat `
  "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" `
  --build build/Release --config Release
```

## Output Locations

The binary is placed in a `bin/` subdirectory of the build tree:

| Config | Executable |
|---|---|
| Debug | `build/Debug/bin/MSFS-Flight-Data-Recorder.exe` |
| Release | `build/Release/bin/MSFS-Flight-Data-Recorder.exe` |

## Runtime Files

All runtime files follow the same rule: **Debug** builds use the **current working directory** (the project root when launched from VS Code); **Release** builds use the **directory containing the exe**.

| File | Purpose |
|---|---|
| `flight_data.db` | SQLite database — created on first run, grows as flights are recorded |
| `settings.ini` | User preferences (panel sizes, hidden data-table fields) — created on first change |
| `msfs_fdr_debug.log` | Qt message log — truncated on each launch; contains Qt warnings and internal app debug messages |
| `flight_data_recorder.log` | Flight recorder event log — appended across launches; contains SimConnect connection changes, recording start/stop, takeoff/touchdown events |

## settings.ini Reference

The file is a standard Windows INI edited automatically by the app as the user resizes panels or changes preferences. All values are human-readable integers or comma-separated strings and can be edited by hand while the app is not running.

```ini
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
```

## Database

`flight_data.db` is a SQLite database. The schema is created automatically on first launch and never migrated — tables and indexes use `CREATE TABLE/INDEX IF NOT EXISTS`.

| Table | Contents |
|---|---|
| `trips` | One row per flight session: departure/destination airport ICAO, runway, times, ATC callsign |
| `trip_data` | Telemetry sampled every 0.3 s while recording: position, altitude, airspeed, engine N1/N2, gear/flaps/spoilers, fuel, autopilot state, and ~60 other variables |
| `trip_events` | Discrete cockpit events (gear up/down, flaps, spoilers, parking brake, anti-ice, etc.) with zulu and local timestamps |
| `trip_touchdowns` | One row per touchdown: airport, runway, airspeed, vertical speed, g-force, pitch/bank/heading, wind direction/speed, and lateral/longitudinal distance from the runway threshold and centreline |

Recording starts automatically when an engine is running on the ground and stops when all engines shut down. A trip that ends abnormally (simulator crash or process kill before engine shutdown) is marked as **Open** in the UI.

## SimConnect

The app connects to MSFS 2024 via SimConnect and retries every 2 seconds until the simulator accepts the connection. The application name sent to MSFS is `"Flight Data Recorder"`.

Both Debug and Release link dynamically against `SimConnect.dll`. The DLL is copied next to the exe by the CMake post-build step and must be present at runtime for the live recording feature to work.

## Project Structure

```
MSFS-Flight-Data-Recorder/
├── MSFS-Flight-Data-Recorder/    C++ source
│   ├── main.cpp                  Entry point: log file, Qt style, window setup
│   ├── types.h                   Core C structs shared across all modules
│   ├── simconnect_defs.h         SimConnect event/definition enums and FLIGHT_DATA_RECORD
│   ├── recorder.h / .cpp         Raw SimConnect layer: data definitions, event subscriptions, dispatch callback
│   ├── recorder_bridge.h / .cpp  Qt wrapper: QTimer-driven dispatch, connection retry, Qt signals
│   ├── gui_notify.h              Free functions called by recorder.cpp to report state changes
│   ├── db.h / .cpp               SQLite write path: schema creation, buffered telemetry flush
│   ├── db_history.h / .cpp       Read-only queries: trip list, telemetry, events, touchdowns
│   ├── logger.h / .cpp           Appends timestamped text to flight_data_recorder.log
│   ├── app_settings.h / .cpp     QSettings wrapper for settings.ini
│   ├── trip_dataset.h            Shared data structs: TripSamplePoint, TripEvent, TripDataset, etc.
│   ├── trip_data_fields.h        X-macro list of all trip_data columns (keeps live and historical paths in sync)
│   ├── main_window.h / .cpp      Top-level QMainWindow shell and cross-feature signal wiring
│   ├── live_status_panel.h/.cpp  Connection/recording status widget and scrolling log
│   ├── trip_history_panel.h/.cpp Trip list table with background dataset loading and trip deletion
│   ├── trajectory_view.h / .cpp  Composite view: owns map, data table, and charts; cursor-sync wiring
│   ├── map_widget.h / .cpp       QWebEngineView hosting map.html (Leaflet/OSM trajectory map)
│   ├── map_bridge.h / .cpp       QWebChannel QObject bridging JS ↔ C++ for the map
│   ├── charts_panel.h / .cpp     QQuickWidget hosting charts_panel.qml (timeline charts)
│   ├── data_table_panel.h / .cpp Per-sample field/value table with hide-field dialog
│   └── resources/
│       ├── charts_panel.qml      QML layout for stacked timeline charts (N1/N2, speed, altitude, gear, etc.)
│       └── map.html              Leaflet map: trajectory polyline, touchdown markers, event markers
├── third_party/sqlite3/          Bundled SQLite3 (sqlite3.h, sqlite3.lib, sqlite3.dll)
├── .vscode/
│   ├── tasks.json                Configure, Build, Clean tasks
│   ├── launch.json               Debug and Run configurations
│   └── scripts/
│       ├── with_vs_env.bat       Sources vcvarsall.bat then invokes CMake/MSBuild
│       └── clean.ps1             Removes the build/ directory
└── CMakeLists.txt
```
