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
; Without a key the Analyze buttons are disabled.
gemini_api_key=
; Auto-managed by the app. The model the AI analysis uses, chosen in the
; drop-down beside the Analyze button: gemma-4-31b-it, gemma-4-26b-a4b-it,
; or other for the one named in custom_model. Default: gemma-4-31b-it.
model=gemma-4-31b-it
; Auto-managed by the app. The model name entered for the drop-down's Other
; choice (model=other).
custom_model=
; Auto-managed by the app. Whether the AI analysis is a progress evaluation
; (the whole climb-out or approach) or, false, a contact evaluation (the
; liftoff or touchdown alone), as the popup's checkbox left it. Default: true.
progress_evaluation=true

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
| `trip_data` | Telemetry sampled at a configurable interval (default 0.5 s) while recording, one row per sample keyed by `id`: 123 numeric variables (position, altitude, airspeed, gear/flaps/spoilers, fuel, weather, …) plus 71 on/off states (autopilot modes, switches, warnings, …) bit-packed into three `bool_group_*` columns; the full list is in `trip_data_fields.h` |
| `trip_engine_data` | Each sample's per-engine values, one row per engine the aircraft has (its NUMBER OF ENGINES, up to the 16 engines MSFS reports), keyed by the sample's `trip_data.id` and the engine number: 32 values (N1, N2, RPM, prop RPM, manifold pressure, torque, oil, fuel flow, EGT, …, and on/off states such as combustion, fire and starter), as MSFS sends them; the full list is `TRIP_ENGINE_FIELDS` in `trip_data_fields.h` |
| `trip_events` | Discrete cockpit events (gear up/down, flaps, spoilers, brakes, parking brake, anti-ice, etc.; each brake application, held or tapped, is stored once; frequent braking is still subject to flood protection; trips recorded with the v2.1.0 and v2.1.1 releases show only short brake taps, as those dropped a held brake entirely) with zulu and local timestamps |
| `trip_liftoffs` | One row per liftoff (the trip's departure, plus any touch-and-go): airport, runway (plus its real facility heading, usually a few degrees off the runway number), airspeed, vertical speed (indicated, and the actual one: NULL for rows recorded before it), pitch/bank/heading, wind direction/speed (these whole-number values rounded from the contact's sample; rows recorded by v2.1.1 and earlier truncated them), lateral/longitudinal distance from the runway threshold and centreline, the matched runway's length, width, displaced threshold at the end used, start position, exact true heading and slope (NULL for rows recorded before these columns, or without a matched runway), and the stored AI analysis report |
| `trip_touchdowns` | One row per touchdown: airport, runway (plus its real facility heading, usually a few degrees off the runway number), airspeed, vertical speed (indicated, and the actual one: NULL for rows recorded before it), g-force, pitch/bank/heading, wind direction/speed (these whole-number values rounded from the contact's sample; rows recorded by v2.1.1 and earlier truncated them), lateral/longitudinal distance from the runway threshold and centreline, the matched runway's length, width, displaced threshold at the end used, start position, exact true heading and slope (NULL for rows recorded before these columns, or without a matched runway), and the stored AI analysis report |
| `trip_groups` | User-defined trip groups: name and list order (`trips.group_id` references a row here; NULL = ungrouped) |

Recording starts automatically when any of the aircraft's engines is running on the ground (unless automatic recording is disabled via the **Recording** indicator in the Live Status panel) and stops when all of them are shut down on the ground or the simulator leaves the flight. A trip that ends abnormally (simulator crash or process kill before engine shutdown) is marked as **Open** in the UI.

A liftoff is recorded when the simulator's on-ground flag clears and the aircraft then stays airborne for at least 0.5 s, or rises at least 0.5 ft above the height it left the ground at, whichever comes first. The row has the values of the moment it left the ground, not of when it was confirmed. Back on the ground before then, it counts as never having left: no liftoff and no touchdown are recorded. This drops the flicker of the flag when the gear skims the runway (touching down fast, with the wings still carrying most of the weight), which would otherwise record a burst of liftoffs and touchdowns a fraction of a second apart. A real bounce, where the aircraft rises off the runway, is still recorded as a liftoff and a second touchdown. A very short, shallow one (under 0.5 s, rising less than 0.5 ft) counts as part of the same touchdown. Trips recorded before this filter keep the contacts they were recorded with.

## SimConnect

The app connects to MSFS 2024 via SimConnect and retries every 2 seconds until the simulator accepts the connection. The application name sent to MSFS is `"Flight Data Recorder"`.

Both Debug and Release link dynamically against `SimConnect.dll`. The DLL is copied next to the exe by the CMake post-build step and must be present at runtime for the live recording feature to work.

## AI Liftoff & Landing Analysis

Clicking a liftoff or touchdown marker on the map opens a popup with two panels. A touchdown where the aircraft crashed (the simulator reset the flight within 10 s after it, with no other liftoff or touchdown in between) is marked with a red icon, an airplane nose-down into the runway, instead of the orange 🛬, its popup is titled **Crash**, as is its KML placemark, and its prompt says it crashed and that nothing after the crash was flown. A crash with no touchdown (into terrain while airborne, say) gets no mark. A teleport or slew that sets the aircraft down on the ground is also marked as a crash, since the recording can't tell it from the simulator's reset after one. The liftoff the simulator's reset makes (it puts the aircraft back in the air) isn't shown on the map, in the profile or in the KML export; its row stays in the database.

- **Left**: raw telemetry for that liftoff or landing: airport (ICAO code, with its name in parentheses), runway (with its real facility heading in parentheses, when known), coordinate, airspeed, vertical speed, touchdown rate (landings only, when known), G-force (landings only), pitch/bank, heading, crab angle, threshold distance and centreline offset (when a runway was matched), wind, and Zulu/local time. The KML export's placemark descriptions list the same fields, without the coordinate.
  - **Vertical speed** is the actual one (the simulator's `VELOCITY WORLD Y`). The cockpit's indicated vertical speed (`VERTICAL SPEED`) lags it, so at a liftoff it can still read negative. A liftoff or touchdown recorded before the actual one was stored shows the indicated one, marked "(indicated)". The charts and the AI prompt's trajectory use the same: the actual vertical speed, or for a trip recorded before it, the indicated one (the prompt says which). The Live Status panel shows the actual one too, and the Data Table both, as recorded: **Vertical Speed** (indicated) and **Velocity World Y** (actual, blank for samples recorded before it).
  - **Touchdown rate** is the sink rate perpendicular to the runway surface as the wheels touched, as the simulator measures it (`PLANE TOUCHDOWN NORMAL VELOCITY`, ft/min, positive into the surface). The simulator keeps the last touchdown's value until the next one and can update it a sample after the on-ground flag sets, so the rate is the first new value from the touchdown's sample up to the next liftoff or touchdown. Without one (no sample before the touchdown, or the value never changed), the row is left out. Trips recorded before this version have it too, since every sample recorded the value.
  - Every heading is marked magnetic (**M**) or true (**T**), magnetic first. The runway shows both, e.g. "08 (74° M, 62° T)": the runway number comes from the magnetic heading, and the true one compares with the wind's (true). The magnetic one is the runway's true heading (the exact facility one where stored, otherwise the whole-degree one) converted with the magnetic variation from the trip's sample at that moment. The aircraft's heading shows the recorded magnetic heading, then in brackets the true one from the same sample (e.g. "334° M (338° T)"), so each compares directly with the runway's. The difference that remains is the **crab** angle: heading minus ground track, "2° R" when the nose points right of the track. A liftoff or touchdown without a matching sample shows only the runway's true heading, the aircraft's magnetic heading, and no crab. The KML export and the AI prompt (as "magnetic"/"true") use the same order. With a runway heading, the prompt also tells the model that a magnetic-versus-true difference is the magnetic variation, not a misalignment, and to judge the alignment and crab only from the true heading and track against the runway's true heading; without a true aircraft heading (no sample), not to judge the alignment from the heading.
- **Right**: an **Analyze** button that streams a graded analysis from a model on the Google Generative Language API, then the **Progress evaluation** checkbox over a drop-down choosing that model, then a **Copy Prompt** button that copies the prompt Analyze would send to the clipboard, exactly as Analyze would send it with the checkbox as it is, to use with another model. Both buttons are as tall as the checkbox and drop-down together. Copy Prompt, and so the checkbox, works without an API key. While an analysis runs, the checkbox, the drop-down and the Other field are disabled, as the analysis was started with them, also in its popup reopened meanwhile.
  - **Progress evaluation** (checked by default; its tooltip, shown as soon as the mouse is over the checkbox, explains both choices): the analysis covers the whole climb-out or approach, with the conditions, the runway data and the final trajectory described below, and its answer adds the segment and Environment sections. Unchecked, it's a **contact evaluation**: the liftoff or touchdown alone, the scope the prompt had before those were added. It sends only the popup's values (airport, runway, airspeed, vertical speed (left out for a crash, whose sample after the contact reads 0), touchdown rate, G-force, pitch and bank, heading and crab, wind, and the position on the runway) and the heading rule, builds no trajectory, and asks for the grade, summary, strengths and areas to improve: a shorter prompt (less of the per-minute token limit below) and a quicker answer. The choice is saved under `[ai]` in settings.ini (`progress_evaluation`) as soon as it changes, and every popup opens with it, also after a restart.
  - **Model**: `gemma-4-31b-it` (the default), `gemma-4-26b-a4b-it`, or **Other…**, which shows a field for any other model's name, e.g. one listed in Google AI Studio. To avoid typos, copy the name with the **Copy to clipboard** button in AI Studio's model selection dialog and paste it there. Analyze waits for a name, and one that isn't one of the two listed (choose those from the list). The choice and the Other name are saved under `[ai]` in settings.ini (`model`, `custom_model`) as soon as they change, and every popup opens with them, also after a restart. A model the service doesn't know is reported as API error 404, naming the model.

The prompt sends the popup's values, every heading in true, including the runway's real heading (not just its two-digit number, which can be off by up to ~10°), so the model can calculate an accurate crosswind component. Without a matched runway, both evaluations' prompts say there are no headwind or crosswind components and to judge the wind against the aircraft's true ground track, and their closing list of what to consider leaves out what needs a runway (the crosswind component, centerline tracking, runway length and remaining, slope, touchdown zone). With a runway matched but its true heading, length or slope not recorded (a contact from before they were stored), it also leaves out what needs that value: the crosswind component, the runway remaining, the slope. It also sends what the trip's samples show around the liftoff or touchdown, derived from them (`db/contact_analysis.cpp`), so trips recorded before this feature get it too: the conditions when the trip loads, the final trajectory when its Analyze or Copy Prompt button is clicked. A progress evaluation sends all of the following; a contact evaluation none of it.

- **Conditions**: time, the sun's elevation and the light (day, civil/nautical/astronomical twilight, night; computed from the position and time, since at high latitude the same clock time can be midnight sun or polar night), position and magnetic variation, ground elevation, outside air temperature, altimeter setting, station pressure and the actual sea-level pressure (so a wrong altimeter setting shows), pressure and density altitude, visibility, whether the aircraft was in cloud, precipitation and its rate, the wind's range over the 30 s before, the surface type and condition, and the aircraft's total weight and fuel weight. Sea-level pressure, cloud and precipitation are recorded only from the version that added them, so older trips don't have them.
- **Runway**, when one was matched: its length, width, displaced threshold at the end used and slope (uphill or downhill in the direction of travel), and the runway remaining ahead of the touchdown or liftoff in ft and % of its length. These come from the simulator's facility data when the contact is recorded, so liftoffs and touchdowns recorded before this feature don't have them.
- **Final trajectory** (for a liftoff, called the liftoff trajectory):
  - **Gates**: the last crossing before a touchdown of 1000, 500, 300, 200, 100 and 50 ft and of 3, 2 and 1 nm, each distance gate with its path angle (to the next closer gate, over the distance flown, so a curved approach doesn't read as steep); or the first crossing after a liftoff of 35, 100, 200, 500 and 1000 ft. A gate that wasn't crossed is left out.
  - **Samples**: every sample from 15 s before to 10 s after a touchdown, or from 10 s before to 10 s after a liftoff, at most 56 (evenly thinned when there are more; at the default 0.5 s sample interval there are at most 51). Then, on the ground side, one sample per second out to 30 s from the contact: the rollout after a touchdown (+11 to +30 s) and the takeoff roll before a liftoff (-30 to -11 s), each second's sample farthest from the contact, so the model sees the deceleration or the acceleration to rotation.
  - Each gate and sample gives its airspeed and ground speed, its vertical speed and G-force, its pitch, bank, heading and track, and says whether the simulator has the aircraft on the ground or airborne, and, per engine (up to 4; for an aircraft with more, 4 spread evenly from the first to the last, which the header names, e.g. "engines 1/3/6/8 of 8 shown", as all 16 MSFS reports would push the prompt past its size limit), the throttle lever position as the aircraft reports it (negative in the reverse range) and its power: N1 (%) for a jet, torque (%) for a turboprop or helicopter turbine, and manifold pressure (inHg; "MP" on each point, spelled out once in the header to keep the prompt within its size limit) for a piston, which shows its power whether the propeller is fixed-pitch or constant-speed (where the governor holds the RPM). Another engine type gets the throttle only. With these the model can tell a bounce (the throttle at idle) from a touch-and-go or go-around (the throttle forward). When the samples recorded the engines' failed and on-fire states, one line names every engine (of all of them, not only the 4 shown) that failed or was on fire at any gate or sample, or says none was.
  - **Weather at each height gate** (not the distance gates, which pass through nearly the same air, about 950/640/320 ft on a 3° path): the wind (true) and its headwind/tailwind and crosswind components against the runway's true heading (the exact one where stored; the trajectory's header tells the model these are relative to the runway, not the aircraft's heading), the vertical wind (up or down, ft/min), the outside air temperature, whether the aircraft was in cloud, the precipitation, and the structural (airframe) ice.
  - **Approach weather and ride** (touchdown: from the last crossing of 1000 ft above the touchdown point down to it) or **climb-out weather and ride** (liftoff: from it up to the first crossing of 1000 ft), stopping at 5 nm from the contact or at the previous or next liftoff/touchdown if that comes first, so a flight that stays low isn't summarized whole:
    - the wind speed range, and the largest headwind change in the segment (from its lowest to its highest headwind, in the order they were flown, with their heights: the two closest in time where the wind held at either), marked as **wind shear** at 15 kt or more;
    - the strongest tailwind and crosswind;
    - the lowest height flown in cloud;
    - **icing conditions** (in cloud or precipitation at -20 to +10 °C), the most structural and pitot ice, and the share of samples with engine anti-ice on (labeled as carburetor heat on a piston engine);
    - the **G-force range while airborne**, as a turbulence proxy. Each value is the instant of its sample (every 0.5 s by default), so a bump between samples is missed: the range is a lower bound on how rough it was. Marked for the model as moderate turbulence or maneuvering at 0.5 G or more from 1 G either way, and as severe turbulence or hard maneuvering at 1.0 G or more.
    - the **vertical wind** range (AMBIENT WIND Y, ft/min, + up: downdrafts and updrafts), with the height where each end was first reached, marked as strong beyond 1000 ft/min either way, with a caution that a swing while the aircraft pitches or rolls fast may be the simulator's reading rather than the air;
    - the airborne sample **closest to the stall**: its angle of attack (INCIDENCE ALPHA) against the stall angle (STALL ALPHA, the angle of the maximum lift), with its height, airspeed and bank, marked as stalled at or past it. When it stalled, this is the first sample at or past the stall angle, where the stall began (not the deepest point after it), and the G-force and vertical wind ranges above stop there: past it the aircraft may be out of control (a crash's tumble), and those readings no longer show the air it flew through.
    - The model is told to judge the crab (and a touchdown's decrab) against the crosswind, the speed additive or rotation and climb speed against the gusts, any excursion against the wind shear, vertical wind and turbulence, and the margin to the stall.
    - Cloud and precipitation are recorded only from the version that added them, and structural ice, vertical wind and angle of attack only from this version onward, so older trips leave those out.
    - Also recorded from this version, but not yet in the prompt until a flight shows how the simulator reports them: the wind's X/Z components (to check whether AMBIENT WIND VELOCITY includes the vertical wind, as a 2023 developer report says), the sideslip (INCIDENCE BETA), and MAX/MIN G FORCE (the simulator's peak G, whose reset behavior the SDK doesn't document).
  - **Highest terrain** under the 5 nm before the touchdown or after the liftoff, or that there's none above the touchdown or liftoff point. It's searched in all the recorded samples, past the previous or next liftoff/touchdown too. When the recording doesn't cover those 5 nm (it starts or ends within them, or the simulator was reset), the line gives how far the search reached instead, so "none above" isn't read as covering terrain no sample was recorded over.
  - With the runway data, each gate and sample also gives its distance along the runway from the marked threshold (touchdown) or the runway start (liftoff), negative before it, and its distance right or left of the extended centerline.
  - **Heights are measured from the touchdown or liftoff point, not radio height.** Radio height follows the terrain under the aircraft (a valley, cliff or the sea before the runway), so each point also gives the terrain below it.
  - The gates stop at the trip's previous or next liftoff/touchdown, so a touch-and-go's circuit isn't counted twice. The samples don't: they cover their fixed seconds whatever happened in them, so a bounce, go-around or touch-and-go shows in them for the model to judge. Each touchdown of a bounced landing gets its own trajectory.
  - **Nothing reaches past a reset**: a sample further from the one before it, up, down or across, than twice the faster of the two samples' vertical or ground speeds over the time between them, plus 100 ft. That's the simulator's reset after a crash (which can't be turned off), or a teleport or slew. A move while the simulator's clock is set back (the time not moving forward) isn't judged one. What follows one was never flown, so the gates, samples, highest terrain, weather and ride, the 30 s wind range and the touchdown rate all stop at it. A bounce between the last sample and a crash's reset has nothing recorded before the reset, so its liftoff has no conditions or trajectory.
  - **When it can't be built**, because the trip has no samples, or a time in its window (the contact's or a sample's) can't be read, the analysis covers the liftoff or touchdown itself only. The prompt tells the model so, and a notice under the button tells you.

Until the answer starts, a status line in the result area says what it's waiting for: "Building the approach trajectory…" (or climb-out; a progress evaluation only), "Sending to gemma-4-31b-it…" (the model asked), then "Waiting for the model…". A retry is counted and says why the attempt before it failed: "Retry 2/3 (got error 500): Sending to gemma-4-31b-it…", or "network error", "stalled" (nothing for 3 minutes, below) or "answer incomplete" in the brackets.

**Rate limit**: the API key's free tier takes at most 16,000 prompt tokens per minute, and limits the requests per minute and per day. The app doesn't count them itself: when the service turns a request down with a rate-limit error (HTTP 429), the retry waits as long as the error says (its RetryInfo delay, "34s" say), or a minute when it doesn't say, counting down in the status line ("Retry 2/4 (got error 429): Waiting 34 s for the rate limit…"), then sends it again. A wait doesn't use up one of the 3 attempts, so a rate limit on the last attempt is still waited out; a 4th rate limit, after 3 such waits, ends the analysis with "Still rate-limited after 3 waits", and Analyze stays disabled while the wait it asks for counts down ("Analyze again in 34 s."); then it's enabled again, without sending by itself. A delay longer than a minute (a per-day quota used up, say) isn't waited out: the error is shown at once, with the wait the service asked for.

A value that wasn't recorded is left out, never sent as 0. In a progress evaluation, the model is also told to describe the airport's terrain, procedures and hazards from its general knowledge, labeled as such. It must check each one against the runway end used and the recorded trajectory before applying it, and say when it doesn't know the airport. It's also told to cite in the Environment section each significant recorded condition, including those marked above, with its value and effect, and elsewhere to keep what the conditions caused apart from technique, neither faulting the pilot for it nor excusing technique by it. The prompt is bounded by construction: at most 32,000 characters with every value present for all 16 engines MSFS reports: within the per-minute limit's 16,000 tokens even at 2 characters per token (the prompts measured 2.2-3.1, since Gemma's tokenizer gives each digit its own token), so a prompt on its own always fits it.

The analysis is returned in a fixed structure:

```
Grade: A+ … F
Summary: 1-2 sentence overall impression
Strengths:  • …
Areas to improve:  • …
Final segment:  • stability at the gates, and the flare, decrab, touchdown and
                  rollout, including any bounce (a landing's), or
Liftoff and climb-out:  • what the samples show the liftoff to be (a takeoff,
                          a touch-and-go or a bounce), then the rotation and
                          initial climb, or a bounce as part of the landing
                          (a liftoff's)
Environment:  • how the conditions, airport, runway and terrain affected it
```

Reports saved before this change keep their old format; analyzing again gives the new one.

While the model is reasoning the toggle label reads **Thinking…** and is non-interactive. When the reasoning phase ends it collapses into a **Show thinking** / **Hide thinking** toggle so the final report is always the first thing visible. A model that answers without reasoning shows just the report, with no toggle. A response counts as complete when the model says it finished and the report isn't empty; if it's cut off or empty, nothing more of it arrives for 3 minutes (the model streams its reasoning as it goes, so a silence that long means a stalled request), the AI service can't be reached, or it reports a server error or rate limit (retried after the wait it asks for, above), the request is made up to three times in all (a rate limit's waits add up to three more, above) before a plain-language error message is shown. A rejected request (e.g. an invalid API key) is not retried. Each failed attempt is written to the debug log as a warning under `MapJS`, saying whether it's retried, why it failed (the server error, the connection error, or the finish reason), any error the service sent mid-answer, how many characters of thinking and answer had arrived, whether it stalled, how long the first data took and the longest wait between pieces. A complete answer logs the same two times as info, so the timeout can be checked against real waits.

### Setup

1. Get a free API key at [aistudio.google.com](https://aistudio.google.com).
2. Open `settings.ini` in a text editor while the app is not running.
3. Set `gemini_api_key=YOUR_KEY` under `[ai]`.
4. Restart the app. The button and the model drop-down become active on the next liftoff or touchdown popup.

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
