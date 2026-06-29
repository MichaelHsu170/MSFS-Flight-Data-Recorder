# MSFS-Flight-Data-Recorder

A flight data recorder for Microsoft Flight Simulator 2024, recording telemetry, events, and landing data to a SQLite database.

## Prerequisites

- **Visual Studio 18 BuildTools** (2026) at `C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools`
- **MSFS 2024 SDK** at `C:\MSFS 2024 SDK\SimConnect SDK`
- **VS Code** with the C/C++ extension

## Build with VS Code

Press **Ctrl+Shift+B** to build. You will be prompted to select **Debug** or **Release**.

The build tasks automatically:
1. Run CMake configure (generating project files in `build/Debug` or `build/Release`)
2. Compile using MSBuild via the MSVC environment wrapper

### Run / Debug

Press **F5** to debug or **Ctrl+F5** to run without debugging. You will be prompted to select Debug or Release.

The output binary is placed at:
- `build/Debug/bin/MSFS_Flight_Data_Recorder.exe`
- `build/Release/bin/MSFS_Flight_Data_Recorder.exe`

### Clean

Run the **CMake: Clean** task via **Ctrl+Shift+P → Tasks: Run Task → CMake: Clean**.

This removes the entire `build/` directory.

## Manual Build (PowerShell)

Configure:
```powershell
.\.vscode\scripts\with_vs_env.bat "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B build/Release -G "Visual Studio 18 2026" -A x64 -DSIMCONNECT_DIR="C:\MSFS 2024 SDK\SimConnect SDK"
```

Build:
```powershell
.\.vscode\scripts\with_vs_env.bat "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build/Release --config Release
```

## Project Structure

- `MSFS-Flight-Data-Recorder/` — C++ source code
- `third_party/sqlite3/` — Bundled SQLite3 library
- `.vscode/tasks.json` — VS Code build tasks (configure, build, clean)
- `.vscode/launch.json` — VS Code debug/run configurations
- `.vscode/scripts/with_vs_env.bat` — MSVC environment setup wrapper

## Database

The program creates `flight_data.db` in the working directory when launched. It records:
- `trips` — One row per flight session (departure/destination airport, times)
- `trip_data` — Telemetry sampled every 0.3 seconds while recording
- `trip_events` — Cockpit events (autopilot, flaps, gear, etc.)
- `trip_touchdowns` — Landing data per touchdown

Recording starts automatically when an engine is running on the ground, and stops when all engines are shut down.

## Notes

- SimConnect is linked **statically** for Release builds and **dynamically** for Debug builds
- `sqlite3.dll` is automatically copied to the output directory after each build
- The `with_vs_env.bat` script sources `vcvarsall.bat` from the BuildTools installation before invoking CMake
