# MSFS-Flight-Data-Recorder

## Build with CMake (Visual Studio Code)

Prerequisites:
- Install CMake (>= 3.15) and the "CMake Tools" extension for VS Code.
- If you use SimConnect, set `SIMCONNECT_DIR` to your MSFS SimConnect SDK root when configuring.

Quick steps (from the workspace root):

1. Create a build directory and configure CMake (from PowerShell):

```powershell
cmake -S . -B build -G "Visual Studio 18 2026" -A x64 -DSIMCONNECT_DIR="C:\MSFS SDK\SimConnect SDK"
```

2. Build (multi-config Visual Studio generator):

```powershell
cmake --build build --config Release
```

3. Run or debug from VS Code using the provided launch configuration (`.vscode/launch.json`).

**Recommended: Use Ctrl+Shift+B** to build automatically via the VS Code tasks (see `.vscode/tasks.json`).

Notes:
- The repository includes `sqlite3.lib` and `sqlite3.dll` under `MSFS-Flight-Data-Recorder/sqlite3` which the CMake script links against.
- If you don't have SimConnect or don't set `SIMCONNECT_DIR`, CMake will configure but you'll get a warning; the build may fail if the code requires SimConnect symbols.

CMake not found / PATH troubleshooting
- If the VS Build Tools installed CMake but VS Code says `cmake` is not found, either add CMake to your system `PATH` or provide the full path to `cmake.exe` in the build task.
- To check if `cmake` is on PATH, run in PowerShell:

```powershell
where cmake
```

- Common install locations to check for `cmake.exe`:
	- `C:\Program Files\CMake\bin\cmake.exe`
	- `C:\Program Files (x86)\CMake\bin\cmake.exe`

- To add CMake to PATH (temporary, PowerShell only):

```powershell
$env:Path += ";C:\\Program Files\\CMake\\bin"
```

- Or permanently add the folder to System Environment Variables → Path, then restart VS Code.

- Alternatively, press Ctrl+Shift+B in VS Code, choose the `CMake: Build` task, and when prompted for `Full path to cmake.exe or just 'cmake' if it's in PATH` enter the full path to `cmake.exe`.

- This workspace includes a helper script `.vscode/scripts/with_vs_env.bat` that attempts to locate Visual Studio via `vswhere` and run `vcvarsall.bat` to set the MSVC build environment before invoking `cmake`.
- The VS Code tasks now call this script automatically. If `vswhere` is available on your machine (usually at `C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe`), the tasks will auto-detect your Visual Studio installation and set up the environment for `cmake` and the MS build tools.

- If neither `vswhere` nor `cmake` are present, the helper attempts to locate Visual Studio in common installation folders (checking VS 18 BuildTools first, then 2022, 2019 as fallbacks) and will try `VsDevCmd.bat` or `vcvarsall.bat` to set the environment.
- If CMake is not on PATH, the wrapper also looks for `C:\Program Files\CMake\bin\cmake.exe` or `C:\Program Files (x86)\CMake\bin\cmake.exe` before falling back to the literal `cmake` command.

- If the wrapper cannot find Visual Studio or CMake automatically, either:
	- Install CMake and/or the Visual Studio Installer (or the full Visual Studio/Build Tools) and ensure their paths are available, or
	- Provide the full path to `cmake.exe` when prompted by the task and set the `VSINSTALLDIR` environment variable to the Visual Studio installation path before launching VS Code.
