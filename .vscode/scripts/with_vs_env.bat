@echo off
REM with_vs_env.bat
REM Usage: with_vs_env.bat <cmake-exe-or-cmake> <cmake args...>

setlocal enabledelayedexpansion

set "PF86=%ProgramFiles(x86)%"
set "PF=%ProgramFiles%"
set "VSWHERE=%PF86%\Microsoft Visual Studio\Installer\vswhere.exe"
set "VSINSTALL="

REM vswhere detection removed to avoid parsing issues when paths contain parentheses

REM Fallback to VSINSTALLDIR environment variable
if not defined VSINSTALL if defined VSINSTALLDIR (
  set "VSINSTALL=%VSINSTALLDIR%"
  echo Using VSINSTALLDIR from environment
)

REM Fallback: check common Visual Studio install paths (check newest first)
REM Check version 18 (2026), then 2022, 2019 as fallbacks
if not defined VSINSTALL (
  if exist "!PF86!\Microsoft Visual Studio\18\BuildTools" set "VSINSTALL=!PF86!\Microsoft Visual Studio\18\BuildTools"
  if not defined VSINSTALL if exist "!PF86!\Microsoft Visual Studio\18\Community" set "VSINSTALL=!PF86!\Microsoft Visual Studio\18\Community"
  if not defined VSINSTALL if exist "!PF86!\Microsoft Visual Studio\18\Professional" set "VSINSTALL=!PF86!\Microsoft Visual Studio\18\Professional"
  if not defined VSINSTALL if exist "!PF86!\Microsoft Visual Studio\18\Enterprise" set "VSINSTALL=!PF86!\Microsoft Visual Studio\18\Enterprise"
)

if defined VSINSTALL (
  if exist "!VSINSTALL!\VC\Auxiliary\Build\vcvarsall.bat" (
    echo Setting up Visual Studio environment from !VSINSTALL!
    call "!VSINSTALL!\VC\Auxiliary\Build\vcvarsall.bat" amd64 >nul
  ) else if exist "!VSINSTALL!\Common7\Tools\VsDevCmd.bat" (
    echo Setting up Visual Studio environment from !VSINSTALL! (VsDevCmd)
    call "!VSINSTALL!\Common7\Tools\VsDevCmd.bat" -arch=amd64 >nul
  ) else (
    echo Warning: vcvarsall.bat or VsDevCmd.bat not found in !VSINSTALL!; continuing without VS env
  )
)

REM First argument is the cmake executable or 'cmake'. If not provided, try to discover cmake.exe in common locations.
set "CMAKE_CMD=%~1"
shift

if "%CMAKE_CMD%"=="" (
  set "CMAKE_CMD=cmake"
  REM Try common CMake installation locations if cmake is not on PATH
  where cmake >nul 2>&1
  if errorlevel 1 (
    if exist "%PF%\CMake\bin\cmake.exe" set "CMAKE_CMD=%PF%\CMake\bin\cmake.exe"
    if exist "%PF86%\CMake\bin\cmake.exe" set "CMAKE_CMD=%PF86%\CMake\bin\cmake.exe"
  )
)

REM Collect remaining arguments into ARGS variable (preserves quotes)
set "ARGS="
:collect_args
if "%~1"=="" goto args_collected
set "ARGS=%ARGS% %1"
shift
goto collect_args
:args_collected

echo Running: "%CMAKE_CMD%"%ARGS%
"%CMAKE_CMD%"%ARGS%

endlocal
