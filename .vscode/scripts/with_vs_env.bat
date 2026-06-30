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

REM %1 must be the cmake executable (tasks.json always supplies the full path).
REM Run %* directly instead of extracting %1 and re-combining with the rest:
REM `shift` does NOT remove the first argument from %*, so "%CMAKE_CMD%" %*
REM would re-include it (duplicating the program) — and consuming %1/%2 via
REM substitution also splits any unquoted value on '=', ',' and ';', which
REM breaks flags like -DCMAKE_PREFIX_PATH=C:\Qt\... whenever the caller (e.g.
REM VS Code's PowerShell-backed shell tasks) passes them without quotes.
REM %* sidesteps both problems by forwarding the original command line as-is.
if "%~1"=="" (
  echo Error: with_vs_env.bat requires the cmake executable as its first argument.
  exit /b 1
)

echo Running: %*
%*

endlocal
