@echo off
REM build.bat <Debug|Release> [test [rebuild]]
REM
REM Single-process build task driven by VS Code's Ctrl+Shift+B menu (see
REM ..\tasks.json). It sets up the MSVC x64 environment once, then
REM configures build\<Config> only the first time (when CMakeCache.txt is
REM missing) before building it. Skipping configure on warm builds is what
REM keeps repeat Ctrl+Shift+B presses fast -- the generated solution's
REM ZERO_CHECK project still reruns configure on its own whenever
REM CMakeLists.txt or other configure inputs actually change. With "test" as
REM the second argument, a successful build is followed by the automated tests
REM (ctest.exe, which ships next to cmake.exe -- see tests\README.md). With
REM "rebuild" as the third argument, everything is recompiled first
REM (--clean-first), so the output shows every file's compiler warnings, not
REM just those of the files that were out of date -- the diff review's
REM warning check (.github\diff-review.md) relies on that.

setlocal enabledelayedexpansion

set "CONFIG=%~1"
if "%CONFIG%"=="" (
  echo Error: build.bat requires a configuration argument ^(Debug or Release^).
  exit /b 1
)

set "ROOT=%~dp0..\.."
set "BUILD_DIR=%ROOT%\build\%CONFIG%"
set "CMAKE=C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"

REM --- Locate and source the MSVC x64 dev environment (needed by both cmake
REM     configure's compiler checks and the MSBuild invocation that
REM     `cmake --build` shells out to). ---
set "PF86=%ProgramFiles(x86)%"
set "VSINSTALL="
if defined VSINSTALLDIR set "VSINSTALL=%VSINSTALLDIR%"

if not defined VSINSTALL if exist "!PF86!\Microsoft Visual Studio\18\BuildTools" set "VSINSTALL=!PF86!\Microsoft Visual Studio\18\BuildTools"
if not defined VSINSTALL if exist "!PF86!\Microsoft Visual Studio\18\Community" set "VSINSTALL=!PF86!\Microsoft Visual Studio\18\Community"
if not defined VSINSTALL if exist "!PF86!\Microsoft Visual Studio\18\Professional" set "VSINSTALL=!PF86!\Microsoft Visual Studio\18\Professional"
if not defined VSINSTALL if exist "!PF86!\Microsoft Visual Studio\18\Enterprise" set "VSINSTALL=!PF86!\Microsoft Visual Studio\18\Enterprise"

if defined VSINSTALL (
  if exist "!VSINSTALL!\VC\Auxiliary\Build\vcvarsall.bat" (
    call "!VSINSTALL!\VC\Auxiliary\Build\vcvarsall.bat" amd64 >nul
  ) else if exist "!VSINSTALL!\Common7\Tools\VsDevCmd.bat" (
    call "!VSINSTALL!\Common7\Tools\VsDevCmd.bat" -arch=amd64 >nul
  ) else (
    echo Warning: vcvarsall.bat/VsDevCmd.bat not found under !VSINSTALL!; continuing without VS env.
  )
) else (
  echo Warning: Visual Studio 18 installation not found; continuing without VS env.
)

if not exist "%BUILD_DIR%\CMakeCache.txt" (
  echo Configuring %CONFIG% into build\%CONFIG% ...
  "%CMAKE%" -S "%ROOT%" -B "%BUILD_DIR%" -G "Visual Studio 18 2026" -A x64 -DSIMCONNECT_DIR="C:\MSFS 2024 SDK\SimConnect SDK" -DCMAKE_PREFIX_PATH="C:\Qt\6.11.1\msvc2022_64"
  if errorlevel 1 exit /b 1
)

set "CLEAN_FIRST="
if /i "%~3"=="rebuild" set "CLEAN_FIRST=--clean-first"
echo Building %CONFIG% ...
"%CMAKE%" --build "%BUILD_DIR%" --config %CONFIG% %CLEAN_FIRST%
set "BUILD_RESULT=%errorlevel%"
if not "%BUILD_RESULT%"=="0" endlocal & exit /b %BUILD_RESULT%

if /i "%~2"=="test" (
  for %%I in ("%CMAKE%") do set "CTEST=%%~dpIctest.exe"
  echo Testing %CONFIG% ...
  "!CTEST!" --test-dir "%BUILD_DIR%" -C %CONFIG% --output-on-failure
  set "BUILD_RESULT=!errorlevel!"
)
endlocal & exit /b %BUILD_RESULT%
