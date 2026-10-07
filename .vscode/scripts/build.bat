@echo off
REM build.bat <Debug|Release> [test|notest [rebuild]]
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
REM warning check (.github\diff-review.md) relies on that. To rebuild without
REM testing, pass "notest" second. Any other second or third argument is an
REM error, so a misplaced "rebuild" (build.bat Release rebuild) isn't silently
REM ignored.

setlocal enabledelayedexpansion

set "CONFIG=%~1"
if "%CONFIG%"=="" (
  echo Error: build.bat requires a configuration argument ^(Debug or Release^).
  exit /b 1
)
set "ARGS_OK=1"
if not "%~2"=="" if /i not "%~2"=="test" if /i not "%~2"=="notest" set "ARGS_OK="
if not "%~3"=="" if /i not "%~3"=="rebuild" set "ARGS_OK="
if not "%~4"=="" set "ARGS_OK="
if not defined ARGS_OK (
  echo Error: usage: build.bat ^<Debug^|Release^> [test^|notest [rebuild]]
  exit /b 1
)

set "ROOT=%~dp0..\.."
set "BUILD_DIR=%ROOT%\build\%CONFIG%"

REM --- Locate and source the MSVC x64 dev environment (needed by both cmake
REM     configure's compiler checks and the MSBuild invocation that
REM     `cmake --build` shells out to): the one already set up in this
REM     shell, else the newest Visual Studio 18 with the C++ x64 tools, of
REM     any edition and wherever installed, as the Visual Studio Installer's
REM     vswhere.exe reports it (the IDE editions install under Program Files,
REM     Build Tools under Program Files (x86)). ---
set "VSINSTALL="
if defined VSINSTALLDIR set "VSINSTALL=%VSINSTALLDIR%"

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not defined VSINSTALL if exist "!VSWHERE!" for /f "usebackq delims=" %%V in (`"!VSWHERE!" -latest -products * -version [18.0^,19.0^) -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSINSTALL=%%V"
REM VsDevCmd.bat (which vcvarsall.bat calls) runs vswhere.exe by bare name
REM and prints "'vswhere.exe' is not recognized" unless its folder is on PATH.
if exist "!VSWHERE!" set "PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer;!PATH!"

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

REM --- cmake.exe: the first one on PATH (which the VS env above includes when
REM     Visual Studio's bundled CMake is installed), else that bundled copy
REM     directly. ---
set "CMAKE="
for /f "delims=" %%C in ('where cmake.exe 2^>nul') do if not defined CMAKE set "CMAKE=%%C"
set "VS_CMAKE=!VSINSTALL!\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
if not defined CMAKE if defined VSINSTALL if exist "!VS_CMAKE!" set "CMAKE=!VS_CMAKE!"
if not defined CMAKE (
  echo Error: cmake.exe not found. Add CMake's bin folder to PATH, or install Visual Studio 18's "C++ CMake tools for Windows".
  endlocal & exit /b 1
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
