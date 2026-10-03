# coverage.ps1 [-Config <Debug|Release>]
#
# Runs the full test suite under OpenCppCoverage and writes a line-coverage
# report. Requires a build that's already up to date (run the "Test" task,
# or build.bat <Config> test, first) and OpenCppCoverage installed
# (winget install --id OpenCppCoverage.OpenCppCoverage -e).
#
# Scope: only MSFS-Flight-Data-Recorder\MSFS-Flight-Data-Recorder (app code),
# excluding build\ (generated/moc/rcc files) and the tests\ themselves.
#
# Output: coverage_report\index.html + coverage.xml. Neither is committed
# (see .gitignore) -- regenerate instead of trusting a stale copy.
#
# Caveat: fdr_core is statically linked into every tst_*.exe, so OpenCppCoverage
# reports the same app-code line set once per test executable; coverage.xml's
# root <coverage> totals and coverage_report's "ctest.exe" summary row are
# that per-file data duplicated once per executable (sum = N x the real
# total), not N independent measurements. The resulting line-rate percentage
# is still correct, but don't read the raw lines-covered/lines-valid counts
# as true line totals -- compute those from the per-file de-duplicated data
# (union hits by filename+line number across every <package>) instead.

param(
    [string]$Config = "Debug"
)

$root = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$buildDir = Join-Path $root "build\$Config"
$cmake = "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$ctest = Join-Path (Split-Path $cmake -Parent) "ctest.exe"
$openCppCoverage = "C:\Program Files\OpenCppCoverage\OpenCppCoverage.exe"
$sources = Join-Path $root "MSFS-Flight-Data-Recorder"
$excluded = Join-Path $root "build"
$reportDir = Join-Path $root "coverage_report"
$xml = Join-Path $root "coverage.xml"

if (-not (Test-Path $ctest)) {
    Write-Host "ERROR: ctest.exe not found at $ctest" -ForegroundColor Red
    exit 1
}
if (-not (Test-Path $openCppCoverage)) {
    Write-Host "ERROR: OpenCppCoverage not found at $openCppCoverage (winget install --id OpenCppCoverage.OpenCppCoverage -e)" -ForegroundColor Red
    exit 1
}
if (-not (Test-Path (Join-Path $buildDir "CMakeCache.txt"))) {
    Write-Host "ERROR: build\$Config isn't configured yet -- build it first (e.g. .\.vscode\scripts\build.bat $Config test)" -ForegroundColor Red
    exit 1
}
if (-not (Test-Path $sources)) {
    Write-Host "ERROR: computed --sources path doesn't exist: $sources" -ForegroundColor Red
    exit 1
}

if (Test-Path $reportDir) {
    Remove-Item -Recurse -Force $reportDir
}

& $openCppCoverage `
    --cover_children `
    --sources "$sources" `
    --excluded_sources "$excluded" `
    --export_type "html:$reportDir" `
    --export_type "cobertura:$xml" `
    -- "$ctest" --test-dir "$buildDir" -C $Config

exit $LASTEXITCODE
