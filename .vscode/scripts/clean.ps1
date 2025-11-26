$buildPath = Join-Path (Join-Path $PSScriptRoot "..") ".." | Join-Path -ChildPath "build"
Write-Host "Attempting to remove: $buildPath"

if (Test-Path $buildPath) {
    try {
        Remove-Item -Path $buildPath -Recurse -Force -ErrorAction Stop
        Write-Host "SUCCESS: Build directory removed" -ForegroundColor Green
    }
    catch {
        Write-Host "ERROR: Failed to remove build directory" -ForegroundColor Red
        Write-Host $_.Exception.Message
        exit 1
    }
    
    # Verify removal
    if (Test-Path $buildPath) {
        Write-Host "WARNING: Build directory still exists after removal attempt" -ForegroundColor Yellow
        exit 1
    }
}
else {
    Write-Host "Build directory does not exist (already clean)" -ForegroundColor Cyan
}
