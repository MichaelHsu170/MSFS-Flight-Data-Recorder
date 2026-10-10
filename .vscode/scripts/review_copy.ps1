# review_copy.ps1 [file...]
#
# Keeps build\review-copy, the throwaway copy of the working tree that the
# diff review's coverage check breaks code in (see .github\diff-review.md),
# so the review never edits the working tree itself. build\ is git-ignored.
#
# With no arguments, syncs the whole copy to the working tree: every file git
# tracks or would track (untracked but not ignored, so no databases or
# settings.ini), and removes the files the working tree no longer has. The
# copy's own build\ is kept, so its builds stay incremental. With file
# arguments (paths from the repository root), copies only those back, which
# is how a break is undone.
#
# Only a file whose content differs is copied, and it's stamped with the
# current time: a copy keeps its source's older timestamp otherwise, older
# than what was built from the broken version, and MSBuild wouldn't rebuild.

param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Files)
$ErrorActionPreference = 'Stop'

$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$copy = Join-Path $root 'build\review-copy'

if (-not $Files) {
    $listed = @(git -C $root -c core.quotePath=false ls-files --cached --others --exclude-standard)
    if ($LASTEXITCODE -ne 0) { exit 1 }
    # Tracked files the working tree deleted are listed too; they aren't copied.
    $Files = @($listed | Where-Object { Test-Path -LiteralPath (Join-Path $root $_) -PathType Leaf })
    if (Test-Path -LiteralPath $copy) {
        $keep = @{}
        foreach ($f in $Files) { $keep[$f.Replace('/', '\')] = $true }
        Get-ChildItem -LiteralPath $copy -Force | Where-Object { $_.Name -ne 'build' } |
            ForEach-Object { if ($_.PSIsContainer) { Get-ChildItem -LiteralPath $_.FullName -Recurse -File -Force } else { $_ } } |
            Where-Object { -not $keep.ContainsKey($_.FullName.Substring($copy.Length + 1)) } |
            Remove-Item -Force
    }
}

$copied = 0
foreach ($f in $Files) {
    $source = Join-Path $root $f
    $target = Join-Path $copy $f
    if ((Test-Path -LiteralPath $target) -and
        (Get-FileHash -LiteralPath $source).Hash -eq (Get-FileHash -LiteralPath $target).Hash) { continue }
    New-Item -ItemType Directory -Force -Path (Split-Path $target) | Out-Null
    Copy-Item -LiteralPath $source -Destination $target -Force
    (Get-Item -LiteralPath $target).LastWriteTime = Get-Date
    $copied++
}
Write-Host "Copied $copied changed file(s) into $copy"
