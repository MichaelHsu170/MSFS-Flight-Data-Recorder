# review_snapshot.ps1
#
# Prints the git tree hash of the working tree as it is now: every file git
# tracks or would track, so no databases or settings.ini. The review loop
# (.github\code-review.md) takes one before each round, so a round's scope is
# "git diff <previous hash> <this hash>" with nothing committed.
#
# It builds the tree in a temporary index under the git-ignored build\, so the
# real index, HEAD and the working tree stay as they were. git keeps the tree
# as an unreferenced object until its normal cleanup, weeks later.

$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$env:GIT_INDEX_FILE = Join-Path $root 'build\review-snapshot.index'
New-Item -ItemType Directory -Force -Path (Split-Path $env:GIT_INDEX_FILE) | Out-Null
try {
    # Starts from HEAD so tracked files that .gitignore also matches stay in.
    git -C $root read-tree HEAD
    if ($LASTEXITCODE -ne 0) { exit 1 }
    git -C $root -c core.safecrlf=false add -A
    if ($LASTEXITCODE -ne 0) { exit 1 }
    git -C $root write-tree
    if ($LASTEXITCODE -ne 0) { exit 1 }
} finally {
    Remove-Item -LiteralPath $env:GIT_INDEX_FILE -Force -ErrorAction SilentlyContinue
}
