#Requires -Version 5.1
<#
.SYNOPSIS
    Runs the native test suites with gcov instrumentation and prints a
    line-by-line coverage summary.

.DESCRIPTION
    1. Builds and runs pio test -e coverage (flags -fprofile-arcs -ftest-coverage -O0).
    2. Invokes gcov on every instrumented source file under src/.
    3. Moves the .gcov reports to .pio/coverage_report/.
    4. Prints a summary table with covered / total lines and percentage.

.NOTES
    Requires w64devkit at $HOME\.tools\w64devkit\bin (gcov 16+).
#>

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Paths
$env:PATH  = "$env:USERPROFILE\.tools\w64devkit\bin;" + $env:PATH
$pio       = "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"
$root      = (Resolve-Path "$PSScriptRoot\..\.." ).Path
$buildDir  = Join-Path $root ".pio\build\coverage"
$reportDir = Join-Path $root ".pio\coverage_report"

# 1. Build + run tests with coverage
Write-Host ""
Write-Host "=== pio test -e coverage ===" -ForegroundColor Cyan
Push-Location $root
& $pio test -e coverage
$testExit = $LASTEXITCODE
Pop-Location

if ($testExit -ne 0) {
    Write-Host "Tests failed (exit $testExit). Aborting gcov." -ForegroundColor Red
    exit $testExit
}

# 2. Prepare report directory
if (Test-Path $reportDir) { Remove-Item $reportDir -Recurse -Force }
New-Item -ItemType Directory -Path $reportDir | Out-Null

# 3. Invoke gcov for each .gcno under src/
Write-Host ""
Write-Host "=== Generating gcov reports ===" -ForegroundColor Cyan

$srcBuildDir = Join-Path $buildDir "src"
if (-not (Test-Path $srcBuildDir)) {
    Write-Warning "Build directory not found: $srcBuildDir"
    exit 1
}

$gcnoFiles = @(Get-ChildItem -Path $srcBuildDir -Recurse -Filter "*.gcno")
if ($gcnoFiles.Count -eq 0) {
    Write-Warning "No .gcno files found in $srcBuildDir"
    exit 1
}

foreach ($gcno in $gcnoFiles) {
    Write-Host "  gcov $($gcno.Name)" -ForegroundColor Gray

    # gcov resolves the source path (stored as relative inside the .gcno)
    # from the CWD, so run it from the project root.
    # The full .gcno path tells gcov where the profiling data lives.
    Push-Location $root
    & gcov $gcno.FullName 2>&1 | Out-Null
    Pop-Location

    # gcov writes .gcov files into the CWD => collect them from $root
    Get-ChildItem $root -Filter "*.gcov" -ErrorAction SilentlyContinue |
        Move-Item -Destination $reportDir -Force
}

# 4. Summary
Write-Host ""
Write-Host "=== Coverage summary ===" -ForegroundColor Cyan

$gcovFiles = @(Get-ChildItem $reportDir -Filter "*.gcov")
if ($gcovFiles.Count -eq 0) {
    Write-Warning "No .gcov files were generated."
    exit 1
}

$header = "{0,-50} {1,8}  {2,8}  {3,7}" -f "File", "Covered", "Total", "%"
$separator = "-" * 80

$totalCovered = 0
$totalLines   = 0
$reportLines  = [System.Collections.Generic.List[string]]::new()
$reportLines.Add($header)
$reportLines.Add($separator)

Write-Host $header
Write-Host $separator

foreach ($gcov in ($gcovFiles | Sort-Object Name)) {
    $content = Get-Content $gcov.FullName

    # Only process files whose source is a .cpp under src/
    $sourceLine = $content | Where-Object { $_ -match '^        -:    0:Source:' } | Select-Object -First 1
    if (-not $sourceLine) { continue }
    $sourcePath = ($sourceLine -replace '^.*Source:', '').Trim()
    if ($sourcePath -notmatch '^src/' -or $sourcePath -notmatch '\.cpp$') { continue }

    # Executable lines: numeric counter or ##### (line 0 is metadata, skipped)
    $execCount    = @($content | Where-Object { $_ -match '^\s+(#####|\d+):\s+[1-9]\d*:' }).Count
    # Covered lines: counter > 0
    $coveredCount = @($content | Where-Object { $_ -match '^\s+[1-9]\d*:\s+[1-9]\d*:' }).Count

    $totalCovered += $coveredCount
    $totalLines   += $execCount

    if ($execCount -gt 0) {
        $pct = [math]::Round(100.0 * $coveredCount / $execCount, 1)
    } else {
        $pct = 0.0
    }

    $color = if ($pct -ge 90) { 'Green' } elseif ($pct -ge 70) { 'Yellow' } else { 'Red' }
    $pctStr = $pct.ToString('0.0', [System.Globalization.CultureInfo]::InvariantCulture)
    $row   = "{0,-50} {1,8}  {2,8}  {3,6}%" -f $gcov.Name, $coveredCount, $execCount, $pctStr
    Write-Host $row -ForegroundColor $color
    $reportLines.Add($row)
}

$reportLines.Add($separator)
Write-Host $separator

if ($totalLines -gt 0) {
    $totalPct = [math]::Round(100.0 * $totalCovered / $totalLines, 1)
} else {
    $totalPct = 0.0
}
$totalPctStr = $totalPct.ToString('0.0', [System.Globalization.CultureInfo]::InvariantCulture)
$totalRow = "{0,-50} {1,8}  {2,8}  {3,6}%" -f "TOTAL", $totalCovered, $totalLines, $totalPctStr

Write-Host $totalRow -ForegroundColor Cyan
$reportLines.Add($totalRow)

# 5. Save to evidence
$date      = Get-Date -Format 'yyyy-MM-dd'
$stamp     = Get-Date -Format 'HHmmss'
$evBase    = Join-Path $root 'evidence'
$evRunDir  = Join-Path $evBase $date
$evLatest  = Join-Path $evBase 'latest'

New-Item -ItemType Directory -Force -Path $evRunDir  | Out-Null
New-Item -ItemType Directory -Force -Path $evLatest  | Out-Null

$stampFile    = Join-Path $evRunDir ("coverage-" + $stamp + ".txt")
$canonFile    = Join-Path $evRunDir 'coverage.txt'
$latestFile   = Join-Path $evLatest 'coverage.txt'

$reportLines | Out-File -Encoding utf8 $stampFile
try { Copy-Item -Force $stampFile $canonFile  } catch { Write-Warning "Could not update dated coverage.txt." }
try { Copy-Item -Force $stampFile $latestFile } catch { Write-Warning "Could not update latest/coverage.txt." }

Write-Host ""
Write-Host "Report saved to: $stampFile" -ForegroundColor Green
Write-Host "gcov reports in: $reportDir" -ForegroundColor Green

