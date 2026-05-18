#Requires -Version 5.1
<#
.SYNOPSIS
    Runs the native test suites with gcov instrumentation and prints a full
    coverage summary: line coverage, branch coverage, uncovered-line detail,
    and trend vs the previous run.

.DESCRIPTION
    1. Reads previous evidence/latest/coverage.txt TOTAL line for trend comparison.
    2. Builds and runs pio test -e coverage (flags -fprofile-arcs -ftest-coverage -O0).
    3. Invokes gcov -b on every instrumented source file under src/  (-b activates
       branch instrumentation in the .gcov output).
    4. Moves the .gcov reports to .pio/coverage_report/.
    5. Parses each .gcov file:
         - Line coverage  (respects GCOV_EXCL_START/STOP and GCOV_EXCL_LINE)
         - Branch coverage (from "branch N taken X" lines, also respects exclusions)
         - Uncovered line numbers (for the detail section)
    6. Prints a summary table: Cov/Exec, Line%, BrCov/Tot, Br%
    7. Prints uncovered line numbers per module (capped at $UNCOV_DETAIL_MAX).
    8. Prints trend vs previous run.
    9. Saves to evidence/.

.NOTES
    Requires w64devkit at $HOME\.tools\w64devkit\bin (gcov 16+).
    Scope: all SITL-compilable src/ modules (mirrors [env:sim] build_src_filter).
    ESP32-only TUs (hal drivers, main.cpp, api_server, etc.) are excluded.
    Per-module minimum threshold: 70%.  Modules below are flagged [BELOW MIN].

    TOTAL line format: "TOTAL  N  N  X.X%   N/N  X.X%"
      First two integers + percentage  = line coverage  (parsed by master pipeline)
      Last  N/N + percentage           = branch coverage (optional, new)
#>

$MIN_MODULE_PCT   = 70.0   # per-module line-coverage minimum (%)
$UNCOV_DETAIL_MAX = 25     # max uncovered line numbers to list per module

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Paths
$env:PATH  = "$env:USERPROFILE\.tools\w64devkit\bin;" + $env:PATH
$pio       = "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"
$root      = (Resolve-Path "$PSScriptRoot\..\.." ).Path
$buildDir  = Join-Path $root ".pio\build\coverage"
$reportDir = Join-Path $root ".pio\coverage_report"
$evLatest  = Join-Path $root 'evidence\latest'

# ── 1. Capture previous TOTAL for trend comparison ───────────────────────────
$prevLinePct = $null
$prevBrPct   = $null
$prevCovTxt  = Join-Path $evLatest 'coverage.txt'
if (Test-Path $prevCovTxt) {
    $prevLines = Get-Content $prevCovTxt -ErrorAction SilentlyContinue
    $prevTL = $prevLines | Where-Object { $_ -match '^TOTAL\s' } | Select-Object -Last 1
    if ($prevTL -and $prevTL -match '(\d+)\s+(\d+)\s+([\d.,]+)%') {
        $prevLinePct = [double]($Matches[3] -replace ',', '.')
    }
    # Branch pct: last "N/N  X.X%" pattern on the TOTAL line (new format only)
    if ($prevTL -and $prevTL -match '(\d+)/(\d+)\s+([\d.,]+)%\s*$') {
        $prevBrPct = [double]($Matches[3] -replace ',', '.')
    }
}

# ── 2. Build + run tests with coverage ───────────────────────────────────────
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

# ── 3. Prepare report directory ───────────────────────────────────────────────
if (Test-Path $reportDir) { Remove-Item $reportDir -Recurse -Force }
New-Item -ItemType Directory -Path $reportDir | Out-Null

# ── 4. Invoke gcov -b for each .gcno under src/ ───────────────────────────────
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
    # Touch the .gcno so its mtime >= source mtime (avoids gcov aborting with
    # "source file is newer than notes file" when only comments were edited).
    (Get-Item $gcno.FullName).LastWriteTime = (Get-Date)
    Push-Location $root
    # -b: emit branch counts in .gcov output
    try { & gcov -b $gcno.FullName 2>$null | Out-Null } catch {}
    Pop-Location
    # gcov writes .gcov files into the CWD => collect them from $root
    Get-ChildItem $root -Filter "*.gcov" -ErrorAction SilentlyContinue |
        Move-Item -Destination $reportDir -Force -ErrorAction SilentlyContinue
}

# ── 5. Parse all .gcov files and build summary ────────────────────────────────
Write-Host ""
Write-Host "=== Coverage summary ===" -ForegroundColor Cyan

$gcovFiles = @(Get-ChildItem $reportDir -Filter "*.gcov")
if ($gcovFiles.Count -eq 0) {
    Write-Warning "No .gcov files were generated."
    exit 1
}

$hdr = "{0,-50} {1,10}  {2,7}  {3,11}  {4,7}" -f "File", "Cov/Exec", "Line%", "BrCov/Tot", "Br%"
$sep = "-" * 105

$totalCovL     = 0; $totalExL  = 0
$totalCovB     = 0; $totalExB  = 0
$belowMinCount = 0

$reportLines = [System.Collections.Generic.List[string]]::new()
$detailLines = [System.Collections.Generic.List[string]]::new()

$reportLines.Add("Scope: SITL-compilable src/ modules (mirrors [env:sim] build_src_filter).")
$reportLines.Add("       ESP32-only TUs (hal drivers, main.cpp, api_server ...) are excluded.")
$reportLines.Add("Per-module minimum threshold: $MIN_MODULE_PCT%  -- modules below are flagged [BELOW MIN].")
$reportLines.Add("")
$reportLines.Add($hdr)
$reportLines.Add($sep)

Write-Host $hdr
Write-Host $sep

foreach ($gcov in ($gcovFiles | Sort-Object Name)) {
    $content = Get-Content $gcov.FullName

    # Only process .cpp sources under src/
    $srcLine = $content | Where-Object { $_ -match '^        -:    0:Source:' } | Select-Object -First 1
    if (-not $srcLine) { continue }
    $srcPath = ($srcLine -replace '^.*Source:', '').Trim()
    if ($srcPath -notmatch '^src/' -or $srcPath -notmatch '\.cpp$') { continue }

    $inExcl    = $false   # inside GCOV_EXCL_START / GCOV_EXCL_STOP block
    $prevExclL = $false   # previous source line was excluded (for branch line tracking)
    $covL      = 0; $execL = 0
    $covB      = 0; $execB = 0
    $uncovNums = [System.Collections.Generic.List[int]]::new()

    foreach ($line in $content) {
        # Block exclusion markers
        if ($line -match 'GCOV_EXCL_START') { $inExcl = $true;  continue }
        if ($line -match 'GCOV_EXCL_STOP')  { $inExcl = $false; continue }

        # Branch lines (gcov -b): "  branch N taken X [fallthrough]"
        # Inherit exclusion from the source line that preceded them.
        if ($line -match '^\s*branch\s+\d+\s+taken\s+(\d+)') {
            if (-not $inExcl -and -not $prevExclL) {
                $execB++
                if ([int]$Matches[1] -gt 0) { $covB++ }
            }
            continue
        }

        # Call lines (also emitted by -b) — skip, don't affect exclusion state
        if ($line -match '^\s*call\s+') { continue }

        # Source / executable lines: "  count:  lineno:  source"
        # Line 0 is metadata (-:    0:...) and won't match [1-9]\d* in lineno.
        if ($line -match '^\s+(-|#+|\d+|\*+):\s+([1-9]\d*):(.*)$') {
            $count  = $Matches[1]
            $lineNo = [int]$Matches[2]
            $src    = $Matches[3]

            # Per-line exclusion marker in the source column
            if ($src -match 'GCOV_EXCL_LINE') {
                $prevExclL = $true
                continue
            }

            if ($inExcl) {
                $prevExclL = $true
                continue
            }

            $prevExclL = $false

            # Non-executable line (comment, blank, pure declaration) — skip
            if ($count -eq '-') { continue }

            $execL++
            if ($count -match '^#') {
                $uncovNums.Add($lineNo)   # uncovered executable line
            } else {
                $covL++
            }
        }
        # Any other line (metadata, etc.) does not change $prevExclL
    }

    $totalCovL += $covL; $totalExL += $execL
    $totalCovB += $covB; $totalExB += $execB

    $lPct  = if ($execL -gt 0) { [math]::Round(100.0 * $covL / $execL, 1) } else { 0.0 }
    $bPct  = if ($execB -gt 0) { [math]::Round(100.0 * $covB / $execB, 1) } else { -1.0 }

    $lPctS   = $lPct.ToString('0.0', [System.Globalization.CultureInfo]::InvariantCulture) + '%'
    $lineStr = "{0}/{1}" -f $covL, $execL
    $brStr   = if ($execB -gt 0) { "{0}/{1}" -f $covB, $execB } else { 'n/a' }
    $brPctS  = if ($bPct -ge 0)  { $bPct.ToString('0.0', [System.Globalization.CultureInfo]::InvariantCulture) + '%' } else { 'n/a' }

    $below    = ($lPct -lt $MIN_MODULE_PCT -and $execL -gt 0)
    $belowTag = if ($below) { '  [BELOW MIN]' } else { '' }
    if ($below) { $belowMinCount++ }

    $color = if ($lPct -ge 90) { 'Green' } elseif ($lPct -ge $MIN_MODULE_PCT) { 'Yellow' } else { 'Red' }
    $row   = "{0,-50} {1,10}  {2,7}  {3,11}  {4,7}{5}" -f `
             $gcov.Name, $lineStr, $lPctS, $brStr, $brPctS, $belowTag
    Write-Host $row -ForegroundColor $color
    $reportLines.Add($row)

    # Accumulate uncovered line detail for the detail section
    if ($uncovNums.Count -gt 0) {
        $modName = [System.IO.Path]::GetFileNameWithoutExtension($gcov.Name)
        $sorted  = @($uncovNums | Sort-Object)
        $shown   = $sorted | Select-Object -First $UNCOV_DETAIL_MAX
        $suffix  = if ($sorted.Count -gt $UNCOV_DETAIL_MAX) {
                       " ... ($($sorted.Count) total, first $UNCOV_DETAIL_MAX shown)"
                   } else { '' }
        $detailLines.Add("  [$modName]  $($sorted.Count) uncovered line(s):")
        $detailLines.Add("    " + ($shown -join ', ') + $suffix)
    }
}

$reportLines.Add($sep)
Write-Host $sep

# ── Totals ────────────────────────────────────────────────────────────────────
$totalLinePct = if ($totalExL -gt 0) { [math]::Round(100.0 * $totalCovL / $totalExL, 1) } else { 0.0 }
$totalBrPct   = if ($totalExB -gt 0) { [math]::Round(100.0 * $totalCovB / $totalExB, 1) } else { -1.0 }

$totalLinePctS = $totalLinePct.ToString('0.0', [System.Globalization.CultureInfo]::InvariantCulture)
$totalBrStr    = if ($totalExB -gt 0) { "$totalCovB/$totalExB" } else { 'n/a' }
$totalBrPctS   = if ($totalBrPct -ge 0) {
                     $totalBrPct.ToString('0.0', [System.Globalization.CultureInfo]::InvariantCulture)
                 } else { 'n/a' }

# TOTAL row — keep "  N  N  X.X%" format so Parse-CoverageOutput in master
# pipeline (regex: (\d+)\s+(\d+)\s+([\d.,]+)%) still extracts line coverage.
# Branch totals appended as "  N/N  X.X%" for the updated parser.
$totalRow = if ($totalExB -gt 0) {
    "{0,-50} {1,8}  {2,8}  {3,6}%   {4,11}  {5,6}%" -f `
    "TOTAL", $totalCovL, $totalExL, $totalLinePctS, $totalBrStr, $totalBrPctS
} else {
    "{0,-50} {1,8}  {2,8}  {3,6}%" -f "TOTAL", $totalCovL, $totalExL, $totalLinePctS
}

Write-Host $totalRow -ForegroundColor Cyan
$reportLines.Add($totalRow)

if ($belowMinCount -gt 0) {
    $belowMsg = "$belowMinCount module(s) below $($MIN_MODULE_PCT)% minimum threshold."
    Write-Host $belowMsg -ForegroundColor Red
    $reportLines.Add("")
    $reportLines.Add($belowMsg)
}

# ── 6. Uncovered line detail ──────────────────────────────────────────────────
if ($detailLines.Count -gt 0) {
    Write-Host ""
    Write-Host "=== Uncovered line detail ===" -ForegroundColor Cyan
    $reportLines.Add("")
    $reportLines.Add("=== Uncovered line detail ===")
    foreach ($dl in $detailLines) {
        Write-Host $dl
        $reportLines.Add($dl)
    }
}

# ── 7. Trend vs previous run ──────────────────────────────────────────────────
Write-Host ""
Write-Host "=== Coverage trend ===" -ForegroundColor Cyan
$reportLines.Add("")
$reportLines.Add("=== Coverage trend ===")

if ($null -ne $prevLinePct) {
    $lineDelta  = $totalLinePct - $prevLinePct
    $lineSign   = if ($lineDelta -ge 0) { '+' } else { '' }
    $lineTrend  = "  Line coverage:   {0:0.0}% -> {1:0.0}%  ({2}{3:0.0}%)" -f `
                  $prevLinePct, $totalLinePct, $lineSign, $lineDelta
    $lineColor  = if ($lineDelta -gt 0) { 'Green' } elseif ($lineDelta -lt 0) { 'Red' } else { 'White' }
    Write-Host $lineTrend -ForegroundColor $lineColor
    $reportLines.Add($lineTrend)
} else {
    $msg = "  (No previous run to compare)"
    Write-Host $msg -ForegroundColor Gray
    $reportLines.Add($msg)
}

if ($totalBrPct -ge 0) {
    if ($null -ne $prevBrPct) {
        $brDelta  = $totalBrPct - $prevBrPct
        $brSign   = if ($brDelta -ge 0) { '+' } else { '' }
        $brTrend  = "  Branch coverage: {0:0.0}% -> {1:0.0}%  ({2}{3:0.0}%)" -f `
                    $prevBrPct, $totalBrPct, $brSign, $brDelta
        $brColor  = if ($brDelta -gt 0) { 'Green' } elseif ($brDelta -lt 0) { 'Red' } else { 'White' }
        Write-Host $brTrend -ForegroundColor $brColor
        $reportLines.Add($brTrend)
    } else {
        $brFirst = "  Branch coverage: (first run with branch data)  $($totalBrPctS)%"
        Write-Host $brFirst -ForegroundColor Gray
        $reportLines.Add($brFirst)
    }
}

# ── 8. Save to evidence ───────────────────────────────────────────────────────
$date        = Get-Date -Format 'yyyy-MM-dd'
$stamp       = Get-Date -Format 'HHmmss'
$evBase      = Join-Path $root 'evidence'
$evRunDir    = Join-Path $evBase $date
$evLatestDir = Join-Path $evBase 'latest'

New-Item -ItemType Directory -Force -Path $evRunDir    | Out-Null
New-Item -ItemType Directory -Force -Path $evLatestDir | Out-Null

$stampFile  = Join-Path $evRunDir  ("coverage-" + $stamp + ".txt")
$canonFile  = Join-Path $evRunDir  'coverage.txt'
$latestFile = Join-Path $evLatestDir 'coverage.txt'

$reportLines | Out-File -Encoding utf8 $stampFile
try { Copy-Item -Force $stampFile $canonFile  } catch { Write-Warning "Could not update dated coverage.txt." }
try { Copy-Item -Force $stampFile $latestFile } catch { Write-Warning "Could not update latest/coverage.txt." }

Write-Host ""
Write-Host "Report saved to: $stampFile" -ForegroundColor Green
Write-Host "gcov reports in: $reportDir" -ForegroundColor Green

# Exit 0 if all modules meet the threshold; exit 1 if any module is below.
exit ([int]($belowMinCount -gt 0))
