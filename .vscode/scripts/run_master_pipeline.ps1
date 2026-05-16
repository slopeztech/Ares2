#Requires -Version 5.1
<#
.SYNOPSIS
    ARES2 master quality pipeline — runs every check and produces a unified report.

.DESCRIPTION
    Sequential pipeline:
      1. Build            pio run
      2. Compile DB       pio run -t compiledb
      3. MISRA Cppcheck   run_cppcheck_misra.ps1
      4. Cppcheck XML     run_cppcheck_practical_xml.ps1
      5. clang-tidy       run_clang_tidy.ps1
      6. Unity tests      run_sitl_sim.ps1  (pio test -e sim)
      7. Coverage         run_coverage.ps1

    After all steps the script reads the evidence files and writes:
      evidence/<date>/master.txt
      evidence/latest/master.txt

    The master report lists PASS / FAIL / WARN for every step plus extracted
    counts (errors, warnings, test ratio, coverage %).  Exit code is 0 only
    when every step passes.

.NOTES
    Prerequisites — same as individual scripts:
      LLVM / clang-tidy in C:\Program Files\LLVM\bin
      Cppcheck in      C:\Program Files\Cppcheck
      w64devkit in     %USERPROFILE%\.tools\w64devkit\bin
#>

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Continue'

# ── Paths ─────────────────────────────────────────────────────────────────────

$root      = (Resolve-Path "$PSScriptRoot\..\.." ).Path
$scripts   = $PSScriptRoot
$pio       = "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"
$env:PATH  = "$env:USERPROFILE\.tools\w64devkit\bin;" + $env:PATH

$date      = Get-Date -Format 'yyyy-MM-dd'
$stamp     = Get-Date -Format 'HHmmss'
$evBase    = Join-Path $root 'evidence'
$evRunDir  = Join-Path $evBase $date
$evLatest  = Join-Path $evBase 'latest'

New-Item -ItemType Directory -Force -Path $evRunDir  | Out-Null
New-Item -ItemType Directory -Force -Path $evLatest  | Out-Null

# ── Result accumulator ────────────────────────────────────────────────────────

class StepResult {
    [string] $Name
    [string] $Status      # PASS | FAIL | WARN | SKIP
    [int]    $ExitCode
    [double] $ElapsedSec
    [string] $Detail      # one-line summary extracted from output
}

$pipeline = [System.Collections.Generic.List[StepResult]]::new()

function Invoke-Step {
    param(
        [string]   $Name,
        [scriptblock] $Body
    )
    $r = [StepResult]::new()
    $r.Name = $Name
    $sw = [System.Diagnostics.Stopwatch]::StartNew()

    Write-Host ""
    Write-Host ("=" * 72) -ForegroundColor DarkCyan
    Write-Host "  STEP: $Name" -ForegroundColor Cyan
    Write-Host ("=" * 72) -ForegroundColor DarkCyan

    try {
        $raw = & $Body
        # Scriptblock may return an array if tool output leaked into the pipeline;
        # take the last element (the actual exit code integer).
        if ($raw -is [array]) { $raw = $raw[-1] }
        $r.ExitCode = if ($raw -ne $null) { [int]$raw } else { 0 }
    } catch {
        Write-Host "EXCEPTION: $_" -ForegroundColor Red
        $r.ExitCode = 99
    }

    $sw.Stop()
    $r.ElapsedSec = [math]::Round($sw.Elapsed.TotalSeconds, 1)
    $pipeline.Add($r)
    return $r
}

# ── Helper: run a sub-script, stream output, return exit code ─────────────────

function Run-Script([string]$ScriptPath) {
    Push-Location $root
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $ScriptPath | Out-Host
    $code = $LASTEXITCODE
    Pop-Location
    return $code
}

# ── Step 1: Build ─────────────────────────────────────────────────────────────

$r1 = Invoke-Step "Build (pio run)" {
    Push-Location $root
    & $pio run | Out-Host
    $c = $LASTEXITCODE
    Pop-Location
    return $c
}

# ── Step 2: Compile DB ────────────────────────────────────────────────────────

$r2 = Invoke-Step "Compile DB (pio compiledb)" {
    Push-Location $root
    & $pio run -t compiledb | Out-Host
    $c = $LASTEXITCODE
    Pop-Location
    return $c
}

# ── Step 3: MISRA Cppcheck ────────────────────────────────────────────────────

$r3 = Invoke-Step "MISRA Cppcheck" {
    Run-Script (Join-Path $scripts 'run_cppcheck_misra.ps1')
}

# ── Step 4: Cppcheck Practical (XML) ─────────────────────────────────────────

$r4 = Invoke-Step "Cppcheck Practical (XML)" {
    Run-Script (Join-Path $scripts 'run_cppcheck_practical_xml.ps1')
}

# ── Step 5: clang-tidy ────────────────────────────────────────────────────────

$r5 = Invoke-Step "clang-tidy" {
    Run-Script (Join-Path $scripts 'run_clang_tidy.ps1')
}

# ── Step 6: Unity tests (sim) ─────────────────────────────────────────────────

$r6 = Invoke-Step "Unity tests (pio test -e sim)" {
    Run-Script (Join-Path $scripts 'run_sitl_sim.ps1')
}

# ── Step 7: Coverage (gcov) ───────────────────────────────────────────────────

$r7 = Invoke-Step "Coverage (gcov)" {
    Run-Script (Join-Path $scripts 'run_coverage.ps1')
}

# ── Parse evidence files for metrics ─────────────────────────────────────────

function Parse-MisraOutput([string]$path) {
    # Returns @{Errors=N; Warnings=N; MisraViolations=N}
    if (-not (Test-Path $path)) { return @{ Errors=0; Warnings=0; MisraViolations=0 } }
    $lines = Get-Content $path
    $errors   = @($lines | Where-Object { $_ -match '\berror\b' -and $_ -notmatch '^\s*#' }).Count
    $warnings = @($lines | Where-Object { $_ -match '\bwarning\b' -and $_ -notmatch '^\s*#' }).Count
    $misra    = @($lines | Where-Object { $_ -match '\[misra' }).Count
    return @{ Errors=$errors; Warnings=$warnings; MisraViolations=$misra }
}

function Parse-CppcheckXml([string]$path) {
    # Returns @{Errors=N; Warnings=N}
    if (-not (Test-Path $path)) { return @{ Errors=0; Warnings=0 } }
    $xml = Get-Content $path -Raw
    # Count <error id="..." severity="error"... and severity="warning"...
    $errors   = ([regex]::Matches($xml, 'severity="error"')).Count
    $warnings = ([regex]::Matches($xml, 'severity="warning"')).Count
    $style    = ([regex]::Matches($xml, 'severity="style"')).Count
    $perf     = ([regex]::Matches($xml, 'severity="performance"')).Count
    return @{ Errors=$errors; Warnings=($warnings + $style + $perf) }
}

function Parse-ClangTidyOutput([string]$path) {
    # Returns @{Errors=N; Warnings=N}
    if (-not (Test-Path $path)) { return @{ Errors=0; Warnings=0 } }
    $lines = Get-Content $path
    $errors   = @($lines | Where-Object { $_ -match ':\s+error:\s+' }).Count
    $warnings = @($lines | Where-Object { $_ -match ':\s+warning:\s+' }).Count
    return @{ Errors=$errors; Warnings=$warnings }
}

function Parse-UnityOutput([string]$path) {
    # Returns @{Tests=N; Failures=N; Ignored=N; Found=$true/$false}
    if (-not (Test-Path $path)) { return @{ Tests=0; Failures=0; Ignored=0; Found=$false } }
    $lines = Get-Content $path
    # Unity summary line: "X Tests Y Failures Z Ignored"
    $summary = $lines | Where-Object { $_ -match '(\d+) Tests\s+(\d+) Failures\s+(\d+) Ignored' } |
               Select-Object -Last 1
    if ($summary -match '(\d+) Tests\s+(\d+) Failures\s+(\d+) Ignored') {
        return @{ Tests=[int]$Matches[1]; Failures=[int]$Matches[2]; Ignored=[int]$Matches[3]; Found=$true }
    }
    return @{ Tests=0; Failures=0; Ignored=0; Found=$false }
}

function Parse-CoverageOutput([string]$path) {
    # Returns @{CoveredLines=N; TotalLines=N; Pct=X.X}
    if (-not (Test-Path $path)) { return @{ CoveredLines=0; TotalLines=0; Pct=0.0 } }
    $lines = Get-Content $path
    $totalLine = $lines | Where-Object { $_ -match '^TOTAL\s' } | Select-Object -Last 1
    if ($totalLine -match '(\d+)\s+(\d+)\s+([\d.,]+)%') {
        $pctNorm = $Matches[3] -replace ',', '.'
        return @{ CoveredLines=[int]$Matches[1]; TotalLines=[int]$Matches[2]; Pct=[double]$pctNorm }
    }
    return @{ CoveredLines=0; TotalLines=0; Pct=0.0 }
}

# Resolve metric file paths from evidence/latest/
$misraTxt   = Join-Path $evLatest 'cppcheck-misra-addon.txt'
$cppXml     = Join-Path $evLatest 'cppcheck-practical.xml'
$tidyTxt    = Join-Path $evLatest 'clang-tidy.txt'
$simTxt     = Join-Path $evLatest 'sitl-sim.txt'
$covTxt     = Join-Path $evLatest 'coverage.txt'

$misraM  = Parse-MisraOutput   $misraTxt
$cppM    = Parse-CppcheckXml   $cppXml
$tidyM   = Parse-ClangTidyOutput $tidyTxt
$unityM  = Parse-UnityOutput   $simTxt
$covM    = Parse-CoverageOutput $covTxt

# ── Annotate step results with details + PASS/FAIL/WARN ─────────────────────

function Set-StepStatus([StepResult]$r, [int]$exit, [string]$detail) {
    $r.ExitCode = $exit
    $r.Detail   = $detail
    $r.Status   = if ($exit -eq 0) { 'PASS' } else { 'FAIL' }
}

Set-StepStatus $r1 $r1.ExitCode "exit=$($r1.ExitCode)"
Set-StepStatus $r2 $r2.ExitCode "exit=$($r2.ExitCode)"

# MISRA: any misra violation or cppcheck error = FAIL; warnings = WARN
if ($r3.ExitCode -ne 0) {
    $r3.Status = 'FAIL'
    $r3.Detail = "errors=$($misraM.Errors)  warnings=$($misraM.Warnings)  misra-violations=$($misraM.MisraViolations)"
} elseif ($misraM.Warnings -gt 0) {
    $r3.Status = 'WARN'
    $r3.Detail = "warnings=$($misraM.Warnings)  misra-violations=$($misraM.MisraViolations)"
} else {
    $r3.Status = 'PASS'
    $r3.Detail = "clean  misra-violations=$($misraM.MisraViolations)"
}

# Cppcheck practical: errors = FAIL; warnings/style = WARN
if ($r4.ExitCode -ne 0 -or $cppM.Errors -gt 0) {
    $r4.Status = 'FAIL'
    $r4.Detail = "errors=$($cppM.Errors)  warnings/style=$($cppM.Warnings)"
} elseif ($cppM.Warnings -gt 0) {
    $r4.Status = 'WARN'
    $r4.Detail = "warnings/style=$($cppM.Warnings)"
} else {
    $r4.Status = 'PASS'
    $r4.Detail = "clean"
}

# clang-tidy: errors = FAIL; warnings = WARN
if ($r5.ExitCode -ne 0 -or $tidyM.Errors -gt 0) {
    $r5.Status = 'FAIL'
    $r5.Detail = "errors=$($tidyM.Errors)  warnings=$($tidyM.Warnings)"
} elseif ($tidyM.Warnings -gt 0) {
    $r5.Status = 'WARN'
    $r5.Detail = "warnings=$($tidyM.Warnings)"
} else {
    $r5.Status = 'PASS'
    $r5.Detail = "clean"
}

# Unity: failures > 0 or no summary found = FAIL
if ($r6.ExitCode -ne 0 -or $unityM.Failures -gt 0) {
    $r6.Status = 'FAIL'
    $r6.Detail = if ($unityM.Found) {
        "$($unityM.Tests) Tests  $($unityM.Failures) Failures  $($unityM.Ignored) Ignored"
    } else {
        "no summary found in output (runner crash?)"
    }
} else {
    $r6.Status = 'PASS'
    $r6.Detail = if ($unityM.Found) {
        "$($unityM.Tests)/$($unityM.Tests) tests passed"
    } else {
        "exit=0 (no summary line found)"
    }
}

# Coverage: treat < 60% as WARN, failure to run as FAIL
if ($r7.ExitCode -ne 0) {
    $r7.Status = 'FAIL'
    $r7.Detail = "gcov run failed (exit=$($r7.ExitCode))"
} elseif ($covM.TotalLines -eq 0) {
    $r7.Status = 'WARN'
    $r7.Detail = "no coverage data found"
} elseif ($covM.Pct -lt 60.0) {
    $r7.Status = 'WARN'
    $r7.Detail = "$($covM.CoveredLines)/$($covM.TotalLines) lines  $($covM.Pct)% (below 60% threshold)"
} else {
    $r7.Status = 'PASS'
    $r7.Detail = "$($covM.CoveredLines)/$($covM.TotalLines) lines  $($covM.Pct)%"
}

# ── Build master report ───────────────────────────────────────────────────────

$runTs     = Get-Date -Format 'yyyy-MM-dd HH:mm:ss'
$totalFail = @($pipeline | Where-Object { $_.Status -eq 'FAIL' }).Count
$totalWarn = @($pipeline | Where-Object { $_.Status -eq 'WARN' }).Count
$totalPass = @($pipeline | Where-Object { $_.Status -eq 'PASS' }).Count
$overallOk = ($totalFail -eq 0)

$bar = "=" * 72

$report = [System.Collections.Generic.List[string]]::new()
$report.Add($bar)
$report.Add("  ARES2 MASTER PIPELINE REPORT")
$report.Add("  Generated : $runTs")
$report.Add("  Commit    : $(git -C $root rev-parse --short HEAD 2>$null)")
$report.Add($bar)
$report.Add("")

# Step summary table
$colW = 40
$report.Add(("{0,-$colW}  {1,-6}  {2,7}s  {3}" -f "Step", "Status", "Time", "Detail"))
$report.Add("-" * 72)

foreach ($r in $pipeline) {
    $statusStr = switch ($r.Status) {
        'PASS' { '[PASS]' }
        'FAIL' { '[FAIL]' }
        'WARN' { '[WARN]' }
        default { '[????]' }
    }
    $report.Add(("{0,-$colW}  {1,-6}  {2,7}s  {3}" -f
        $r.Name, $statusStr, $r.ElapsedSec, $r.Detail))
}

$report.Add("-" * 72)
$report.Add("")
$report.Add("SUMMARY  Pass=$totalPass  Warn=$totalWarn  Fail=$totalFail")

if ($overallOk) {
    $report.Add("OVERALL  PASS -- all checks green")
} else {
    $report.Add("OVERALL  FAIL -- $totalFail step(s) failed")
}

$report.Add("")

# ── Append first-failure extracts ────────────────────────────────────────────

$extractMap = @{
    $r3 = $misraTxt
    $r4 = $cppXml
    $r5 = $tidyTxt
    $r6 = $simTxt
    $r7 = $covTxt
}

foreach ($entry in $extractMap.GetEnumerator()) {
    $step = $entry.Key
    $file = $entry.Value
    if ($step.Status -in @('FAIL','WARN') -and (Test-Path $file)) {
        $report.Add($bar)
        $report.Add("  DETAIL: $($step.Name)  [$($step.Status)]")
        $report.Add($bar)
        # Emit up to 40 lines from the evidence file (trim blank runs)
        $excerpt = Get-Content $file | Select-Object -First 40
        $report.AddRange([string[]]$excerpt)
        $report.Add("  ... (see $file for full output)")
        $report.Add("")
    }
}

$report.Add($bar)

# ── Write master report ───────────────────────────────────────────────────────

$masterStamp    = Join-Path $evRunDir ("master-" + $stamp + ".txt")
$masterCanon    = Join-Path $evRunDir 'master.txt'
$masterLatest   = Join-Path $evLatest 'master.txt'

$report | Out-File -Encoding utf8 $masterStamp
try { Copy-Item -Force $masterStamp $masterCanon  } catch { Write-Warning "Could not update dated master.txt."  }
try { Copy-Item -Force $masterStamp $masterLatest } catch { Write-Warning "Could not update latest/master.txt." }

# ── Print master report to terminal ──────────────────────────────────────────

Write-Host ""
foreach ($line in $report) {
    if ($line -match '\[FAIL\]') {
        Write-Host $line -ForegroundColor Red
    } elseif ($line -match '\[WARN\]') {
        Write-Host $line -ForegroundColor Yellow
    } elseif ($line -match '\[PASS\]') {
        Write-Host $line -ForegroundColor Green
    } elseif ($line -match '^OVERALL.*FAIL') {
        Write-Host $line -ForegroundColor Red
    } elseif ($line -match '^OVERALL.*PASS') {
        Write-Host $line -ForegroundColor Green
    } else {
        Write-Host $line
    }
}

Write-Host ""
Write-Host "Master report: $masterLatest" -ForegroundColor Cyan

# ── Exit ─────────────────────────────────────────────────────────────────────

exit $(if ($overallOk) { 0 } else { 1 })
