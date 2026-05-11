$ErrorActionPreference = 'Continue'

$date     = Get-Date -Format 'yyyy-MM-dd'
$base     = 'evidence'
$runDir   = Join-Path $base $date
$latestDir = Join-Path $base 'latest'
$stamp    = Get-Date -Format 'HHmmss'

New-Item -ItemType Directory -Force -Path $runDir    | Out-Null
New-Item -ItemType Directory -Force -Path $latestDir | Out-Null

$runFile       = Join-Path $runDir    ("sitl-sim-" + $stamp + ".txt")
$runCanonical  = Join-Path $runDir    'sitl-sim.txt'
$latestFile    = Join-Path $latestDir 'sitl-sim.txt'

$pio = "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"
$env:PATH = "$env:USERPROFILE\.tools\w64devkit\bin;$env:PATH"

& $pio test -e sim 2>&1 |
    ForEach-Object {
        if ($_ -is [System.Management.Automation.ErrorRecord]) { $_.Exception.Message } else { $_ }
    } |
    Tee-Object -FilePath $runFile

$code = $LASTEXITCODE

if (Test-Path $runFile)
{
    try { Copy-Item -Force $runFile $runCanonical } catch { Write-Host 'Warning: could not refresh canonical dated SITL file (possibly locked).' }
    try { Copy-Item -Force $runFile $latestFile   } catch { Write-Host 'Warning: could not refresh latest SITL file (possibly locked).' }
}

exit $code
