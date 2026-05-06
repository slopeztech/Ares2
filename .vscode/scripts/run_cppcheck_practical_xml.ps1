$ErrorActionPreference = 'Continue'

$date = Get-Date -Format 'yyyy-MM-dd'
$base = 'evidence'
$runDir = Join-Path $base $date
$latestDir = Join-Path $base 'latest'
$stamp = Get-Date -Format 'HHmmss'

New-Item -ItemType Directory -Force -Path $runDir | Out-Null
New-Item -ItemType Directory -Force -Path $latestDir | Out-Null

$runFile = Join-Path $runDir ("cppcheck-practical-" + $stamp + ".xml")
$runCanonical = Join-Path $runDir 'cppcheck-practical.xml'
$latestFile = Join-Path $latestDir 'cppcheck-practical.xml'

$cppArgs = @(
    '-j', '4',
    '--max-configs=1',
    '--check-level=normal',
    '--enable=warning,style,performance,portability',
    '--inline-suppr',
    '--suppress=missingIncludeSystem',
    '--suppress=unusedFunction',
    '--suppress=*:.pio/*',
    '--std=c++17',
    '--quiet',
    '--xml',
    '--xml-version=2',
    '-Isrc',
    '-Iinclude',
    'src'
)

$sw = [System.Diagnostics.Stopwatch]::StartNew()
Write-Host 'Cppcheck practical XML started...'
& 'C:/Program Files/Cppcheck/cppcheck.exe' @cppArgs 2>&1 |
    ForEach-Object { if ($_ -is [System.Management.Automation.ErrorRecord]) { $_.Exception.Message } else { $_ } } |
    Out-File -Encoding utf8 $runFile

$code = $LASTEXITCODE
if (Test-Path $runFile)
{
    try { Copy-Item -Force $runFile $runCanonical } catch { Write-Host 'Warning: could not refresh canonical dated practical XML (possibly locked).' }
    try { Copy-Item -Force $runFile $latestFile } catch { Write-Host 'Warning: could not refresh latest practical XML (possibly locked).' }
}

$sw.Stop()
Write-Host ("Cppcheck practical XML finished in {0:N1}s (exit={1})." -f $sw.Elapsed.TotalSeconds, $code)

exit $code
