$ErrorActionPreference = 'Continue'

$date = Get-Date -Format 'yyyy-MM-dd'
$base = 'evidence'
$runDir = Join-Path $base $date
$latestDir = Join-Path $base 'latest'
$stamp = Get-Date -Format 'HHmmss'

New-Item -ItemType Directory -Force -Path $runDir | Out-Null
New-Item -ItemType Directory -Force -Path $latestDir | Out-Null

$runFile = Join-Path $runDir ("cppcheck-practical-" + $stamp + ".txt")
$runCanonical = Join-Path $runDir 'cppcheck-practical.txt'
$latestFile = Join-Path $latestDir 'cppcheck-practical.txt'

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
    '--error-exitcode=1',
    '-Isrc',
    '-Iinclude',
    'src'
)

$sw = [System.Diagnostics.Stopwatch]::StartNew()
Write-Host 'Cppcheck practical started...'
& 'C:/Program Files/Cppcheck/cppcheck.exe' @cppArgs 2>&1 | Out-File -Encoding utf8 $runFile

$code = $LASTEXITCODE
if (Test-Path $runFile)
{
    try { Copy-Item -Force $runFile $runCanonical } catch { Write-Host 'Warning: could not refresh canonical dated practical file (possibly locked).' }
    try { Copy-Item -Force $runFile $latestFile } catch { Write-Host 'Warning: could not refresh latest practical file (possibly locked).' }
}

$sw.Stop()
Write-Host ("Cppcheck practical finished in {0:N1}s (exit={1})." -f $sw.Elapsed.TotalSeconds, $code)

exit $code
