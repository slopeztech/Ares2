$ErrorActionPreference = 'Continue'

$date = Get-Date -Format 'yyyy-MM-dd'
$base = 'evidence'
$runDir = Join-Path $base $date
$latestDir = Join-Path $base 'latest'
$stamp = Get-Date -Format 'HHmmss'

New-Item -ItemType Directory -Force -Path $runDir | Out-Null
New-Item -ItemType Directory -Force -Path $latestDir | Out-Null

$runFile = Join-Path $runDir ("cppcheck-misra-addon-" + $stamp + ".txt")
$runCanonical = Join-Path $runDir 'cppcheck-misra-addon.txt'
$latestFile = Join-Path $latestDir 'cppcheck-misra-addon.txt'

$cppArgs = @(
    '--project=compile_commands.json',
    '--addon=.tools/cppcheck-addons/misra.py',
    '--file-filter=src/*',
    '-iC:/Users/Sergio/.platformio',
    '-i.pio',
    '--enable=warning,style,performance,portability',
    '--inline-suppr',
    '--suppress=missingIncludeSystem',
    '--suppress=unusedFunction',
    '--suppress=misra-config',
    '--suppress=*:.pio/*',
    '--suppress=*:.platformio/*',
    '--suppress=*:*framework-arduinoespressif32*',
    '--suppress=misra-c2012-12.3:src/ams/mission_script_engine.cpp',
    '--suppress=misra-c2012-12.3:src/api/api_server.cpp',
    '--suppress=misra-c2012-12.3:src/drivers/baro/bmp280_driver.cpp',
    '--suppress=misra-c2012-12.3:src/drivers/gps/bn220_driver.cpp',
    '--suppress=misra-c2012-12.3:src/drivers/radio/dxlr03_driver.cpp',
    '--std=c++17',
    '--error-exitcode=1'
)

& 'C:/Program Files/Cppcheck/cppcheck.exe' @cppArgs 2>&1 | Out-File -Encoding utf8 $runFile

$code = $LASTEXITCODE
if (Test-Path $runFile)
{
    try { Copy-Item -Force $runFile $runCanonical } catch { Write-Host 'Warning: could not refresh canonical dated MISRA file (possibly locked).' }
    try { Copy-Item -Force $runFile $latestFile } catch { Write-Host 'Warning: could not refresh latest MISRA file (possibly locked).' }
}

exit $code
