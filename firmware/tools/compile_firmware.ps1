# Compile both Thenar Walker sketches for a classic ESP32 (esp32:esp32 core 3.x).
# Uses arduino-cli from PATH, $env:ARDUINO_CLI, or the copy in ..\thenar-bin\tools.
# Nothing is flashed. To upload: add  --upload -p COMx  to the compile line yourself.
$ErrorActionPreference = 'Stop'
$fw = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$cli = $env:ARDUINO_CLI
if (-not $cli) { $c = Get-Command arduino-cli -ErrorAction SilentlyContinue; if ($c) { $cli = $c.Source } }
if (-not $cli) { $cli = Join-Path $fw '..\..\thenar-bin\tools\arduino-cli\arduino-cli.exe' }
if (-not (Test-Path $cli)) { throw 'arduino-cli not found: install it and run  arduino-cli core install esp32:esp32' }
$build = Join-Path $env:TEMP 'thenar_walker_build'
$fail = 0
foreach ($sketch in 'thenar_walker_bot', 'thenar_walker_controller') {
    Write-Host "== $sketch"
    $ErrorActionPreference = 'Continue'   # compiler warnings arrive on stderr
    & $cli compile --fqbn esp32:esp32:esp32 --libraries (Join-Path $fw 'libraries') `
        --build-path (Join-Path $build $sketch) --warnings default (Join-Path $fw $sketch) 2>&1 |
        Select-String -Pattern 'error|warning: (?!.*Example credentials)|Sketch uses|Global variables' | ForEach-Object { "$_" }
    if ($LASTEXITCODE -ne 0) { $fail = 1; Write-Host "FAILED: $sketch" }
    $ErrorActionPreference = 'Stop'
}
exit $fail
