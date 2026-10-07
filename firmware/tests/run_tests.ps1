# Compile and run the host unit tests with PlatformIO's bundled MinGW g++ (no system compiler needed).
$ErrorActionPreference = 'Stop'
$gpp = Join-Path $env:USERPROFILE '.platformio\packages\toolchain-gccmingw32\bin\g++.exe'
if (-not (Test-Path $gpp)) { throw "MinGW g++ not found at $gpp (install PlatformIO or any g++ and edit this path)" }
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$exe = Join-Path $env:TEMP 'thenar_walker_tests.exe'
& $gpp -std=gnu++14 -O1 -Wall -Wextra -static -o $exe (Join-Path $here 'test_core.cpp')
if ($LASTEXITCODE -ne 0) { throw 'compile failed' }
& $exe
exit $LASTEXITCODE
