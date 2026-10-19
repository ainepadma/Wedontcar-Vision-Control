param([string]$Compiler = 'gcc')

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$outputDirectory = Join-Path $projectRoot 'demo\mdk\Objects\host-tests'
$null = New-Item -ItemType Directory -Path $outputDirectory -Force
$outputExecutable = Join-Path $outputDirectory 'test_imu660rb.exe'
$compilerCommand = Get-Command $Compiler -ErrorAction Stop

& $compilerCommand.Source -std=c99 -Wall -Wextra -Werror -pedantic -O2 `
    '-I' (Join-Path $projectRoot 'libraries\zf_device') `
    (Join-Path $PSScriptRoot 'test_imu660rb.c') `
    '-lm' '-o' $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "IMU660RB test compilation failed ($LASTEXITCODE)." }
& $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "IMU660RB host tests failed ($LASTEXITCODE)." }
