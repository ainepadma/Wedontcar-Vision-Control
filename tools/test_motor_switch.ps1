param([string]$Compiler = 'gcc')

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$outputDirectory = Join-Path $projectRoot 'demo\mdk\Objects\host-tests'
$null = New-Item -ItemType Directory -Path $outputDirectory -Force
$outputExecutable = Join-Path $outputDirectory 'test_motor_switch.exe'
$compilerCommand = Get-Command $Compiler -ErrorAction Stop

& $compilerCommand.Source -std=c99 -Wall -Wextra -Werror -pedantic -O2 `
    '-I' (Join-Path $projectRoot 'demo\user\inc') `
    (Join-Path $PSScriptRoot 'test_motor_switch.c') `
    '-o' $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "Motor-switch test compilation failed ($LASTEXITCODE)." }
& $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "Motor-switch host tests failed ($LASTEXITCODE)." }
