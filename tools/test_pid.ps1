param([string]$Compiler = 'gcc')

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$outputDirectory = Join-Path $projectRoot 'demo\mdk\Objects\host-tests'
$null = New-Item -ItemType Directory -Path $outputDirectory -Force
$outputExecutable = Join-Path $outputDirectory 'test_pid.exe'
$compilerCommand = Get-Command $Compiler -ErrorAction Stop

& $compilerCommand.Source -std=c99 -Wall -Wextra -Werror -Wconversion -Wshadow -pedantic -O2 `
    '-I' (Join-Path $projectRoot 'demo\user\inc') `
    (Join-Path $PSScriptRoot 'test_pid.c') `
    (Join-Path $projectRoot 'demo\user\src\pid.c') `
    '-lm' '-o' $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "PID test compilation failed ($LASTEXITCODE)." }
& $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "PID host tests failed ($LASTEXITCODE)." }
