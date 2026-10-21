param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$outputDirectory = Join-Path $projectRoot 'demo\mdk\Objects\host-tests'
$null = New-Item -ItemType Directory -Path $outputDirectory -Force
$outputExecutable = Join-Path $outputDirectory 'test_ring_routes.exe'
$compilerCommand = Get-Command $Compiler -ErrorAction Stop
& $compilerCommand.Source -std=c99 -Wall -Wextra -Werror -Wshadow -pedantic -O2 `
    '-I' (Join-Path $projectRoot 'demo\user\inc') `
    (Join-Path $PSScriptRoot 'test_ring_routes.c') `
    (Join-Path $projectRoot 'demo\user\src\track_vision.c') `
    (Join-Path $projectRoot 'demo\user\src\ring_control.c') `
    (Join-Path $projectRoot 'demo\user\src\track_control.c') `
    '-lm' '-o' $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "Ring-routing test compilation failed ($LASTEXITCODE)." }
& $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "Ring-routing tests failed ($LASTEXITCODE)." }
