param([string]$Compiler = 'gcc')

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$outputDirectory = Join-Path $projectRoot 'demo\mdk\Objects\host-tests'
$null = New-Item -ItemType Directory -Path $outputDirectory -Force
$outputExecutable = Join-Path $outputDirectory 'test_track_vision.exe'
$compilerCommand = Get-Command $Compiler -ErrorAction Stop

& $compilerCommand.Source -std=c99 -Wall -Wextra -Werror -pedantic -O2 `
    '-I' (Join-Path $projectRoot 'demo\user\inc') `
    (Join-Path $PSScriptRoot 'test_track_vision.c') `
    (Join-Path $projectRoot 'demo\user\src\track_vision.c') `
    '-o' $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "Track-vision test compilation failed ($LASTEXITCODE)." }
& $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "Track-vision host tests failed ($LASTEXITCODE)." }
