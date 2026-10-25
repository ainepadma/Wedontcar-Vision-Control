param([string]$Compiler = 'E:\mingw64\bin\gcc.exe')

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$outputDirectory = Join-Path $projectRoot 'demo\mdk\Objects\host-tests'
$null = New-Item -ItemType Directory -Path $outputDirectory -Force
$outputExecutable = Join-Path $outputDirectory 'test_line_following.exe'
$compilerCommand = Get-Command $Compiler -ErrorAction Stop

& $compilerCommand.Source -std=c99 -Wall -Wextra -Werror -pedantic -O2 `
    '-I' (Join-Path $projectRoot 'demo\user\inc') `
    (Join-Path $PSScriptRoot 'test_line_following.c') `
    (Join-Path $projectRoot 'demo\user\src\track_vision.c') `
    (Join-Path $projectRoot 'demo\user\src\ring_corner.c') `
    (Join-Path $projectRoot 'demo\user\src\track_control.c') `
    (Join-Path $projectRoot 'demo\user\src\pid.c') `
    '-lm' '-o' $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "Line-following integration compilation failed ($LASTEXITCODE)." }
& $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "Line-following integration tests failed ($LASTEXITCODE)." }
