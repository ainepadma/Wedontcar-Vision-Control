param(
    [string]$Compiler = 'E:\mingw64\bin\gcc.exe',
    [string[]]$Frames = @()
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$outputDirectory = Join-Path $projectRoot 'demo\mdk\Objects\host-tests'
$null = New-Item -ItemType Directory -Path $outputDirectory -Force
$outputExecutable = Join-Path $outputDirectory 'test_track_preprocess.exe'
$compilerCommand = Get-Command $Compiler -ErrorAction Stop
& $compilerCommand.Source -std=c99 -Wall -Wextra -Werror -pedantic -O2 `
    '-I' (Join-Path $projectRoot 'demo\user\inc') `
    (Join-Path $PSScriptRoot 'test_track_preprocess.c') `
    (Join-Path $projectRoot 'demo\user\src\img_process.c') `
    (Join-Path $projectRoot 'demo\user\src\track_vision.c') `
    (Join-Path $projectRoot 'demo\user\src\ring_corner.c') `
    (Join-Path $projectRoot 'demo\user\src\track_control.c') `
    '-lm' '-o' $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "Track preprocessing compilation failed ($LASTEXITCODE)." }
& $outputExecutable @Frames
if ($LASTEXITCODE -ne 0) { throw "Track preprocessing tests failed ($LASTEXITCODE)." }
