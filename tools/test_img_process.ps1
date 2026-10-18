param([string]$Compiler = 'gcc')

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$outputDirectory = Join-Path $projectRoot 'demo\mdk\Objects\host-tests'
$null = New-Item -ItemType Directory -Path $outputDirectory -Force
$outputExecutable = Join-Path $outputDirectory 'test_img_process.exe'
$compilerCommand = Get-Command $Compiler -ErrorAction Stop

& $compilerCommand.Source -std=c99 -Wall -Wextra -Werror -pedantic -O2 `
    '-I' (Join-Path $projectRoot 'demo\user\inc') `
    (Join-Path $PSScriptRoot 'test_img_process.c') `
    (Join-Path $projectRoot 'demo\user\src\img_process.c') `
    '-o' $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "Image-processing test compilation failed ($LASTEXITCODE)." }
& $outputExecutable
if ($LASTEXITCODE -ne 0) { throw "Image-processing host tests failed ($LASTEXITCODE)." }
