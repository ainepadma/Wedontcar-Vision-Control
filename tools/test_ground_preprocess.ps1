param([string]$Compiler='gcc', [string[]]$Frames=@())
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$outputDirectory=Join-Path $projectRoot 'demo\mdk\Objects\host-tests'
$null=New-Item -ItemType Directory -Path $outputDirectory -Force
$executable=Join-Path $outputDirectory 'test_ground_preprocess.exe'
$compilerCommand=Get-Command $Compiler -ErrorAction Stop
& $compilerCommand.Source -std=c99 -Wall -Wextra -Werror -Wshadow -pedantic -O2 `
    '-I' (Join-Path $projectRoot 'demo\user\inc') `
    (Join-Path $PSScriptRoot 'test_ground_preprocess.c') `
    (Join-Path $projectRoot 'demo\user\src\img_process.c') `
    (Join-Path $projectRoot 'demo\user\src\track_vision.c') `
    (Join-Path $projectRoot 'demo\user\src\ring_corner.c') `
    '-lm' '-o' $executable
if($LASTEXITCODE -ne 0) { throw 'Ground preprocessing test compilation failed.' }
& $executable @Frames
if($LASTEXITCODE -ne 0) { throw 'Ground preprocessing tests failed.' }
