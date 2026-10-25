param([string]$Compiler='gcc')
$ErrorActionPreference='Stop'
$projectRoot=Split-Path -Parent $PSScriptRoot
$outputDirectory=Join-Path $projectRoot 'demo\mdk\Objects\host-tests'
$null=New-Item -ItemType Directory -Path $outputDirectory -Force
$compilerCommand=Get-Command $Compiler -ErrorAction Stop
foreach($suite in @('test_vision_stability','test_ring_feature_switch')) {
    $executable=Join-Path $outputDirectory ($suite+'.exe')
    $arguments=@('-std=c99','-Wall','-Wextra','-Werror','-Wshadow','-pedantic','-O2',
        '-I',(Join-Path $projectRoot 'demo\user\inc'),(Join-Path $PSScriptRoot ($suite+'.c')),
        (Join-Path $projectRoot 'demo\user\src\track_vision.c'),
        (Join-Path $projectRoot 'demo\user\src\ring_corner.c'))
    if($suite -eq 'test_vision_stability') {
        $arguments+=(Join-Path $projectRoot 'demo\user\src\img_process.c')
    } else {
        $arguments+='-DRING_FEATURE_ENABLED=0'
        $arguments+=(Join-Path $projectRoot 'demo\user\src\ring_control.c')
        $arguments+=(Join-Path $projectRoot 'demo\user\src\track_control.c')
    }
    $arguments+=@('-lm','-o',$executable)
    & $compilerCommand.Source @arguments
    if($LASTEXITCODE -ne 0) {throw "$suite compilation failed."}
    & $executable
    if($LASTEXITCODE -ne 0) {throw "$suite failed."}
}
