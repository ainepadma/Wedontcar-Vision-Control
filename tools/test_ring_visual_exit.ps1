param([string]$Compiler='gcc')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$dest=Join-Path $root 'demo/mdk/Objects/visual-exit-tests'
New-Item -ItemType Directory -Force $dest | Out-Null
foreach($suite in @('test_ring_visual_exit','test_ring_exit_directions','test_ring_s3_lock','test_ring_border_angle')) {
 $exe=Join-Path $dest ($suite+'.exe')
 & $Compiler -std=c99 -Wall -Wextra -Werror -Wno-sign-compare -pedantic -O2 -I (Join-Path $root 'demo/user/inc') (Join-Path $PSScriptRoot ($suite+'.c')) (Join-Path $root 'demo/user/src/ring_corner.c') (Join-Path $root 'demo/user/src/ring_control.c') -lm -o $exe
 if($LASTEXITCODE -ne 0) {throw 'Compile failed'}
 & $exe
 if($LASTEXITCODE -ne 0) {throw 'Test failed'}
}
