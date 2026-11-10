param([string]$Compiler='gcc')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$dest=Join-Path $root 'demo\mdk\Objects\revision4-tests'
New-Item -ItemType Directory -Force $dest | Out-Null
foreach ($suite in @('test_ring_revision4', 'test_ring_s3_outer_handoff')) {
    $exe=Join-Path $dest ($suite+'.exe')
    & $Compiler -std=c99 -Wall -Wextra -Werror -Wno-sign-compare -pedantic -O2 -I (Join-Path $root 'demo/user/inc') (Join-Path $PSScriptRoot ($suite+'.c')) (Join-Path $root 'demo/user/src/track_vision.c') (Join-Path $root 'demo/user/src/ring_corner.c') (Join-Path $root 'demo/user/src/ring_control.c') -lm -o $exe
    if ($LASTEXITCODE -ne 0) {throw "$suite compilation failed"}
    & $exe (Join-Path $PSScriptRoot 'fixtures/revision4')
    if ($LASTEXITCODE -ne 0) {throw "$suite failed"}
}
