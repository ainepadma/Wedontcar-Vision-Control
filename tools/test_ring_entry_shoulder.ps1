param([string]$Compiler='gcc')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$dest=Join-Path $root 'demo/mdk/Objects/entry-shoulder-tests'
New-Item -ItemType Directory -Force $dest | Out-Null
$exe=Join-Path $dest 'test_ring_entry_shoulder.exe'
& $Compiler -std=c99 -Wall -Wextra -Werror -Wno-sign-compare -pedantic -O2 -I (Join-Path $root 'demo/user/inc') (Join-Path $PSScriptRoot 'test_ring_entry_shoulder.c') (Join-Path $root 'demo/user/src/track_vision.c') (Join-Path $root 'demo/user/src/ring_corner.c') (Join-Path $root 'demo/user/src/ring_control.c') -lm -o $exe
if($LASTEXITCODE -ne 0) {throw 'Compile failed'}
& $exe (Join-Path $PSScriptRoot 'fixtures/right_entry/entry_curved.bin')
if($LASTEXITCODE -ne 0) {throw 'Test failed'}
