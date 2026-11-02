param([string]$Compiler='gcc')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$dest=Join-Path $root 'demo\mdk\Objects\pdf-ring-tests'
New-Item -ItemType Directory -Force $dest | Out-Null
foreach($suite in @('test_pdf_ring','test_native_ring_photos')) {
 $sources=@((Join-Path $PSScriptRoot ($suite+'.c')),(Join-Path $root 'demo\user\src\ring_control.c'),(Join-Path $root 'demo\user\src\ring_corner.c'))
 if($suite -eq 'test_native_ring_photos') {$sources += (Join-Path $root 'demo\user\src\track_vision.c')}
 $exe=Join-Path $dest ($suite+'.exe')
 & $Compiler -std=c99 -Wall -Wextra -Werror -Wno-sign-compare -pedantic -O2 -I (Join-Path $root 'demo\user\inc') @sources -lm -o $exe
 if($LASTEXITCODE -ne 0) {throw 'Compile failed'}
 if($suite -eq 'test_native_ring_photos') {& $exe (Join-Path $PSScriptRoot 'fixtures\native_ring')} else {& $exe}
 if($LASTEXITCODE -ne 0) {throw 'Test failed'}
}
