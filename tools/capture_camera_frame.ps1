param(
    [string]$Port = 'COM6',
    [int]$Baud = 115200,
    [ValidateRange(1, 2147483647)][int]$Frames = 2,
    [ValidateRange(1, 752)][int]$Width = 376,
    [ValidateRange(1, 480)][int]$Height = 240,
    [ValidateRange(1, 86400)][int]$TimeoutSeconds = 60,
    [Parameter(Mandatory = $true)][string]$OutputDirectory
)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
$null = New-Item -ItemType Directory -Path $OutputDirectory -Force
$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, 'None', 8, 'One')
$serial.ReadTimeout = 500
$frameLength = $Width * $Height
$serial.ReadBufferSize = [Math]::Max(262144, $frameLength * 2)
# V2 gray-image header: dimensions are little-endian; byte 1 is an 8-bit checksum.
$header = [byte[]](0xAA, 0, 2, 2,
    ($Width -band 0xFF), (($Width -shr 8) -band 0xFF),
    ($Height -band 0xFF), (($Height -shr 8) -band 0xFF), 1, 0, 0, 0)
$headerChecksum = 0
foreach ($headerByte in $header) { $headerChecksum += $headerByte }
$header[1] = [byte]($headerChecksum -band 0xFF)
$buffer = [System.Collections.Generic.List[byte]]::new()
$chunk = [byte[]]::new(32768)
$captured = 0
$clock = [Diagnostics.Stopwatch]::StartNew()
try {
    $serial.Open()
    $serial.DiscardInBuffer()
    while ($captured -lt $Frames -and $clock.Elapsed.TotalSeconds -lt $TimeoutSeconds) {
        try { $count = $serial.Read($chunk, 0, $chunk.Length) } catch [TimeoutException] { continue }
        for ($i = 0; $i -lt $count; $i++) { $buffer.Add($chunk[$i]) }
        while ($buffer.Count -ge ($frameLength + $header.Length)) {
            $start = -1
            for ($i = 0; $i -le $buffer.Count - $header.Length; $i++) {
                if ($buffer[$i] -ne 0xAA) { continue }
                $matches = $true
                for ($j = 1; $j -lt $header.Length; $j++) {
                    if ($buffer[$i + $j] -ne $header[$j]) { $matches = $false; break }
                }
                if ($matches) { $start = $i; break }
            }
            if ($start -lt 0) {
                $buffer.RemoveRange(0, $buffer.Count - $header.Length + 1)
                break
            }
            if ($start -gt 0) {
                $prefix = [Text.Encoding]::ASCII.GetString($buffer.GetRange(0, $start).ToArray())
                [regex]::Matches($prefix, '#CAM [^\r\n]+') | ForEach-Object { $_.Value }
                $buffer.RemoveRange(0, $start)
            }
            if ($buffer.Count -lt ($frameLength + $header.Length)) { break }
            $pixels = $buffer.GetRange($header.Length, $frameLength).ToArray()
            $buffer.RemoveRange(0, $frameLength + $header.Length)
            $captured++
            $base = Join-Path $OutputDirectory ('frame-{0:D2}' -f $captured)
            [IO.File]::WriteAllBytes($base + '.raw', $pixels)
            $bitmap = [Drawing.Bitmap]::new($Width, $Height)
            try {
                for ($y = 0; $y -lt $Height; $y++) {
                    for ($x = 0; $x -lt $Width; $x++) {
                        $v = $pixels[$y * $Width + $x]
                        $bitmap.SetPixel($x, $y, [Drawing.Color]::FromArgb($v, $v, $v))
                    }
                }
                $bitmap.Save($base + '.png', [Drawing.Imaging.ImageFormat]::Png)
            } finally { $bitmap.Dispose() }
            $lowerRegionStart = [int][Math]::Floor($Height / 3.0) * $Width
            foreach ($region in @(@{Name='full'; Start=0}, @{Name='lower_two_thirds'; Start=$lowerRegionStart})) {
                $sorted = $pixels[$region.Start..($frameLength-1)] | Sort-Object
                $stats = $sorted | Measure-Object -Average -Minimum -Maximum
                [pscustomobject]@{
                    Frame=$captured; Width=$Width; Height=$Height
                    Region=$region.Name; Min=$stats.Minimum; Max=$stats.Maximum
                    Mean=[math]::Round($stats.Average,2)
                    P10=$sorted[[int][Math]::Floor(($sorted.Count-1)*0.10)]
                    P50=$sorted[[int][Math]::Floor(($sorted.Count-1)*0.50)]
                    P90=$sorted[[int][Math]::Floor(($sorted.Count-1)*0.90)]
                    P99=$sorted[[int][Math]::Floor(($sorted.Count-1)*0.99)]
                    Image=$base+'.png'
                } | ConvertTo-Json -Compress
            }
            if ($captured -ge $Frames) { break }
        }
    }
    if ($captured -lt $Frames) { throw "Only captured $captured of $Frames complete $Width x $Height camera frames in $TimeoutSeconds seconds." }
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
