param(
    [string]$Port = 'COM6',
    [int]$Baud = 115200,
    [int]$Frames = 2,
    [Parameter(Mandatory = $true)][string]$OutputDirectory
)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
$null = New-Item -ItemType Directory -Path $OutputDirectory -Force
$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, 'None', 8, 'One')
$serial.ReadTimeout = 500
$serial.ReadBufferSize = 262144
$header = [byte[]](0xAA, 0xE3, 2, 2, 188, 0, 120, 0, 1, 0, 0, 0)
$buffer = [System.Collections.Generic.List[byte]]::new()
$chunk = [byte[]]::new(32768)
$frameLength = 188 * 120
$captured = 0
$clock = [Diagnostics.Stopwatch]::StartNew()
try {
    $serial.Open()
    $serial.DiscardInBuffer()
    while ($captured -lt $Frames -and $clock.Elapsed.TotalSeconds -lt 25) {
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
            $bitmap = [Drawing.Bitmap]::new(188, 120)
            try {
                for ($y = 0; $y -lt 120; $y++) {
                    for ($x = 0; $x -lt 188; $x++) {
                        $v = $pixels[$y * 188 + $x]
                        $bitmap.SetPixel($x, $y, [Drawing.Color]::FromArgb($v, $v, $v))
                    }
                }
                $bitmap.Save($base + '.png', [Drawing.Imaging.ImageFormat]::Png)
            } finally { $bitmap.Dispose() }
            foreach ($region in @(@{Name='full'; Start=0}, @{Name='lower_two_thirds'; Start=40*188})) {
                $sorted = $pixels[$region.Start..($frameLength-1)] | Sort-Object
                $stats = $sorted | Measure-Object -Average -Minimum -Maximum
                [pscustomobject]@{
                    Frame=$captured; Region=$region.Name; Min=$stats.Minimum; Max=$stats.Maximum
                    Mean=[math]::Round($stats.Average,2)
                    P10=$sorted[[int]($sorted.Count*0.10)]; P50=$sorted[[int]($sorted.Count*0.50)]
                    P90=$sorted[[int]($sorted.Count*0.90)]; P99=$sorted[[int]($sorted.Count*0.99)]
                    Image=$base+'.png'
                } | ConvertTo-Json -Compress
            }
            if ($captured -ge $Frames) { break }
        }
    }
    if ($captured -lt $Frames) { throw "Only captured $captured of $Frames complete camera frames in 25 seconds." }
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
