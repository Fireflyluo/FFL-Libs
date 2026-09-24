# 将 img_bin/*.bin 经 COM4 写入板载 W25Q（固件需带 UART PROTO）
$ErrorActionPreference = "Stop"
$PortName = "COM4"
if ($env:IMG_COM -and $env:IMG_COM.Length -gt 0) { $PortName = $env:IMG_COM }
Write-Host "using $PortName"
$base = Split-Path -Parent $PSScriptRoot
$binDir = Join-Path $base "img_bin"
$imgs = @(
  @{ f = "img0.bin"; addr = 0x000000 },
  @{ f = "img1.bin"; addr = 0x010000 },
  @{ f = "img2.bin"; addr = 0x020000 }
)

function Read-LineOK($port) {
  $sw = [Diagnostics.Stopwatch]::StartNew()
  $s = ""
  while ($sw.Elapsed.TotalSeconds -lt 5) {
    try {
      $c = [char]$port.ReadByte()
      if ($c -eq "`n") {
        if ($s -match "OK") { return $true }
        $s = ""
      } else {
        $s += $c
      }
    } catch { }
  }
  return $false
}

$port = New-Object System.IO.Ports.SerialPort
$port.PortName = $PortName
$port.BaudRate = 115200
$port.DataBits = 8
$port.Parity = [System.IO.Ports.Parity]::None
$port.StopBits = [System.IO.Ports.StopBits]::One
$port.ReadTimeout = 3000
$port.Open()
$port.DiscardInBuffer()
$port.Write("P`n")
Start-Sleep -Milliseconds 80

foreach ($it in $imgs) {
  $path = Join-Path $binDir $it.f
  $data = [System.IO.File]::ReadAllBytes($path)
  $addr = $it.addr
  Write-Host "write $($it.f) @0x$('{0:X6}' -f $addr) len=$($data.Length)"

  $nsec = [math]::Ceiling($data.Length / 4096)
  for ($s = 0; $s -lt $nsec; $s++) {
    $port.DiscardInBuffer()
    $port.Write("E $($addr + $s * 4096)`n")
    if (-not (Read-LineOK $port)) { Write-Host "  erase timeout"; exit 1 }
  }

  $off = 0
  while ($off -lt $data.Length) {
    $n = [math]::Min(256, $data.Length - $off)
    $port.DiscardInBuffer()
    $port.Write("W $($addr + $off) $n`n")
    if (-not (Read-LineOK $port)) { Write-Host "  W ack timeout at $off"; exit 1 }
    $chunk = New-Object byte[] $n
    [Array]::Copy($data, $off, $chunk, 0, $n)
    $port.Write($chunk, 0, $n)
    if (-not (Read-LineOK $port)) { Write-Host "  W done timeout at $off"; exit 1 }
    $off += $n
    if (($off % 4096) -eq 0) { Write-Host "  $off" }
  }

  $port.DiscardInBuffer()
  $port.Write("R $addr 32`n")
  $back = New-Object byte[] 32
  $got = 0
  $sw = [Diagnostics.Stopwatch]::StartNew()
  while ($got -lt 32 -and $sw.Elapsed.TotalSeconds -lt 2) {
    try {
      $b = $port.ReadByte()
      if ($b -ge 0) { $back[$got++] = [byte]$b }
    } catch { }
  }
  $ok = $true
  if ($got -lt 32) { $ok = $false }
  else {
    for ($i = 0; $i -lt 32; $i++) {
      if ($back[$i] -ne $data[$i]) { $ok = $false; break }
    }
  }
  if ($ok) { Write-Host "  head OK" } else { Write-Host "  VERIFY FAIL got=$got" }
}
$port.Write("Q`n")
$port.Close()
Write-Host "done"
