# USB CDC 写图入口；分包、CRC、写后 ACK 由 Python 工具统一实现。
$ErrorActionPreference = "Stop"
$script = Join-Path $PSScriptRoot "usb_upload_img.py"
if (-not (Test-Path -LiteralPath $script)) { throw "missing $script" }
& py -3 $script @args
exit $LASTEXITCODE
