<#
.SYNOPSIS
  One-time setup: test, build and flash both AAI16 edge nodes (node1 and node2) in one go.

.DESCRIPTION
  Run this once on a new laptop, or whenever the firmware or the Wi-Fi/broker settings change.
  It is a friendly front end to tools\build_all.ps1, which does the actual work:

    1. creates the Python environment (.venv) and installs the requirements, if they are missing
    2. runs every host test suite (unless -SkipTests)
    3. builds node1 and node2, each in its own folder (build_node1, build_node2) with its own settings
    4. checks the settings that went into each image (node id, broker, PSRAM, 1 kHz tick, ESP-NN)
    5. flashes each board to its serial port

  Wi-Fi SSID, password and broker address are asked for once and remembered in the git-ignored build folders.
  The password is never printed and never written anywhere that git tracks.

  Needs: Windows PowerShell, Python 3.10+, ESP-IDF 6.1 (an ESP-IDF PowerShell, or the paths in tools\idf_env.ps1).

.PARAMETER Node1Port
  Serial port of node1, for example COM12. Leave it out to be asked (blank = build without flashing).

.PARAMETER Node2Port
  Serial port of node2, for example COM11. Leave it out to be asked (blank = build without flashing).

.PARAMETER Ssid
  Wi-Fi network name (2.4 GHz). Asked for if not given and not remembered from a previous build.

.PARAMETER Password
  Wi-Fi password. Asked for (hidden) if not given and not remembered.

.PARAMETER Broker
  IP address of the laptop running Mosquitto, as the boards reach it (for example 172.20.10.2).

.PARAMETER SkipTests
  Skip the host test suites (about 3 minutes) when you only changed settings.

.PARAMETER NoFlash
  Build only.

.EXAMPLE
  .\build_and_flash.ps1
  Asks for the ports and the network settings, then tests, builds and flashes both boards.

.EXAMPLE
  .\build_and_flash.ps1 -Node1Port COM12 -Node2Port COM11 -SkipTests
  Rebuilds and flashes both boards with the remembered Wi-Fi and broker settings.
#>
[CmdletBinding()]
param(
    [string]$Node1Port,
    [string]$Node2Port,
    [string]$Ssid,
    [string]$Password,
    [string]$Broker,
    [switch]$SkipTests,
    [switch]$NoFlash
)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
Set-Location $root

Write-Host "AAI16 configurable edge node: one-time build and flash" -ForegroundColor Cyan
Write-Host "Project folder: $root`n"

# ---------------------------------------------------------------------------------------------- serial ports
$ports = @{}
if (-not $NoFlash) {
    $available = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    if ($available) { Write-Host ("Serial ports found: " + ($available -join ', ')) }
    else { Write-Host "No serial ports found. Plug in the boards, or continue to build without flashing." -ForegroundColor Yellow }
    if (-not $Node1Port) { $Node1Port = Read-Host "COM port of node1 (blank = do not flash node1)" }
    if (-not $Node2Port) { $Node2Port = Read-Host "COM port of node2 (blank = do not flash node2)" }
    if ($Node1Port) { $ports['node1'] = $Node1Port.Trim().ToUpper() }
    if ($Node2Port) { $ports['node2'] = $Node2Port.Trim().ToUpper() }
    if ($ports.Count -eq 2 -and $ports['node1'] -eq $ports['node2']) { throw "node1 and node2 cannot use the same port ($($ports['node1']))" }
}

# ---------------------------------------------------------------------------------------------- broker hint
if (-not $Broker -and -not $env:AAI16_BROKER) {
    $ips = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Where-Object { $_.IPAddress -notlike '127.*' -and $_.IPAddress -notlike '169.254.*' } |
        Select-Object -ExpandProperty IPAddress
    if ($ips) { Write-Host ("This laptop's IPv4 addresses (use the one on the boards' Wi-Fi as the broker): " + ($ips -join ', ')) }
}

# ---------------------------------------------------------------------------------------------- build (and flash)
$buildArgs = @{ Nodes = @('node1', 'node2') }
if ($Ssid)      { $buildArgs['Ssid'] = $Ssid }
if ($Password)  { $buildArgs['Password'] = $Password }
if ($Broker)    { $buildArgs['Broker'] = $Broker }
if ($SkipTests) { $buildArgs['SkipTests'] = $true }

& (Join-Path $root 'tools\build_all.ps1') @buildArgs
if (-not $?) { throw "build failed: see the messages above" }

# tools\build_all.ps1 flashes every node it built, so flash here only the boards that were given a port
if ($ports.Count -gt 0) {
    if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) { . (Join-Path $root 'tools\idf_env.ps1') }
    foreach ($node in @('node1', 'node2')) {
        if (-not $ports.ContainsKey($node)) { Write-Host "  note $node not flashed (no port given)" -ForegroundColor Yellow; continue }
        Write-Host "`n=== Flashing $node on $($ports[$node])" -ForegroundColor Cyan
        & idf.py -B "build_$node" -p $ports[$node] flash
        if ($LASTEXITCODE -ne 0) { throw "flashing $node on $($ports[$node]) failed (wrong port, or hold BOOT while pressing RESET and retry)" }
        Write-Host "  ok   flashed $node" -ForegroundColor Green
    }
}

Write-Host "`nDone. Next steps:" -ForegroundColor Green
Write-Host "  1. Start Mosquitto on this laptop (port 1883, listening on all interfaces)."
Write-Host "  2. .\.venv\Scripts\python.exe tools\json_bridge_all.py --broker 127.0.0.1   (leave it running)"
Write-Host "  3. Open MQTTX and follow docs\mqttx_setup.md, then the demo in docs\demo_guide.md."
