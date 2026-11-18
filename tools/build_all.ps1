<#
.SYNOPSIS
  One command to check, test, build (and optionally flash) the AAI16 ESP32-S3 edge nodes.

.DESCRIPTION
  1. checks the tools (Python venv with the project requirements, ESP-IDF 6.x),
  2. runs every host test suite (tools\run_all_tests.py),
  3. builds each node into its OWN build folder with its OWN settings (node id, Wi-Fi, broker),
     generated from sdkconfig.defaults + a small per-node file, so no `idf.py menuconfig` is needed,
  4. checks the settings that ended up in each build,
  5. with -Flash, flashes each node to its serial port.

  The Wi-Fi SSID/password are taken from (first match wins): the parameters, the environment variables
  AAI16_WIFI_SSID / AAI16_WIFI_PASSWORD / AAI16_BROKER, the settings of the previous build of that node, or a prompt.
  They are written only into <build folder>\node.defaults and <build folder>\sdkconfig, which are git-ignored
  (build_*/), and the password is never printed. Your root `sdkconfig` and `build\` folder are never touched.

.EXAMPLE
  .\tools\build_all.ps1 -Ssid "MyHotspot" -Password "secret" -Broker 172.20.10.2
  Tests, then builds node1 and node2.

.EXAMPLE
  .\tools\build_all.ps1 -SkipTests -Flash -Ports @{ node1 = 'COM12'; node2 = 'COM11' }
  Builds both nodes (reusing the Wi-Fi settings of the last build) and flashes them.

.EXAMPLE
  .\tools\build_all.ps1 -Nodes node2 -NoWifi
  Builds only node2 without network (serial console only).

.EXAMPLE
  .\tools\build_all.ps1 -TestsOnly
  Only the checks and the host tests.

.EXAMPLE
  .\tools\build_all.ps1 -SkipTests -Reference -Nodes node1
  Builds the reference image (ESP-NN off) into build_ref_node1 for the inference-speed comparison; the script checks that the
  finished binary really contains no ESP-NN kernels (and that the normal image does).
#>
[CmdletBinding()]
param(
    [string[]]$Nodes = @('node1', 'node2'),   # node ids (1-15 chars: a-z 0-9 _ -); each gets its own build folder
    [string]$Ssid,                            # Wi-Fi SSID (2.4 GHz)
    [string]$Password,                        # Wi-Fi password (never printed)
    [string]$Broker,                          # MQTT broker IP or host name, for example 172.20.10.2 (port 1883)
    [switch]$NoWifi,                          # empty SSID: the node runs with the serial console only
    [switch]$Flash,                           # flash after a successful build (needs -Ports)
    [hashtable]$Ports,                        # @{ node1 = 'COM12'; node2 = 'COM11' }
    [switch]$SkipTests,                       # skip the host test suites
    [switch]$TestsOnly,                       # checks + host tests only, no firmware build
    [switch]$Clean,                           # delete the build folder of each node first (settings are kept)
    [switch]$Reference,                       # build the NON-accelerated reference image (no ESP-NN) into build_ref_<node>
    [string]$BuildPrefix = 'build_'           # build folder = <repo>\<prefix><node>
)

# Native tools (idf.py, python, TensorFlow) print warnings to stderr; that must not abort the script. Failures are caught by
# explicit exit-code checks (Check-Exit) and by Fail, which throws; file operations below use -ErrorAction Stop.
$ErrorActionPreference = 'Continue'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$startTime = Get-Date
if ($Reference -and -not $PSBoundParameters.ContainsKey('BuildPrefix')) { $BuildPrefix = 'build_ref_' }

function Step($text) { Write-Host "`n=== $text" -ForegroundColor Cyan }
function Ok($text)   { Write-Host "  ok   $text" -ForegroundColor Green }
function Warn($text) { Write-Host "  note $text" -ForegroundColor Yellow }
function Fail($text) { Write-Host "  FAIL $text" -ForegroundColor Red; throw $text }
function Check-Exit($what) { if ($LASTEXITCODE -ne 0) { Fail "$what failed (exit code $LASTEXITCODE)" } }

# ------------------------------------------------------------------------------------------------ 1. tools
Step "1/5 Checking tools"
$venvPy = Join-Path $root '.venv\Scripts\python.exe'
if (-not (Test-Path $venvPy)) {
    Warn "no .venv yet: creating it and installing the requirements (TensorFlow is large; this takes a few minutes)"
    $sysPy = $null
    foreach ($cand in @('py', 'python')) { if (Get-Command $cand -ErrorAction SilentlyContinue) { $sysPy = $cand; break } }
    if (-not $sysPy) { Fail "Python 3.10+ not found. Install it from python.org, then run this script again." }
    if ($sysPy -eq 'py') { & py -3 -m venv (Join-Path $root '.venv') } else { & python -m venv (Join-Path $root '.venv') }
    Check-Exit "creating the virtual environment"
}
& $venvPy -c "import ziglang, google.protobuf, paho.mqtt.client, numpy, tensorflow" 2>$null
if ($LASTEXITCODE -ne 0) {
    Warn "installing the Python requirements into .venv"
    & $venvPy -m pip install --quiet -r (Join-Path $root 'tools\requirements.txt') -r (Join-Path $root 'ml\requirements.txt')
    Check-Exit "pip install"
    & $venvPy -c "import ziglang, google.protobuf, paho.mqtt.client, numpy, tensorflow"
    Check-Exit "importing the Python requirements"
}
Ok ("python: " + (& $venvPy --version))

if (-not $TestsOnly) {
    if (-not (Get-Command idf.py -ErrorAction SilentlyContinue)) {
        $idfEnv = Join-Path $root 'tools\idf_env.ps1'
        if (-not (Test-Path $idfEnv)) { Fail "ESP-IDF not found and tools\idf_env.ps1 is missing. Open an ESP-IDF PowerShell, or fix the paths in tools\idf_env.ps1." }
        . $idfEnv
    }
    $idfVersion = (& idf.py --version | Out-String).Trim()
    Check-Exit "idf.py --version"
    if ($idfVersion -notmatch 'v?6\.') { Warn "expected ESP-IDF 6.x, found: $idfVersion (the project was built with 6.1)" }
    Ok "esp-idf: $idfVersion"
}

# ------------------------------------------------------------------------------------------------ 2. tests
if ($SkipTests) {
    Step "2/5 Host tests"; Warn "skipped (-SkipTests)"
} else {
    Step "2/5 Running every host test suite (about 3 minutes)"
    & $venvPy (Join-Path $root 'tools\run_all_tests.py')
    Check-Exit "host tests"
    Ok "all host suites pass"
}
if ($TestsOnly) {
    Write-Host "`nDone (tests only) in $([int]((Get-Date) - $startTime).TotalSeconds) s." -ForegroundColor Green
    return
}

# ------------------------------------------------------------------------------------------------ 3. build
function Quote-Kconfig([string]$v) { '"' + ($v -replace '\\', '\\' -replace '"', '\"') + '"' }

function Read-Previous($defaultsFile, $key) {
    if (-not (Test-Path $defaultsFile)) { return $null }
    $line = Select-String -Path $defaultsFile -Pattern "^$key=" | Select-Object -First 1
    if (-not $line) { return $null }
    $v = $line.Line.Substring($key.Length + 1).Trim()
    if ($v.Length -ge 2 -and $v.StartsWith('"') -and $v.EndsWith('"')) { $v = $v.Substring(1, $v.Length - 2) -replace '\\"', '"' -replace '\\\\', '\' }
    return $v
}

function Plain-Text([System.Security.SecureString]$s) {
    $b = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($s)
    try { return [Runtime.InteropServices.Marshal]::PtrToStringAuto($b) } finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($b) }
}

if ($BuildPrefix -notmatch '^[A-Za-z0-9_.-]*$') { Fail "-BuildPrefix may only contain letters, digits, _ . -" }
foreach ($n in $Nodes) { if ($n -notmatch '^[a-z0-9_-]{1,15}$') { Fail "node id '$n' must be 1-15 characters from a-z 0-9 _ -" } }
if ($Flash -and -not $Ports) { Fail ("-Flash needs -Ports, for example -Ports @{ node1='COM12'; node2='COM11' }. Connected ports: " + ((Get-CimInstance Win32_SerialPort | ForEach-Object { $_.DeviceID }) -join ', ')) }

$built = @()
$i = 0
foreach ($node in $Nodes) {
    $i++
    Step "3/5 Building $node ($i of $($Nodes.Count))"
    $buildDir = Join-Path $root ($BuildPrefix + $node)
    $defaultsFile = Join-Path $buildDir 'node.defaults'

    # settings: parameters > environment > previous build of this node > prompt
    $ssid = $Ssid; $pass = $Password; $broker = $Broker
    if (-not $ssid)   { $ssid   = $env:AAI16_WIFI_SSID }
    if (-not $pass)   { $pass   = $env:AAI16_WIFI_PASSWORD }
    if (-not $broker) { $broker = $env:AAI16_BROKER }
    if (-not $NoWifi) {
        if (-not $ssid)   { $ssid   = Read-Previous $defaultsFile 'CONFIG_AAI16_WIFI_SSID' }
        if (-not $pass)   { $pass   = Read-Previous $defaultsFile 'CONFIG_AAI16_WIFI_PASSWORD' }
        if (-not $broker) {
            $prev = Read-Previous $defaultsFile 'CONFIG_AAI16_MQTT_URI'
            if ($prev -match '^mqtt://([^:/]+)') { $broker = $Matches[1] }
        }
        if ($ssid) { Warn "Wi-Fi SSID/password: using the given or remembered values (password hidden)" }
        if (-not $ssid)   { $ssid = Read-Host "Wi-Fi SSID for $node (2.4 GHz)" }
        if (-not $pass)   { $pass = Plain-Text (Read-Host "Wi-Fi password for $node (hidden)" -AsSecureString) }
        if (-not $broker) { $broker = Read-Host "MQTT broker IP or host name (the laptop running Mosquitto)" }
        if (-not $ssid -or -not $broker) { Fail "Wi-Fi SSID and broker are required (use -NoWifi for a serial-only node)" }
    } else {
        $ssid = ''; $pass = ''
        if (-not $broker) { $broker = '192.168.1.10' }
    }

    if ($Clean -and (Test-Path $buildDir)) {
        $full = (Resolve-Path $buildDir).Path
        if (-not $full.StartsWith($root) -or (Split-Path $full -Leaf) -notlike "$BuildPrefix*") { Fail "refusing to delete $full" }
        Remove-Item -Recurse -Force $full -ErrorAction Stop
        Warn "cleaned $full"
    }
    New-Item -ItemType Directory -Force $buildDir -ErrorAction Stop | Out-Null

    # per-node settings file (git-ignored: build_*/). Written fresh each time so the build always matches the arguments.
    $lines = @(
        "# generated by tools\build_all.ps1 - do not commit",
        "CONFIG_AAI16_NODE_ID=$(Quote-Kconfig $node)",
        "CONFIG_AAI16_WIFI_SSID=$(Quote-Kconfig $ssid)",
        "CONFIG_AAI16_WIFI_PASSWORD=$(Quote-Kconfig $pass)",
        "CONFIG_AAI16_MQTT_URI=$(Quote-Kconfig "mqtt://${broker}:1883")"
    )
    Set-Content -Path $defaultsFile -Value $lines -Encoding ASCII -ErrorAction Stop
    # sdkconfig is regenerated from the defaults on every run (an old sdkconfig would silently keep stale values)
    $sdk = Join-Path $buildDir 'sdkconfig'
    if (Test-Path $sdk) { Remove-Item -Force $sdk -ErrorAction Stop }

    # idf.py is given RELATIVE paths (we are in the repo root): the repo path contains spaces, and some idf.py launchers split
    # an absolute path with spaces into two arguments ("No such option -D"). Relative paths have no spaces.
    $rel = "$BuildPrefix$node"
    $defaultsList = 'sdkconfig.defaults'
    if ($Reference) { $defaultsList += ';sdkconfig.ref.defaults' }
    & idf.py -B $rel "-DSDKCONFIG=$rel/sdkconfig" "-DIDF_TARGET=esp32s3" "-DSDKCONFIG_DEFAULTS=$defaultsList;$rel/node.defaults" build
    Check-Exit "building $node"

    # ---- verify what really went into this build
    $cfg = Get-Content $sdk -ErrorAction Stop
    function Cfg($key) { ($cfg | Where-Object { $_ -match "^$key=" } | Select-Object -First 1) -replace "^$key=", '' }
    $checks = @(
        @{ n = 'node id';        ok = ((Cfg 'CONFIG_AAI16_NODE_ID') -eq "`"$node`"") },
        @{ n = 'MQTT broker';    ok = ((Cfg 'CONFIG_AAI16_MQTT_URI') -eq "`"mqtt://${broker}:1883`"") },
        @{ n = 'Wi-Fi SSID set'; ok = ($NoWifi -or ((Cfg 'CONFIG_AAI16_WIFI_SSID') -eq (Quote-Kconfig $ssid))) },
        @{ n = 'target esp32s3'; ok = ((Cfg 'CONFIG_IDF_TARGET') -eq '"esp32s3"') },
        @{ n = 'PSRAM (octal)';  ok = (((Cfg 'CONFIG_SPIRAM') -eq 'y') -and ((Cfg 'CONFIG_SPIRAM_MODE_OCT') -eq 'y')) },
        @{ n = 'FreeRTOS 1 kHz'; ok = ((Cfg 'CONFIG_FREERTOS_HZ') -eq '1000') },
        @{ n = $(if ($Reference) { 'ESP-NN disabled (reference build)' } else { 'ESP-NN enabled' });
           ok = $(if ($Reference) { (Cfg 'CONFIG_ESP_TFLITE_MICRO_USE_ESP_NN') -ne 'y' } else { (Cfg 'CONFIG_ESP_TFLITE_MICRO_USE_ESP_NN') -eq 'y' }) }
    )
    foreach ($c in $checks) { if ($c.ok) { Ok $c.n } else { Fail "build setting check failed for ${node}: $($c.n)" } }
    $bin = Join-Path $buildDir 'aai16_edge_node.bin'
    if (-not (Test-Path $bin)) { Fail "expected image not found: $bin" }
    Ok ("image: {0:N0} bytes, {1}" -f (Get-Item $bin).Length, $bin)
    # ---- the finished binary must agree with the configuration: ESP-NN kernels present in the normal image, absent in the reference
    $nm = Get-Command xtensa-esp32s3-elf-nm -ErrorAction SilentlyContinue
    $nmPath = $null
    if ($nm) { $nmPath = $nm.Source }
    elseif ($env:IDF_TOOLS_PATH) {
        $found = Get-ChildItem (Join-Path $env:IDF_TOOLS_PATH 'tools\xtensa-esp-elf') -Recurse -Filter 'xtensa-esp32s3-elf-nm.exe' -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($found) { $nmPath = $found.FullName }
    }
    $elf = Join-Path $buildDir 'aai16_edge_node.elf'
    if ($nmPath -and (Test-Path $elf)) {
        $nnSymbols = @(& $nmPath $elf | Select-String ' esp_nn_').Count
        if ($Reference -and $nnSymbols -ne 0) { Fail "reference image still contains $nnSymbols ESP-NN symbols" }
        if (-not $Reference -and $nnSymbols -eq 0) { Fail "normal image contains no ESP-NN symbols: acceleration is not in the binary" }
        $expect = if ($Reference) { 'expected 0: reference image' } else { 'expected above 0: accelerated image' }
        Ok "binary check: $nnSymbols ESP-NN symbols ($expect)"
    } else { Warn "ESP-NN binary check skipped (xtensa nm tool or ELF not found)" }
    $built += [pscustomobject]@{ Node = $node; Image = $bin; Dir = $buildDir; Rel = $rel; Broker = $broker; Wifi = [bool]$ssid }
}

# ------------------------------------------------------------------------------------------------ 4. flash
if ($Flash) {
    foreach ($b in $built) {
        $port = $Ports[$b.Node]
        if (-not $port) { Fail "no serial port given for $($b.Node) in -Ports" }
        Step "4/5 Flashing $($b.Node) on $port"
        & idf.py -B $b.Rel -p $port flash
        Check-Exit "flashing $($b.Node)"
        Ok "flashed $($b.Node)"
    }
} else {
    Step "4/5 Flash"
    Warn ("not requested. To flash: -Flash -Ports @{ node1='COM12'; node2='COM11' }  (connected ports: " + ((Get-CimInstance Win32_SerialPort | ForEach-Object { $_.DeviceID }) -join ', ') + ")")
}

# ------------------------------------------------------------------------------------------------ 5. summary
Step "5/5 Summary"
$built | Format-Table Node, Broker, Wifi, Image -AutoSize
Write-Host ("Finished in {0} s." -f [int]((Get-Date) - $startTime).TotalSeconds) -ForegroundColor Green
Write-Host "Next: start Mosquitto on the laptop, then  .\.venv\Scripts\python.exe tools\json_bridge_all.py --broker <ip>  and open MQTTX." -ForegroundColor Green
