<#
.SYNOPSIS
  Runs one FlexiCargo demo step with the project's Python environment.

.EXAMPLE
  .\demo\run.ps1 status
  .\demo\run.ps1 watch
  .\demo\run.ps1 modeA -DryRun        (shows what would be sent, touches nothing)

  Steps: status check-wiring reset prepare setup models modeA bridge motion env swap-bme280 modeB badmap upload timing watch
#>
param(
    [Parameter(Mandatory = $true, Position = 0)][string]$Step,
    [switch]$DryRun,
    [double]$Seconds = 0
)
$root = Split-Path -Parent $PSScriptRoot
$py = Join-Path $root '.venv\Scripts\python.exe'
if (-not (Test-Path $py)) { Write-Error "Python venv not found at $py (run from the project folder after setting up .venv)"; exit 1 }
$argsList = @((Join-Path $root 'demo\demo.py'), $Step)
if ($DryRun) { $argsList += '--dry-run' }
if ($Seconds -gt 0) { $argsList += @('--seconds', $Seconds) }
& $py @argsList
exit $LASTEXITCODE
