# Sets up an ESP-IDF 6.1 shell for this project WITHOUT modifying the install (session variables only).
# Usage (PowerShell, from the repo root):   . .\tools\idf_env.ps1 ; idf.py --version
# Paths are the ones the Espressif installer recorded (C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1).
$env:IDF_PATH = 'C:\esp\v6.1\esp-idf'
$env:IDF_TOOLS_PATH = 'C:\Espressif\tools'
$env:IDF_PYTHON_ENV_PATH = 'C:\Espressif\tools\python\v6.1\venv'
$env:ESP_ROM_ELF_DIR = 'C:\Espressif\tools\esp-rom-elfs\20241011/'
$env:IDF_CCACHE_ENABLE = '1'
$env:IDF_COMPONENT_LOCAL_STORAGE_URL = 'file://C:\Espressif\tools'
$env:PYTHONUTF8 = '1'
$v = Get-Content (Join-Path $env:IDF_PATH 'tools\cmake\version.cmake') | Select-String 'set\(IDF_VERSION_(MAJOR|MINOR|PATCH) (\d+)\)'
$n = $v | ForEach-Object { $_.Matches[0].Groups[2].Value }
$env:ESP_IDF_VERSION = ($n -join '.')
$bins = @(
    'C:\Espressif\tools\python\v6.1\venv\Scripts',
    'C:\Espressif\tools\xtensa-esp-elf\esp-15.2.0_20251204\xtensa-esp-elf\bin',
    'C:\Espressif\tools\cmake\4.0.3\bin',
    'C:\Espressif\tools\ninja\1.12.1',
    'C:\Espressif\tools\ccache\4.12.1\ccache-4.12.1-windows-x86_64',
    'C:\Espressif\tools\idf-exe\1.0.3'
)
$env:PATH = ($bins -join ';') + ';' + $env:PATH

function idf.py { & "$env:IDF_PYTHON_ENV_PATH\Scripts\python.exe" "$env:IDF_PATH\tools\idf.py" @args }
