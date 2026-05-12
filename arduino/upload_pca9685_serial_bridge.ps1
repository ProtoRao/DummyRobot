param(
    [string]$Port = "COM7"
)

$ErrorActionPreference = "Stop"

$cli = "C:\Program Files\Arduino CLI\arduino-cli.exe"
$sketchPath = Join-Path $PSScriptRoot "pca9685_serial_bridge"
$repoRoot = Split-Path -Parent $PSScriptRoot
$buildPath = Join-Path $repoRoot ".arduino-build\pca9685_serial_bridge"
$configDir = Join-Path $repoRoot ".arduino-cli-data"
$fqbn = "arduino:avr:uno"

& $cli `
    --config-dir $configDir `
    compile --fqbn $fqbn --build-path $buildPath --upload --port $Port $sketchPath
