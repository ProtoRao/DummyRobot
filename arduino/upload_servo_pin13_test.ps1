param(
    [string]$Port = "COM7"
)

$ErrorActionPreference = "Stop"

$cli = "C:\Program Files\Arduino CLI\arduino-cli.exe"
$sketchPath = Join-Path $PSScriptRoot "servo_pin13_test"
$repoRoot = Split-Path -Parent $PSScriptRoot
$buildPath = Join-Path $repoRoot ".arduino-build\servo_pin13_test"
$configDir = Join-Path $repoRoot ".arduino-cli-data"
$fqbn = "arduino:avr:uno"

& $cli `
    --config-dir $configDir `
    compile --fqbn $fqbn --build-path $buildPath --upload --port $Port $sketchPath
