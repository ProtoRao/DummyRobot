param(
    [string]$Port = "COM7"
)

$ErrorActionPreference = "Stop"

$cli = "C:\Program Files\Arduino CLI\arduino-cli.exe"
$repoRoot = Split-Path -Parent $PSScriptRoot
$configFile = Join-Path $repoRoot ".arduino-cli.yaml"
$sketchPath = Join-Path $PSScriptRoot "pca9685_servo_test"
$buildPath = Join-Path $repoRoot ".arduino-build\pca9685_servo_test"
$fqbn = "arduino:avr:uno"

& $cli --config-file $configFile compile --fqbn $fqbn --build-path $buildPath --upload --port $Port $sketchPath
