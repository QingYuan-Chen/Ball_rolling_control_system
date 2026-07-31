param(
    [ValidateSet("debug", "release")]
    [string] $Configuration = "debug",
    [string] $OpenOcd = ""
)

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$firmware = Join-Path $projectRoot (
    "build\{0}\BallControl_STM32F407" -f $Configuration)
$config = Join-Path $projectRoot "openocd_daplink_stm32f407.cfg"

if (-not (Test-Path -LiteralPath $firmware -PathType Leaf)) {
    throw "Firmware not found: $firmware. Build preset '$Configuration' first."
}
if (-not (Test-Path -LiteralPath $config -PathType Leaf)) {
    throw "OpenOCD config not found: $config"
}

if ([string]::IsNullOrWhiteSpace($OpenOcd)) {
    $command = Get-Command openocd -ErrorAction SilentlyContinue
    if ($null -ne $command) {
        $OpenOcd = $command.Source
    } else {
        $knownPath =
            "C:\Program Files\xpack-openocd-0.12.0-7\bin\openocd.exe"
        if (Test-Path -LiteralPath $knownPath -PathType Leaf) {
            $OpenOcd = $knownPath
        }
    }
}
if ([string]::IsNullOrWhiteSpace($OpenOcd) -or
    -not (Test-Path -LiteralPath $OpenOcd -PathType Leaf)) {
    throw "OpenOCD executable not found. Pass -OpenOcd with its absolute path."
}

& $OpenOcd -f $config `
    -c "program $firmware verify reset exit"
if ($LASTEXITCODE -ne 0) {
    throw "DAPLink flash failed with OpenOCD exit code $LASTEXITCODE."
}
