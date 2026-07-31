$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$gccCommand = Get-Command gcc -ErrorAction SilentlyContinue

if ($null -eq $gccCommand) {
    $knownGcc = "C:\mingw64\bin\gcc.exe"
    if (-not (Test-Path -LiteralPath $knownGcc -PathType Leaf)) {
        throw "Host GCC was not found."
    }
    $gcc = $knownGcc
} else {
    $gcc = $gccCommand.Source
}

$testBuild = Join-Path $projectRoot "build\host-tests"
New-Item -ItemType Directory -Force -Path $testBuild | Out-Null

$visionExe = Join-Path $testBuild "test_raspberry_pi_vision_protocol.exe"
& $gcc -std=c11 -Wall -Wextra -Werror `
    -I (Join-Path $projectRoot "Modules\RaspberryPiVision") `
    (Join-Path $projectRoot "tests\host\test_raspberry_pi_vision_protocol.c") `
    (Join-Path $projectRoot "Modules\RaspberryPiVision\raspberry_pi_vision_protocol.c") `
    -o $visionExe
if ($LASTEXITCODE -ne 0) {
    throw "Vision protocol host test compilation failed."
}
& $visionExe
if ($LASTEXITCODE -ne 0) {
    throw "Vision protocol host test failed."
}

$visionAdapterExe = Join-Path $testBuild `
    "test_raspberry_pi_vision_stm32f407.exe"
& $gcc -std=c11 -Wall -Wextra -Werror `
    -I (Join-Path $projectRoot "Modules\RaspberryPiVision") `
    (Join-Path $projectRoot `
        "tests\host\test_raspberry_pi_vision_stm32f407.c") `
    (Join-Path $projectRoot `
        "Modules\RaspberryPiVision\raspberry_pi_vision_protocol.c") `
    (Join-Path $projectRoot `
        "Modules\RaspberryPiVision\raspberry_pi_vision_stm32f407.c") `
    -o $visionAdapterExe
if ($LASTEXITCODE -ne 0) {
    throw "Vision STM32F407 adapter host test compilation failed."
}
& $visionAdapterExe
if ($LASTEXITCODE -ne 0) {
    throw "Vision STM32F407 adapter host test failed."
}

$wirelessExe = Join-Path $testBuild "test_tianmengxing_wireless_serial.exe"
& $gcc -std=c11 -Wall -Wextra -Werror `
    -I (Join-Path $projectRoot "Modules\Debug") `
    (Join-Path $projectRoot "tests\host\test_tianmengxing_wireless_serial.c") `
    (Join-Path $projectRoot "Modules\Debug\tianmengxing_wireless_serial.c") `
    -o $wirelessExe
if ($LASTEXITCODE -ne 0) {
    throw "Wireless serial host test compilation failed."
}
& $wirelessExe
if ($LASTEXITCODE -ne 0) {
    throw "Wireless serial host test failed."
}

$gyroExe = Join-Path $testBuild "test_single_axis_gyro.exe"
& $gcc -std=c11 -Wall -Wextra -Werror `
    -I (Join-Path $projectRoot "Modules\SingleAxisGyro") `
    (Join-Path $projectRoot "tests\host\test_single_axis_gyro.c") `
    (Join-Path $projectRoot "Modules\SingleAxisGyro\single_axis_gyro.c") `
    -o $gyroExe
if ($LASTEXITCODE -ne 0) {
    throw "Single-axis gyro host test compilation failed."
}
& $gyroExe
if ($LASTEXITCODE -ne 0) {
    throw "Single-axis gyro host test failed."
}

$controlExe = Join-Path $testBuild "test_ball_control_core.exe"
& $gcc -std=c11 -Wall -Wextra -Werror `
    -I (Join-Path $projectRoot "Modules\Control") `
    (Join-Path $projectRoot "tests\host\test_ball_control_core.c") `
    (Join-Path $projectRoot "Modules\Control\ball_control_core.c") `
    -lm -o $controlExe
if ($LASTEXITCODE -ne 0) {
    throw "Ball control core host test compilation failed."
}
& $controlExe
if ($LASTEXITCODE -ne 0) {
    throw "Ball control core host test failed."
}
