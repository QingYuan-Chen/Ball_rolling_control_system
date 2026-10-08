# 构建与测试

[返回文档索引](README.md)

本文命令均从仓库根目录执行。主机测试和编译不需要连接开发板；烧录单独列出。

## 依赖

| 用途 | 依赖 |
| --- | --- |
| 配置与构建 | CMake 3.25 或更新版本、Ninja |
| MCU 编译 | ARM GNU Toolchain：`arm-none-eabi-gcc/g++/objcopy/size` 在 PATH 中 |
| Windows 主机测试 | Windows 原生 GCC；脚本先查 PATH，再尝试 `C:\mingw64\bin\gcc.exe` |
| 下载与调试（可选） | OpenOCD、CMSIS-DAP/DAPLink、正确连接的 SWD |

顶层 CMake 声明的最低版本为 3.22，但仓库的 preset 文件使用 version 6，
因此按现有 preset 操作需要 CMake 3.25 或更新版本。

CLion 可直接打开仓库根目录并读取 debug/release preset。OpenOCD 缺失不影响
固件构建；CMake 未找到它时不会生成 `flash` 目标。

## 完整应用构建

```powershell
cmake --preset debug -DBALLCONTROL_SINGLE_AXIS_GYRO_TEST=OFF
cmake --build --preset debug
```

Release 构建：

```powershell
cmake --preset release -DBALLCONTROL_SINGLE_AXIS_GYRO_TEST=OFF
cmake --build --preset release
```

两种配置分别生成到 `build/debug/` 和 `build/release/`：

- `BallControl_STM32F407`：ELF，无扩展名，用于调试和烧录。
- `BallControl_STM32F407.hex`、`BallControl_STM32F407.bin`：固件镜像。
- `BallControl_STM32F407.map`：链接布局。

`build/` 已被 Git 忽略。不要把这些产物当作源码提交。

## 独立陀螺仪测试模式

```powershell
cmake --preset debug -DBALLCONTROL_SINGLE_AXIS_GYRO_TEST=ON
cmake --build --preset debug
```

该模式选择另一份应用源码，不运行完整视觉接收与滚球控制逻辑。
详见[单轴陀螺仪模块](../Modules/SingleAxisGyro/README.md)。

CMake 会缓存选项；测试后恢复完整应用时必须重新执行带 `OFF` 的配置命令，
不能仅再次执行 build 就认为回到了默认模式。

## 主机测试

```powershell
.\tools\run_host_tests.ps1
```

脚本使用 `-std=c11 -Wall -Wextra -Werror` 编译测试并依次运行：

| 测试程序 | 主要范围 |
| --- | --- |
| `test_raspberry_pi_vision_protocol` | V1/V2 黄金帧、拆包/粘包、CRC/格式/语义恢复、超时与连续帧 |
| `test_raspberry_pi_vision_stm32f407` | USB 入队、环形缓冲回绕/溢出、重连会话清理 |
| `test_tianmengxing_wireless_serial` | 保留的调试串口缓冲与读写接口 |
| `test_single_axis_gyro` | 陀螺仪协议与角度归零状态机 |
| `test_ball_control_core` | 配置校验、投影、Kalman、控制输出及算法层安全条件 |

这些测试不覆盖完整应用的硬件 DMA 时序、机械启动、电机响应状态机与端到端
闭环，不能代替实机验证。

## Windows 中文路径

2026-10-08 使用 MinGW GCC 15.2.0 在包含中文的路径中运行测试时，链接器报告
`cannot open output file ... No such file or directory`；同一目录映射到
英文盘符后测试通过。失败发生在生成程序阶段，并非测试断言失败。

建议新克隆使用纯英文路径。现有中文目录可在确认 `R:` 未被占用后临时映射：

```powershell
$repoPath = (Get-Location).Path
if (Test-Path -LiteralPath 'R:\') {
    throw 'R: 已被占用，请选用其他空闲盘符并同步替换后续命令。'
}
subst R: "$repoPath"
if ($LASTEXITCODE -ne 0) { throw '创建盘符映射失败。' }
try {
    Push-Location -LiteralPath 'R:\'
    .\tools\run_host_tests.ps1
    cmake --preset debug -DBALLCONTROL_SINGLE_AXIS_GYRO_TEST=OFF
    if ($LASTEXITCODE -ne 0) { throw 'CMake 配置失败。' }
    cmake --build --preset debug
    if ($LASTEXITCODE -ne 0) { throw '固件构建失败。' }
}
finally {
    Pop-Location
    subst R: /D
}
```

`subst R: /D` 只解除映射，不删除仓库文件。CMake 缓存记录配置时的绝对路径，
以后使用同一构建目录时应恢复相同映射；切换到其他路径时使用新的构建目录。
映射解除后不要直接双击或调用依赖 `R:` 缓存的构建命令。

## DAPLink 烧录（会改变开发板固件）

先阅读[硬件连接](hardware.md)和[上板与联调](bringup.md)，初次检查不接电机
动力电源。烧录前确认镜像对应完整应用还是独立测试模式。

SWD 连接包括 SWDIO、SWCLK、GND；仓库 OpenOCD 配置要求 NRST 已连接。
确认后可执行以下一种方式：

```powershell
cmake --build --preset debug --target flash
```

或使用现有脚本（需先构建，脚本不会代为编译）：

```powershell
.\tools\flash_daplink.ps1 -Configuration debug
```

如 OpenOCD 未被找到，脚本接受 `-OpenOcd` 绝对路径。NRST 无法连接时，
先检查接线和电源；确认目标身份后才考虑手动覆盖复位策略：

```powershell
openocd -f .\openocd_daplink_stm32f407.cfg `
  -c "reset_config none" -c "adapter speed 500" `
  -c "init; halt; program build/debug/BallControl_STM32F407 verify reset exit"
```

## 2026-10-08 本地验证记录

源码基线：`46d3d14383643ddb2cc9c5b545a2e6bff3f2f302`。
环境：Windows、CMake 3.28.1、ARM GCC 13.3.1、MinGW GCC 15.2.0；
本地中文工作目录使用临时英文盘符映射。

- 5 组主机测试全部通过。
- 完整应用 ARM Debug 构建通过。
- FLASH 使用 56,468 B / 512 KiB；RAM 使用 13,528 B / 128 KiB；
  CCMRAM 未使用。
- 本次全量 Debug 构建有 31 条警告，涉及 USB 适配的名称遮蔽和类型转换、
  ST HAL/中间件、newlib 系统调用占位与 ELF RWX 段。未在文档整理中修改或
  屏蔽这些警告。
- Release、独立陀螺仪模式构建、烧录、USB 实机通信、电机运动及闭环性能
  未在本次验证中执行。

资源占用随编译器和配置变化，不应把历史数字视为所有环境的固定结果。
