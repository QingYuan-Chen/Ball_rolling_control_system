# STM32F407 单轴陀螺仪测试

本目录将 `SingleAxisGyro` 的可移植串口协议核心接入当前 STM32F407 HAL
工程。没有复制 MSPM0 DriverLib、SysConfig 或 TI 中断代码。

## 当前测试连接

| 单轴陀螺仪 | 天空星 STM32F407VET6 | 说明 |
| --- | --- | --- |
| TX | PD9 / USART3_RX | 模块发送到 MCU 接收 |
| RX | PD8 / USART3_TX | MCU 配置命令发送到模块；本测试不自动配置 |
| GND | GND | 必须共地 |
| VCC | 按模块要求供电 | 手册允许 3.3-16 V，典型 5 V |

USART3 使用 115200、8N1。USART2（PD5/PD6）恢复给 ZDT_X42S 电机，测试
应用会初始化电机接收驱动但不会自动使能或运动。DAPLink 继续使用开发板的
独立 SWD 调试/串口通道，不再由 USART3 提供调试控制台。

## 驱动结构

- `single_axis_gyro.h/.c`：不依赖 HAL 的协议核心；解析 5 字节角度/角速度帧，
  校验累加和，并提供一致快照。
- `single_axis_gyro_stm32f407.h/.c`：USART3 单字节 RX 中断、HAL 发送与错误
  恢复适配。
- `App/Src/single_axis_gyro_test_app.c`：每 100 ms 在 OLED 显示 Yaw、角速度、
  原始值、合法帧数、校验错误、接收字节和 UART 错误。

手册参数表写角速度量程为 ±400°/s，但串口公式和配套例程使用
`raw / 32768 * 2000°/s`。当前测试按后者显示；实机需通过已知角速度进一步
确认量程。

测试程序不会在上电时修改输出率、归零、校准或保存模块参数。测试状态只在
OLED显示；USART3不再接收DAPLink调试命令。

## 构建

`BALLCONTROL_SINGLE_AXIS_GYRO_TEST` 默认关闭。需要构建本测试时显式开启：

```powershell
.\tools\run_host_tests.ps1
cmake --preset debug -DBALLCONTROL_SINGLE_AXIS_GYRO_TEST=ON
cmake --build --preset debug
```

恢复完整树莓派视觉应用时重新配置：

```powershell
cmake --preset debug -DBALLCONTROL_SINGLE_AXIS_GYRO_TEST=OFF
cmake --build --preset debug
```
