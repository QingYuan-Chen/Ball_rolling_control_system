# 工程结构与运行流程

[返回文档索引](README.md)

## 仓库边界与目录职责

本仓库是 STM32F407VET6 固件。树莓派的摄像头采集、Hailo 推理、模型和
发送服务位于外部工程；此处只维护 MCU 接收与控制部分。

| 路径 | 职责与主要入口 |
| --- | --- |
| [Core/Src/main.c](../Core/Src/main.c) | HAL、168 MHz 时钟、GPIO/DMA/UART/I2C/TIM6 初始化和主循环 |
| [App/Src/ball_control_app.c](../App/Src/ball_control_app.c) | 完整应用、启动状态、控制调度、电机状态机和 OLED |
| [App/Src/single_axis_gyro_test_app.c](../App/Src/single_axis_gyro_test_app.c) | 独立陀螺仪 OLED 测试；由 CMake 选项切换 |
| [Modules/Control/](../Modules/Control/) | 不依赖 HAL 的标定校验、视觉投影、Kalman 和控制计算 |
| [Modules/RaspberryPiVision/](../Modules/RaspberryPiVision/) | 不依赖 HAL 的 V1/V2 解析核心及 USB 接收队列适配 |
| [Modules/Motor/](../Modules/Motor/) | USART2 电机命令、DMA 收发、响应解析与广播急停 |
| [Modules/SingleAxisGyro/](../Modules/SingleAxisGyro/) | 单轴角度/角速度解析、USART3 适配、角度归零 |
| [Modules/JY61P/](../Modules/JY61P/) | USART6 IMU 接收，可选加速度前馈数据来源 |
| [Modules/OLED/](../Modules/OLED/) | SSD1306 绘图、字体和异步刷新 |
| [Modules/Debug/](../Modules/Debug/) | 保留的调试控制台和串口缓冲模块；当前完整应用未启用控制台 |
| [USB_DEVICE/](../USB_DEVICE/) | CDC 设备描述符、USB 回调与 HAL 适配 |
| [Drivers/](../Drivers/)、[Middlewares/](../Middlewares/) | ST 依赖库，分别保留各自许可证 |
| [tests/host/](../tests/host/)、[tools/](../tools/) | 主机测试和辅助脚本 |

本次仅整理文档及其目录，不拆分应用源文件，也不调整上述依赖关系。

## 启动与主循环

`main()` 初始化板级外设后调用 `BallControl_Init()`，再初始化 USB、启动
TIM6 中断，循环调用 `BallControl_Process()`。

完整应用的每轮处理顺序：

1. 消费电机、单轴陀螺仪、JY61P 和 USB 接收数据。
2. 推进机械启动状态机，更新树莓派启动状态。
3. 检查 PA0 急停按键。
4. 接收新的视觉测量，处理已产生的定时节拍。
5. 推进电机使能、参数设置与位置命令状态机。
6. 若调试控制台已初始化则读取命令；当前初始化明确关闭了该入口。
7. 按周期更新 OLED、LED，执行 `__WFI()` 等待中断。

## 中断与周期任务

| 任务 | 当前实现 |
| --- | --- |
| TIM6 | 1 kHz 中断只增加节拍计数 |
| 状态预测 | 每 5 个节拍执行一次，名义频率 200 Hz |
| 控制计算 | 每 10 个节拍执行一次，名义频率 100 Hz |
| 视觉校正 | 主循环收到新的目标包时处理 |
| 完整应用 OLED | 每 200 ms 更新绘图，逐页 DMA 刷新 |
| 独立陀螺仪测试 OLED | 每 100 ms 更新 |
| LED | 每 500 ms 翻转 |

UART DMA/IDLE 回调发布接收进度，USB OUT 回调只入队；协议解析与控制计算
在主循环执行。OLED 上电初始化仍有阻塞操作；陀螺仪启动归零的配置发送是短
UART 发送，等待间隔由状态机管理。不能将整个程序概括为“完全无阻塞”。

## 控制数据流

```text
USB 视觉目标 → 原图 Q12.4 坐标 → 标定轴投影（mm）
                                     ↓
                              Kalman 位置/速度
                                     ↓
                  位置外环 → 摆杆角度目标 ← 可选 JY61P 加速度前馈
                                     ↓
单轴陀螺仪角度/角速度 ──────────→ 角度内环
                                     ↓
                    限幅与标定换算 → 电机目标 → USART2
```

位置外环支持 PD 和带积分的 LQI 模式；两者都经过角度内环。增益通过配置传入，
固件不会自动求解或整定 LQI 增益。小球坐标只来自视觉，不由 JY61P 积分产生。

测量年龄计算使用 `本地处理时间差 + latency_ms - prediction_ms`（下限为零）。
V2 Unix 时间戳可被解析和读取，但当前控制计算未用它做两端时钟同步。

## 三类状态不能混为一谈

- **视觉启动就绪**：USB、协议、Heartbeat 健康位与目标流通过检查。
- **机械启动就绪**：电机回零状态及零位附近连续三次位置读数通过检查，
  再完成陀螺仪角度归零。软件判据不等于独立测量确认机构完全静止。
- **闭环启用**：还必须具有完整标定、有效估计和传感器输入，并显式请求使能。

完整应用的 `SYSTEM READY` 页面需要前两类就绪，但不表示第三类已启用。
上电配置由 `BallControlCore_LoadSafeDefaults()` 清零，标定标志无效；
启动代码没有调用 `BallControl_RequestControlEnable(true)`。

视觉/传感器条件失效、电机应答异常、急停及已请求使能时的节拍积压超过
20 ms 等条件会撤销控制并触发广播急停。具体上板行为、超时和限制见
[上板与联调](bringup.md)。

## 工程文件为何保持原位

- `CMakeLists.txt` 是实际构建入口，显式列出源码和依赖。
- `CMakePresets.json` 选择构建目录及 `cmake/arm-none-eabi-gcc.cmake`。
- `BallControl_STM32F407.ioc` 用于查看/调整 CubeMX 配置，本身不参与编译。
- 链接脚本、OpenOCD 配置和 `tools/` 中的脚本均有现有相对路径依赖。

因此不为目录美观而搬动这些文件。CubeMX 重新生成代码仍需备份并检查差异，
不能假定所有手写初始化与驱动都能被无损保留。
