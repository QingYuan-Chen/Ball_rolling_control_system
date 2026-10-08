# 车载平衡滚球运动控制系统

基于 STM32F407VET6 的单自由度滚球控制固件，使用 STM32 HAL、CMake 和
ARM GNU Toolchain。树莓派提供视觉坐标，STM32 负责接收、状态估计、摆杆控制
和外设管理。本仓库不包含树莓派视觉程序、模型或数据集。

## 当前状态

- 默认构建完整应用；可切换为独立单轴陀螺仪 OLED 测试。
- 已实现 USB CDC 视觉接收、USART 电机/传感器通信、OLED 状态显示和串级控制框架。
- 视觉协议兼容 V1/V2，正式发送端按 V2 对接。
- 标定参数默认无效，当前启动代码未请求开启闭环；需要实测标定后才能启用。
- 字符调试控制台当前未启用，不能通过发送 `t`、`x`、`+/-` 操作电机。

**安全提醒：MCU 不自动开启闭环，不等于机构上电不会运动。项目已有记录说明
电机自身配置了上电自动回零；接通动力电源前必须固定机构、确认活动范围和断电手段。**

## 快速开始

从仓库根目录执行，需准备支持 preset version 6 的 CMake（3.25 或更新版本）、
Ninja、ARM GNU Toolchain；主机测试另需 Windows 原生 GCC。

```powershell
# 编译完整应用，不连接或烧录硬件
cmake --preset debug -DBALLCONTROL_SINGLE_AXIS_GYRO_TEST=OFF
cmake --build --preset debug

# 运行主机测试，不访问开发板
.\tools\run_host_tests.ps1
```

Windows 建议使用纯英文路径。本次验证中，MinGW 无法在中文路径下输出测试
程序；英文盘符映射可解决，具体步骤见[构建与测试](docs/build-and-test.md)。

## 文档导航

| 需要了解 | 文档 |
| --- | --- |
| 全部文档与阅读顺序 | [文档索引](docs/README.md) |
| 目录职责、运行流程与控制状态 | [工程结构与运行流程](docs/architecture.md) |
| 引脚、电气注意事项和 DMA 分配 | [硬件连接](docs/hardware.md) |
| 工具链、测试、构建模式和烧录 | [构建与测试](docs/build-and-test.md) |
| 实机检查顺序、显示状态和已知限制 | [上板与联调](docs/bringup.md) |
| 树莓派通信格式、接收机制和黄金帧 | [视觉通信模块](Modules/RaspberryPiVision/README.md) |
| 单轴陀螺仪接线与独立测试 | [陀螺仪模块](Modules/SingleAxisGyro/README.md) |
| 历史迁移提示词 | [历史归档](docs/archive/README.md) |

## 仓库目录

```text
App/          应用入口、调度与状态管理
Core/         启动、时钟、外设初始化和中断
Modules/      控制算法、协议与设备模块
USB_DEVICE/   USB CDC 接口与板级适配
Drivers/      ST HAL、CMSIS 依赖
Middlewares/  ST USB Device 中间件
cmake/        ARM GCC 工具链配置
docs/         当前维护文档；archive/ 保存历史资料
tests/host/   5 组主机测试
tools/        主机测试与 DAPLink 烧录脚本
build/        本地生成产物，不纳入版本控制
```

根目录的 CMake 配置、CubeMX `.ioc`、链接脚本与 OpenOCD 配置保留原位，
避免改变现有工具入口。源码目录和文件名保持不变。

## 验证边界

2026-10-08 对源码提交 `46d3d14383643ddb2cc9c5b545a2e6bff3f2f302`
进行了本地主机测试和 ARM Debug 构建：5 组测试通过，构建通过但存在警告。
环境、资源占用和未验证事项见[构建与测试](docs/build-and-test.md)。

本次文档整理未修改程序、通信协议、控制参数或构建脚本，也未进行烧录、
传感器实测或带电机闭环验证。
