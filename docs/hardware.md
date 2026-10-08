# STM32F407VET6 硬件连接

[返回文档索引](README.md)

引脚和 DMA 分配已按当前 [main.c](../Core/Src/main.c)、
[HAL MSP](../Core/Src/stm32f4xx_hal_msp.c) 及设备驱动核对。
底板接口名、电压范围和电气注意事项继承原硬件迁移记录；本仓库未包含其完整
原理图与设备手册，本次也未重新测量，接线前应按手头硬件资料复核。

网络名 `TX5/RX5` 只是底板标注；PC6/PC7 在当前程序中使用 USART6。

## 题目执行机构核对

题目中的滚球机构是一根沿小车长度方向摆动的 25 cm 带凹槽摆杆，左端铰接，
右端连接一个摆杆角度控制机构。这是单自由度机构，对应唯一一台
ZDT_X42S 闭环步进电机。题目不存在第二根摆杆或第二台摆杆执行电机。

## 引脚与外设表

| 功能 | 底板接口 | STM32 引脚 | 实例/方式 | 参数 | 中断与缓冲 |
|---|---|---|---|---|---|
| 电机：MCU TX → 电机 RX | CN3 TX2 | PD5 | USART2_TX AF7 | 115200, 8N1, 3.3 V TTL | DMA1 Stream6 Channel4 |
| 电机：MCU RX ← 电机 TX | CN3 RX2 | PD6 | USART2_RX AF7 | 同上 | DMA1 Stream5 Channel4，64 字节循环 DMA + IDLE |
| 陀螺仪：MCU TX → 模块 RX | CN5 TX3 | PD8 | USART3_TX AF7 | 115200, 8N1, TTL | 短配置命令；完整应用启动归零，独立测试不自动配置 |
| 陀螺仪：MCU RX ← 模块 TX | CN5 RX3 | PD9 | USART3_RX AF7 | 同上 | DMA1 Stream1 Channel4，128 字节循环 DMA + IDLE |
| JY61P：MCU TX → 模块 RX | H1/CN4 TX5 | PC6 | USART6_TX AF8 | 9600, 8N1, 3.3 V TTL | 配置命令发送 |
| JY61P：MCU RX ← 模块 TX | H1/CN4 RX5 | PC7 | USART6_RX AF8 | 同上 | DMA2 Stream1 Channel5，128 字节循环 DMA + IDLE |
| OLED SCL | OLED SCL | PB6 | I2C1_SCL AF4 | 100 kHz, 3.3 V | TX 使用 DMA1 Stream7 Channel1 |
| OLED SDA | OLED SDA | PB7 | I2C1_SDA AF4 | 默认 7 位地址 0x3C | 逐页异步刷新，上电初始化仍有阻塞操作 |
| 树莓派 USB D- | 核心板 Type-C | PA11 | USB_OTG_FS_DM AF10 | USB CDC Full Speed | OUT 回调入 1024 字节环形缓冲，主循环解析 |
| 树莓派 USB D+ | 核心板 Type-C | PA12 | USB_OTG_FS_DP AF10 | 同上 | 同上 |
| SWDIO | 核心板调试口 | PA13 | SWD | 3.3 V | DAPLink/OpenOCD |
| SWCLK | 核心板调试口 | PA14 | SWD | 3.3 V | DAPLink/OpenOCD |
| 天空星板载 KEY | 核心板按键 | PA0 | GPIO 输入（原记录为板载10 kΩ下拉） | 按下接3.3 V | 主循环消抖后广播急停；控制安全条件也检查原始电平 |
| KEY3 | 底板 SW4 | PE3 | GPIO 输入上拉 | 按下接地 | 硬件问题待处理，应用暂不读取 |
| KEY4 | 底板 SW5 | PE4 | GPIO 输入上拉 | 按下接地 | 硬件问题待处理，应用暂不读取 |
| 核心板 LED | 板载 | PB2 | GPIO 推挽输出 | 500 ms 翻转 | 状态心跳 |

底板另引出 SPI1（PA5/PB4/PB5、PD7 CS）和 SPI2
（PB13/PB14/PB15、PB12 CS），当前控制需求未使用，固件不初始化。

## 总线与电气结论

- 唯一一台摆杆执行电机使用 USART2 TTL 通信，地址为 1。软件不提供地址 2
  选择或双电机同步控制入口。
- ZDT 电机通信信号支持 3.3 V TTL，但电机动力电源必须使用独立的
  10~29 V、足够电流电源。控制板、电机和树莓派必须共地。
- DAPLink 使用 SWD；USART3 已由陀螺仪占用。当前应用没有启用字符调试
  控制台，不能因 DAPLink 具备串口功能就认为可以发送调试命令。
- OLED 的 I2C 总线需要上拉到 3.3 V；若 OLED 模块没有板载上拉，应外加
  约 4.7 kΩ 上拉。MCU 内部弱上拉不能代替总线电阻。
- 天空星 Type-C 直接连接 PA11/PA12，是 USB Device 接口，不是硬件 USART。
  树莓派必须作为 USB Host。
- 同时连接 DAPLink、核心板 Type-C 和底板电源前，应确认 5 V 电源路径，
  避免多个电源输出互相回灌。仅共地不等于允许并联 5 V 输出。

## 已发现的底板问题

CN6 标注为 `TX4/RX4`，连接 PC10/PC13。PC10 可以复用为 UART4_TX，
但 PC13 没有 UART4_RX 复用功能，因此 CN6 不能作为完整 UART4 使用。
当前固件明确不初始化 CN6，不能通过软件修复该引脚错误。

## 当前收发与恢复实现

- USART2 电机：一条命令对应一条短响应；发送前登记地址、命令和响应长度，
  DMA 接收数据在主循环解析。正常控制使用非阻塞应答状态机；机械启动单次
  应答超时为 200 ms，控制应答超时来自标定配置。150 ms 阻塞等待仅保留在
  当前未启用的人工调试路径中。
- USART3 单轴陀螺仪：循环 DMA + IDLE，主循环解析 5 字节协议帧。
  UART 错误后清理接收状态并重新启动 DMA，OLED 读取快照。
- USART6 JY61P：循环 DMA + UART IDLE/传输完成事件；解析器按 0x55 帧头、
  类型和校验恢复同步。UART 错误时停止 DMA、清 ORE 并重新启动。
- USB CDC：OUT endpoint回调只把完整数据块复制到1024字节静态环形缓冲，
  然后马上重新挂接64字节接收缓冲；主循环是唯一协议输入消费者并调用
  `RPV_FeedBytes()`。协议核心负责帧头搜索、长度、CRC16/CCITT-FALSE、
  语义检查、序号与超时。缓冲溢出时丢弃整个USB数据块并计数。

各 UART DMA/IDLE 回调只发布接收进度；电机控制、视觉解析和显示不在这些
回调中执行。完整调度说明见[工程结构与运行流程](architecture.md)。

## 时钟、启动与调试

- HSE：8 MHz；SYSCLK/HCLK：168 MHz；APB1：42 MHz；APB2：84 MHz。
- PLLQ=7，USB 内核时钟为 48 MHz。
- 从内部 Flash `0x08000000` 启动；正常运行保持 BOOT0 为低。
- SWD 使用 PA13/PA14，未被普通 GPIO 初始化；OpenOCD 使用 CMSIS-DAP、
  SWD 和 `stm32f4x.cfg`，初始适配器速度 1 MHz。
- LSE/RTC 不属于当前需求，固件不依赖 LSE 是否焊接。
