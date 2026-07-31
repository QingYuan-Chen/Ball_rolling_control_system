# BallControl STM32F407 HAL migration

本目录是“车载平衡滚球运动控制系统”的 STM32F407VET6 迁移工程。
工程使用 STM32 HAL、CMake 和 ARM GNU Toolchain，可直接由 CLion 打开。
原 MSPM0 工程仅用于协议与模块行为参考，底层 UART/GPIO/DMA 均已按新硬件重写。

`BallControl_STM32F407.ioc` 保存了对应的 STM32CubeMX 配置。`.ioc` 本身不是
参与编译的源文件；它用于在 CubeMX 中查看或调整引脚、时钟、DMA 和 USB
中间件，然后生成初始化代码。实际编译入口仍是本目录的 `CMakeLists.txt`。

## 当前构建模式：完整系统与串级控制框架

`BALLCONTROL_SINGLE_AXIS_GYRO_TEST` 当前默认关闭，构建原完整 BallControl
应用。OLED显示板级启动、树莓派 USB/协议/WiFi/视觉状态、视觉坐标、JY61P和
单轴陀螺仪数据。USART2（PD5/PD6）用于电机，USART3不再作为字符命令控制台，避免
连接的单轴陀螺仪数据被误解释为电机命令。完整应用在后台运行串级控制框架；
单轴陀螺仪专用OLED测试代码仍保留在
`Modules/SingleAxisGyro`，需要时可单独开启测试选项。

## 已实现

- 168 MHz 系统时钟：天空星板载 8 MHz HSE，PLLQ 输出 USB 所需 48 MHz。
- DAPLink：使用开发板 SWD 调试/串口通道；当前不占用 USART3。
- 一台 ZDT_X42S：USART2（PD5/PD6、CN3），115200 8N1，地址 1。
- 单轴陀螺仪：USART3（PD8/PD9、CN5），115200 8N1，循环 DMA + IDLE。
- JY61P：USART6（PC6/PC7、原网络名 TX5/RX5），9600 8N1，循环 DMA + IDLE。
- OLED：I2C1（PB6/PB7），100 kHz，默认 7 位地址 0x3C，后台 DMA 刷新。
- 树莓派：天空星板载 Type-C 的 USB OTG FS（PA11/PA12），USB CDC 虚拟串口。
- 按键：天空星板载用户按键（PA0，高电平按下）为电机广播急停；KEY3/KEY4
  因硬件问题暂不参与应用逻辑。
- 树莓派目标帧、CRC、超时/失联状态与 JY61P 帧解析已独立封装。
- USB CDC中断只把数据放入1024字节静态环形缓冲，协议解析在主循环执行。
- 视觉协议同时接收冻结的V1和带采集时间戳/IP的V2；OLED显示原图像素坐标及
  树莓派IPv4，AX/AY在同一行。

## DMA与主循环调度

为后续100 Hz控制、状态估计和滤波计算预留CPU，底层数据通路采用以下分工：

| 通路 | DMA映射 | 执行方式 |
| --- | --- | --- |
| 电机 USART2 RX | DMA1 Stream5 Channel4 | 64字节循环DMA + IDLE，主循环解析应答 |
| 电机 USART2 TX | DMA1 Stream6 Channel4 | 普通命令DMA发送；广播急停可抢占普通发送 |
| 单轴陀螺仪 USART3 RX | DMA1 Stream1 Channel4 | 128字节循环DMA + IDLE，主循环解析 |
| JY61P USART6 RX | DMA2 Stream1 Channel5 | 128字节循环DMA + IDLE，主循环解析 |
| OLED I2C1 TX | DMA1 Stream7 Channel1 | 逐页异步状态机，绘图缓冲与发送快照分离 |
| 树莓派 USB CDC | USB FS中断 | 中断只入静态队列，主循环解析协议 |

UART的DMA/IDLE回调只发布接收计数，不解析协议、不刷新OLED、不执行控制计算。
主循环依次消费USART2、USART3、USART6和USB队列，空闲时执行`__WFI()`。电机
驱动提供非阻塞应答查询API；原`EmmV5_WaitResponse()`仅保留给人工调试命令，
正式控制循环不得调用。陀螺仪角度归零使用非阻塞时间状态机；两条5字节
配置命令各自只占用一次短UART发送，不位于周期控制路径。OLED上电初始化仍是
低频阻塞操作。

## 串级控制框架

- TIM6以84 MHz计数时钟、83分频、999自动重装产生1 kHz基础节拍；中断仅递增
  生产计数，全部浮点计算和通信状态机都在主循环执行。
- 常速度Kalman状态在200 Hz预测，摄像头新帧到达时异步校正；测量年龄按
  `本地排队时间 + latency_ms - prediction_ms`补偿。树莓派当前关闭预测时，
  `prediction_ms`应为0。
- 控制器在100 Hz执行。位置外环使用小球位置、Kalman速度及可选积分项生成
  摆杆角度目标；摆杆角度内环始终使用USART3陀螺仪角度误差和角速度生成
  电机轴角度命令。PD和LQI模式都会经过角度内环。
- JY61P加速度前馈叠加在位置外环的摆杆角度目标上。摄像头坐标先通过待标定的
  一维视轴投影换算为毫米，控制环不直接使用原始像素。
- 上电后MCU先轮询电机`0x3B`回零状态，并以`0x36`实时位置连续三次接近零位
  作为补充确认；确认电机静止在零位后，USART3发送`55 AA 13 8E 5F`解锁，
  间隔100 ms发送`55 AA 15 00 00`执行陀螺仪角度归零。归零后的连续三帧角度
  接近0°才开放控制安全门；不执行零偏校准，也不发送保存命令。
- USART2电机通路使用非阻塞状态机依次完成使能、绝对位置参数配置和目标下发；
  每一步都检查应答、拒绝和超时，不在控制路径调用阻塞等待。
- 视觉无效/过期、树莓派启动状态丢失、陀螺仪或可选IMU前馈过期、PA0急停、
  电机应答异常以及1 kHz任务积压超过20 ms，都会撤销软件使能并发送广播急停。

控制配置默认由`BallControlCore_LoadSafeDefaults()`初始化为“全部未标定”。必须
实测并填写视觉原点/方向/毫米每像素、电机零位/方向/每摆杆角度单位、陀螺仪
方向、Kalman噪声、位置外环和角度内环增益。陀螺仪已经在每次启动时执行硬件
角度归零，因此`gyro.angle_zero_deg`通常设置为0，仅用于补偿实测残差。然后从
主循环依次调用：

```c
BallControl_Config config;

BallControlCore_LoadSafeDefaults(&config);
/* 仅在完成对应实机标定后填写各字段并将各 valid 置为 true。 */
if (BallControl_ApplyControlConfig(&config)) {
    BallControl_SetControlSetpoint(0.0f);
    /* 视觉、树莓派、陀螺仪和可选IMU全部就绪后，才会接受true。 */
    (void) BallControl_RequestControlEnable(true);
}
```

当前工程没有写入任何猜测标定值，也没有调用
`BallControl_RequestControlEnable(true)`；因此MCU不会自动使能电机或下发控制
运动命令。但电机已经由自身参数配置为上电自动回零，接通动力电源后仍可能立即
运动，测试时必须固定机构并留出安全范围。下一阶段应先完成机构角度标定和低速、
小角度验证，再写入初始保守参数。

> STM32F407 没有 `USART0`。这里所说的“板载 Type-C 串口”实际是 USB CDC
> 虚拟串口；树莓派应从 USB Host 口连接天空星 Type-C，Linux 通常枚举为
> `/dev/ttyACM*`。CDC 的波特率设置只是 line coding，实际传输由 USB Full
> Speed 总线完成。小球 X/Y 仍只来自树莓派摄像头帧，JY61P 不参与小球定位。

## CLion 构建

1. 在 CLion 中打开本目录。
2. 让 CLion 读取 `CMakePresets.json`，选择 `debug` 或 `release` preset。
3. 构建目标 `BallControl_STM32F407`。

如需检查 CubeMX 配置，可单独用 STM32CubeMX 打开
`BallControl_STM32F407.ioc`。不要在没有备份的情况下覆盖现有手写驱动；
工程已将 CubeMX 的“保留用户代码”和“备份原文件”选项写入配置。

命令行等价操作：

```powershell
cmake --preset debug
cmake --build --preset debug
```

产物位于 `build/debug/`：

- `BallControl_STM32F407`：ELF，可用于调试和 OpenOCD 烧录；
- `BallControl_STM32F407.hex`；
- `BallControl_STM32F407.bin`；
- `BallControl_STM32F407.map`。

## DAPLink 烧录和调试

SWD 连接：`SWDIO`、`SWCLK`、`GND`，建议同时连接 `NRST`。按当前硬件连接，
DAPLink 的调试/串口功能直接使用开发板 SWD 调试口，不再连接 CN5 的 PD8/PD9；
这两个引脚已完整分配给单轴陀螺仪 USART3。

工程保留了用户提供的 `openocd_daplink_stm32f407.cfg`。连接目标板后可运行：

```powershell
cmake --build --preset debug --target flash
```

或：

```powershell
.\tools\flash_daplink.ps1 -Configuration debug
```

配置默认要求DAPLink的`NRST`已连接。若SWDIO/SWCLK可读但出现
`timed out while waiting for target halted`，先补接NRST；无法接NRST时可用
软件停机方式执行同样的写入和verify：

```powershell
openocd -f .\openocd_daplink_stm32f407.cfg `
  -c "reset_config none" -c "adapter speed 500" `
  -c "init; halt; program build/debug/BallControl_STM32F407 verify reset exit"
```

当前陀螺仪测试固件不通过 USART3 输出调试文字，运行状态直接显示在 OLED。

## 首次硬件测试顺序

1. 不连接电机动力电源，先烧录并验证 OLED、LED 和 USART3 陀螺仪数据。
2. OLED 应显示 `DATA OK`，角度和角速度随模块转动变化，`SUM ERR` 与 `U3E`
   应保持为 0。
3. 连接树莓派 USB，确认枚举为 CDC，并只发送视觉测试帧验证 CRC/超时。
   OLED依次显示USB、协议、视觉服务、WiFi和视觉状态。树莓派在服务、WiFi、
   相机和推理均正常后先发送`STARTUP_READY` Heartbeat，再开始目标帧；
   MCU收到确认和目标流后自动显示`SYSTEM READY`。
4. 连接唯一一台摆杆执行电机，固定机构或解除机械负载，先用 `t` 做只读查询。
5. 确认急停 `x` 和天空星板载用户按键有效后，才用 `+`/`-` 做
   30 rpm、256 脉冲小步测试。

MCU固件不会自动使能或下发控制运动命令；电机自身的上电自动回零配置仍会导致
上电运动。控制安全门必须同时满足电机回零确认、陀螺仪角度归零、视觉与标定就绪。

## 主机协议测试

```powershell
.\tools\run_host_tests.ps1
```

测试覆盖V1/V2黄金帧、V2全部拆包位置、混合版本粘包、噪声、
CRC/格式/语义恢复、V1扩展字段清零、连续600帧、USB接收环形缓冲和无线调试
串口环形缓冲。视觉模块说明见
`Modules/RaspberryPiVision/README.md`；交给树莓派端开发者的迁移提示词见
`docs/RASPBERRY_PI_STARTUP_READY_PROMPT.md`。完整引脚和电气检查见
`docs/HARDWARE_MIGRATION.md`。
