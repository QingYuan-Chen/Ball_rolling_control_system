# STM32F407 Raspberry Pi视觉接收

本模块接收树莓派5通过天空星板载Type-C发送的视觉坐标。物理传输为
STM32F407 USB OTG FS Device/CDC，不是STM32硬件UART；CDC在Linux上表现为
`/dev/ttyACM*`。接收器同时支持冻结的V1和正式部署使用的V2扩展。

## 文件与分层

- `raspberry_pi_vision_protocol.h/.c`：从
  `RaspberryPiVisionProtocol`模块复用的可移植协议核心，不依赖HAL；本工程
  仅将超时边界对齐为达到100/300 ms即分别进入STALE/LINK_LOST。
- `raspberry_pi_vision_stm32f407.h/.c`：STM32F407 USB CDC薄适配层。
- `USB_DEVICE/App/usbd_cdc_if.c`：USB配置/断开和OUT数据回调。
- `App/Src/ball_control_app.c`：主循环消费、状态显示及KEY3/KEY4启动交互。

协议核心负责帧头搜索、长度、版本、CRC、语义检查、一致快照和超时状态。
适配层不依赖OLED、电机、JY61P或控制算法。

## 协议版本

通用帧头版本保持`0x01`，V2使用新的消息ID，不改写V1线格式：

| 消息 | MSG_ID | Payload | 总长 |
|---|---:|---:|---:|
| V1 Heartbeat | `0x01` | 14字节 | 24字节 |
| V2 Heartbeat | `0x02` | 26字节 | 36字节 |
| V1 Primary Target | `0x11` | 32字节 | 42字节 |
| V2 Primary Target | `0x12` | 40字节 | 50字节 |

V2 Target在`frame_id`后增加`capture_unix_ms (uint64_le)`；V2 Heartbeat在
V1字段后增加4字节IPv4和`system_unix_ms (uint64_le)`。V1输入会把这些扩展
字段和有效标志清零，应用层可同时兼容两类发送端。

约定：时间戳为0表示未知，IPv4 `0.0.0.0`表示当前没有有效地址。目标和心跳
共享同一个`uint16` sequence。双方测试使用以下V2黄金向量：

```text
# Target V2，50字节，CRC=0x2DA5
AA 55 01 12 2A 00 28 00 D2 04 00 00 13 09 42 5B
98 01 00 00 07 00 1F 00 00 00 12 24 EC 27 78 16
08 06 84 07 83 FF FF FF 50 00 00 00 12 00 1E 00
A5 2D

# Heartbeat V2，36字节，IP=192.168.5.148，CRC=0x0F32
AA 55 01 02 2B 00 1A 00 E8 03 00 00 77 00 00 3C
00 3C 02 00 00 00 C0 A8 05 94 60 0A 42 5B 98 01
00 00 32 0F
```

## CubeMX配置

`BallControl_STM32F407.ioc`中的视觉接口应保持：

- `USB_OTG_FS`：Device Only；
- `USB_DEVICE`：Communication Device Class (CDC)；
- PA11：`USB_OTG_FS_DM`；
- PA12：`USB_OTG_FS_DP`；
- OTG FS内核时钟：48 MHz，当前由8 MHz HSE和`PLLQ=7`提供；
- `OTG_FS_IRQn`：已启用。

USB OTG FS不需要UART实例、UART波特率、UART IDLE或UART RX DMA。CDC主机请求
的`460800` line coding会被保存并返回，但不会改变12 Mbit/s USB Full Speed
物理速率。

## 接收上下文

`CDC_Receive_FS()`运行在USB中断调用链中，只调用：

```c
BallControl_UsbCdcReceive(buffer, length);
```

该入口只把完整USB OUT数据块复制到1024字节静态单生产者/单消费者环形缓冲，
不执行CRC、printf、OLED刷新、PID或电机控制。空间不足时丢弃整个USB数据块并
增加溢出和丢字节计数，不向解析器提交半块数据。

主循环调用：

```c
(void) RaspberryPiVision_STM32F407_ReceiverProcess(
    &receiver, HAL_GetTick());
```

只有这个主循环消费端调用`RPV_FeedBytes()`，同一个Parser不存在第二个输入
生产者。模块不使用动态内存。

## 应用API示例

```c
static RaspberryPiVision_STM32F407_Receiver_t receiver;

void App_Init(void)
{
    RaspberryPiVision_STM32F407_ReceiverInit(&receiver);
}

/* USB CDC配置完成/断开回调。 */
void App_SetUsbConnected(bool connected)
{
    RaspberryPiVision_STM32F407_ReceiverSetUsbConnected(
        &receiver, connected);
}

/* USB OUT回调：只入队。 */
void App_UsbReceive(const uint8_t *data, uint32_t length)
{
    (void) RaspberryPiVision_STM32F407_ReceiverEnqueueUsbData(
        &receiver, data, length);
}

/* 主循环：解析并读取一致快照。 */
void App_Process(void)
{
    RPV_TargetSnapshot_t target;
    RPV_TargetState_t state;
    uint32_t now_ms = HAL_GetTick();

    (void) RaspberryPiVision_STM32F407_ReceiverProcess(
        &receiver, now_ms);
    state =
        RaspberryPiVision_STM32F407_ReceiverGetTargetState(
            &receiver, now_ms);

    if ((state == RPV_TARGET_STATE_VALID) &&
        RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
            &receiver, &target)) {
        uint16_t x =
            RPV_Q4ToRoundedPixelClamped(
                target.target_x_q4, RPV_IMAGE_WIDTH_PIXELS);
        uint16_t y =
            RPV_Q4ToRoundedPixelClamped(
                target.target_y_q4, RPV_IMAGE_HEIGHT_PIXELS);
        /* 应用层使用x/y；不要在USB回调中执行显示或控制。 */
        (void) x;
        (void) y;
    }
}
```

应用还可以读取：

- `ReceiverGetLatestHeartbeat()`：视觉服务、相机、推理、FPS、错误码、树莓派
  IPv4和系统Unix毫秒；
- `ReceiverGetStats()`：协议计数、USB包数、队列长度、溢出与丢字节；
- `ReceiverGetTargetState()`：默认100 ms目标超时和300 ms链路超时。

V2目标快照还提供`capture_unix_ms`和`capture_timestamp_valid`。V2心跳快照
提供`ipv4_address[4]`、`system_unix_ms`和`extended_fields_valid`。

只有`RPV_TARGET_STATE_VALID`且`RPV_TARGET_VALID`置位时才能使用坐标。
`NOT_FOUND`、`VISION_FAULT`、`STALE`和`LINK_LOST`都不得继续使用旧坐标。

## OLED和树莓派启动确认

启动期间OLED按顺序显示：

1. `BOARD`：MCU本地驱动是否初始化；
2. `USB`：树莓派是否完成CDC枚举；
3. `PROTO`：是否收到CRC和格式合法的协议帧；
4. `PI`：Heartbeat的`SERVICE_READY`；
5. `WIFI`：Heartbeat的`WIFI_CONNECTED`；
6. `VISION`：`CAMERA_RUNNING`和`INFERENCE_RUNNING`；
7. `WAIT PI CONFIRM`或当前故障原因。

Heartbeat的`system_flags`启用两个原保留位：

- bit5，`RPV_SYSTEM_WIFI_CONNECTED`；
- bit6，`RPV_SYSTEM_STARTUP_READY`。

树莓派只有在服务、WiFi、相机和推理都健康时才设置`STARTUP_READY`，并且必须
先成功写出该Heartbeat，之后才发送目标流。MCU收到确认并收到未过期的目标流
后自动进入`SYSTEM READY`。USB断开、300 ms链路丢失、Heartbeat超过1500 ms
未更新、目标流过期或健康位异常都会自动撤销READY。

KEY3/KEY4因硬件问题暂不参与应用逻辑。PA0天空星板载按键仍只用于电机广播
急停；树莓派启动确认不会使能或移动电机。

进入`SYSTEM READY`后的OLED每200 ms在主循环按以下8行刷新：

```text
SYSTEM READY
AX:... AY:...
IMU F:...
VSN:...
X:... Y:...
IP:192.168.5.148
MOTOR ADDR:...
PI START: OK
```

收到V1心跳、V2报告`0.0.0.0`或链路尚未就绪时，IP行显示`IP:---`，不会复用
上一条V2心跳中的旧地址。

## 树莓派联调

连接后先在树莓派执行：

```bash
ls -l /dev/serial/by-id/
udevadm info --query=property --name=/dev/ttyACM0
```

设备描述符为：

```text
VID:PID 0483:5740
Manufacturer: BallControl
Product: BallControl Vision CDC
```

使用实际枚举出的`/dev/serial/by-id/...`稳定路径，不要猜测字符串，也不要在
正式服务中依赖可能随插拔变化的`/dev/ttyACM0`。

```bash
python3 vision_uart_test_sender.py \
  --serial-device "$CDC_DEVICE" \
  --protocol-version v2 \
  --ipv4 192.168.5.148 \
  --mode golden --duration 10
```

将示例IPv4替换为树莓派当前地址。

先运行黄金发送器验证帧解析，再启动包含启动确认逻辑的正式视觉服务。期望
OLED依次进入`PROTO:OK`、`PI:OK`、`WIFI:OK`、`VISION:OK`，收到
`STARTUP_READY`和目标流后自动显示`SYSTEM READY`。停止发送后进入
`LINK LOST`并撤销READY。

树莓派端修改要求和完整测试清单见
`docs/RASPBERRY_PI_STARTUP_READY_PROMPT.md`。

## 测试

```powershell
.\tools\run_host_tests.ps1
cmake --build --preset debug
```

宿主测试覆盖V1/V2黄金帧逐字段、混合版本粘包、所有V2两段拆包位置、噪声
前缀、CRC错误恢复、错误版本、错误长度、坐标越界、V1扩展字段清零、环形缓冲
回绕/溢出恢复、USB新会话清旧快照，以及连续600个V1目标帧。

2026-07-30验证：三组主机测试通过，ARM Debug构建成功；FLASH使用40,156字节
（7.66%），RAM使用11,384字节（8.69%）。最终固件烧录后V2实机快照读取时已
接受1,430个目标和24个心跳，CRC、格式、语义、USB溢出和丢字节计数全部为0。
