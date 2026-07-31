# 给树莓派端会话的启动确认协议修改提示词

> 2026-07-30状态说明：下面代码块记录最初为V1加入启动确认时的约束，不能再
> 单独作为当前正式协议规范。现已在完整保留V1 `0x01/0x11`和黄金向量的前提下
> 增加V2：Target `0x12`/40字节Payload/50字节总长，Heartbeat
> `0x02`/26字节Payload/36字节总长。V2携带帧采集Unix毫秒、树莓派IPv4和
> 系统Unix毫秒；正式服务使用`--protocol-version v2`。继续开发前必须以
> `Modules/RaspberryPiVision/README.md`和树莓派工程
> `raspberry_pi_vision/README.md`为当前事实来源。

```text
你负责修改树莓派5视觉发送端，使其通过STM32F407原生USB CDC向MCU报告启动
状态，并且只有在树莓派启动完成、WiFi已连接、视觉服务/相机/推理均正常后，
先发送启动确认Heartbeat，再开始发送PRIMARY_TARGET坐标。

树莓派工程：
/home/b/raspberry-pi-vision

Windows参考副本：
C:\Users\MECHREU\Desktop\树莓派5

开始前必须完整阅读：
1. README.md
2. raspberry_pi_vision/README.md
3. hailo_vision_uart.py
4. vision_uart_test_sender.py
5. raspberry_pi_vision/protocol.py
6. raspberry_pi_vision/transport.py
7. raspberry_pi_vision/publisher.py
8. raspberry_pi_vision/runtime.py
9. deploy/raspberry-pi-vision.service
10. tests目录中全部视觉通信测试

物理传输已经改为：
- STM32F407 USB OTG FS CDC ACM
- VID:PID=0483:5740
- Manufacturer=BallControl
- Product=BallControl Vision CDC
- Linux通常枚举为/dev/ttyACM*。

先在树莓派实机执行：
lsusb
ls -l /dev/serial/by-id/
udevadm info --query=property --name=/dev/ttyACM0

正式服务必须使用实机发现的/dev/serial/by-id/...稳定路径，不要猜测路径，
也不要长期依赖/dev/ttyACM0。现有PosixSerialPort可直接操作CDC ACM；
--baudrate 460800可以保留为line coding兼容参数，它不限制USB实际速率。

一、不得改变的帧协议

- 方向仍为树莓派到STM32，不新增MCU ACK、重传或MCU控制命令。
- 小端序。
- 帧格式：
  AA 55 | VERSION | MSG_ID | SEQUENCE_U16 |
  PAYLOAD_LEN_U16 | PAYLOAD | CRC16_U16
- VERSION=0x01。
- CRC-16/CCITT-FALSE：
  poly=0x1021、init=0xFFFF、refin=false、refout=false、xorout=0。
- CRC覆盖VERSION至Payload末尾，不含SOF和CRC字段。
- PRIMARY_TARGET仍为MSG_ID=0x11、Payload 32字节、总长42字节。
- HEARTBEAT仍为MSG_ID=0x01、Payload 14字节、总长24字节。
- Q12.4坐标、字段偏移、状态位、共享sequence和42字节黄金帧不得改变。
- protocol.py中的CRC和组帧实现不得重写。

二、Heartbeat system_flags兼容扩展

保留现有定义：
- bit0 SERVICE_READY
- bit1 CAMERA_RUNNING
- bit2 INFERENCE_RUNNING
- bit3 CALIBRATION_LOADED
- bit4 TRACKER_RUNNING

启用两个原保留位：
- bit5 WIFI_CONNECTED，值0x0020
- bit6 STARTUP_READY，值0x0040

将SYSTEM_STATUS_MASK从0x001F更新为0x007F。

STARTUP_READY是树莓派发给MCU的聚合启动确认，只有以下条件同时满足时才能置1：

SERVICE_READY=1
WIFI_CONNECTED=1
CAMERA_RUNNING=1
INFERENCE_RUNNING=1

收到Heartbeat本身说明用户服务已经在树莓派启动；SERVICE_READY只应在脚本配置、
模型和发送线程初始化成功后置1。STARTUP_READY不得仅凭进程存在或固定延时置1。
TRACKER_RUNNING和CALIBRATION_LOADED继续按实际状态上报，但暂不作为启动确认的
强制条件。

WiFi连接不要通过外网ping判断。默认检查wlan0：
- /sys/class/net/wlan0/operstate为up；
- wlan0拥有scope global的IPv4地址；
- 存在经wlan0的默认路由。

如果项目已有可靠NetworkManager或网络状态接口，应复用。接口名应允许通过
命令行参数配置，默认wlan0，并为判断逻辑添加单元测试。

三、启动发送时序

1. 视觉进程启动后立即启动CDC发送线程和独立状态发布循环。
2. STARTUP_READY=0期间不发送有效PRIMARY_TARGET坐标。
3. 等待启动条件期间以10Hz发送Heartbeat，使MCU在300ms链路超时内持续看到
   启动状态；状态变化时立即额外发布一次Heartbeat。
4. 条件全部满足后构造STARTUP_READY=1的Heartbeat。
5. 必须确认该Heartbeat已经被本地发送线程完整写入CDC文件描述符，之后才能
   打开PRIMARY_TARGET发送闸门。这里的“确认”是本地write_all成功，不是MCU
   ACK；禁止增加反向协议。
6. 可为LatestPacketTransmitter增加submit_status_confirmed()，复用现有
   _PacketCompletion机制，不要另建第二个写线程。
7. 启动确认写入成功后：
   - PRIMARY_TARGET按现有最高60Hz策略发送；
   - Heartbeat恢复1Hz，并持续携带WIFI_CONNECTED和STARTUP_READY状态；
   - Heartbeat和TARGET继续共享同一个uint16 sequence源。
8. 启动条件未满足或确认Heartbeat写入失败时，保持坐标闸门关闭并周期重试，
   不得先发坐标后补确认。

四、运行中故障

运行中只要WiFi、相机、推理或服务健康条件任一失效：
- 立即关闭有效坐标发送闸门；
- 立即发送STARTUP_READY=0且反映真实健康位的Heartbeat；
- 立即发送一帧TARGET_VALID=0的PRIMARY_TARGET安全状态包，所有目标坐标、
  track_id、confidence、框和角度字段清零；
- 回到10Hz启动状态Heartbeat；
- 条件恢复后必须再次完成“READY Heartbeat本地写入成功→开放坐标”的顺序。

不要重复发送旧有效坐标。没有目标但视觉系统健康时，可以继续使用现有
TARGET_VALID=0、CAMERA_OK=1、INFERENCE_OK=1的60Hz状态包；它不算有效坐标。

五、建议修改位置

- raspberry_pi_vision/protocol.py
  只新增WIFI_CONNECTED、STARTUP_READY常量并把SYSTEM_STATUS_MASK改为0x007F；
  不改帧结构、CRC或现有字段。
- raspberry_pi_vision/transport.py
  复用单写线程；增加状态包“本地写入完成”能力时复用_PacketCompletion。
- raspberry_pi_vision/publisher.py
  增加启动闸门、10Hz预启动Heartbeat、状态变化立即发布和故障撤销逻辑。
- raspberry_pi_vision/runtime.py / hailo_vision_uart.py
  提供真实WiFi、服务、相机和推理健康状态；增加可配置WiFi接口参数。
- deploy/raspberry-pi-vision.service
  换成实机BallControl CDC的by-id路径；Description改为STM32F407；必要时增加
  网络启动顺序，但脚本内部仍必须处理WiFi掉线和恢复。
- README.md和raspberry_pi_vision/README.md
  更新USB CDC、system_flags和启动时序。

六、必须测试

- 原42字节PRIMARY_TARGET黄金向量保持完全一致，CRC仍为0xE808。
- Heartbeat带0x0020/0x0040后CRC正确且仍为24字节。
- WiFi未连接时STARTUP_READY必须为0且无有效坐标。
- 服务、WiFi、相机、推理全部正常时，READY Heartbeat在线上先于首个有效目标。
- READY Heartbeat本地写失败时不得发送有效目标。
- WiFi掉线、相机停帧、推理停滞均立即清READY并停止旧坐标。
- 条件恢复后重新确认再恢复坐标。
- USB CDC拔插后发送线程能够重连，并重新执行启动确认，不能沿用旧READY状态。
- 运行全部现有Python测试并补齐上述测试。
- 使用vision_uart_test_sender.py和正式脚本分别实机验证。

交付时列出实际修改文件、测试结果、实机CDC by-id路径和未完成验证。不要修改
STM32工程，不修改模型/跟踪算法，不做无关重构。
```
