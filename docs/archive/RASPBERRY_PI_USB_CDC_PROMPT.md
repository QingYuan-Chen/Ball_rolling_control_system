# 给树莓派端开发者的USB CDC迁移提示词

> 本提示词仅包含早期USB CDC物理传输迁移要求，已被
> `RASPBERRY_PI_STARTUP_READY_PROMPT.md`取代。请使用新提示词完成
> WiFi/视觉启动确认和坐标发送闸门，不要单独使用本文。

```text
你负责把现有“树莓派5视觉坐标发送端”从CH340/硬件UART设备迁移到
STM32F407VET6的原生USB CDC设备。只修改树莓派端，禁止修改STM32工程。

开始前完整阅读：
1. /home/b/raspberry-pi-vision/README.md
2. /home/b/raspberry-pi-vision/raspberry_pi_vision/README.md
3. /home/b/raspberry-pi-vision/hailo_vision_uart.py
4. /home/b/raspberry-pi-vision/vision_uart_test_sender.py
5. /home/b/raspberry-pi-vision/raspberry_pi_vision/transport.py
6. /home/b/raspberry-pi-vision/raspberry_pi_vision/protocol.py
7. /home/b/raspberry-pi-vision/deploy/raspberry-pi-vision.service
8. /home/b/raspberry-pi-vision/tests下全部视觉通信测试

STM32 USB设备已确认：
- USB Full Speed CDC ACM
- VID:PID = 0483:5740
- Manufacturer = BallControl
- Product = BallControl Vision CDC
- Linux通常枚举为/dev/ttyACM*，但正式服务必须使用实机发现的
  /dev/serial/by-id/...稳定路径，禁止猜测by-id名称。

先在树莓派实机执行并记录：
lsusb
ls -l /dev/serial/by-id/
udevadm info --query=property --name=/dev/ttyACM0

需要修改：
1. deploy/raspberry-pi-vision.service：
   - Description中的MSPM0G3507改为STM32F407；
   - ExecStart的--serial-device从
     /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
     改为实机枚举出的BallControl Vision CDC稳定by-id路径；
   - --baudrate 460800可以保留。它对CDC只是line coding兼容参数，不限制
     USB的实际速率；
   - 保留Restart、停止流程、0.30/0.20阈值和单槽最新包发送策略。
2. 顶层README.md和raspberry_pi_vision/README.md：
   - 把CH340、ttyUSB0、天猛星/MSPM0接法更新为STM32F407原生USB CDC、
     ttyACM和实机by-id路径；
   - 明确USB CDC不是STM32 UART，不使用460800 UART带宽计算；
   - 端到端prediction/downstream delay继续以实测为准，不臆造USB延迟。
3. 如果修改日志或命名，可把用户可见的UART字样改为serial/USB CDC，但保持
   现有PacketPort、LatestPacketTransmitter接口和非阻塞重连行为，避免无关
   重构。

以下协议已经冻结，绝对不能修改：
- 方向仍为树莓派到STM32，禁止新增ACK、重传或MCU命令；
- AA 55帧头、VERSION=0x01、消息ID、字段偏移和小端序；
- CRC-16/CCITT-FALSE参数和覆盖范围；
- PRIMARY_TARGET 42字节/最高60Hz；
- HEARTBEAT 24字节/1Hz；
- Q12.4坐标、状态位、健康位和黄金向量；
- 最新目标覆盖旧目标的发送策略。

不要修改raspberry_pi_vision/protocol.py中的协议定义。transport.py当前基于
termios和POSIX文件描述符，理论上可直接打开ttyACM；先用现有实现实测，不要
为了CDC另造传输协议。如果必须改transport.py，只允许做设备名称/日志泛化和
拔插恢复修复，write_all()的完整写入、超时和非阻塞语义必须保留。

验证顺序：
1. 运行现有全部Python测试，必须全部通过；
2. 使用实机by-id路径运行vision_uart_test_sender.py黄金模式10秒；
3. 确认发送600个42字节帧，频率约60Hz，无写入错误；
4. 分别测试golden、no-target和vision-fault模式；
5. 启动正式hailo_vision_uart.py，确认Heartbeat和PRIMARY_TARGET持续发送；
6. 更新并安装systemd用户服务，验证启动、停止、USB拔插后重连和journal；
7. 报告实际修改文件、测试结果、实机设备路径；未做的实机验证必须明确说明。

不要修改STM32代码，不修改黄金向量，不顺手重构视觉推理、跟踪或模型部分。
```
