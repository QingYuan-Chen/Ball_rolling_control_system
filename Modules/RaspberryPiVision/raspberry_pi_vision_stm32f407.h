/**
 * @file raspberry_pi_vision_stm32f407.h
 * @brief STM32F407 USB CDC到树莓派视觉协议核心的无动态内存适配层。
 *
 * USB CDC接收回调是唯一字节生产者，只负责把完整USB OUT数据块复制到静态
 * 环形缓冲。主循环调用ReceiverProcess()消费缓冲并执行协议解析。应用层只
 * 通过快照和状态API读取结果，不直接访问RPV_Parser_t。
 */

#ifndef RASPBERRY_PI_VISION_STM32F407_H_
#define RASPBERRY_PI_VISION_STM32F407_H_

#include <stdbool.h>
#include <stdint.h>

#include "raspberry_pi_vision_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RPV_STM32F407_TARGET_TIMEOUT_MS 100U
#define RPV_STM32F407_LINK_TIMEOUT_MS   300U
#define RPV_STM32F407_USB_RX_BUFFER_SIZE 1024U

typedef struct {
    RPV_Stats_t protocol;
    uint32_t usb_packet_count;
    uint32_t usb_rx_overflow_count;
    uint32_t usb_dropped_byte_count;
    uint16_t queued_byte_count;
    bool usb_connected;
} RaspberryPiVision_STM32F407_Stats_t;

typedef struct {
    RPV_Parser_t parser;
    uint8_t usb_rx_buffer[RPV_STM32F407_USB_RX_BUFFER_SIZE];
    volatile uint16_t usb_rx_write_position;
    volatile uint16_t usb_rx_read_position;
    volatile uint32_t usb_packet_count;
    volatile uint32_t usb_rx_overflow_count;
    volatile uint32_t usb_dropped_byte_count;
    volatile uint32_t usb_session_generation;
    uint32_t processed_usb_session_generation;
    volatile bool usb_connected;
} RaspberryPiVision_STM32F407_Receiver_t;

/** 初始化解析器、USB接收缓冲和统计量。 */
void RaspberryPiVision_STM32F407_ReceiverInit(
    RaspberryPiVision_STM32F407_Receiver_t *receiver);

/** USB CDC配置或断开时更新物理连接状态；本函数不修改协议快照。 */
void RaspberryPiVision_STM32F407_ReceiverSetUsbConnected(
    RaspberryPiVision_STM32F407_Receiver_t *receiver, bool connected);

/**
 * USB CDC OUT回调入口。
 *
 * 只复制数据并发布新的写位置，不执行CRC、打印、显示或控制。缓冲空间不足时
 * 丢弃整个USB数据块，防止把半个数据块交给协议解析器。
 */
bool RaspberryPiVision_STM32F407_ReceiverEnqueueUsbData(
    RaspberryPiVision_STM32F407_Receiver_t *receiver,
    const uint8_t *data, uint32_t length);

/**
 * 主循环消费USB缓冲并调用RPV_FeedBytes()。
 *
 * @return 本次成功接受的目标帧与心跳帧总数。
 */
uint32_t RaspberryPiVision_STM32F407_ReceiverProcess(
    RaspberryPiVision_STM32F407_Receiver_t *receiver,
    uint32_t received_at_ms);

bool RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
    const RaspberryPiVision_STM32F407_Receiver_t *receiver,
    RPV_TargetSnapshot_t *target);

bool RaspberryPiVision_STM32F407_ReceiverGetLatestHeartbeat(
    const RaspberryPiVision_STM32F407_Receiver_t *receiver,
    RPV_HeartbeatSnapshot_t *heartbeat);

void RaspberryPiVision_STM32F407_ReceiverGetStats(
    const RaspberryPiVision_STM32F407_Receiver_t *receiver,
    RaspberryPiVision_STM32F407_Stats_t *stats);

RPV_TargetState_t
RaspberryPiVision_STM32F407_ReceiverEvaluateTargetState(
    const RaspberryPiVision_STM32F407_Receiver_t *receiver,
    uint32_t now_ms, uint32_t target_timeout_ms,
    uint32_t link_timeout_ms);

RPV_TargetState_t RaspberryPiVision_STM32F407_ReceiverGetTargetState(
    const RaspberryPiVision_STM32F407_Receiver_t *receiver,
    uint32_t now_ms);

bool RaspberryPiVision_STM32F407_ReceiverIsUsbConnected(
    const RaspberryPiVision_STM32F407_Receiver_t *receiver);

#ifdef __cplusplus
}
#endif

#endif /* RASPBERRY_PI_VISION_STM32F407_H_ */
