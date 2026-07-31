#include "raspberry_pi_vision_stm32f407.h"

#include <stddef.h>
#include <string.h>

#if defined(__GNUC__)
#define RPV_STM32F407_COMPILER_BARRIER() \
    __asm volatile ("" ::: "memory")
#else
#define RPV_STM32F407_COMPILER_BARRIER() do { } while (0)
#endif

_Static_assert(
    RPV_STM32F407_USB_RX_BUFFER_SIZE > RPV_MAX_FRAME_SIZE,
    "USB receive ring must hold at least one complete vision frame");
_Static_assert(
    RPV_STM32F407_USB_RX_BUFFER_SIZE <= UINT16_MAX,
    "USB receive ring positions must fit in uint16_t");

static uint16_t ring_advance(uint16_t position, uint32_t amount)
{
    return (uint16_t) (
        ((uint32_t) position + amount) %
        RPV_STM32F407_USB_RX_BUFFER_SIZE);
}

static uint16_t ring_used(uint16_t read_position,
    uint16_t write_position)
{
    if (write_position >= read_position) {
        return (uint16_t) (write_position - read_position);
    }
    return (uint16_t) (
        RPV_STM32F407_USB_RX_BUFFER_SIZE -
        (uint32_t) read_position + write_position);
}

void RaspberryPiVision_STM32F407_ReceiverInit(
    RaspberryPiVision_STM32F407_Receiver_t *receiver)
{
    if (receiver == NULL) {
        return;
    }

    (void) memset(receiver, 0, sizeof(*receiver));
    RPV_Init(&receiver->parser);
}

void RaspberryPiVision_STM32F407_ReceiverSetUsbConnected(
    RaspberryPiVision_STM32F407_Receiver_t *receiver, bool connected)
{
    if ((receiver != NULL) &&
        (receiver->usb_connected != connected)) {
        receiver->usb_connected = connected;
        RPV_STM32F407_COMPILER_BARRIER();
        receiver->usb_session_generation++;
    }
}

bool RaspberryPiVision_STM32F407_ReceiverEnqueueUsbData(
    RaspberryPiVision_STM32F407_Receiver_t *receiver,
    const uint8_t *data, uint32_t length)
{
    uint16_t read_position;
    uint16_t write_position;
    uint16_t used;
    uint32_t free_space;
    uint32_t first_length;

    if ((receiver == NULL) || (data == NULL) || (length == 0U)) {
        return false;
    }

    receiver->usb_packet_count++;
    read_position = receiver->usb_rx_read_position;
    write_position = receiver->usb_rx_write_position;
    used = ring_used(read_position, write_position);
    free_space = RPV_STM32F407_USB_RX_BUFFER_SIZE -
                 (uint32_t) used - 1U;

    if (length > free_space) {
        receiver->usb_rx_overflow_count++;
        receiver->usb_dropped_byte_count += length;
        return false;
    }

    first_length = RPV_STM32F407_USB_RX_BUFFER_SIZE -
                   (uint32_t) write_position;
    if (first_length > length) {
        first_length = length;
    }
    (void) memcpy(&receiver->usb_rx_buffer[write_position],
                  data, first_length);
    if (length > first_length) {
        (void) memcpy(receiver->usb_rx_buffer,
                      &data[first_length], length - first_length);
    }

    RPV_STM32F407_COMPILER_BARRIER();
    receiver->usb_rx_write_position =
        ring_advance(write_position, length);
    return true;
}

uint32_t RaspberryPiVision_STM32F407_ReceiverProcess(
    RaspberryPiVision_STM32F407_Receiver_t *receiver,
    uint32_t received_at_ms)
{
    uint16_t read_position;
    uint16_t write_position;
    uint32_t accepted = 0U;

    if (receiver == NULL) {
        return 0U;
    }

    /*
     * USB配置/断开回调只发布新会话代号。解析器只能由主循环重置，
     * 避免在OTG中断中与RPV_FeedBytes()并发修改快照。每次新会话
     * 丢弃旧快照和切换期间排队的字节，必须重新收到本会话的协议帧、
     * Heartbeat和Target后才允许应用进入READY。
     */
    if (receiver->processed_usb_session_generation !=
        receiver->usb_session_generation) {
        write_position = receiver->usb_rx_write_position;
        RPV_Init(&receiver->parser);
        RPV_STM32F407_COMPILER_BARRIER();
        receiver->usb_rx_read_position = write_position;
        receiver->processed_usb_session_generation =
            receiver->usb_session_generation;
        return 0U;
    }

    read_position = receiver->usb_rx_read_position;
    write_position = receiver->usb_rx_write_position;
    RPV_STM32F407_COMPILER_BARRIER();

    if (write_position > read_position) {
        accepted = RPV_FeedBytes(&receiver->parser,
            &receiver->usb_rx_buffer[read_position],
            (uint32_t) (write_position - read_position),
            received_at_ms);
    } else if (write_position < read_position) {
        accepted = RPV_FeedBytes(&receiver->parser,
            &receiver->usb_rx_buffer[read_position],
            RPV_STM32F407_USB_RX_BUFFER_SIZE -
                (uint32_t) read_position,
            received_at_ms);
        if (write_position != 0U) {
            accepted += RPV_FeedBytes(&receiver->parser,
                receiver->usb_rx_buffer, write_position,
                received_at_ms);
        }
    }

    RPV_STM32F407_COMPILER_BARRIER();
    receiver->usb_rx_read_position = write_position;
    return accepted;
}

bool RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
    const RaspberryPiVision_STM32F407_Receiver_t *receiver,
    RPV_TargetSnapshot_t *target)
{
    return (receiver != NULL) &&
           RPV_GetLatestTarget(&receiver->parser, target);
}

bool RaspberryPiVision_STM32F407_ReceiverGetLatestHeartbeat(
    const RaspberryPiVision_STM32F407_Receiver_t *receiver,
    RPV_HeartbeatSnapshot_t *heartbeat)
{
    return (receiver != NULL) &&
           RPV_GetLatestHeartbeat(&receiver->parser, heartbeat);
}

void RaspberryPiVision_STM32F407_ReceiverGetStats(
    const RaspberryPiVision_STM32F407_Receiver_t *receiver,
    RaspberryPiVision_STM32F407_Stats_t *stats)
{
    uint16_t read_position;
    uint16_t write_position;

    if ((receiver == NULL) || (stats == NULL)) {
        return;
    }

    RPV_GetStats(&receiver->parser, &stats->protocol);
    stats->usb_packet_count = receiver->usb_packet_count;
    stats->usb_rx_overflow_count =
        receiver->usb_rx_overflow_count;
    stats->usb_dropped_byte_count =
        receiver->usb_dropped_byte_count;
    read_position = receiver->usb_rx_read_position;
    write_position = receiver->usb_rx_write_position;
    stats->queued_byte_count = ring_used(
        read_position, write_position);
    stats->usb_connected = receiver->usb_connected;
}

RPV_TargetState_t
RaspberryPiVision_STM32F407_ReceiverEvaluateTargetState(
    const RaspberryPiVision_STM32F407_Receiver_t *receiver,
    uint32_t now_ms, uint32_t target_timeout_ms,
    uint32_t link_timeout_ms)
{
    return RPV_EvaluateTargetState(
        (receiver == NULL) ? NULL : &receiver->parser,
        now_ms, target_timeout_ms, link_timeout_ms);
}

RPV_TargetState_t
RaspberryPiVision_STM32F407_ReceiverGetTargetState(
    const RaspberryPiVision_STM32F407_Receiver_t *receiver,
    uint32_t now_ms)
{
    return
        RaspberryPiVision_STM32F407_ReceiverEvaluateTargetState(
            receiver, now_ms, RPV_STM32F407_TARGET_TIMEOUT_MS,
            RPV_STM32F407_LINK_TIMEOUT_MS);
}

bool RaspberryPiVision_STM32F407_ReceiverIsUsbConnected(
    const RaspberryPiVision_STM32F407_Receiver_t *receiver)
{
    return (receiver != NULL) && receiver->usb_connected;
}
