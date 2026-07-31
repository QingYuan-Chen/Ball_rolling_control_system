/**
 * @file tianmengxing_wireless_serial.h
 * @brief 天猛星无线串口的可移植收发与环形缓冲驱动。
 *
 * 协议核心不依赖 MSPM0 DriverLib。接收侧按单生产者（UART ISR）、
 * 单消费者（主循环）方式使用。
 */

#ifndef TIANMENGXING_WIRELESS_SERIAL_H_
#define TIANMENGXING_WIRELESS_SERIAL_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TIANMENGXING_WIRELESS_SERIAL_RX_BUFFER_SIZE 256U

typedef enum {
    TIANMENGXING_WIRELESS_SERIAL_STATUS_OK = 0,
    TIANMENGXING_WIRELESS_SERIAL_STATUS_INVALID_ARGUMENT = -1,
    TIANMENGXING_WIRELESS_SERIAL_STATUS_IO_ERROR = -2
} TianmengxingWirelessSerial_Status_t;

typedef bool (*TianmengxingWirelessSerial_Write_t)(
    void *context, const uint8_t *data, uint32_t length);

typedef struct {
    TianmengxingWirelessSerial_Write_t write;
    void *context;
} TianmengxingWirelessSerial_IO_t;

typedef struct {
    uint32_t rx_received_bytes;
    uint32_t rx_dropped_bytes;
    uint32_t tx_bytes;
    uint16_t rx_available;
} TianmengxingWirelessSerial_Stats_t;

typedef struct {
    TianmengxingWirelessSerial_IO_t io;
    uint8_t rx_buffer[TIANMENGXING_WIRELESS_SERIAL_RX_BUFFER_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint32_t rx_received_bytes;
    volatile uint32_t rx_dropped_bytes;
    volatile uint32_t tx_bytes;
} TianmengxingWirelessSerial_t;

/** 初始化驱动对象。仅接收数据时，io 可以为 NULL。 */
void TianmengxingWirelessSerial_Init(
    TianmengxingWirelessSerial_t *serial,
    const TianmengxingWirelessSerial_IO_t *io);

/** 由 UART 接收中断逐字节调用；缓冲满时丢弃新字节并返回 false。 */
bool TianmengxingWirelessSerial_FeedRxByte(
    TianmengxingWirelessSerial_t *serial, uint8_t byte);

/** 返回当前可读取的字节数。 */
uint16_t TianmengxingWirelessSerial_Available(
    const TianmengxingWirelessSerial_t *serial);

/** 从接收缓冲区读取数据，返回实际读取长度。 */
uint32_t TianmengxingWirelessSerial_Read(
    TianmengxingWirelessSerial_t *serial,
    uint8_t *data, uint32_t capacity);

/** 发送任意二进制数据。 */
TianmengxingWirelessSerial_Status_t TianmengxingWirelessSerial_Write(
    TianmengxingWirelessSerial_t *serial,
    const uint8_t *data, uint32_t length);

/** 发送以 '\0' 结束的 ASCII/UTF-8 字符串，不发送结束符。 */
TianmengxingWirelessSerial_Status_t TianmengxingWirelessSerial_WriteString(
    TianmengxingWirelessSerial_t *serial, const char *text);

/** 清空尚未读取的数据。 */
void TianmengxingWirelessSerial_ClearRx(
    TianmengxingWirelessSerial_t *serial);

/** 获取统计信息快照。 */
bool TianmengxingWirelessSerial_GetStats(
    const TianmengxingWirelessSerial_t *serial,
    TianmengxingWirelessSerial_Stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* TIANMENGXING_WIRELESS_SERIAL_H_ */
