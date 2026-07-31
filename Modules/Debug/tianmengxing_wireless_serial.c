#include "tianmengxing_wireless_serial.h"

#include <stddef.h>
#include <string.h>

static uint16_t next_index(uint16_t index)
{
    index++;
    if (index >= TIANMENGXING_WIRELESS_SERIAL_RX_BUFFER_SIZE) {
        index = 0U;
    }
    return index;
}

void TianmengxingWirelessSerial_Init(
    TianmengxingWirelessSerial_t *serial,
    const TianmengxingWirelessSerial_IO_t *io)
{
    if (serial == NULL) {
        return;
    }

    (void) memset(serial, 0, sizeof(*serial));
    if (io != NULL) {
        serial->io = *io;
    }
}

bool TianmengxingWirelessSerial_FeedRxByte(
    TianmengxingWirelessSerial_t *serial, uint8_t byte)
{
    uint16_t next_head;

    if (serial == NULL) {
        return false;
    }

    next_head = next_index(serial->rx_head);
    if (next_head == serial->rx_tail) {
        serial->rx_dropped_bytes++;
        return false;
    }

    serial->rx_buffer[serial->rx_head] = byte;
    serial->rx_head = next_head;
    serial->rx_received_bytes++;
    return true;
}

uint16_t TianmengxingWirelessSerial_Available(
    const TianmengxingWirelessSerial_t *serial)
{
    uint16_t head;
    uint16_t tail;

    if (serial == NULL) {
        return 0U;
    }

    head = serial->rx_head;
    tail = serial->rx_tail;
    return (head >= tail) ?
               (uint16_t) (head - tail) :
               (uint16_t) (TIANMENGXING_WIRELESS_SERIAL_RX_BUFFER_SIZE -
                   tail + head);
}

uint32_t TianmengxingWirelessSerial_Read(
    TianmengxingWirelessSerial_t *serial,
    uint8_t *data, uint32_t capacity)
{
    uint32_t count = 0U;

    if ((serial == NULL) || ((data == NULL) && (capacity != 0U))) {
        return 0U;
    }

    while ((count < capacity) && (serial->rx_tail != serial->rx_head)) {
        data[count++] = serial->rx_buffer[serial->rx_tail];
        serial->rx_tail = next_index(serial->rx_tail);
    }
    return count;
}

TianmengxingWirelessSerial_Status_t TianmengxingWirelessSerial_Write(
    TianmengxingWirelessSerial_t *serial,
    const uint8_t *data, uint32_t length)
{
    if ((serial == NULL) || ((data == NULL) && (length != 0U))) {
        return TIANMENGXING_WIRELESS_SERIAL_STATUS_INVALID_ARGUMENT;
    }
    if (length == 0U) {
        return TIANMENGXING_WIRELESS_SERIAL_STATUS_OK;
    }
    if (serial->io.write == NULL) {
        return TIANMENGXING_WIRELESS_SERIAL_STATUS_INVALID_ARGUMENT;
    }
    if (!serial->io.write(serial->io.context, data, length)) {
        return TIANMENGXING_WIRELESS_SERIAL_STATUS_IO_ERROR;
    }

    serial->tx_bytes += length;
    return TIANMENGXING_WIRELESS_SERIAL_STATUS_OK;
}

TianmengxingWirelessSerial_Status_t TianmengxingWirelessSerial_WriteString(
    TianmengxingWirelessSerial_t *serial, const char *text)
{
    uint32_t length = 0U;

    if (text == NULL) {
        return TIANMENGXING_WIRELESS_SERIAL_STATUS_INVALID_ARGUMENT;
    }
    while (text[length] != '\0') {
        length++;
    }
    return TianmengxingWirelessSerial_Write(
        serial, (const uint8_t *) text, length);
}

void TianmengxingWirelessSerial_ClearRx(
    TianmengxingWirelessSerial_t *serial)
{
    if (serial != NULL) {
        serial->rx_tail = serial->rx_head;
    }
}

bool TianmengxingWirelessSerial_GetStats(
    const TianmengxingWirelessSerial_t *serial,
    TianmengxingWirelessSerial_Stats_t *stats)
{
    if ((serial == NULL) || (stats == NULL)) {
        return false;
    }

    stats->rx_received_bytes = serial->rx_received_bytes;
    stats->rx_dropped_bytes = serial->rx_dropped_bytes;
    stats->tx_bytes = serial->tx_bytes;
    stats->rx_available = TianmengxingWirelessSerial_Available(serial);
    return true;
}
