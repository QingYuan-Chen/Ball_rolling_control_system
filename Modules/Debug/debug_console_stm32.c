#include "debug_console_stm32.h"

#include <stddef.h>

static bool DebugConsole_HalWrite(void *context, const uint8_t *data,
                                  uint32_t length)
{
    DebugConsole *console = (DebugConsole *) context;

    if ((console == NULL) || (console->uart == NULL) ||
        (length > UINT16_MAX)) {
        return false;
    }
    return HAL_UART_Transmit(console->uart, (uint8_t *) data,
                            (uint16_t) length, 200U) == HAL_OK;
}

bool DebugConsole_Init(DebugConsole *console, UART_HandleTypeDef *uart)
{
    TianmengxingWirelessSerial_IO_t io;

    if ((console == NULL) || (uart == NULL)) {
        return false;
    }

    console->uart = uart;
    console->rx_byte = 0U;
    console->uart_error_count = 0U;
    io.write = DebugConsole_HalWrite;
    io.context = console;
    TianmengxingWirelessSerial_Init(&console->serial, &io);

    return HAL_UART_Receive_IT(console->uart, &console->rx_byte, 1U) ==
           HAL_OK;
}

void DebugConsole_RxCompleteCallback(DebugConsole *console)
{
    if ((console == NULL) || (console->uart == NULL)) {
        return;
    }

    (void) TianmengxingWirelessSerial_FeedRxByte(
        &console->serial, console->rx_byte);
    if (HAL_UART_Receive_IT(console->uart, &console->rx_byte, 1U) !=
        HAL_OK) {
        console->uart_error_count++;
    }
}

void DebugConsole_ErrorCallback(DebugConsole *console)
{
    if ((console == NULL) || (console->uart == NULL)) {
        return;
    }

    console->uart_error_count++;
    (void) HAL_UART_AbortReceive(console->uart);
    __HAL_UART_CLEAR_OREFLAG(console->uart);
    (void) HAL_UART_Receive_IT(console->uart, &console->rx_byte, 1U);
}

uint32_t DebugConsole_Read(DebugConsole *console, uint8_t *data,
                           uint32_t capacity)
{
    if (console == NULL) {
        return 0U;
    }
    return TianmengxingWirelessSerial_Read(
        &console->serial, data, capacity);
}

bool DebugConsole_Write(DebugConsole *console, const uint8_t *data,
                        uint32_t length)
{
    if (console == NULL) {
        return false;
    }
    return TianmengxingWirelessSerial_Write(
               &console->serial, data, length) ==
           TIANMENGXING_WIRELESS_SERIAL_STATUS_OK;
}

bool DebugConsole_WriteString(DebugConsole *console, const char *text)
{
    if (console == NULL) {
        return false;
    }
    return TianmengxingWirelessSerial_WriteString(
               &console->serial, text) ==
           TIANMENGXING_WIRELESS_SERIAL_STATUS_OK;
}

