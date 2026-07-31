#ifndef BALLCONTROL_DEBUG_CONSOLE_STM32_H
#define BALLCONTROL_DEBUG_CONSOLE_STM32_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "tianmengxing_wireless_serial.h"

typedef struct {
    UART_HandleTypeDef *uart;
    TianmengxingWirelessSerial_t serial;
    uint8_t rx_byte;
    volatile uint32_t uart_error_count;
} DebugConsole;

bool DebugConsole_Init(DebugConsole *console, UART_HandleTypeDef *uart);
void DebugConsole_RxCompleteCallback(DebugConsole *console);
void DebugConsole_ErrorCallback(DebugConsole *console);
uint32_t DebugConsole_Read(DebugConsole *console, uint8_t *data,
                           uint32_t capacity);
bool DebugConsole_Write(DebugConsole *console, const uint8_t *data,
                        uint32_t length);
bool DebugConsole_WriteString(DebugConsole *console, const char *text);

#endif

