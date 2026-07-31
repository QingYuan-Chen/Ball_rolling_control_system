#ifndef BALLCONTROL_JY61P_STM32_H
#define BALLCONTROL_JY61P_STM32_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define JY61P_DATA_ACCELERATION (1UL << 0)
#define JY61P_DATA_GYROSCOPE    (1UL << 1)
#define JY61P_DATA_ANGLE        (1UL << 2)
#define JY61P_DATA_QUATERNION   (1UL << 3)
#define JY61P_RX_DMA_BUFFER_SIZE 128U

typedef struct {
    float acceleration_g[3];
    float angular_velocity_dps[3];
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float temperature_c;
    float quaternion[4];
    uint32_t valid_flags;
    uint32_t received_byte_count;
    uint32_t header_byte_count;
    uint32_t valid_frame_count;
    uint32_t checksum_error_count;
    uint32_t uart_error_count;
    uint32_t rx_overrun_count;
    uint32_t last_valid_frame_ms;
    uint32_t last_acceleration_frame_ms;
    uint8_t last_received_byte;
    bool receive_armed;
} JY61P_Data;

typedef struct {
    UART_HandleTypeDef *uart;
    volatile int16_t acceleration[3];
    volatile int16_t angular_velocity[3];
    volatile int16_t angle[3];
    volatile int16_t temperature;
    volatile int16_t quaternion[4];
    volatile uint32_t valid_flags;
    volatile uint32_t received_byte_count;
    volatile uint32_t header_byte_count;
    volatile uint32_t valid_frame_count;
    volatile uint32_t checksum_error_count;
    volatile uint32_t uart_error_count;
    volatile uint32_t rx_overrun_count;
    volatile uint32_t last_valid_frame_ms;
    volatile uint32_t last_acceleration_frame_ms;
    volatile uint8_t last_received_byte;
    uint8_t frame[11];
    uint8_t frame_index;
    uint8_t rx_dma_buffer[JY61P_RX_DMA_BUFFER_SIZE];
    volatile uint32_t rx_published_count;
    uint32_t rx_consumed_count;
    volatile uint16_t rx_event_position;
    volatile bool receive_armed;
    volatile bool rx_restart_requested;
} JY61P_Driver;

bool JY61P_Init(JY61P_Driver *driver, UART_HandleTypeDef *uart);
void JY61P_RxEventCallback(JY61P_Driver *driver, uint16_t position);
void JY61P_ProcessRx(JY61P_Driver *driver);
void JY61P_ErrorCallback(JY61P_Driver *driver);
void JY61P_FeedBytes(JY61P_Driver *driver, const uint8_t *data,
                     uint16_t length);
void JY61P_RecordUartError(JY61P_Driver *driver);
bool JY61P_GetData(JY61P_Driver *driver, JY61P_Data *data);
bool JY61P_WriteRegister(JY61P_Driver *driver, uint8_t register_address,
                         uint16_t value);
bool JY61P_ZeroYaw(JY61P_Driver *driver);

#endif
