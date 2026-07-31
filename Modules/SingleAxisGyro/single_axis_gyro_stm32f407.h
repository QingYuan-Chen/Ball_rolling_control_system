/**
 * @file single_axis_gyro_stm32f407.h
 * @brief STM32F407 HAL circular-DMA UART adapter for SingleAxisGyro.
 */

#ifndef SINGLE_AXIS_GYRO_STM32F407_H_
#define SINGLE_AXIS_GYRO_STM32F407_H_

#include <stdbool.h>
#include <stdint.h>

#include "single_axis_gyro.h"
#include "stm32f4xx_hal.h"

#define SINGLE_AXIS_GYRO_RX_DMA_BUFFER_SIZE 128U

typedef struct {
    uint32_t received_byte_count;
    uint32_t uart_error_count;
    uint32_t rx_overrun_count;
    uint32_t last_byte_ms;
    uint32_t last_valid_frame_ms;
    bool receive_armed;
} SingleAxisGyro_STM32F407_Statistics_t;

typedef struct {
    SingleAxisGyro_t protocol;
    UART_HandleTypeDef *uart;
    uint8_t rx_dma_buffer[SINGLE_AXIS_GYRO_RX_DMA_BUFFER_SIZE];
    volatile uint32_t rx_published_count;
    uint32_t rx_consumed_count;
    volatile uint16_t rx_event_position;
    volatile uint32_t received_byte_count;
    volatile uint32_t uart_error_count;
    volatile uint32_t rx_overrun_count;
    volatile uint32_t last_byte_ms;
    volatile uint32_t last_valid_frame_ms;
    volatile bool receive_armed;
    volatile bool rx_restart_requested;
} SingleAxisGyro_STM32F407_t;

bool SingleAxisGyro_STM32F407_Init(
    SingleAxisGyro_STM32F407_t *receiver,
    UART_HandleTypeDef *uart, float rate_full_scale_dps);

void SingleAxisGyro_STM32F407_RxEventCallback(
    SingleAxisGyro_STM32F407_t *receiver, uint16_t position);

void SingleAxisGyro_STM32F407_Process(
    SingleAxisGyro_STM32F407_t *receiver);

void SingleAxisGyro_STM32F407_ErrorCallback(
    SingleAxisGyro_STM32F407_t *receiver);

bool SingleAxisGyro_STM32F407_GetSample(
    const SingleAxisGyro_STM32F407_t *receiver,
    SingleAxisGyro_Sample_t *sample);

bool SingleAxisGyro_STM32F407_RequestYawZero(
    SingleAxisGyro_STM32F407_t *receiver, uint32_t now_ms);

SingleAxisGyro_YawZeroState_t
SingleAxisGyro_STM32F407_GetYawZeroState(
    const SingleAxisGyro_STM32F407_t *receiver);

void SingleAxisGyro_STM32F407_GetStatistics(
    const SingleAxisGyro_STM32F407_t *receiver,
    SingleAxisGyro_STM32F407_Statistics_t *statistics);

#endif /* SINGLE_AXIS_GYRO_STM32F407_H_ */
