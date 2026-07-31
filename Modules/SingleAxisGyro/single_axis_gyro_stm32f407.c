#include "single_axis_gyro_stm32f407.h"

#include <stddef.h>
#include <string.h>

#define SINGLE_AXIS_GYRO_TX_TIMEOUT_MS 100U

static bool UartWrite(void *context, const uint8_t *data, uint32_t length)
{
    SingleAxisGyro_STM32F407_t *receiver =
        (SingleAxisGyro_STM32F407_t *) context;

    if ((receiver == NULL) || (receiver->uart == NULL) ||
        (data == NULL) || (length > UINT16_MAX)) {
        return false;
    }

    return HAL_UART_Transmit(receiver->uart, (uint8_t *) data,
        (uint16_t) length, SINGLE_AXIS_GYRO_TX_TIMEOUT_MS) == HAL_OK;
}

static void DelayMs(void *context, uint32_t delay_ms)
{
    (void) context;
    HAL_Delay(delay_ms);
}

static bool StartReceiveDma(SingleAxisGyro_STM32F407_t *receiver)
{
    HAL_StatusTypeDef status;

    receiver->rx_event_position = 0U;
    receiver->rx_published_count = 0U;
    receiver->rx_consumed_count = 0U;
    status = HAL_UARTEx_ReceiveToIdle_DMA(
        receiver->uart, receiver->rx_dma_buffer,
        SINGLE_AXIS_GYRO_RX_DMA_BUFFER_SIZE);
    receiver->receive_armed = status == HAL_OK;
    if (receiver->receive_armed) {
        __HAL_DMA_DISABLE_IT(receiver->uart->hdmarx, DMA_IT_HT);
    }
    return receiver->receive_armed;
}

bool SingleAxisGyro_STM32F407_Init(
    SingleAxisGyro_STM32F407_t *receiver,
    UART_HandleTypeDef *uart, float rate_full_scale_dps)
{
    SingleAxisGyro_IO_t io;

    if ((receiver == NULL) || (uart == NULL)) {
        return false;
    }

    (void) memset(receiver, 0, sizeof(*receiver));
    receiver->uart = uart;
    io.write = UartWrite;
    io.delay_ms = DelayMs;
    io.context = receiver;
    SingleAxisGyro_Init(
        &receiver->protocol, &io, rate_full_scale_dps);

    return StartReceiveDma(receiver);
}

void SingleAxisGyro_STM32F407_RxEventCallback(
    SingleAxisGyro_STM32F407_t *receiver, uint16_t position)
{
    uint16_t normalized_position;
    uint16_t previous_position;
    uint32_t received;

    if ((receiver == NULL) || !receiver->receive_armed ||
        (position > SINGLE_AXIS_GYRO_RX_DMA_BUFFER_SIZE)) {
        return;
    }

    normalized_position =
        (position == SINGLE_AXIS_GYRO_RX_DMA_BUFFER_SIZE) ? 0U :
        position;
    previous_position = receiver->rx_event_position;
    if ((position == SINGLE_AXIS_GYRO_RX_DMA_BUFFER_SIZE) &&
        (previous_position == 0U)) {
        received = (HAL_UARTEx_GetRxEventType(receiver->uart) ==
                    HAL_UART_RXEVENT_IDLE) ?
                   0U : SINGLE_AXIS_GYRO_RX_DMA_BUFFER_SIZE;
    } else if (normalized_position >= previous_position) {
        received = (uint32_t) (normalized_position - previous_position);
    } else {
        received = (uint32_t) SINGLE_AXIS_GYRO_RX_DMA_BUFFER_SIZE -
                   previous_position + normalized_position;
    }
    receiver->rx_event_position = normalized_position;
    receiver->rx_published_count += received;
}

void SingleAxisGyro_STM32F407_Process(
    SingleAxisGyro_STM32F407_t *receiver)
{
    uint32_t published_count;
    uint32_t available;

    if ((receiver == NULL) || (receiver->uart == NULL)) {
        return;
    }

    if (receiver->rx_restart_requested) {
        receiver->rx_restart_requested = false;
        (void) HAL_UART_AbortReceive(receiver->uart);
        __HAL_UART_CLEAR_OREFLAG(receiver->uart);
        receiver->protocol.frame_index = 0U;
        if (!StartReceiveDma(receiver)) {
            receiver->uart_error_count++;
            receiver->rx_restart_requested = true;
        }
        return;
    }

    published_count = receiver->rx_published_count;
    available = published_count - receiver->rx_consumed_count;
    if (available > SINGLE_AXIS_GYRO_RX_DMA_BUFFER_SIZE) {
        receiver->rx_overrun_count++;
        receiver->rx_consumed_count =
            published_count - SINGLE_AXIS_GYRO_RX_DMA_BUFFER_SIZE;
        receiver->protocol.frame_index = 0U;
    }

    while (receiver->rx_consumed_count != published_count) {
        SingleAxisGyro_FrameResult_t result;
        uint32_t index = receiver->rx_consumed_count %
                         SINGLE_AXIS_GYRO_RX_DMA_BUFFER_SIZE;

        receiver->received_byte_count++;
        receiver->last_byte_ms = HAL_GetTick();
        result = SingleAxisGyro_FeedByte(
            &receiver->protocol, receiver->rx_dma_buffer[index]);
        if ((result == SINGLE_AXIS_GYRO_FRAME_RATE) ||
            (result == SINGLE_AXIS_GYRO_FRAME_YAW)) {
            receiver->last_valid_frame_ms = receiver->last_byte_ms;
        }
        receiver->rx_consumed_count++;
    }
}

void SingleAxisGyro_STM32F407_ErrorCallback(
    SingleAxisGyro_STM32F407_t *receiver)
{
    if ((receiver == NULL) || (receiver->uart == NULL)) {
        return;
    }

    receiver->uart_error_count++;
    receiver->receive_armed = false;
    receiver->rx_restart_requested = true;
}

bool SingleAxisGyro_STM32F407_GetSample(
    const SingleAxisGyro_STM32F407_t *receiver,
    SingleAxisGyro_Sample_t *sample)
{
    return (receiver != NULL) &&
        SingleAxisGyro_GetSample(&receiver->protocol, sample);
}

void SingleAxisGyro_STM32F407_GetStatistics(
    const SingleAxisGyro_STM32F407_t *receiver,
    SingleAxisGyro_STM32F407_Statistics_t *statistics)
{
    uint32_t primask;

    if ((receiver == NULL) || (statistics == NULL)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    statistics->received_byte_count = receiver->received_byte_count;
    statistics->uart_error_count = receiver->uart_error_count;
    statistics->rx_overrun_count = receiver->rx_overrun_count;
    statistics->last_byte_ms = receiver->last_byte_ms;
    statistics->last_valid_frame_ms = receiver->last_valid_frame_ms;
    statistics->receive_armed = receiver->receive_armed;
    if (primask == 0U) {
        __enable_irq();
    }
}
