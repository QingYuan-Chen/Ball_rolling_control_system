#include "emm_v5.h"

#include <stddef.h>
#include <string.h>

#define EMM_V5_COMMAND_ENABLE            0xF3U
#define EMM_V5_COMMAND_FAST_PARAMS       0xF1U
#define EMM_V5_COMMAND_FAST_POSITION     0xFCU
#define EMM_V5_COMMAND_STOP              0xFEU
#define EMM_V5_COMMAND_VERSION           0x1FU
#define EMM_V5_COMMAND_CURRENT_POSITION  0x36U
#define EMM_V5_COMMAND_CURRENT_SPEED     0x35U
#define EMM_V5_COMMAND_STATUS            0x3AU

#define EMM_V5_ACK_LENGTH                4U
#define EMM_V5_POSITION_RESPONSE_LENGTH  8U
#define EMM_V5_SPEED_RESPONSE_LENGTH     6U
#define EMM_V5_VERSION_RESPONSE_LENGTH   7U
#define EMM_V5_MAX_SPEED_RPM             5000U

static void EmmV5_ResetParser(EmmV5_Driver *driver)
{
    driver->index = 0U;
    driver->complete = false;
    (void) memset(driver->bytes, 0, sizeof(driver->bytes));
}

static void EmmV5_PrepareResponse(EmmV5_Driver *driver, uint8_t address,
                                  uint8_t command, uint8_t length)
{
    driver->expected_address = address;
    driver->expected_command = command;
    driver->expected_length = length;
    driver->response_pending = true;
    EmmV5_ResetParser(driver);
}

static void EmmV5_ReceiveByte(EmmV5_Driver *driver, uint8_t byte)
{
    uint8_t index = driver->index;

    if ((driver->expected_length == 0U) || driver->complete) {
        driver->statistics.unexpected_byte_count++;
        return;
    }

    if (index == 0U) {
        if (byte != driver->expected_address) {
            driver->statistics.unexpected_byte_count++;
            return;
        }
    } else if ((index == 1U) && (byte != driver->expected_command)) {
        driver->statistics.invalid_frame_count++;
        driver->index = 0U;
        if (byte == driver->expected_address) {
            driver->bytes[0] = byte;
            driver->index = 1U;
        }
        return;
    }

    if (index >= EMM_V5_MAX_RESPONSE_LENGTH) {
        driver->statistics.invalid_frame_count++;
        EmmV5_ResetParser(driver);
        return;
    }

    driver->bytes[index] = byte;
    index++;
    driver->index = index;
    if (index == driver->expected_length) {
        if (driver->bytes[index - 1U] == EMM_V5_CHECK_BYTE) {
            driver->complete = true;
            driver->statistics.valid_frame_count++;
        } else {
            driver->statistics.invalid_frame_count++;
            EmmV5_ResetParser(driver);
        }
    }
}

static bool EmmV5_SendFrame(EmmV5_Driver *driver, const uint8_t *frame,
                            uint8_t frame_length, uint8_t response_length)
{
    if ((driver == NULL) || (driver->uart == NULL) || (frame == NULL) ||
        (frame_length < 3U) ||
        (frame_length > EMM_V5_MAX_TX_FRAME_LENGTH) ||
        (response_length > EMM_V5_MAX_RESPONSE_LENGTH)) {
        return false;
    }
    if (driver->tx_busy || driver->response_pending) {
        return false;
    }

    (void) memcpy(driver->tx_buffer, frame, frame_length);
    EmmV5_PrepareResponse(driver, frame[0], frame[1], response_length);
    driver->tx_busy = true;
    if (HAL_UART_Transmit_DMA(driver->uart, driver->tx_buffer,
                              frame_length) != HAL_OK) {
        driver->tx_busy = false;
        driver->response_pending = false;
        driver->expected_length = 0U;
        return false;
    }
    return true;
}

static bool EmmV5_StartReceiveDma(EmmV5_Driver *driver)
{
    HAL_StatusTypeDef status;

    driver->rx_event_position = 0U;
    driver->rx_published_count = 0U;
    driver->rx_consumed_count = 0U;
    status = HAL_UARTEx_ReceiveToIdle_DMA(
        driver->uart, driver->rx_dma_buffer,
        EMM_V5_RX_DMA_BUFFER_SIZE);
    driver->receive_armed = status == HAL_OK;
    if (driver->receive_armed) {
        __HAL_DMA_DISABLE_IT(driver->uart->hdmarx, DMA_IT_HT);
    }
    return driver->receive_armed;
}

bool EmmV5_Init(EmmV5_Driver *driver, UART_HandleTypeDef *uart)
{
    if ((driver == NULL) || (uart == NULL)) {
        return false;
    }

    (void) memset(driver, 0, sizeof(*driver));
    driver->uart = uart;
    return EmmV5_StartReceiveDma(driver);
}

void EmmV5_RxEventCallback(EmmV5_Driver *driver, uint16_t position)
{
    uint16_t normalized_position;
    uint16_t previous_position;
    uint32_t received;

    if ((driver == NULL) || !driver->receive_armed ||
        (position > EMM_V5_RX_DMA_BUFFER_SIZE)) {
        return;
    }

    normalized_position =
        (position == EMM_V5_RX_DMA_BUFFER_SIZE) ? 0U : position;
    previous_position = driver->rx_event_position;
    if ((position == EMM_V5_RX_DMA_BUFFER_SIZE) &&
        (previous_position == 0U)) {
        received = (HAL_UARTEx_GetRxEventType(driver->uart) ==
                    HAL_UART_RXEVENT_IDLE) ?
                   0U : EMM_V5_RX_DMA_BUFFER_SIZE;
    } else if (normalized_position >= previous_position) {
        received = (uint32_t) (normalized_position - previous_position);
    } else {
        received = (uint32_t) EMM_V5_RX_DMA_BUFFER_SIZE -
                   previous_position + normalized_position;
    }
    driver->rx_event_position = normalized_position;
    driver->rx_published_count += received;
}

void EmmV5_ProcessRx(EmmV5_Driver *driver)
{
    uint32_t published_count;
    uint32_t available;

    if ((driver == NULL) || (driver->uart == NULL)) {
        return;
    }

    if (driver->rx_restart_requested) {
        driver->rx_restart_requested = false;
        (void) HAL_UART_AbortReceive(driver->uart);
        __HAL_UART_CLEAR_OREFLAG(driver->uart);
        EmmV5_ResetParser(driver);
        driver->response_pending = false;
        driver->expected_length = 0U;
        if (!EmmV5_StartReceiveDma(driver)) {
            driver->statistics.uart_error_count++;
            driver->rx_restart_requested = true;
        }
        return;
    }

    published_count = driver->rx_published_count;
    available = published_count - driver->rx_consumed_count;
    if (available > EMM_V5_RX_DMA_BUFFER_SIZE) {
        driver->statistics.rx_overrun_count++;
        driver->rx_consumed_count =
            published_count - EMM_V5_RX_DMA_BUFFER_SIZE;
        EmmV5_ResetParser(driver);
    }

    while (driver->rx_consumed_count != published_count) {
        uint32_t index = driver->rx_consumed_count %
                         EMM_V5_RX_DMA_BUFFER_SIZE;
        EmmV5_ReceiveByte(driver, driver->rx_dma_buffer[index]);
        driver->rx_consumed_count++;
    }
}

void EmmV5_TxCompleteCallback(EmmV5_Driver *driver)
{
    if (driver != NULL) {
        driver->tx_busy = false;
    }
}

void EmmV5_ErrorCallback(EmmV5_Driver *driver)
{
    if ((driver == NULL) || (driver->uart == NULL)) {
        return;
    }

    driver->statistics.uart_error_count++;
    driver->receive_armed = false;
    driver->rx_restart_requested = true;
}

bool EmmV5_TryGetResponse(EmmV5_Driver *driver,
                          EmmV5_Response *response)
{
    uint8_t index;

    if ((driver == NULL) || (response == NULL)) {
        return false;
    }
    if (!driver->complete) {
        return false;
    }

    response->length = driver->expected_length;
    for (index = 0U; index < response->length; ++index) {
        response->bytes[index] = driver->bytes[index];
    }
    driver->complete = false;
    driver->response_pending = false;
    driver->expected_length = 0U;
    return true;
}

bool EmmV5_ResponsePending(const EmmV5_Driver *driver)
{
    return (driver != NULL) && driver->response_pending;
}

void EmmV5_CancelResponse(EmmV5_Driver *driver, bool timed_out)
{
    if (driver == NULL) {
        return;
    }

    if (timed_out && driver->response_pending) {
        driver->statistics.timeout_count++;
    }
    driver->response_pending = false;
    driver->expected_length = 0U;
    EmmV5_ResetParser(driver);
}

bool EmmV5_WaitResponse(EmmV5_Driver *driver, uint32_t timeout_ms,
                        EmmV5_Response *response)
{
    uint32_t start;

    if ((driver == NULL) || (response == NULL)) {
        return false;
    }

    start = HAL_GetTick();
    while ((uint32_t) (HAL_GetTick() - start) <= timeout_ms) {
        EmmV5_ProcessRx(driver);
        if (EmmV5_TryGetResponse(driver, response)) {
            return true;
        }
        __WFI();
    }

    EmmV5_CancelResponse(driver, true);
    return false;
}

void EmmV5_GetStatistics(EmmV5_Driver *driver,
                         EmmV5_Statistics *statistics)
{
    uint32_t primask;

    if ((driver == NULL) || (statistics == NULL)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *statistics = driver->statistics;
    if (primask == 0U) {
        __enable_irq();
    }
}

bool EmmV5_Enable(EmmV5_Driver *driver, uint8_t address, bool enabled,
                  bool synchronous)
{
    const uint8_t frame[] = {
        address, EMM_V5_COMMAND_ENABLE, 0xABU,
        enabled ? 1U : 0U, synchronous ? 1U : 0U, EMM_V5_CHECK_BYTE
    };
    return EmmV5_SendFrame(
        driver, frame, (uint8_t) sizeof(frame), EMM_V5_ACK_LENGTH);
}

bool EmmV5_SetFastPositionParameters(
    EmmV5_Driver *driver, uint8_t address, uint16_t speed_rpm,
    uint8_t acceleration, EmmV5_MotionMode mode, bool synchronous)
{
    const uint8_t frame[] = {
        address, EMM_V5_COMMAND_FAST_PARAMS,
        (uint8_t) (speed_rpm >> 8U), (uint8_t) speed_rpm,
        acceleration, (uint8_t) mode,
        synchronous ? 1U : 0U, EMM_V5_CHECK_BYTE
    };

    if ((speed_rpm > EMM_V5_MAX_SPEED_RPM) ||
        (mode > EMM_V5_MOVE_RELATIVE_TO_CURRENT)) {
        return false;
    }
    return EmmV5_SendFrame(
        driver, frame, (uint8_t) sizeof(frame), EMM_V5_ACK_LENGTH);
}

bool EmmV5_StartFastPosition(EmmV5_Driver *driver, uint8_t address,
                             int32_t pulses)
{
    const uint32_t raw = (uint32_t) pulses;
    const uint8_t frame[] = {
        address, EMM_V5_COMMAND_FAST_POSITION,
        (uint8_t) (raw >> 24U), (uint8_t) (raw >> 16U),
        (uint8_t) (raw >> 8U), (uint8_t) raw, EMM_V5_CHECK_BYTE
    };
    return EmmV5_SendFrame(
        driver, frame, (uint8_t) sizeof(frame), EMM_V5_ACK_LENGTH);
}

bool EmmV5_StopNow(EmmV5_Driver *driver, uint8_t address,
                   bool synchronous)
{
    const uint8_t frame[] = {
        address, EMM_V5_COMMAND_STOP, 0x98U,
        synchronous ? 1U : 0U, EMM_V5_CHECK_BYTE
    };
    return EmmV5_SendFrame(
        driver, frame, (uint8_t) sizeof(frame), EMM_V5_ACK_LENGTH);
}

bool EmmV5_EmergencyStopBroadcast(EmmV5_Driver *driver)
{
    const uint8_t frame[] = {
        0U, EMM_V5_COMMAND_STOP, 0x98U, 0U, EMM_V5_CHECK_BYTE
    };

    if ((driver == NULL) || (driver->uart == NULL)) {
        return false;
    }

    if (driver->tx_busy) {
        (void) HAL_UART_AbortTransmit(driver->uart);
        driver->tx_busy = false;
    }
    EmmV5_CancelResponse(driver, false);
    (void) memcpy(driver->tx_buffer, frame, sizeof(frame));
    driver->tx_busy = true;
    if (HAL_UART_Transmit_DMA(driver->uart, driver->tx_buffer,
                              (uint16_t) sizeof(frame)) == HAL_OK) {
        return true;
    }

    driver->tx_busy = false;
    return HAL_UART_Transmit(driver->uart, driver->tx_buffer,
                             (uint16_t) sizeof(frame), 10U) == HAL_OK;
}

bool EmmV5_ReadVersion(EmmV5_Driver *driver, uint8_t address)
{
    const uint8_t frame[] = {
        address, EMM_V5_COMMAND_VERSION, EMM_V5_CHECK_BYTE
    };
    return EmmV5_SendFrame(driver, frame, (uint8_t) sizeof(frame),
                           EMM_V5_VERSION_RESPONSE_LENGTH);
}

bool EmmV5_ReadCurrentPosition(EmmV5_Driver *driver, uint8_t address)
{
    const uint8_t frame[] = {
        address, EMM_V5_COMMAND_CURRENT_POSITION, EMM_V5_CHECK_BYTE
    };
    return EmmV5_SendFrame(driver, frame, (uint8_t) sizeof(frame),
                           EMM_V5_POSITION_RESPONSE_LENGTH);
}

bool EmmV5_ReadCurrentSpeed(EmmV5_Driver *driver, uint8_t address)
{
    const uint8_t frame[] = {
        address, EMM_V5_COMMAND_CURRENT_SPEED, EMM_V5_CHECK_BYTE
    };
    return EmmV5_SendFrame(driver, frame, (uint8_t) sizeof(frame),
                           EMM_V5_SPEED_RESPONSE_LENGTH);
}

bool EmmV5_ReadStatus(EmmV5_Driver *driver, uint8_t address)
{
    const uint8_t frame[] = {
        address, EMM_V5_COMMAND_STATUS, EMM_V5_CHECK_BYTE
    };
    return EmmV5_SendFrame(
        driver, frame, (uint8_t) sizeof(frame), EMM_V5_ACK_LENGTH);
}

bool EmmV5_ResponseIsAccepted(const EmmV5_Response *response)
{
    return (response != NULL) &&
           (response->length == EMM_V5_ACK_LENGTH) &&
           ((response->bytes[2] == EMM_V5_RESPONSE_OK) ||
            (response->bytes[2] == EMM_V5_RESPONSE_REACHED)) &&
           (response->bytes[3] == EMM_V5_CHECK_BYTE);
}

bool EmmV5_DecodeVersion(const EmmV5_Response *response,
                         EmmV5_Version *version)
{
    if ((response == NULL) || (version == NULL) ||
        (response->length != EMM_V5_VERSION_RESPONSE_LENGTH) ||
        (response->bytes[1] != EMM_V5_COMMAND_VERSION) ||
        (response->bytes[6] != EMM_V5_CHECK_BYTE)) {
        return false;
    }

    version->firmware_version =
        ((uint16_t) response->bytes[2] << 8U) | response->bytes[3];
    version->hardware_series = response->bytes[4] >> 4U;
    version->hardware_type = response->bytes[4] & 0x0FU;
    version->hardware_version = response->bytes[5];
    return true;
}

bool EmmV5_DecodeCurrentPosition(const EmmV5_Response *response,
                                 int64_t *position_units)
{
    uint32_t magnitude;

    if ((response == NULL) || (position_units == NULL) ||
        (response->length != EMM_V5_POSITION_RESPONSE_LENGTH) ||
        (response->bytes[1] != EMM_V5_COMMAND_CURRENT_POSITION) ||
        (response->bytes[2] > 1U) ||
        (response->bytes[7] != EMM_V5_CHECK_BYTE)) {
        return false;
    }

    magnitude = ((uint32_t) response->bytes[3] << 24U) |
                ((uint32_t) response->bytes[4] << 16U) |
                ((uint32_t) response->bytes[5] << 8U) |
                (uint32_t) response->bytes[6];
    *position_units = (response->bytes[2] == 0U) ?
        (int64_t) magnitude : -(int64_t) magnitude;
    return true;
}

int64_t EmmV5_PositionUnitsToMillidegrees(int64_t position_units)
{
    return (position_units * 360000LL) / 65536LL;
}

bool EmmV5_DecodeCurrentSpeed(const EmmV5_Response *response,
                              int16_t *speed_rpm)
{
    uint16_t magnitude;

    if ((response == NULL) || (speed_rpm == NULL) ||
        (response->length != EMM_V5_SPEED_RESPONSE_LENGTH) ||
        (response->bytes[1] != EMM_V5_COMMAND_CURRENT_SPEED) ||
        (response->bytes[2] > 1U) ||
        (response->bytes[5] != EMM_V5_CHECK_BYTE)) {
        return false;
    }

    magnitude = ((uint16_t) response->bytes[3] << 8U) |
                response->bytes[4];
    if (magnitude > EMM_V5_MAX_SPEED_RPM) {
        return false;
    }
    *speed_rpm = (response->bytes[2] == 0U) ?
        (int16_t) magnitude : -(int16_t) magnitude;
    return true;
}

bool EmmV5_DecodeStatus(const EmmV5_Response *response,
                        uint8_t *status_flags)
{
    if ((response == NULL) || (status_flags == NULL) ||
        (response->length != EMM_V5_ACK_LENGTH) ||
        (response->bytes[1] != EMM_V5_COMMAND_STATUS) ||
        (response->bytes[3] != EMM_V5_CHECK_BYTE)) {
        return false;
    }
    *status_flags = response->bytes[2];
    return true;
}
