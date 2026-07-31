#include "jy61p_stm32.h"

#include <stddef.h>
#include <string.h>

#define JY61P_FRAME_HEADER          0x55U
#define JY61P_FRAME_LENGTH          11U
#define JY61P_FRAME_ACCELERATION    0x51U
#define JY61P_FRAME_GYROSCOPE       0x52U
#define JY61P_FRAME_ANGLE           0x53U
#define JY61P_FRAME_QUATERNION      0x59U
#define JY61P_REGISTER_CALIBRATION  0x01U
#define JY61P_REGISTER_UNLOCK       0x69U
#define JY61P_UNLOCK_VALUE          0xB588U
#define JY61P_CALIBRATE_YAW_ZERO    0x0004U

static bool JY61P_StartReceiveDma(JY61P_Driver *driver)
{
    HAL_StatusTypeDef status;

    driver->rx_event_position = 0U;
    driver->rx_published_count = 0U;
    driver->rx_consumed_count = 0U;
    status = HAL_UARTEx_ReceiveToIdle_DMA(
        driver->uart, driver->rx_dma_buffer,
        JY61P_RX_DMA_BUFFER_SIZE);
    driver->receive_armed = status == HAL_OK;
    if (driver->receive_armed) {
        __HAL_DMA_DISABLE_IT(driver->uart->hdmarx, DMA_IT_HT);
    }
    return driver->receive_armed;
}

static int16_t JY61P_ReadInt16(const uint8_t *bytes)
{
    return (int16_t) ((uint16_t) bytes[0] |
                     ((uint16_t) bytes[1] << 8U));
}

static bool JY61P_ChecksumIsValid(const uint8_t *frame)
{
    uint8_t checksum = 0U;
    uint8_t index;

    for (index = 0U; index < (JY61P_FRAME_LENGTH - 1U); ++index) {
        checksum = (uint8_t) (checksum + frame[index]);
    }
    return checksum == frame[JY61P_FRAME_LENGTH - 1U];
}

static void JY61P_ParseFrame(JY61P_Driver *driver, const uint8_t *frame)
{
    uint32_t now_ms;

    if (!JY61P_ChecksumIsValid(frame)) {
        driver->checksum_error_count++;
        return;
    }

    switch (frame[1]) {
        case JY61P_FRAME_ACCELERATION:
            now_ms = HAL_GetTick();
            driver->acceleration[0] = JY61P_ReadInt16(&frame[2]);
            driver->acceleration[1] = JY61P_ReadInt16(&frame[4]);
            driver->acceleration[2] = JY61P_ReadInt16(&frame[6]);
            driver->temperature = JY61P_ReadInt16(&frame[8]);
            driver->valid_flags |= JY61P_DATA_ACCELERATION;
            driver->last_acceleration_frame_ms = now_ms;
            break;
        case JY61P_FRAME_GYROSCOPE:
            driver->angular_velocity[0] = JY61P_ReadInt16(&frame[2]);
            driver->angular_velocity[1] = JY61P_ReadInt16(&frame[4]);
            driver->angular_velocity[2] = JY61P_ReadInt16(&frame[6]);
            driver->temperature = JY61P_ReadInt16(&frame[8]);
            driver->valid_flags |= JY61P_DATA_GYROSCOPE;
            break;
        case JY61P_FRAME_ANGLE:
            driver->angle[0] = JY61P_ReadInt16(&frame[2]);
            driver->angle[1] = JY61P_ReadInt16(&frame[4]);
            driver->angle[2] = JY61P_ReadInt16(&frame[6]);
            driver->valid_flags |= JY61P_DATA_ANGLE;
            break;
        case JY61P_FRAME_QUATERNION:
            driver->quaternion[0] = JY61P_ReadInt16(&frame[2]);
            driver->quaternion[1] = JY61P_ReadInt16(&frame[4]);
            driver->quaternion[2] = JY61P_ReadInt16(&frame[6]);
            driver->quaternion[3] = JY61P_ReadInt16(&frame[8]);
            driver->valid_flags |= JY61P_DATA_QUATERNION;
            break;
        default:
            return;
    }
    driver->last_valid_frame_ms = HAL_GetTick();
    driver->valid_frame_count++;
}

static void JY61P_ProcessByte(JY61P_Driver *driver, uint8_t byte)
{
    driver->received_byte_count++;
    driver->last_received_byte = byte;
    if (byte == JY61P_FRAME_HEADER) {
        driver->header_byte_count++;
    }

    if (driver->frame_index == 0U) {
        if (byte == JY61P_FRAME_HEADER) {
            driver->frame[driver->frame_index++] = byte;
        }
        return;
    }

    driver->frame[driver->frame_index++] = byte;
    if (driver->frame_index >= JY61P_FRAME_LENGTH) {
        JY61P_ParseFrame(driver, driver->frame);
        driver->frame_index = 0U;
    }
}

bool JY61P_Init(JY61P_Driver *driver, UART_HandleTypeDef *uart)
{
    if ((driver == NULL) || (uart == NULL)) {
        return false;
    }
    (void) memset(driver, 0, sizeof(*driver));
    driver->uart = uart;
    return JY61P_StartReceiveDma(driver);
}

void JY61P_RxEventCallback(JY61P_Driver *driver, uint16_t position)
{
    uint16_t normalized_position;
    uint16_t previous_position;
    uint32_t received;

    if ((driver == NULL) || !driver->receive_armed ||
        (position > JY61P_RX_DMA_BUFFER_SIZE)) {
        return;
    }

    normalized_position =
        (position == JY61P_RX_DMA_BUFFER_SIZE) ? 0U : position;
    previous_position = driver->rx_event_position;
    if ((position == JY61P_RX_DMA_BUFFER_SIZE) &&
        (previous_position == 0U)) {
        received = (HAL_UARTEx_GetRxEventType(driver->uart) ==
                    HAL_UART_RXEVENT_IDLE) ?
                   0U : JY61P_RX_DMA_BUFFER_SIZE;
    } else if (normalized_position >= previous_position) {
        received = (uint32_t) (normalized_position - previous_position);
    } else {
        received = (uint32_t) JY61P_RX_DMA_BUFFER_SIZE -
                   previous_position + normalized_position;
    }
    driver->rx_event_position = normalized_position;
    driver->rx_published_count += received;
}

void JY61P_ProcessRx(JY61P_Driver *driver)
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
        driver->frame_index = 0U;
        if (!JY61P_StartReceiveDma(driver)) {
            driver->uart_error_count++;
            driver->rx_restart_requested = true;
        }
        return;
    }

    published_count = driver->rx_published_count;
    available = published_count - driver->rx_consumed_count;
    if (available > JY61P_RX_DMA_BUFFER_SIZE) {
        driver->rx_overrun_count++;
        driver->rx_consumed_count =
            published_count - JY61P_RX_DMA_BUFFER_SIZE;
        driver->frame_index = 0U;
    }

    while (driver->rx_consumed_count != published_count) {
        uint32_t index = driver->rx_consumed_count %
                         JY61P_RX_DMA_BUFFER_SIZE;
        JY61P_ProcessByte(driver, driver->rx_dma_buffer[index]);
        driver->rx_consumed_count++;
    }
}

void JY61P_ErrorCallback(JY61P_Driver *driver)
{
    if ((driver == NULL) || (driver->uart == NULL)) {
        return;
    }

    JY61P_RecordUartError(driver);
    driver->receive_armed = false;
    driver->rx_restart_requested = true;
}

void JY61P_FeedBytes(JY61P_Driver *driver, const uint8_t *data,
                     uint16_t length)
{
    uint16_t index;

    if ((driver == NULL) || ((data == NULL) && (length != 0U))) {
        return;
    }
    for (index = 0U; index < length; ++index) {
        JY61P_ProcessByte(driver, data[index]);
    }
}

void JY61P_RecordUartError(JY61P_Driver *driver)
{
    if (driver != NULL) {
        driver->uart_error_count++;
        driver->frame_index = 0U;
    }
}

bool JY61P_GetData(JY61P_Driver *driver, JY61P_Data *data)
{
    int16_t acceleration[3];
    int16_t angular_velocity[3];
    int16_t angle[3];
    int16_t quaternion[4];
    int16_t temperature;
    uint32_t valid_flags;
    uint32_t primask;
    uint8_t index;

    if ((driver == NULL) || (data == NULL)) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    for (index = 0U; index < 3U; ++index) {
        acceleration[index] = driver->acceleration[index];
        angular_velocity[index] = driver->angular_velocity[index];
        angle[index] = driver->angle[index];
    }
    for (index = 0U; index < 4U; ++index) {
        quaternion[index] = driver->quaternion[index];
    }
    temperature = driver->temperature;
    valid_flags = driver->valid_flags;
    data->received_byte_count = driver->received_byte_count;
    data->header_byte_count = driver->header_byte_count;
    data->valid_frame_count = driver->valid_frame_count;
    data->checksum_error_count = driver->checksum_error_count;
    data->uart_error_count = driver->uart_error_count;
    data->rx_overrun_count = driver->rx_overrun_count;
    data->last_valid_frame_ms = driver->last_valid_frame_ms;
    data->last_acceleration_frame_ms =
        driver->last_acceleration_frame_ms;
    data->last_received_byte = driver->last_received_byte;
    data->receive_armed = driver->receive_armed;
    if (primask == 0U) {
        __enable_irq();
    }

    for (index = 0U; index < 3U; ++index) {
        data->acceleration_g[index] =
            ((float) acceleration[index] / 32768.0f) * 16.0f;
        data->angular_velocity_dps[index] =
            ((float) angular_velocity[index] / 32768.0f) * 2000.0f;
    }
    data->roll_deg = ((float) angle[0] / 32768.0f) * 180.0f;
    data->pitch_deg = ((float) angle[1] / 32768.0f) * 180.0f;
    data->yaw_deg = ((float) angle[2] / 32768.0f) * 180.0f;
    data->temperature_c = (float) temperature / 100.0f;
    for (index = 0U; index < 4U; ++index) {
        data->quaternion[index] = (float) quaternion[index] / 32768.0f;
    }
    data->valid_flags = valid_flags;
    return valid_flags != 0U;
}

bool JY61P_WriteRegister(JY61P_Driver *driver, uint8_t register_address,
                         uint16_t value)
{
    const uint8_t command[5] = {
        0xFFU, 0xAAU, register_address,
        (uint8_t) value, (uint8_t) (value >> 8U)
    };

    if ((driver == NULL) || (driver->uart == NULL)) {
        return false;
    }
    return HAL_UART_Transmit(driver->uart, (uint8_t *) command,
                             (uint16_t) sizeof(command), 100U) == HAL_OK;
}

bool JY61P_ZeroYaw(JY61P_Driver *driver)
{
    if (!JY61P_WriteRegister(
            driver, JY61P_REGISTER_UNLOCK, JY61P_UNLOCK_VALUE)) {
        return false;
    }
    HAL_Delay(200U);
    if (!JY61P_WriteRegister(
            driver, JY61P_REGISTER_CALIBRATION,
            JY61P_CALIBRATE_YAW_ZERO)) {
        return false;
    }
    HAL_Delay(500U);
    return true;
}
