#include "ball_control_app.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#include "emm_v5.h"
#include "main.h"
#include "oled_ssd1306.h"
#include "single_axis_gyro_stm32f407.h"

#define GYRO_RATE_FULL_SCALE_DPS  2000.0f
#define GYRO_DATA_TIMEOUT_MS      1000U
#define OLED_UPDATE_PERIOD_MS     100U
#define LED_TOGGLE_PERIOD_MS      500U
#define KEY_DEBOUNCE_MS           30U

static EmmV5_Driver g_motor;
static OledSsd1306 g_oled;
static SingleAxisGyro_STM32F407_t g_gyro;
static bool g_motor_initialized;
static bool g_gyro_initialized;
static bool g_estop_key_raw_pressed;
static bool g_estop_key_stable_pressed;
static uint32_t g_estop_key_change_ms;
static uint32_t g_next_oled_update_ms;
static uint32_t g_next_led_toggle_ms;

static int32_t FloatToMilli(float value)
{
    float scaled = value * 1000.0f;

    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    return (int32_t) scaled;
}

static void FormatMilli(int32_t value_milli, char *buffer, size_t capacity)
{
    bool negative;
    uint32_t magnitude;

    if ((buffer == NULL) || (capacity == 0U)) {
        return;
    }

    negative = value_milli < 0;
    magnitude = negative ?
        (uint32_t) (-(value_milli + 1)) + 1U :
        (uint32_t) value_milli;
    if (negative) {
        (void) snprintf(buffer, capacity, "-%" PRIu32 ".%03" PRIu32,
            magnitude / 1000U, magnitude % 1000U);
    } else {
        (void) snprintf(buffer, capacity, "%" PRIu32 ".%03" PRIu32,
            magnitude / 1000U, magnitude % 1000U);
    }
}

static const char *GyroLinkState(
    const SingleAxisGyro_Sample_t *sample,
    const SingleAxisGyro_STM32F407_Statistics_t *statistics,
    uint32_t now_ms)
{
    if (!g_gyro_initialized || !statistics->receive_armed) {
        return "UART ERROR";
    }
    if (sample->valid_frame_count == 0U) {
        return "WAIT DATA";
    }
    if ((uint32_t) (now_ms - statistics->last_valid_frame_ms) >=
        GYRO_DATA_TIMEOUT_MS) {
        return "DATA STALE";
    }
    return "DATA OK";
}

static void Oled_Update(uint32_t now_ms)
{
    SingleAxisGyro_Sample_t sample = {0};
    SingleAxisGyro_STM32F407_Statistics_t statistics = {0};
    char line[24];
    char value[12];

    if (!g_oled.online) {
        return;
    }

    (void) SingleAxisGyro_STM32F407_GetSample(&g_gyro, &sample);
    SingleAxisGyro_STM32F407_GetStatistics(&g_gyro, &statistics);

    OledSsd1306_Clear(&g_oled);
    OledSsd1306_DrawText(
        &g_oled, 0U, 0U, "GYRO USART3 TEST", &afont8x6);
    (void) snprintf(line, sizeof(line), "LINK:%s",
        GyroLinkState(&sample, &statistics, now_ms));
    OledSsd1306_DrawText(&g_oled, 0U, 8U, line, &afont8x6);

    if ((sample.valid_mask & SINGLE_AXIS_GYRO_VALID_YAW) != 0U) {
        FormatMilli(FloatToMilli(sample.yaw_deg), value, sizeof(value));
        (void) snprintf(line, sizeof(line), "YAW:%s deg", value);
    } else {
        (void) snprintf(line, sizeof(line), "YAW:--- deg");
    }
    OledSsd1306_DrawText(&g_oled, 0U, 16U, line, &afont8x6);

    if ((sample.valid_mask & SINGLE_AXIS_GYRO_VALID_RATE) != 0U) {
        FormatMilli(
            FloatToMilli(sample.angular_rate_dps), value, sizeof(value));
        (void) snprintf(line, sizeof(line), "RATE:%s dps", value);
    } else {
        (void) snprintf(line, sizeof(line), "RATE:--- dps");
    }
    OledSsd1306_DrawText(&g_oled, 0U, 24U, line, &afont8x6);

    (void) snprintf(line, sizeof(line), "RAW Y:%d W:%d",
        sample.raw_yaw, sample.raw_angular_rate);
    OledSsd1306_DrawText(&g_oled, 0U, 32U, line, &afont8x6);
    (void) snprintf(line, sizeof(line), "FRAME:%" PRIu32,
        sample.valid_frame_count);
    OledSsd1306_DrawText(&g_oled, 0U, 40U, line, &afont8x6);
    (void) snprintf(line, sizeof(line), "SUM ERR:%" PRIu32,
        sample.checksum_error_count);
    OledSsd1306_DrawText(&g_oled, 0U, 48U, line, &afont8x6);
    (void) snprintf(line, sizeof(line), "M2:%s U3E:%" PRIu32,
        g_motor_initialized ? "OK" : "ERR",
        statistics.uart_error_count);
    OledSsd1306_DrawText(&g_oled, 0U, 56U, line, &afont8x6);
    (void) OledSsd1306_RefreshAsync(&g_oled);
}

void BallControl_Init(void)
{
    uint32_t now_ms;

    (void) OledSsd1306_Init(&g_oled, &hi2c1);
    g_motor_initialized = EmmV5_Init(&g_motor, &huart2);
    g_gyro_initialized = SingleAxisGyro_STM32F407_Init(
        &g_gyro, &huart3, GYRO_RATE_FULL_SCALE_DPS);

    now_ms = HAL_GetTick();
    g_next_oled_update_ms = now_ms;
    g_next_led_toggle_ms = now_ms;
    g_estop_key_raw_pressed =
        HAL_GPIO_ReadPin(CORE_USER_KEY_GPIO_PORT,
            CORE_USER_KEY_PIN) == GPIO_PIN_SET;
    g_estop_key_stable_pressed = g_estop_key_raw_pressed;
    g_estop_key_change_ms = now_ms;
}

void BallControl_Process(void)
{
    uint32_t now_ms = HAL_GetTick();
    bool estop_key_pressed =
        HAL_GPIO_ReadPin(CORE_USER_KEY_GPIO_PORT,
            CORE_USER_KEY_PIN) == GPIO_PIN_SET;

    EmmV5_ProcessRx(&g_motor);
    SingleAxisGyro_STM32F407_Process(&g_gyro);

    if (estop_key_pressed != g_estop_key_raw_pressed) {
        g_estop_key_raw_pressed = estop_key_pressed;
        g_estop_key_change_ms = now_ms;
    }
    if ((g_estop_key_stable_pressed != g_estop_key_raw_pressed) &&
        ((uint32_t) (now_ms - g_estop_key_change_ms) >=
         KEY_DEBOUNCE_MS)) {
        g_estop_key_stable_pressed = g_estop_key_raw_pressed;
        if (g_estop_key_stable_pressed && g_motor_initialized) {
            (void) EmmV5_EmergencyStopBroadcast(&g_motor);
        }
    }

    if ((int32_t) (now_ms - g_next_oled_update_ms) >= 0) {
        Oled_Update(now_ms);
        g_next_oled_update_ms = now_ms + OLED_UPDATE_PERIOD_MS;
    }
    if ((int32_t) (now_ms - g_next_led_toggle_ms) >= 0) {
        HAL_GPIO_TogglePin(CORE_LED_GPIO_PORT, CORE_LED_PIN);
        g_next_led_toggle_ms = now_ms + LED_TOGGLE_PERIOD_MS;
    }

    __WFI();
}

void BallControl_Timer1kHzCallback(void)
{
}

bool BallControl_ApplyControlConfig(const BallControl_Config *config)
{
    (void) config;
    return false;
}

bool BallControl_RequestControlEnable(bool enabled)
{
    return !enabled;
}

void BallControl_SetControlSetpoint(float setpoint_mm)
{
    (void) setpoint_mm;
}

void BallControl_GetControlStatus(BallControl_Status *status)
{
    if (status != NULL) {
        BallControl_Status empty = {0};
        *status = empty;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if (uart == &huart2) {
        EmmV5_RxEventCallback(&g_motor, size);
    } else if (uart == &huart3) {
        SingleAxisGyro_STM32F407_RxEventCallback(&g_gyro, size);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart2) {
        EmmV5_TxCompleteCallback(&g_motor);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart2) {
        EmmV5_ErrorCallback(&g_motor);
    } else if (uart == &huart3) {
        SingleAxisGyro_STM32F407_ErrorCallback(&g_gyro);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *i2c)
{
    OledSsd1306_TxCompleteCallback(&g_oled, i2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *i2c)
{
    OledSsd1306_ErrorCallback(&g_oled, i2c);
}

void BallControl_UsbCdcReceive(const uint8_t *data, uint32_t length)
{
    (void) data;
    (void) length;
}

void BallControl_UsbCdcSetConnected(bool connected)
{
    (void) connected;
}
