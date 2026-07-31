#include "ball_control_app.h"

#include <inttypes.h>
#include <limits.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "debug_console_stm32.h"
#include "emm_v5.h"
#include "jy61p_stm32.h"
#include "main.h"
#include "oled_ssd1306.h"
#include "raspberry_pi_vision_stm32f407.h"
#include "single_axis_gyro_stm32f407.h"

#define MOTOR_ADDRESS                1U
#define MOTOR_RESPONSE_TIMEOUT_MS    150U
#define MOTOR_TEST_SPEED_RPM         30U
#define MOTOR_TEST_ACCELERATION      10U
#define MOTOR_TEST_STEP_PULSES       256L
#define OLED_UPDATE_PERIOD_MS        200U
#define LED_TOGGLE_PERIOD_MS         500U
#define KEY_DEBOUNCE_MS              30U
#define VISION_HEARTBEAT_TIMEOUT_MS  1500U
#define GYRO_RATE_FULL_SCALE_DPS     2000.0f
#define GYRO_RATE_FULL_SCALE_TENTHS  20000L
#define CONTROL_ESTIMATOR_TICKS      5U
#define CONTROL_OUTPUT_TICKS         10U
#define CONTROL_MAX_TICK_BACKLOG     20U
#define CONTROL_ESTIMATOR_DT_S       0.005f
#define CONTROL_OUTPUT_DT_S          0.010f

typedef enum {
    STARTUP_STAGE_BOARD_FAULT = 0,
    STARTUP_STAGE_WAIT_USB,
    STARTUP_STAGE_WAIT_PROTOCOL,
    STARTUP_STAGE_LINK_LOST,
    STARTUP_STAGE_WAIT_TARGET,
    STARTUP_STAGE_WAIT_HEARTBEAT,
    STARTUP_STAGE_PI_STARTING,
    STARTUP_STAGE_WAIT_WIFI,
    STARTUP_STAGE_CAMERA_FAULT,
    STARTUP_STAGE_INFERENCE_FAULT,
    STARTUP_STAGE_WAIT_PI_CONFIRM,
    STARTUP_STAGE_VISION_FAULT,
    STARTUP_STAGE_READY
} StartupStage;

typedef enum {
    CONTROL_MOTOR_IDLE = 0,
    CONTROL_MOTOR_WAIT_ENABLE,
    CONTROL_MOTOR_SEND_PARAMETERS,
    CONTROL_MOTOR_WAIT_PARAMETERS,
    CONTROL_MOTOR_READY,
    CONTROL_MOTOR_WAIT_POSITION,
    CONTROL_MOTOR_FAULT
} ControlMotorState;

static DebugConsole g_debug;
static EmmV5_Driver g_motor;
static JY61P_Driver g_imu;
static OledSsd1306 g_oled;
static RaspberryPiVision_STM32F407_Receiver_t g_vision;
static SingleAxisGyro_STM32F407_t g_angle_gyro;
static BallControl_Core g_control_core;
static BallControl_Output g_control_output;

static uint32_t g_next_oled_update_ms;
static uint32_t g_next_led_toggle_ms;
static bool g_estop_key_raw_pressed;
static bool g_estop_key_stable_pressed;
static uint32_t g_estop_key_change_ms;
static bool g_motor_initialized;
static bool g_imu_initialized;
static bool g_angle_gyro_initialized;
static bool g_debug_initialized;
static StartupStage g_startup_stage;
static volatile uint32_t g_control_tick_produced;
static uint32_t g_control_tick_consumed;
static uint32_t g_control_scheduler_overrun_count;
static bool g_control_target_sequence_valid;
static uint16_t g_control_last_target_sequence;
static bool g_control_output_valid;
static ControlMotorState g_control_motor_state;
static uint32_t g_control_motor_deadline_ms;
static uint32_t g_control_motor_fault_count;
static int32_t g_control_pending_motor_target;
static int32_t g_control_last_motor_target;
static bool g_control_last_motor_target_valid;

static void Control_Disarm(bool fault, bool force_stop);
static void Control_EnterFault(const char *reason);
static void Control_UpdateVision(uint32_t now_ms);
static void Control_ProcessTicks(uint32_t now_ms);
static void Control_MotorProcess(uint32_t now_ms);

static void Debug_Printf(const char *format, ...)
{
    char buffer[256];
    va_list arguments;
    int length;

    if (!g_debug_initialized) {
        return;
    }

    va_start(arguments, format);
    length = vsnprintf(buffer, sizeof(buffer), format, arguments);
    va_end(arguments);

    if (length <= 0) {
        return;
    }
    if ((size_t) length >= sizeof(buffer)) {
        length = (int) sizeof(buffer) - 1;
    }
    (void) DebugConsole_Write(
        &g_debug, (const uint8_t *) buffer, (uint32_t) length);
}

static void Format_Int64(int64_t value, char *buffer, size_t capacity)
{
    char reversed[20];
    uint64_t magnitude;
    size_t digit_count = 0U;
    size_t output = 0U;

    if ((buffer == NULL) || (capacity == 0U)) {
        return;
    }

    if (value < 0) {
        magnitude = (uint64_t) (-(value + 1)) + 1U;
        if (output + 1U < capacity) {
            buffer[output++] = '-';
        }
    } else {
        magnitude = (uint64_t) value;
    }

    do {
        reversed[digit_count++] =
            (char) ('0' + (char) (magnitude % 10U));
        magnitude /= 10U;
    } while ((magnitude != 0U) &&
             (digit_count < sizeof(reversed)));

    while ((digit_count != 0U) && (output + 1U < capacity)) {
        buffer[output++] = reversed[--digit_count];
    }
    buffer[output] = '\0';
}

static int32_t ScaleRawToTenths(int16_t raw, int32_t full_scale_tenths)
{
    int32_t scaled = (int32_t) raw * full_scale_tenths;

    scaled += (scaled >= 0) ? 16384L : -16384L;
    return scaled / 32768L;
}

static void Format_Tenths(int32_t value, char *buffer, size_t capacity)
{
    bool negative;
    uint32_t magnitude;

    if ((buffer == NULL) || (capacity == 0U)) {
        return;
    }

    negative = value < 0;
    magnitude = negative ?
        (uint32_t) (-(value + 1)) + 1U : (uint32_t) value;
    (void) snprintf(buffer, capacity, "%s%" PRIu32 ".%" PRIu32,
                    negative ? "-" : "", magnitude / 10U,
                    magnitude % 10U);
}

static bool Motor_WaitAccepted(const char *operation)
{
    EmmV5_Response response;

    if (!EmmV5_WaitResponse(
            &g_motor, MOTOR_RESPONSE_TIMEOUT_MS, &response)) {
        Debug_Printf("%s: timeout\r\n", operation);
        return false;
    }
    if (!EmmV5_ResponseIsAccepted(&response)) {
        Debug_Printf("%s: rejected, code=0x%02X\r\n",
                     operation, response.bytes[2]);
        return false;
    }
    return true;
}

static void Motor_ReadVersion(void)
{
    EmmV5_Response response;
    EmmV5_Version version;

    if (!EmmV5_ReadVersion(&g_motor, MOTOR_ADDRESS) ||
        !EmmV5_WaitResponse(
            &g_motor, MOTOR_RESPONSE_TIMEOUT_MS, &response) ||
        !EmmV5_DecodeVersion(&response, &version)) {
        Debug_Printf("motor %u version: timeout/invalid\r\n",
                     MOTOR_ADDRESS);
        return;
    }
    Debug_Printf("motor %u fw=%u hw=%u.%u.%u\r\n",
                 MOTOR_ADDRESS, version.firmware_version,
                 version.hardware_series, version.hardware_type,
                 version.hardware_version);
}

static void Motor_ReadStatus(void)
{
    EmmV5_Response response;
    uint8_t status;

    if (!EmmV5_ReadStatus(&g_motor, MOTOR_ADDRESS) ||
        !EmmV5_WaitResponse(
            &g_motor, MOTOR_RESPONSE_TIMEOUT_MS, &response) ||
        !EmmV5_DecodeStatus(&response, &status)) {
        Debug_Printf("motor %u status: timeout/invalid\r\n",
                     MOTOR_ADDRESS);
        return;
    }
    Debug_Printf("motor %u status=0x%02X\r\n",
                 MOTOR_ADDRESS, status);
}

static void Motor_ReadPosition(void)
{
    EmmV5_Response response;
    int64_t position;
    char position_text[24];
    char millidegree_text[24];

    if (!EmmV5_ReadCurrentPosition(
            &g_motor, MOTOR_ADDRESS) ||
        !EmmV5_WaitResponse(
            &g_motor, MOTOR_RESPONSE_TIMEOUT_MS, &response) ||
        !EmmV5_DecodeCurrentPosition(&response, &position)) {
        Debug_Printf("motor %u position: timeout/invalid\r\n",
                     MOTOR_ADDRESS);
        return;
    }
    Format_Int64(position, position_text, sizeof(position_text));
    Format_Int64(EmmV5_PositionUnitsToMillidegrees(position),
                 millidegree_text, sizeof(millidegree_text));
    Debug_Printf("motor %u position=%s units (%s mdeg)\r\n",
                 MOTOR_ADDRESS, position_text,
                 millidegree_text);
}

static void Motor_ReadSpeed(void)
{
    EmmV5_Response response;
    int16_t speed;

    if (!EmmV5_ReadCurrentSpeed(&g_motor, MOTOR_ADDRESS) ||
        !EmmV5_WaitResponse(
            &g_motor, MOTOR_RESPONSE_TIMEOUT_MS, &response) ||
        !EmmV5_DecodeCurrentSpeed(&response, &speed)) {
        Debug_Printf("motor %u speed: timeout/invalid\r\n",
                     MOTOR_ADDRESS);
        return;
    }
    Debug_Printf("motor %u speed=%d rpm\r\n",
                 MOTOR_ADDRESS, speed);
}

static void Motor_CommunicationTest(void)
{
    EmmV5_Statistics before;
    EmmV5_Statistics after;

    EmmV5_GetStatistics(&g_motor, &before);
    Debug_Printf("motor %u read-only test\r\n",
                 MOTOR_ADDRESS);
    Motor_ReadVersion();
    Motor_ReadStatus();
    Motor_ReadPosition();
    Motor_ReadSpeed();
    EmmV5_GetStatistics(&g_motor, &after);
    Debug_Printf(
        "motor RX delta: valid=%" PRIu32 " invalid=%" PRIu32
        " unexpected=%" PRIu32 " timeout=%" PRIu32
        " uart_error=%" PRIu32 "\r\n",
        after.valid_frame_count - before.valid_frame_count,
        after.invalid_frame_count - before.invalid_frame_count,
        after.unexpected_byte_count - before.unexpected_byte_count,
        after.timeout_count - before.timeout_count,
        after.uart_error_count - before.uart_error_count);
}

static void Motor_SetEnabled(bool enabled)
{
    if (EmmV5_Enable(
            &g_motor, MOTOR_ADDRESS, enabled, false) &&
        Motor_WaitAccepted(enabled ? "enable" : "disable")) {
        Debug_Printf("motor %u %s\r\n", MOTOR_ADDRESS,
                     enabled ? "enabled" : "disabled");
    }
}

static bool Control_ManualMotorCommandAllowed(void)
{
    BallControl_Status status;

    BallControlCore_GetStatus(&g_control_core, &status);
    if (status.enable_requested ||
        ((g_control_motor_state != CONTROL_MOTOR_IDLE) &&
         (g_control_motor_state != CONTROL_MOTOR_FAULT))) {
        Debug_Printf("manual motor command rejected: control active\r\n");
        return false;
    }
    return true;
}

static void Motor_MoveSmallStep(int32_t pulses)
{
    if (!EmmV5_Enable(
            &g_motor, MOTOR_ADDRESS, true, false) ||
        !Motor_WaitAccepted("enable")) {
        return;
    }
    if (!EmmV5_SetFastPositionParameters(
            &g_motor, MOTOR_ADDRESS, MOTOR_TEST_SPEED_RPM,
            MOTOR_TEST_ACCELERATION,
            EMM_V5_MOVE_RELATIVE_TO_CURRENT, false) ||
        !Motor_WaitAccepted("set fast-position parameters")) {
        return;
    }
    if (!EmmV5_StartFastPosition(
            &g_motor, MOTOR_ADDRESS, pulses) ||
        !Motor_WaitAccepted("start fast-position")) {
        return;
    }
    Debug_Printf(
        "motor %u move accepted: %" PRId32
        " pulses, %u rpm, accel=%u\r\n",
        MOTOR_ADDRESS, pulses, MOTOR_TEST_SPEED_RPM,
        MOTOR_TEST_ACCELERATION);
}

static const char *Vision_StateName(RPV_TargetState_t state)
{
    switch (state) {
        case RPV_TARGET_STATE_NO_DATA:
            return "NO DATA";
        case RPV_TARGET_STATE_LINK_LOST:
            return "LINK LOST";
        case RPV_TARGET_STATE_STALE:
            return "STALE";
        case RPV_TARGET_STATE_VISION_FAULT:
            return "VISION ERR";
        case RPV_TARGET_STATE_NOT_FOUND:
            return "NOT FOUND";
        case RPV_TARGET_STATE_VALID:
            return "VALID";
        default:
            return "UNKNOWN";
    }
}

static StartupStage Vision_GetStartupStage(uint32_t now_ms)
{
    RaspberryPiVision_STM32F407_Stats_t stats;
    RPV_HeartbeatSnapshot_t heartbeat;
    RPV_TargetState_t target_state;

    if (!g_motor_initialized || !g_imu_initialized ||
        !g_angle_gyro_initialized || !g_oled.online) {
        return STARTUP_STAGE_BOARD_FAULT;
    }
    if (!RaspberryPiVision_STM32F407_ReceiverIsUsbConnected(
            &g_vision)) {
        return STARTUP_STAGE_WAIT_USB;
    }

    RaspberryPiVision_STM32F407_ReceiverGetStats(
        &g_vision, &stats);
    if (!stats.protocol.has_protocol_frame) {
        return STARTUP_STAGE_WAIT_PROTOCOL;
    }

    target_state =
        RaspberryPiVision_STM32F407_ReceiverGetTargetState(
            &g_vision, now_ms);
    if (target_state == RPV_TARGET_STATE_LINK_LOST) {
        return STARTUP_STAGE_LINK_LOST;
    }

    if (!RaspberryPiVision_STM32F407_ReceiverGetLatestHeartbeat(
            &g_vision, &heartbeat)) {
        return STARTUP_STAGE_WAIT_HEARTBEAT;
    }
    if ((uint32_t) (now_ms - heartbeat.received_at_ms) >=
        VISION_HEARTBEAT_TIMEOUT_MS) {
        return STARTUP_STAGE_WAIT_HEARTBEAT;
    }
    if ((heartbeat.system_flags &
         RPV_SYSTEM_SERVICE_READY) == 0U) {
        return STARTUP_STAGE_PI_STARTING;
    }
    if ((heartbeat.system_flags &
         RPV_SYSTEM_WIFI_CONNECTED) == 0U) {
        return STARTUP_STAGE_WAIT_WIFI;
    }
    if ((heartbeat.system_flags &
         RPV_SYSTEM_CAMERA_RUNNING) == 0U) {
        return STARTUP_STAGE_CAMERA_FAULT;
    }
    if ((heartbeat.system_flags &
         RPV_SYSTEM_INFERENCE_RUNNING) == 0U) {
        return STARTUP_STAGE_INFERENCE_FAULT;
    }
    if ((heartbeat.system_flags &
         RPV_SYSTEM_STARTUP_READY) == 0U) {
        return STARTUP_STAGE_WAIT_PI_CONFIRM;
    }
    if ((target_state == RPV_TARGET_STATE_NO_DATA) ||
        (target_state == RPV_TARGET_STATE_STALE) ||
        (target_state == RPV_TARGET_STATE_NOT_FOUND)) {
        return STARTUP_STAGE_WAIT_TARGET;
    }
    if (target_state == RPV_TARGET_STATE_VISION_FAULT) {
        return STARTUP_STAGE_VISION_FAULT;
    }

    return STARTUP_STAGE_READY;
}

static const char *Startup_Instruction(StartupStage stage)
{
    switch (stage) {
        case STARTUP_STAGE_BOARD_FAULT:
            return "CHECK BOARD";
        case STARTUP_STAGE_WAIT_USB:
            return "CONNECT PI USB";
        case STARTUP_STAGE_WAIT_PROTOCOL:
            return "START PI SERVICE";
        case STARTUP_STAGE_LINK_LOST:
            return "PI LINK LOST";
        case STARTUP_STAGE_WAIT_TARGET:
            return "WAIT TARGET DATA";
        case STARTUP_STAGE_WAIT_HEARTBEAT:
            return "WAIT HEARTBEAT";
        case STARTUP_STAGE_PI_STARTING:
            return "PI STARTING";
        case STARTUP_STAGE_WAIT_WIFI:
            return "WAIT WIFI";
        case STARTUP_STAGE_CAMERA_FAULT:
            return "CAMERA ERROR";
        case STARTUP_STAGE_INFERENCE_FAULT:
            return "INFERENCE ERROR";
        case STARTUP_STAGE_WAIT_PI_CONFIRM:
            return "WAIT PI CONFIRM";
        case STARTUP_STAGE_VISION_FAULT:
            return "VISION ERROR";
        case STARTUP_STAGE_READY:
            return "SYSTEM READY";
        default:
            return "UNKNOWN";
    }
}

static void Imu_PrintSnapshot(void)
{
    JY61P_Data data = {0};
    int32_t ax_mg;
    int32_t ay_mg;

    if (!JY61P_GetData(&g_imu, &data)) {
        Debug_Printf(
            "JY61P: no valid frame, rx=%" PRIu32
            ", checksum=%" PRIu32 ", uart=%" PRIu32 "\r\n",
            data.received_byte_count, data.checksum_error_count,
            data.uart_error_count);
        return;
    }
    ax_mg = (int32_t) (data.acceleration_g[0] * 1000.0f);
    ay_mg = (int32_t) (data.acceleration_g[1] * 1000.0f);
    Debug_Printf(
        "JY61P: AX=%" PRId32 "mg AY=%" PRId32
        "mg frames=%" PRIu32 " checksum=%" PRIu32
        " uart=%" PRIu32 "\r\n",
        ax_mg, ay_mg, data.valid_frame_count,
        data.checksum_error_count, data.uart_error_count);
}

static void Vision_PrintSnapshot(void)
{
    RPV_TargetSnapshot_t target;
    RaspberryPiVision_STM32F407_Stats_t stats;
    RPV_TargetState_t state =
        RaspberryPiVision_STM32F407_ReceiverGetTargetState(
            &g_vision, HAL_GetTick());

    RaspberryPiVision_STM32F407_ReceiverGetStats(
        &g_vision, &stats);
    Debug_Printf(
        "vision: %s rx=%" PRIu32 " valid=%" PRIu32
        " crc=%" PRIu32 " format=%" PRIu32
        " semantic=%" PRIu32 " usb_packets=%" PRIu32
        " overflow=%" PRIu32 " queued=%u",
        Vision_StateName(state), stats.protocol.rx_byte_count,
        stats.protocol.accepted_frame_count,
        stats.protocol.crc_error_count,
        stats.protocol.format_error_count,
        stats.protocol.semantic_error_count,
        stats.usb_packet_count, stats.usb_rx_overflow_count,
        stats.queued_byte_count);
    if ((state == RPV_TARGET_STATE_VALID) &&
        RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
            &g_vision, &target)) {
        Debug_Printf(" x=%u y=%u confidence=%u.%02u%%",
                     RPV_Q4ToRoundedPixelClamped(
                         target.target_x_q4, RPV_IMAGE_WIDTH_PIXELS),
                     RPV_Q4ToRoundedPixelClamped(
                         target.target_y_q4, RPV_IMAGE_HEIGHT_PIXELS),
                     target.confidence / 100U,
                     target.confidence % 100U);
    }
    Debug_Printf("\r\n");
}

static void Debug_PrintHelp(void)
{
    Debug_Printf(
        "\r\nBallControl STM32F407 HAL safe test\r\n"
        "h/? : help\r\n"
        "t   : motor read-only communication test\r\n"
        "p/r/s/m: position/speed/status/version\r\n");
    Debug_Printf(
        "a   : JY61P AX/AY and receive statistics\r\n"
        "o   : scan I2C bus and reinitialize OLED\r\n"
        "v   : Raspberry Pi vision snapshot/statistics\r\n"
        "e/d : enable/disable motor 1\r\n"
        "+/- : explicit 256-pulse move at 30 rpm\r\n"
        "x   : broadcast immediate stop (also on-board KEY)\r\n"
        "z   : zero JY61P yaw\r\n"
        "startup: automatic Raspberry Pi heartbeat confirmation\r\n"
        "KEY3/KEY4 startup actions are disabled.\r\n"
        "Control requires calibrated config plus explicit enable API.\r\n"
        "No motor is enabled or moved automatically.\r\n");
}

static void Oled_ProbeAndReinitialize(void)
{
    uint8_t address;
    uint32_t device_count = 0U;

    if (OledSsd1306_IsBusy(&g_oled)) {
        Debug_Printf("OLED DMA busy; retry later\r\n");
        return;
    }

    Debug_Printf("I2C1 scan:");
    for (address = 0x03U; address <= 0x77U; ++address) {
        if (HAL_I2C_IsDeviceReady(
                &hi2c1, (uint16_t) address << 1U, 2U, 10U) == HAL_OK) {
            Debug_Printf(" 0x%02X", address);
            device_count++;
        }
    }
    if (device_count == 0U) {
        Debug_Printf(" no ACK\r\n");
    } else {
        Debug_Printf("\r\n");
    }

    (void) OledSsd1306_Init(&g_oled, &hi2c1);
    Debug_Printf("OLED address 0x3C: %s, errors=%" PRIu32 "\r\n",
                 g_oled.online ? "OK" : "NACK", g_oled.error_count);
    if (g_oled.online) {
        g_next_oled_update_ms = HAL_GetTick();
    }
}

static void Debug_HandleCommand(uint8_t command)
{
    switch (command) {
        case 't':
        case 'T':
            if (Control_ManualMotorCommandAllowed()) {
                Motor_CommunicationTest();
            }
            break;
        case 'm':
        case 'M':
            if (Control_ManualMotorCommandAllowed()) {
                Motor_ReadVersion();
            }
            break;
        case 's':
        case 'S':
            if (Control_ManualMotorCommandAllowed()) {
                Motor_ReadStatus();
            }
            break;
        case 'p':
        case 'P':
            if (Control_ManualMotorCommandAllowed()) {
                Motor_ReadPosition();
            }
            break;
        case 'r':
        case 'R':
            if (Control_ManualMotorCommandAllowed()) {
                Motor_ReadSpeed();
            }
            break;
        case 'a':
        case 'A':
            Imu_PrintSnapshot();
            break;
        case 'o':
        case 'O':
            Oled_ProbeAndReinitialize();
            break;
        case 'v':
        case 'V':
            Vision_PrintSnapshot();
            break;
        case 'e':
        case 'E':
            if (Control_ManualMotorCommandAllowed()) {
                Motor_SetEnabled(true);
            }
            break;
        case 'd':
        case 'D':
            (void) BallControl_RequestControlEnable(false);
            if (Control_ManualMotorCommandAllowed()) {
                Motor_SetEnabled(false);
            }
            break;
        case '+':
            if (Control_ManualMotorCommandAllowed()) {
                Motor_MoveSmallStep(MOTOR_TEST_STEP_PULSES);
            }
            break;
        case '-':
            if (Control_ManualMotorCommandAllowed()) {
                Motor_MoveSmallStep(-MOTOR_TEST_STEP_PULSES);
            }
            break;
        case 'x':
        case 'X':
            (void) BallControl_RequestControlEnable(false);
            Debug_Printf("control disarmed; broadcast stop sent\r\n");
            break;
        case 'z':
        case 'Z':
            Debug_Printf(JY61P_ZeroYaw(&g_imu) ?
                         "JY61P yaw zero command sent\r\n" :
                         "JY61P yaw zero command failed\r\n");
            break;
        case 'h':
        case 'H':
        case '?':
            Debug_PrintHelp();
            break;
        case '\r':
        case '\n':
        case ' ':
        case '\t':
            break;
        default:
            Debug_Printf("unknown command 0x%02X; send h\r\n", command);
            break;
    }
}

static void Oled_UpdateStartup(StartupStage stage)
{
    RaspberryPiVision_STM32F407_Stats_t stats;
    RPV_HeartbeatSnapshot_t heartbeat = {0};
    bool heartbeat_received;
    char line[24];

    RaspberryPiVision_STM32F407_ReceiverGetStats(
        &g_vision, &stats);
    heartbeat_received =
        RaspberryPiVision_STM32F407_ReceiverGetLatestHeartbeat(
            &g_vision, &heartbeat);

    OledSsd1306_Clear(&g_oled);
    OledSsd1306_DrawText(
        &g_oled, 0U, 0U, "SYSTEM STARTUP", &afont8x6);
    (void) snprintf(line, sizeof(line), "BOARD:%s",
                    (g_motor_initialized && g_imu_initialized &&
                     g_angle_gyro_initialized) ?
                    "OK" : "ERROR");
    OledSsd1306_DrawText(&g_oled, 0U, 8U, line, &afont8x6);
    (void) snprintf(line, sizeof(line), "USB:%s",
                    stats.usb_connected ? "OK" : "WAIT");
    OledSsd1306_DrawText(&g_oled, 0U, 16U, line, &afont8x6);
    (void) snprintf(line, sizeof(line), "PROTO:%s",
                    stats.protocol.has_protocol_frame ?
                    "OK" : "WAIT");
    OledSsd1306_DrawText(&g_oled, 0U, 24U, line, &afont8x6);
    (void) snprintf(
        line, sizeof(line), "PI:%s",
        heartbeat_received &&
        ((heartbeat.system_flags &
          RPV_SYSTEM_SERVICE_READY) != 0U) ? "OK" : "WAIT");
    OledSsd1306_DrawText(&g_oled, 0U, 32U, line, &afont8x6);
    (void) snprintf(
        line, sizeof(line), "WIFI:%s",
        heartbeat_received &&
        ((heartbeat.system_flags &
          RPV_SYSTEM_WIFI_CONNECTED) != 0U) ? "OK" : "WAIT");
    OledSsd1306_DrawText(&g_oled, 0U, 40U, line, &afont8x6);
    (void) snprintf(
        line, sizeof(line), "VISION:%s",
        heartbeat_received &&
        ((heartbeat.system_flags &
          (RPV_SYSTEM_CAMERA_RUNNING |
           RPV_SYSTEM_INFERENCE_RUNNING)) ==
         (RPV_SYSTEM_CAMERA_RUNNING |
          RPV_SYSTEM_INFERENCE_RUNNING)) ? "OK" : "WAIT");
    OledSsd1306_DrawText(&g_oled, 0U, 48U, line, &afont8x6);
    OledSsd1306_DrawText(
        &g_oled, 0U, 56U, Startup_Instruction(stage),
        &afont8x6);
    (void) OledSsd1306_RefreshAsync(&g_oled);
}

static void Oled_Update(void)
{
    JY61P_Data imu = {0};
    SingleAxisGyro_Sample_t angle_gyro = {0};
    RPV_TargetSnapshot_t target;
    RPV_HeartbeatSnapshot_t heartbeat = {0};
    RPV_TargetState_t vision_state;
    char line[26];
    char angle_text[9];
    char rate_text[10];
    bool heartbeat_received;
    bool ipv4_valid;
    bool imu_valid;
    bool angle_gyro_valid;

    if (!g_oled.online) {
        return;
    }

    if (g_startup_stage != STARTUP_STAGE_READY) {
        Oled_UpdateStartup(g_startup_stage);
        return;
    }

    imu_valid = JY61P_GetData(&g_imu, &imu);
    angle_gyro_valid = SingleAxisGyro_STM32F407_GetSample(
        &g_angle_gyro, &angle_gyro);
    vision_state =
        RaspberryPiVision_STM32F407_ReceiverGetTargetState(
            &g_vision, HAL_GetTick());
    heartbeat_received =
        RaspberryPiVision_STM32F407_ReceiverGetLatestHeartbeat(
            &g_vision, &heartbeat);
    ipv4_valid =
        heartbeat_received && heartbeat.extended_fields_valid &&
        ((heartbeat.ipv4_address[0] != 0U) ||
         (heartbeat.ipv4_address[1] != 0U) ||
         (heartbeat.ipv4_address[2] != 0U) ||
         (heartbeat.ipv4_address[3] != 0U));

    OledSsd1306_Clear(&g_oled);
    OledSsd1306_DrawText(
        &g_oled, 0U, 0U, "SYSTEM READY", &afont8x6);

    if (imu_valid) {
        int32_t ax_mg =
            (int32_t) (imu.acceleration_g[0] * 1000.0f);
        int32_t ay_mg =
            (int32_t) (imu.acceleration_g[1] * 1000.0f);
        (void) snprintf(
            line, sizeof(line), "AX:%" PRId32 " AY:%" PRId32,
            ax_mg, ay_mg);
        OledSsd1306_DrawText(&g_oled, 0U, 8U, line, &afont8x6);
        (void) snprintf(line, sizeof(line), "IMU F:%" PRIu32,
                        imu.valid_frame_count);
    } else {
        OledSsd1306_DrawText(
            &g_oled, 0U, 8U, "AX:---- AY:----", &afont8x6);
        (void) snprintf(line, sizeof(line), "IMU:NO DATA");
    }
    OledSsd1306_DrawText(&g_oled, 0U, 16U, line, &afont8x6);

    if (angle_gyro_valid &&
        ((angle_gyro.valid_mask &
          (SINGLE_AXIS_GYRO_VALID_YAW |
           SINGLE_AXIS_GYRO_VALID_RATE)) ==
         (SINGLE_AXIS_GYRO_VALID_YAW |
          SINGLE_AXIS_GYRO_VALID_RATE))) {
        Format_Tenths(
            ScaleRawToTenths(angle_gyro.raw_yaw, 1800L),
            angle_text, sizeof(angle_text));
        Format_Tenths(
            ScaleRawToTenths(angle_gyro.raw_angular_rate,
                             GYRO_RATE_FULL_SCALE_TENTHS),
            rate_text, sizeof(rate_text));
        (void) snprintf(line, sizeof(line), "ANG:%s W:%s",
                        angle_text, rate_text);
    } else {
        (void) snprintf(line, sizeof(line), "ANG:--- W:---");
    }
    OledSsd1306_DrawText(&g_oled, 0U, 24U, line, &afont8x6);
    if ((vision_state == RPV_TARGET_STATE_VALID) &&
        RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
            &g_vision, &target)) {
        (void) snprintf(line, sizeof(line), "X:%u Y:%u",
                        RPV_Q4ToRoundedPixelClamped(
                            target.target_x_q4,
                            RPV_IMAGE_WIDTH_PIXELS),
                        RPV_Q4ToRoundedPixelClamped(
                            target.target_y_q4,
                            RPV_IMAGE_HEIGHT_PIXELS));
    } else {
        (void) snprintf(line, sizeof(line), "X:---- Y:----");
    }
    OledSsd1306_DrawText(&g_oled, 0U, 32U, line, &afont8x6);
    if (ipv4_valid) {
        (void) snprintf(
            line, sizeof(line), "IP:%u.%u.%u.%u",
            heartbeat.ipv4_address[0], heartbeat.ipv4_address[1],
            heartbeat.ipv4_address[2], heartbeat.ipv4_address[3]);
    } else {
        (void) snprintf(line, sizeof(line), "IP:---");
    }
    OledSsd1306_DrawText(&g_oled, 0U, 40U, line, &afont8x6);
    (void) snprintf(line, sizeof(line), "MOTOR ADDR:%u",
                    MOTOR_ADDRESS);
    OledSsd1306_DrawText(&g_oled, 0U, 48U, line, &afont8x6);
    OledSsd1306_DrawText(
        &g_oled, 0U, 56U, "PI START: OK", &afont8x6);
    (void) OledSsd1306_RefreshAsync(&g_oled);
}

static uint32_t Control_MeasurementAgeMs(
    uint32_t now_ms, const RPV_TargetSnapshot_t *target)
{
    uint64_t age_ms =
        (uint64_t) (uint32_t) (now_ms - target->received_at_ms) +
        target->latency_ms;

    if (age_ms >= target->prediction_ms) {
        age_ms -= target->prediction_ms;
    } else {
        age_ms = 0U;
    }
    return age_ms > UINT32_MAX ? UINT32_MAX : (uint32_t) age_ms;
}

static float Control_WrapSignedDegrees(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void Control_ReadSensorInput(
    uint32_t now_ms, BallControl_SensorInput *input)
{
    SingleAxisGyro_Sample_t gyro = {0};
    SingleAxisGyro_STM32F407_Statistics_t gyro_stats = {0};
    JY61P_Data imu = {0};
    const BallControl_Config *config = &g_control_core.config;

    (void) memset(input, 0, sizeof(*input));
    if (g_angle_gyro_initialized && config->gyro.valid &&
        SingleAxisGyro_STM32F407_GetSample(&g_angle_gyro, &gyro)) {
        SingleAxisGyro_STM32F407_GetStatistics(
            &g_angle_gyro, &gyro_stats);
        if (gyro_stats.receive_armed &&
            ((gyro.valid_mask &
              (SINGLE_AXIS_GYRO_VALID_YAW |
               SINGLE_AXIS_GYRO_VALID_RATE)) ==
             (SINGLE_AXIS_GYRO_VALID_YAW |
              SINGLE_AXIS_GYRO_VALID_RATE)) &&
            ((uint32_t) (now_ms - gyro_stats.last_valid_frame_ms) <
             config->gyro.timeout_ms)) {
            input->gyro_angle_deg =
                Control_WrapSignedDegrees(
                    gyro.yaw_deg - config->gyro.angle_zero_deg) *
                config->gyro.angle_direction;
            input->gyro_rate_dps =
                gyro.angular_rate_dps * config->gyro.rate_direction;
            input->gyro_valid = true;
        }
    }

    if (config->imu_feedforward.enabled && g_imu_initialized &&
        (config->imu_feedforward.acceleration_axis < 3U) &&
        JY61P_GetData(&g_imu, &imu) && imu.receive_armed &&
        ((imu.valid_flags & JY61P_DATA_ACCELERATION) != 0U) &&
        ((uint32_t) (now_ms - imu.last_acceleration_frame_ms) <
         config->imu_feedforward.timeout_ms)) {
        input->imu_acceleration_g =
            imu.acceleration_g[
                config->imu_feedforward.acceleration_axis] *
            config->imu_feedforward.acceleration_direction;
        input->imu_acceleration_valid = true;
    }
}

static void Control_Disarm(bool fault, bool force_stop)
{
    BallControlCore_SetEnabled(&g_control_core, false);
    (void) memset(&g_control_output, 0, sizeof(g_control_output));
    g_control_output_valid = false;
    g_control_pending_motor_target = 0;
    g_control_last_motor_target = 0;
    g_control_last_motor_target_valid = false;
    g_control_motor_deadline_ms = 0U;
    if (force_stop && g_motor_initialized) {
        (void) EmmV5_EmergencyStopBroadcast(&g_motor);
    } else {
        EmmV5_CancelResponse(&g_motor, false);
    }
    g_control_motor_state = fault ?
        CONTROL_MOTOR_FAULT : CONTROL_MOTOR_IDLE;
    if (fault) {
        g_control_motor_fault_count++;
    }
}

static void Control_EnterFault(const char *reason)
{
    if (g_control_motor_state == CONTROL_MOTOR_FAULT) {
        BallControlCore_SetEnabled(&g_control_core, false);
        return;
    }
    Debug_Printf("control fault: %s\r\n",
                 reason != NULL ? reason : "unknown");
    Control_Disarm(true, true);
}

static void Control_UpdateVision(uint32_t now_ms)
{
    RPV_TargetSnapshot_t target;
    RPV_TargetState_t state =
        RaspberryPiVision_STM32F407_ReceiverGetTargetState(
            &g_vision, now_ms);

    if ((state != RPV_TARGET_STATE_VALID) ||
        !RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
            &g_vision, &target) ||
        ((target.status_flags & RPV_TARGET_VALID) == 0U)) {
        BallControlCore_InvalidateVision(&g_control_core);
        return;
    }

    if (g_control_target_sequence_valid &&
        (target.packet_sequence == g_control_last_target_sequence)) {
        return;
    }
    g_control_target_sequence_valid = true;
    g_control_last_target_sequence = target.packet_sequence;
    if (target.confidence < g_control_core.config.minimum_confidence) {
        BallControlCore_InvalidateVision(&g_control_core);
        return;
    }

    (void) BallControlCore_AcceptVisionMeasurement(
        &g_control_core, target.frame_id,
        target.target_x_q4, target.target_y_q4,
        Control_MeasurementAgeMs(now_ms, &target));
}

static void Control_RunOutputStep(uint32_t now_ms)
{
    BallControl_SensorInput input;

    Control_ReadSensorInput(now_ms, &input);
    g_control_output_valid = BallControlCore_Compute(
        &g_control_core, &input, CONTROL_OUTPUT_DT_S,
        &g_control_output);
    if (g_control_output_valid && g_control_output.valid) {
        g_control_pending_motor_target =
            g_control_output.motor_target_units;
    } else {
        g_control_output_valid = false;
        (void) memset(&g_control_output, 0, sizeof(g_control_output));
    }
}

static void Control_ProcessTicks(uint32_t now_ms)
{
    uint32_t produced = g_control_tick_produced;
    uint32_t backlog = produced - g_control_tick_consumed;

    if (backlog > CONTROL_MAX_TICK_BACKLOG) {
        uint32_t skipped = backlog - CONTROL_MAX_TICK_BACKLOG;
        BallControl_Status status;

        g_control_scheduler_overrun_count += skipped;
        g_control_tick_consumed += skipped;
        BallControlCore_GetStatus(&g_control_core, &status);
        if (status.enable_requested) {
            Control_EnterFault("1 kHz scheduler overrun");
        }
    }

    while (g_control_tick_consumed != produced) {
        g_control_tick_consumed++;
        if ((g_control_tick_consumed % CONTROL_ESTIMATOR_TICKS) == 0U) {
            BallControlCore_Predict(
                &g_control_core, CONTROL_ESTIMATOR_DT_S);
        }
        if ((g_control_tick_consumed % CONTROL_OUTPUT_TICKS) == 0U) {
            Control_RunOutputStep(now_ms);
        }
    }
}

static bool Control_MotorTargetChanged(int32_t target)
{
    int64_t difference;

    if (!g_control_last_motor_target_valid) {
        return true;
    }
    difference = (int64_t) target - g_control_last_motor_target;
    if (difference < 0) {
        difference = -difference;
    }
    return (uint64_t) difference >=
        g_control_core.config.actuator.minimum_command_delta_units;
}

static bool Control_MotorCanRun(const BallControl_Status *status)
{
    return g_motor_initialized && g_motor.receive_armed &&
        (g_startup_stage == STARTUP_STAGE_READY) &&
        (HAL_GPIO_ReadPin(CORE_USER_KEY_GPIO_PORT,
                          CORE_USER_KEY_PIN) != GPIO_PIN_SET) &&
        status->enable_requested && status->configuration_ready &&
        status->estimator_initialized && status->vision_valid &&
        (status->state == BALL_CONTROL_STATE_ACTIVE) &&
        g_control_output_valid && g_control_output.valid;
}

static bool Control_MotorDeadlineExpired(uint32_t now_ms)
{
    return (g_control_motor_deadline_ms != 0U) &&
        ((int32_t) (now_ms - g_control_motor_deadline_ms) >= 0);
}

static void Control_MotorProcess(uint32_t now_ms)
{
    BallControl_Status status;
    EmmV5_Response response;

    BallControlCore_GetStatus(&g_control_core, &status);
    if (!status.enable_requested) {
        if (g_control_motor_state != CONTROL_MOTOR_FAULT) {
            g_control_motor_state = CONTROL_MOTOR_IDLE;
            g_control_motor_deadline_ms = 0U;
        }
        return;
    }

    if (!Control_MotorCanRun(&status)) {
        if ((g_control_motor_state != CONTROL_MOTOR_IDLE) ||
            (status.state != BALL_CONTROL_STATE_DISABLED)) {
            Control_EnterFault("control safety gate lost");
        }
        return;
    }

    switch (g_control_motor_state) {
        case CONTROL_MOTOR_IDLE:
            if (EmmV5_Enable(
                    &g_motor, MOTOR_ADDRESS, true, false)) {
                g_control_motor_state = CONTROL_MOTOR_WAIT_ENABLE;
                g_control_motor_deadline_ms = now_ms +
                    g_control_core.config.actuator.response_timeout_ms;
            } else if (g_control_motor_deadline_ms == 0U) {
                g_control_motor_deadline_ms = now_ms +
                    g_control_core.config.actuator.response_timeout_ms;
            } else if (Control_MotorDeadlineExpired(now_ms)) {
                Control_EnterFault("motor enable send timeout");
            }
            break;

        case CONTROL_MOTOR_WAIT_ENABLE:
            if (EmmV5_TryGetResponse(&g_motor, &response)) {
                if (!EmmV5_ResponseIsAccepted(&response)) {
                    Control_EnterFault("motor enable rejected");
                    break;
                }
                g_control_motor_state = CONTROL_MOTOR_SEND_PARAMETERS;
                g_control_motor_deadline_ms = now_ms +
                    g_control_core.config.actuator.response_timeout_ms;
            } else if (Control_MotorDeadlineExpired(now_ms)) {
                EmmV5_CancelResponse(&g_motor, true);
                Control_EnterFault("motor enable response timeout");
            }
            break;

        case CONTROL_MOTOR_SEND_PARAMETERS:
            if (EmmV5_SetFastPositionParameters(
                    &g_motor, MOTOR_ADDRESS,
                    g_control_core.config.actuator.speed_rpm,
                    g_control_core.config.actuator.acceleration,
                    EMM_V5_MOVE_ABSOLUTE, false)) {
                g_control_motor_state = CONTROL_MOTOR_WAIT_PARAMETERS;
                g_control_motor_deadline_ms = now_ms +
                    g_control_core.config.actuator.response_timeout_ms;
            } else if (Control_MotorDeadlineExpired(now_ms)) {
                Control_EnterFault("motor parameter send timeout");
            }
            break;

        case CONTROL_MOTOR_WAIT_PARAMETERS:
            if (EmmV5_TryGetResponse(&g_motor, &response)) {
                if (!EmmV5_ResponseIsAccepted(&response)) {
                    Control_EnterFault("motor parameters rejected");
                    break;
                }
                g_control_motor_state = CONTROL_MOTOR_READY;
                g_control_motor_deadline_ms = 0U;
            } else if (Control_MotorDeadlineExpired(now_ms)) {
                EmmV5_CancelResponse(&g_motor, true);
                Control_EnterFault("motor parameter response timeout");
            }
            break;

        case CONTROL_MOTOR_READY:
            if (!Control_MotorTargetChanged(
                    g_control_pending_motor_target)) {
                g_control_motor_deadline_ms = 0U;
                break;
            }
            if (EmmV5_StartFastPosition(
                    &g_motor, MOTOR_ADDRESS,
                    g_control_pending_motor_target)) {
                g_control_last_motor_target =
                    g_control_pending_motor_target;
                g_control_last_motor_target_valid = true;
                g_control_motor_state = CONTROL_MOTOR_WAIT_POSITION;
                g_control_motor_deadline_ms = now_ms +
                    g_control_core.config.actuator.response_timeout_ms;
            } else if (g_control_motor_deadline_ms == 0U) {
                g_control_motor_deadline_ms = now_ms +
                    g_control_core.config.actuator.response_timeout_ms;
            } else if (Control_MotorDeadlineExpired(now_ms)) {
                Control_EnterFault("motor position send timeout");
            }
            break;

        case CONTROL_MOTOR_WAIT_POSITION:
            if (EmmV5_TryGetResponse(&g_motor, &response)) {
                if (!EmmV5_ResponseIsAccepted(&response)) {
                    Control_EnterFault("motor position rejected");
                    break;
                }
                g_control_motor_state = CONTROL_MOTOR_READY;
                g_control_motor_deadline_ms = 0U;
            } else if (Control_MotorDeadlineExpired(now_ms)) {
                EmmV5_CancelResponse(&g_motor, true);
                Control_EnterFault("motor position response timeout");
            }
            break;

        case CONTROL_MOTOR_FAULT:
        default:
            Control_EnterFault("invalid motor state");
            break;
    }
}

void BallControl_Init(void)
{
    BallControl_Config control_config;
    bool debug_ok;
    bool motor_ok;
    bool imu_ok;
    bool angle_gyro_ok;
    uint32_t now_ms;

    RaspberryPiVision_STM32F407_ReceiverInit(&g_vision);
    debug_ok = false;
    g_debug_initialized = false;
    (void) OledSsd1306_Init(&g_oled, &hi2c1);
    motor_ok = EmmV5_Init(&g_motor, &huart2);
    angle_gyro_ok = SingleAxisGyro_STM32F407_Init(
        &g_angle_gyro, &huart3, GYRO_RATE_FULL_SCALE_DPS);
    imu_ok = JY61P_Init(&g_imu, &huart6);
    g_motor_initialized = motor_ok;
    g_angle_gyro_initialized = angle_gyro_ok;
    g_imu_initialized = imu_ok;

    BallControlCore_LoadSafeDefaults(&control_config);
    BallControlCore_Init(&g_control_core, &control_config);
    (void) memset(&g_control_output, 0, sizeof(g_control_output));
    g_control_tick_produced = 0U;
    g_control_tick_consumed = 0U;
    g_control_scheduler_overrun_count = 0U;
    g_control_target_sequence_valid = false;
    g_control_last_target_sequence = 0U;
    g_control_output_valid = false;
    g_control_motor_state = CONTROL_MOTOR_IDLE;
    g_control_motor_deadline_ms = 0U;
    g_control_motor_fault_count = 0U;
    g_control_pending_motor_target = 0;
    g_control_last_motor_target = 0;
    g_control_last_motor_target_valid = false;

    if (debug_ok) {
        Debug_Printf(
            "\r\nBallControl STM32F407VET6 HAL boot\r\n"
            "SYSCLK=168MHz HSE=8MHz\r\n"
            "single-axis gyro USART3 PD8/PD9 115200 8N1 DMA\r\n"
            "motor bus USART2 PD5/PD6 115200 8N1\r\n");
        Debug_Printf(
            "vision USB CDC PA11/PA12 (Type-C, not USART0)\r\n"
            "vision RX: 1024-byte ISR queue, main-loop parser\r\n"
            "JY61P USART6 PC6/PC7 9600 8N1 DMA\r\n"
            "OLED I2C1 PB6/PB7 100kHz address 0x3C\r\n"
            "CN6 PC10/PC13 disabled: PC13 has no UART4_RX AF\r\n");
        Debug_Printf(
            "init: motor=%s angle-gyro=%s vision=USB-CDC imu=%s\r\n",
                     motor_ok ? "OK" : "FAIL",
                     angle_gyro_ok ? "OK" : "FAIL",
                     imu_ok ? "OK" : "FAIL");
    }

    if (debug_ok) {
        Debug_Printf("init: OLED=%s\r\n",
                     g_oled.online ? "OK" : "NACK");
        Debug_PrintHelp();
    }

    now_ms = HAL_GetTick();
    g_next_oled_update_ms = now_ms;
    g_next_led_toggle_ms = now_ms;
    g_estop_key_raw_pressed =
        HAL_GPIO_ReadPin(CORE_USER_KEY_GPIO_PORT,
                         CORE_USER_KEY_PIN) == GPIO_PIN_SET;
    g_estop_key_stable_pressed = g_estop_key_raw_pressed;
    g_estop_key_change_ms = now_ms;
    g_startup_stage = Vision_GetStartupStage(now_ms);
}

void BallControl_Process(void)
{
    uint8_t command;
    uint32_t now = HAL_GetTick();
    StartupStage previous_startup_stage;
    bool estop_key_pressed =
        HAL_GPIO_ReadPin(CORE_USER_KEY_GPIO_PORT,
                         CORE_USER_KEY_PIN) == GPIO_PIN_SET;

    EmmV5_ProcessRx(&g_motor);
    SingleAxisGyro_STM32F407_Process(&g_angle_gyro);
    JY61P_ProcessRx(&g_imu);
    (void) RaspberryPiVision_STM32F407_ReceiverProcess(
        &g_vision, now);

    previous_startup_stage = g_startup_stage;
    g_startup_stage = Vision_GetStartupStage(now);
    if (g_startup_stage != previous_startup_stage) {
        Debug_Printf("startup: %s\r\n",
                     Startup_Instruction(g_startup_stage));
    }

    if (estop_key_pressed != g_estop_key_raw_pressed) {
        g_estop_key_raw_pressed = estop_key_pressed;
        g_estop_key_change_ms = now;
    }
    if ((g_estop_key_stable_pressed != g_estop_key_raw_pressed) &&
        ((uint32_t) (now - g_estop_key_change_ms) >= KEY_DEBOUNCE_MS)) {
        g_estop_key_stable_pressed = g_estop_key_raw_pressed;
        if (g_estop_key_stable_pressed) {
            (void) BallControl_RequestControlEnable(false);
            Debug_Printf(
                "on-board KEY: control disarmed and stop sent\r\n");
        }
    }

    Control_UpdateVision(now);
    Control_ProcessTicks(now);
    Control_MotorProcess(now);

    if (g_debug_initialized) {
        while (DebugConsole_Read(&g_debug, &command, 1U) == 1U) {
            Debug_HandleCommand(command);
        }
    }

    if ((int32_t) (now - g_next_oled_update_ms) >= 0) {
        Oled_Update();
        g_next_oled_update_ms = now + OLED_UPDATE_PERIOD_MS;
    }
    if ((int32_t) (now - g_next_led_toggle_ms) >= 0) {
        HAL_GPIO_TogglePin(CORE_LED_GPIO_PORT, CORE_LED_PIN);
        g_next_led_toggle_ms = now + LED_TOGGLE_PERIOD_MS;
    }

    __WFI();
}

void BallControl_Timer1kHzCallback(void)
{
    g_control_tick_produced++;
}

bool BallControl_ApplyControlConfig(const BallControl_Config *config)
{
    BallControl_Status status;

    if ((config == NULL) || !BallControlCore_ConfigIsReady(config)) {
        return false;
    }
    BallControlCore_GetStatus(&g_control_core, &status);
    if (status.enable_requested ||
        ((g_control_motor_state != CONTROL_MOTOR_IDLE) &&
         (g_control_motor_state != CONTROL_MOTOR_FAULT))) {
        return false;
    }

    BallControlCore_Configure(&g_control_core, config);
    g_control_target_sequence_valid = false;
    g_control_last_target_sequence = 0U;
    (void) memset(&g_control_output, 0, sizeof(g_control_output));
    g_control_output_valid = false;
    g_control_pending_motor_target = 0;
    g_control_last_motor_target = 0;
    g_control_last_motor_target_valid = false;
    g_control_motor_deadline_ms = 0U;
    g_control_motor_state = CONTROL_MOTOR_IDLE;
    return true;
}

bool BallControl_RequestControlEnable(bool enabled)
{
    BallControl_Status status;
    BallControl_SensorInput input;
    uint32_t now_ms;

    if (!enabled) {
        Control_Disarm(false, true);
        return true;
    }

    now_ms = HAL_GetTick();
    BallControlCore_GetStatus(&g_control_core, &status);
    Control_ReadSensorInput(now_ms, &input);
    if (!status.configuration_ready || !status.estimator_initialized ||
        !status.vision_valid || !g_motor_initialized ||
        !g_motor.receive_armed || !input.gyro_valid ||
        (g_control_core.config.imu_feedforward.enabled &&
         !input.imu_acceleration_valid) ||
        (g_startup_stage != STARTUP_STAGE_READY) ||
        (HAL_GPIO_ReadPin(CORE_USER_KEY_GPIO_PORT,
                          CORE_USER_KEY_PIN) == GPIO_PIN_SET)) {
        return false;
    }

    if (g_control_motor_state == CONTROL_MOTOR_FAULT) {
        g_control_motor_state = CONTROL_MOTOR_IDLE;
    }
    g_control_motor_deadline_ms = 0U;
    g_control_last_motor_target_valid = false;
    BallControlCore_SetEnabled(&g_control_core, true);
    return true;
}

void BallControl_SetControlSetpoint(float setpoint_mm)
{
    BallControlCore_SetSetpoint(&g_control_core, setpoint_mm);
}

void BallControl_GetControlStatus(BallControl_Status *status)
{
    BallControlCore_GetStatus(&g_control_core, status);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if (uart == &huart2) {
        EmmV5_RxEventCallback(&g_motor, size);
    } else if (uart == &huart3) {
        SingleAxisGyro_STM32F407_RxEventCallback(
            &g_angle_gyro, size);
    } else if (uart == &huart6) {
        JY61P_RxEventCallback(&g_imu, size);
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
        SingleAxisGyro_STM32F407_ErrorCallback(&g_angle_gyro);
    } else if (uart == &huart6) {
        JY61P_ErrorCallback(&g_imu);
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
    (void) RaspberryPiVision_STM32F407_ReceiverEnqueueUsbData(
        &g_vision, data, length);
}

void BallControl_UsbCdcSetConnected(bool connected)
{
    RaspberryPiVision_STM32F407_ReceiverSetUsbConnected(
        &g_vision, connected);
}
