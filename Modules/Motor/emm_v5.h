#ifndef BALLCONTROL_EMM_V5_H
#define BALLCONTROL_EMM_V5_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define EMM_V5_CHECK_BYTE              0x6BU
#define EMM_V5_RESPONSE_OK             0x02U
#define EMM_V5_RESPONSE_REACHED        0x9FU
#define EMM_V5_MAX_RESPONSE_LENGTH     8U
#define EMM_V5_RX_DMA_BUFFER_SIZE      64U
#define EMM_V5_MAX_TX_FRAME_LENGTH     8U

typedef enum {
    EMM_V5_MOVE_RELATIVE_TO_LAST_TARGET = 0U,
    EMM_V5_MOVE_ABSOLUTE = 1U,
    EMM_V5_MOVE_RELATIVE_TO_CURRENT = 2U
} EmmV5_MotionMode;

typedef struct {
    uint8_t bytes[EMM_V5_MAX_RESPONSE_LENGTH];
    uint8_t length;
} EmmV5_Response;

typedef struct {
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint32_t unexpected_byte_count;
    uint32_t timeout_count;
    uint32_t uart_error_count;
    uint32_t rx_overrun_count;
} EmmV5_Statistics;

typedef struct {
    uint16_t firmware_version;
    uint8_t hardware_series;
    uint8_t hardware_type;
    uint8_t hardware_version;
} EmmV5_Version;

typedef enum {
    EMM_V5_HOMING_IDLE_OR_COMPLETE = 0,
    EMM_V5_HOMING_IN_PROGRESS,
    EMM_V5_HOMING_FAILED
} EmmV5_HomingState;

typedef struct {
    uint8_t raw_flags;
    bool encoder_ready;
    bool calibration_ready;
    bool overtemperature_fault;
    bool overcurrent_fault;
    EmmV5_HomingState state;
} EmmV5_HomingStatus;

typedef struct {
    UART_HandleTypeDef *uart;
    uint8_t rx_dma_buffer[EMM_V5_RX_DMA_BUFFER_SIZE];
    uint8_t tx_buffer[EMM_V5_MAX_TX_FRAME_LENGTH];
    volatile uint32_t rx_published_count;
    uint32_t rx_consumed_count;
    volatile uint16_t rx_event_position;
    uint8_t expected_address;
    uint8_t expected_command;
    uint8_t expected_length;
    uint8_t index;
    uint8_t bytes[EMM_V5_MAX_RESPONSE_LENGTH];
    bool complete;
    bool response_pending;
    volatile bool receive_armed;
    volatile bool rx_restart_requested;
    volatile bool tx_busy;
    volatile EmmV5_Statistics statistics;
} EmmV5_Driver;

bool EmmV5_Init(EmmV5_Driver *driver, UART_HandleTypeDef *uart);
void EmmV5_RxEventCallback(EmmV5_Driver *driver, uint16_t position);
void EmmV5_ProcessRx(EmmV5_Driver *driver);
void EmmV5_TxCompleteCallback(EmmV5_Driver *driver);
void EmmV5_ErrorCallback(EmmV5_Driver *driver);
bool EmmV5_TryGetResponse(EmmV5_Driver *driver,
                          EmmV5_Response *response);
bool EmmV5_ResponsePending(const EmmV5_Driver *driver);
void EmmV5_CancelResponse(EmmV5_Driver *driver, bool timed_out);
bool EmmV5_WaitResponse(EmmV5_Driver *driver, uint32_t timeout_ms,
                        EmmV5_Response *response);
void EmmV5_GetStatistics(EmmV5_Driver *driver,
                         EmmV5_Statistics *statistics);

bool EmmV5_Enable(EmmV5_Driver *driver, uint8_t address, bool enabled,
                  bool synchronous);
bool EmmV5_SetFastPositionParameters(
    EmmV5_Driver *driver, uint8_t address, uint16_t speed_rpm,
    uint8_t acceleration, EmmV5_MotionMode mode, bool synchronous);
bool EmmV5_StartFastPosition(EmmV5_Driver *driver, uint8_t address,
                             int32_t pulses);
bool EmmV5_StopNow(EmmV5_Driver *driver, uint8_t address,
                   bool synchronous);
bool EmmV5_EmergencyStopBroadcast(EmmV5_Driver *driver);
bool EmmV5_ReadVersion(EmmV5_Driver *driver, uint8_t address);
bool EmmV5_ReadCurrentPosition(EmmV5_Driver *driver, uint8_t address);
bool EmmV5_ReadCurrentSpeed(EmmV5_Driver *driver, uint8_t address);
bool EmmV5_ReadStatus(EmmV5_Driver *driver, uint8_t address);
bool EmmV5_ReadHomingStatus(EmmV5_Driver *driver, uint8_t address);

bool EmmV5_ResponseIsAccepted(const EmmV5_Response *response);
bool EmmV5_DecodeVersion(const EmmV5_Response *response,
                         EmmV5_Version *version);
bool EmmV5_DecodeCurrentPosition(const EmmV5_Response *response,
                                 int64_t *position_units);
int64_t EmmV5_PositionUnitsToMillidegrees(int64_t position_units);
bool EmmV5_DecodeCurrentSpeed(const EmmV5_Response *response,
                              int16_t *speed_rpm);
bool EmmV5_DecodeStatus(const EmmV5_Response *response,
                        uint8_t *status_flags);
bool EmmV5_DecodeHomingStatus(const EmmV5_Response *response,
                              EmmV5_HomingStatus *status);

#endif
