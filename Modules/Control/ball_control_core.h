#ifndef BALLCONTROL_CONTROL_CORE_H
#define BALLCONTROL_CONTROL_CORE_H

#include <stdbool.h>
#include <stdint.h>

#define BALL_CONTROL_CONFIDENCE_MAX 10000U

typedef enum {
    BALL_CONTROL_MODE_PD = 0,
    BALL_CONTROL_MODE_LQI = 1
} BallControl_Mode;

typedef enum {
    BALL_CONTROL_STATE_DISABLED = 0,
    BALL_CONTROL_STATE_WAIT_CALIBRATION,
    BALL_CONTROL_STATE_WAIT_VISION,
    BALL_CONTROL_STATE_WAIT_GYRO,
    BALL_CONTROL_STATE_WAIT_IMU,
    BALL_CONTROL_STATE_ACTIVE,
    BALL_CONTROL_STATE_FAULT
} BallControl_State;

typedef struct {
    bool valid;
    float origin_x_px;
    float origin_y_px;
    float axis_x;
    float axis_y;
    float mm_per_pixel;
} BallControl_VisionCalibration;

typedef struct {
    bool valid;
    int32_t motor_zero_units;
    float motor_units_per_beam_degree;
    float max_beam_angle_deg;
    uint16_t speed_rpm;
    uint8_t acceleration;
    uint32_t response_timeout_ms;
    uint32_t minimum_command_delta_units;
} BallControl_ActuatorCalibration;

typedef struct {
    bool valid;
    float angle_zero_deg;
    float angle_direction;
    float rate_direction;
    uint32_t timeout_ms;
} BallControl_GyroCalibration;

typedef struct {
    bool enabled;
    uint8_t acceleration_axis;
    float acceleration_direction;
    float angle_deg_per_g;
    uint32_t timeout_ms;
} BallControl_ImuFeedforward;

typedef struct {
    bool valid;
    float acceleration_noise_mm_s2;
    float measurement_variance_mm2;
    float age_variance_mm2_per_s;
    float initial_position_variance_mm2;
    float initial_velocity_variance_mm2_s2;
} BallControl_EstimatorConfig;

typedef struct {
    bool valid;
    BallControl_Mode mode;
    /* Position outer-loop gains: ball state -> beam angle setpoint. */
    float position_gain_deg_per_mm;
    float velocity_gain_deg_per_mm_s;
    /* Beam angle inner-loop gains: angle error/rate -> actuator angle. */
    float beam_angle_gain;
    float beam_rate_gain_s;
    float integral_gain_deg_per_mm_s;
    float integral_limit_mm_s;
} BallControl_FeedbackConfig;

typedef struct {
    BallControl_VisionCalibration vision;
    BallControl_ActuatorCalibration actuator;
    BallControl_GyroCalibration gyro;
    BallControl_ImuFeedforward imu_feedforward;
    BallControl_EstimatorConfig estimator;
    BallControl_FeedbackConfig feedback;
    uint16_t minimum_confidence;
    uint32_t maximum_measurement_age_ms;
} BallControl_Config;

typedef struct {
    bool gyro_valid;
    float gyro_angle_deg;
    float gyro_rate_dps;
    bool imu_acceleration_valid;
    float imu_acceleration_g;
} BallControl_SensorInput;

typedef struct {
    bool valid;
    float beam_angle_command_deg;
    float actuator_angle_command_deg;
    int32_t motor_target_units;
    bool saturated;
} BallControl_Output;

typedef struct {
    BallControl_State state;
    bool enable_requested;
    bool configuration_ready;
    bool estimator_initialized;
    bool vision_valid;
    float setpoint_mm;
    float position_mm;
    float velocity_mm_s;
    float position_error_mm;
    float integral_error_mm_s;
    float beam_angle_command_deg;
    float actuator_angle_command_deg;
    float beam_angle_error_deg;
    uint32_t last_frame_id;
    uint32_t accepted_measurement_count;
    uint32_t rejected_measurement_count;
    uint32_t saturated_output_count;
} BallControl_Status;

typedef struct {
    float position_mm;
    float velocity_mm_s;
    float covariance_00;
    float covariance_01;
    float covariance_10;
    float covariance_11;
    bool initialized;
} BallControl_Kalman;

typedef struct {
    BallControl_Config config;
    BallControl_Kalman kalman;
    BallControl_Status status;
} BallControl_Core;

void BallControlCore_LoadSafeDefaults(BallControl_Config *config);
bool BallControlCore_ConfigIsReady(const BallControl_Config *config);

void BallControlCore_Init(BallControl_Core *core,
                          const BallControl_Config *config);
void BallControlCore_Configure(BallControl_Core *core,
                               const BallControl_Config *config);
void BallControlCore_Reset(BallControl_Core *core);
void BallControlCore_SetEnabled(BallControl_Core *core, bool enabled);
void BallControlCore_SetSetpoint(BallControl_Core *core,
                                 float setpoint_mm);
void BallControlCore_InvalidateVision(BallControl_Core *core);

bool BallControlCore_ProjectVisionQ4(
    const BallControl_VisionCalibration *calibration,
    uint16_t target_x_q4, uint16_t target_y_q4,
    float *position_mm);

bool BallControlCore_AcceptVisionMeasurement(
    BallControl_Core *core, uint32_t frame_id,
    uint16_t target_x_q4, uint16_t target_y_q4,
    uint32_t measurement_age_ms);

void BallControlCore_Predict(BallControl_Core *core, float dt_s);

bool BallControlCore_Compute(
    BallControl_Core *core, const BallControl_SensorInput *input,
    float dt_s, BallControl_Output *output);

bool BallControlCore_BeamAngleToMotorUnits(
    const BallControl_ActuatorCalibration *calibration,
    float beam_angle_deg, int32_t *motor_target_units);

void BallControlCore_GetStatus(const BallControl_Core *core,
                               BallControl_Status *status);

#endif
