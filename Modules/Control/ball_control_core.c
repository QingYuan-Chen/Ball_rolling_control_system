#include "ball_control_core.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define BALL_CONTROL_Q4_SCALE 16.0f
#define BALL_CONTROL_MIN_AXIS_NORM 0.0001f
#define BALL_CONTROL_MIN_VARIANCE 0.000001f

static float ClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static bool IsFinitePositive(float value)
{
    return isfinite(value) && (value > 0.0f);
}

static bool VisionCalibrationIsValid(
    const BallControl_VisionCalibration *calibration)
{
    float norm_squared;

    if ((calibration == NULL) || !calibration->valid ||
        !isfinite(calibration->origin_x_px) ||
        !isfinite(calibration->origin_y_px) ||
        !isfinite(calibration->axis_x) ||
        !isfinite(calibration->axis_y) ||
        !IsFinitePositive(calibration->mm_per_pixel)) {
        return false;
    }

    norm_squared = calibration->axis_x * calibration->axis_x +
                   calibration->axis_y * calibration->axis_y;
    return norm_squared > BALL_CONTROL_MIN_AXIS_NORM;
}

static bool ActuatorCalibrationIsValid(
    const BallControl_ActuatorCalibration *calibration)
{
    return (calibration != NULL) && calibration->valid &&
           isfinite(calibration->motor_units_per_beam_degree) &&
           (fabsf(calibration->motor_units_per_beam_degree) > 0.0f) &&
           IsFinitePositive(calibration->max_beam_angle_deg) &&
           (calibration->speed_rpm > 0U) &&
           (calibration->speed_rpm <= 5000U) &&
           (calibration->acceleration > 0U) &&
           (calibration->response_timeout_ms > 0U);
}

static bool GyroCalibrationIsValid(
    const BallControl_GyroCalibration *calibration)
{
    return (calibration != NULL) && calibration->valid &&
           isfinite(calibration->angle_zero_deg) &&
           isfinite(calibration->angle_direction) &&
           isfinite(calibration->rate_direction) &&
           (fabsf(calibration->angle_direction) > 0.0f) &&
           (fabsf(calibration->rate_direction) > 0.0f) &&
           (calibration->timeout_ms > 0U);
}

static bool EstimatorConfigIsValid(
    const BallControl_EstimatorConfig *config)
{
    return (config != NULL) && config->valid &&
           IsFinitePositive(config->acceleration_noise_mm_s2) &&
           IsFinitePositive(config->measurement_variance_mm2) &&
           isfinite(config->age_variance_mm2_per_s) &&
           (config->age_variance_mm2_per_s >= 0.0f) &&
           IsFinitePositive(config->initial_position_variance_mm2) &&
           IsFinitePositive(config->initial_velocity_variance_mm2_s2);
}

static bool FeedbackConfigIsValid(
    const BallControl_FeedbackConfig *config)
{
    if ((config == NULL) || !config->valid ||
        ((config->mode != BALL_CONTROL_MODE_PD) &&
         (config->mode != BALL_CONTROL_MODE_LQI)) ||
        !isfinite(config->position_gain_deg_per_mm) ||
        !isfinite(config->velocity_gain_deg_per_mm_s) ||
        !isfinite(config->beam_angle_gain) ||
        !isfinite(config->beam_rate_gain_s) ||
        !isfinite(config->integral_gain_deg_per_mm_s) ||
        !isfinite(config->integral_limit_mm_s) ||
        (config->beam_angle_gain <= 0.0f) ||
        (config->beam_rate_gain_s < 0.0f)) {
        return false;
    }

    if (config->mode == BALL_CONTROL_MODE_LQI) {
        return config->integral_limit_mm_s > 0.0f;
    }
    return true;
}

static bool ImuFeedforwardConfigIsValid(
    const BallControl_ImuFeedforward *config)
{
    if ((config == NULL) || !config->enabled) {
        return true;
    }
    return (config->acceleration_axis < 3U) &&
           isfinite(config->acceleration_direction) &&
           (fabsf(config->acceleration_direction) > 0.0f) &&
           isfinite(config->angle_deg_per_g) &&
           (config->timeout_ms > 0U);
}

static void KalmanReset(BallControl_Core *core)
{
    (void) memset(&core->kalman, 0, sizeof(core->kalman));
    core->status.estimator_initialized = false;
    core->status.position_mm = 0.0f;
    core->status.velocity_mm_s = 0.0f;
}

static void KalmanInitialize(BallControl_Core *core, float position_mm)
{
    core->kalman.position_mm = position_mm;
    core->kalman.velocity_mm_s = 0.0f;
    core->kalman.covariance_00 =
        core->config.estimator.initial_position_variance_mm2;
    core->kalman.covariance_01 = 0.0f;
    core->kalman.covariance_10 = 0.0f;
    core->kalman.covariance_11 =
        core->config.estimator.initial_velocity_variance_mm2_s2;
    core->kalman.initialized = true;
    core->status.estimator_initialized = true;
}

static void KalmanCorrect(BallControl_Core *core, float measurement_mm,
                          float measurement_variance_mm2)
{
    BallControl_Kalman *filter = &core->kalman;
    float innovation_variance =
        filter->covariance_00 + measurement_variance_mm2;
    float gain_position;
    float gain_velocity;
    float innovation;
    float a00;
    float a10;
    float p00;
    float p01;
    float p10;
    float p11;

    if (innovation_variance < BALL_CONTROL_MIN_VARIANCE) {
        innovation_variance = BALL_CONTROL_MIN_VARIANCE;
    }

    gain_position = filter->covariance_00 / innovation_variance;
    gain_velocity = filter->covariance_10 / innovation_variance;
    innovation = measurement_mm - filter->position_mm;
    filter->position_mm += gain_position * innovation;
    filter->velocity_mm_s += gain_velocity * innovation;

    a00 = 1.0f - gain_position;
    a10 = -gain_velocity;
    p00 = a00 * a00 * filter->covariance_00 +
          gain_position * gain_position * measurement_variance_mm2;
    p01 = a00 * (a10 * filter->covariance_00 +
                 filter->covariance_01) +
          gain_position * gain_velocity * measurement_variance_mm2;
    p10 = a00 * (a10 * filter->covariance_00 +
                 filter->covariance_10) +
          gain_position * gain_velocity * measurement_variance_mm2;
    p11 = a10 * (a10 * filter->covariance_00 +
                 filter->covariance_01 + filter->covariance_10) +
          filter->covariance_11 +
          gain_velocity * gain_velocity * measurement_variance_mm2;
    filter->covariance_00 = p00;
    filter->covariance_01 = 0.5f * (p01 + p10);
    filter->covariance_10 = filter->covariance_01;
    filter->covariance_11 = p11;
}

void BallControlCore_LoadSafeDefaults(BallControl_Config *config)
{
    if (config == NULL) {
        return;
    }

    (void) memset(config, 0, sizeof(*config));
    config->feedback.mode = BALL_CONTROL_MODE_PD;
    config->minimum_confidence = 0U;
    config->maximum_measurement_age_ms = 100U;
}

bool BallControlCore_ConfigIsReady(const BallControl_Config *config)
{
    if (config == NULL) {
        return false;
    }
    return VisionCalibrationIsValid(&config->vision) &&
           ActuatorCalibrationIsValid(&config->actuator) &&
           GyroCalibrationIsValid(&config->gyro) &&
           EstimatorConfigIsValid(&config->estimator) &&
           FeedbackConfigIsValid(&config->feedback) &&
           ImuFeedforwardConfigIsValid(&config->imu_feedforward) &&
           (config->minimum_confidence <=
            BALL_CONTROL_CONFIDENCE_MAX) &&
           (config->maximum_measurement_age_ms > 0U);
}

void BallControlCore_Init(BallControl_Core *core,
                          const BallControl_Config *config)
{
    if (core == NULL) {
        return;
    }

    (void) memset(core, 0, sizeof(*core));
    if (config != NULL) {
        core->config = *config;
    } else {
        BallControlCore_LoadSafeDefaults(&core->config);
    }
    core->status.state = BALL_CONTROL_STATE_DISABLED;
    core->status.configuration_ready =
        BallControlCore_ConfigIsReady(&core->config);
}

void BallControlCore_Configure(BallControl_Core *core,
                               const BallControl_Config *config)
{
    bool enable_requested;
    float setpoint_mm;

    if ((core == NULL) || (config == NULL)) {
        return;
    }

    enable_requested = core->status.enable_requested;
    setpoint_mm = core->status.setpoint_mm;
    core->config = *config;
    BallControlCore_Reset(core);
    core->status.enable_requested = enable_requested;
    core->status.setpoint_mm = setpoint_mm;
    core->status.configuration_ready =
        BallControlCore_ConfigIsReady(&core->config);
}

void BallControlCore_Reset(BallControl_Core *core)
{
    bool enable_requested;
    bool configuration_ready;
    float setpoint_mm;

    if (core == NULL) {
        return;
    }

    enable_requested = core->status.enable_requested;
    configuration_ready =
        BallControlCore_ConfigIsReady(&core->config);
    setpoint_mm = core->status.setpoint_mm;
    (void) memset(&core->status, 0, sizeof(core->status));
    core->status.enable_requested = enable_requested;
    core->status.configuration_ready = configuration_ready;
    core->status.setpoint_mm = setpoint_mm;
    core->status.state = enable_requested ?
        BALL_CONTROL_STATE_WAIT_CALIBRATION :
        BALL_CONTROL_STATE_DISABLED;
    KalmanReset(core);
}

void BallControlCore_SetEnabled(BallControl_Core *core, bool enabled)
{
    if (core == NULL) {
        return;
    }

    core->status.enable_requested = enabled;
    if (!enabled) {
        core->status.state = BALL_CONTROL_STATE_DISABLED;
        core->status.integral_error_mm_s = 0.0f;
        core->status.beam_angle_command_deg = 0.0f;
        core->status.actuator_angle_command_deg = 0.0f;
        core->status.beam_angle_error_deg = 0.0f;
    }
}

void BallControlCore_SetSetpoint(BallControl_Core *core,
                                 float setpoint_mm)
{
    if ((core != NULL) && isfinite(setpoint_mm)) {
        core->status.setpoint_mm = setpoint_mm;
    }
}

void BallControlCore_InvalidateVision(BallControl_Core *core)
{
    if (core != NULL) {
        core->status.vision_valid = false;
    }
}

bool BallControlCore_ProjectVisionQ4(
    const BallControl_VisionCalibration *calibration,
    uint16_t target_x_q4, uint16_t target_y_q4,
    float *position_mm)
{
    float axis_norm;
    float delta_x;
    float delta_y;

    if ((position_mm == NULL) ||
        !VisionCalibrationIsValid(calibration)) {
        return false;
    }

    axis_norm = sqrtf(calibration->axis_x * calibration->axis_x +
                      calibration->axis_y * calibration->axis_y);
    delta_x = (float) target_x_q4 / BALL_CONTROL_Q4_SCALE -
              calibration->origin_x_px;
    delta_y = (float) target_y_q4 / BALL_CONTROL_Q4_SCALE -
              calibration->origin_y_px;
    *position_mm =
        (delta_x * calibration->axis_x +
         delta_y * calibration->axis_y) /
        axis_norm * calibration->mm_per_pixel;
    return isfinite(*position_mm);
}

bool BallControlCore_AcceptVisionMeasurement(
    BallControl_Core *core, uint32_t frame_id,
    uint16_t target_x_q4, uint16_t target_y_q4,
    uint32_t measurement_age_ms)
{
    float measurement_mm;
    float measurement_variance;
    float age_s;

    if ((core == NULL) ||
        !EstimatorConfigIsValid(&core->config.estimator) ||
        (measurement_age_ms > core->config.maximum_measurement_age_ms) ||
        !BallControlCore_ProjectVisionQ4(
            &core->config.vision, target_x_q4, target_y_q4,
            &measurement_mm)) {
        if (core != NULL) {
            core->status.rejected_measurement_count++;
            core->status.vision_valid = false;
        }
        return false;
    }

    if (!core->kalman.initialized) {
        KalmanInitialize(core, measurement_mm);
    } else {
        age_s = (float) measurement_age_ms * 0.001f;
        measurement_mm += core->kalman.velocity_mm_s * age_s;
        measurement_variance =
            core->config.estimator.measurement_variance_mm2 +
            core->config.estimator.age_variance_mm2_per_s * age_s;
        KalmanCorrect(core, measurement_mm, measurement_variance);
    }

    core->status.last_frame_id = frame_id;
    core->status.accepted_measurement_count++;
    core->status.vision_valid = true;
    core->status.position_mm = core->kalman.position_mm;
    core->status.velocity_mm_s = core->kalman.velocity_mm_s;
    return true;
}

void BallControlCore_Predict(BallControl_Core *core, float dt_s)
{
    BallControl_Kalman *filter;
    float acceleration_variance;
    float dt2;
    float dt3;
    float dt4;
    float p00;
    float p01;
    float p10;
    float p11;

    if ((core == NULL) || !core->kalman.initialized ||
        !IsFinitePositive(dt_s)) {
        return;
    }

    filter = &core->kalman;
    acceleration_variance =
        core->config.estimator.acceleration_noise_mm_s2 *
        core->config.estimator.acceleration_noise_mm_s2;
    dt2 = dt_s * dt_s;
    dt3 = dt2 * dt_s;
    dt4 = dt2 * dt2;

    filter->position_mm += filter->velocity_mm_s * dt_s;
    p00 = filter->covariance_00 +
          dt_s * (filter->covariance_01 +
                  filter->covariance_10) +
          dt2 * filter->covariance_11 +
          0.25f * dt4 * acceleration_variance;
    p01 = filter->covariance_01 +
          dt_s * filter->covariance_11 +
          0.5f * dt3 * acceleration_variance;
    p10 = filter->covariance_10 +
          dt_s * filter->covariance_11 +
          0.5f * dt3 * acceleration_variance;
    p11 = filter->covariance_11 + dt2 * acceleration_variance;
    filter->covariance_00 = p00;
    filter->covariance_01 = 0.5f * (p01 + p10);
    filter->covariance_10 = filter->covariance_01;
    filter->covariance_11 = p11;

    core->status.position_mm = filter->position_mm;
    core->status.velocity_mm_s = filter->velocity_mm_s;
}

bool BallControlCore_BeamAngleToMotorUnits(
    const BallControl_ActuatorCalibration *calibration,
    float beam_angle_deg, int32_t *motor_target_units)
{
    float clamped_angle;
    float target;

    if ((motor_target_units == NULL) ||
        !ActuatorCalibrationIsValid(calibration) ||
        !isfinite(beam_angle_deg)) {
        return false;
    }

    clamped_angle = ClampFloat(
        beam_angle_deg, -calibration->max_beam_angle_deg,
        calibration->max_beam_angle_deg);
    target = (float) calibration->motor_zero_units +
             clamped_angle *
             calibration->motor_units_per_beam_degree;
    if (!isfinite(target) || (target > (float) INT32_MAX) ||
        (target < (float) INT32_MIN)) {
        return false;
    }

    target += (target >= 0.0f) ? 0.5f : -0.5f;
    *motor_target_units = (int32_t) target;
    return true;
}

bool BallControlCore_Compute(
    BallControl_Core *core, const BallControl_SensorInput *input,
    float dt_s, BallControl_Output *output)
{
    const BallControl_FeedbackConfig *feedback;
    float error;
    float integral;
    float beam_angle_command;
    float clamped_beam_angle_command;
    float beam_angle_error;
    float actuator_angle_command;
    float clamped_actuator_angle_command;

    if (output != NULL) {
        (void) memset(output, 0, sizeof(*output));
    }
    if ((core == NULL) || (input == NULL) || (output == NULL) ||
        !IsFinitePositive(dt_s)) {
        return false;
    }

    core->status.configuration_ready =
        BallControlCore_ConfigIsReady(&core->config);
    if (!core->status.enable_requested) {
        core->status.state = BALL_CONTROL_STATE_DISABLED;
        core->status.integral_error_mm_s = 0.0f;
        return false;
    }
    if (!core->status.configuration_ready) {
        core->status.state = BALL_CONTROL_STATE_WAIT_CALIBRATION;
        return false;
    }
    if (!core->status.vision_valid || !core->kalman.initialized) {
        core->status.state = BALL_CONTROL_STATE_WAIT_VISION;
        core->status.integral_error_mm_s = 0.0f;
        return false;
    }
    if (!input->gyro_valid) {
        core->status.state = BALL_CONTROL_STATE_WAIT_GYRO;
        core->status.integral_error_mm_s = 0.0f;
        return false;
    }
    if (core->config.imu_feedforward.enabled &&
        !input->imu_acceleration_valid) {
        core->status.state = BALL_CONTROL_STATE_WAIT_IMU;
        core->status.integral_error_mm_s = 0.0f;
        return false;
    }

    feedback = &core->config.feedback;
    error = core->status.setpoint_mm - core->kalman.position_mm;
    integral = core->status.integral_error_mm_s;
    if (feedback->mode == BALL_CONTROL_MODE_LQI) {
        integral = ClampFloat(
            integral + error * dt_s,
            -feedback->integral_limit_mm_s,
            feedback->integral_limit_mm_s);
    } else {
        integral = 0.0f;
    }

    beam_angle_command =
        feedback->position_gain_deg_per_mm * error -
        feedback->velocity_gain_deg_per_mm_s *
            core->kalman.velocity_mm_s;
    if (feedback->mode == BALL_CONTROL_MODE_LQI) {
        beam_angle_command +=
            feedback->integral_gain_deg_per_mm_s * integral;
    }
    if (core->config.imu_feedforward.enabled) {
        beam_angle_command +=
            core->config.imu_feedforward.angle_deg_per_g *
            input->imu_acceleration_g;
    }

    clamped_beam_angle_command = ClampFloat(
        beam_angle_command, -core->config.actuator.max_beam_angle_deg,
        core->config.actuator.max_beam_angle_deg);
    beam_angle_error =
        clamped_beam_angle_command - input->gyro_angle_deg;
    actuator_angle_command =
        clamped_beam_angle_command +
        feedback->beam_angle_gain * beam_angle_error -
        feedback->beam_rate_gain_s * input->gyro_rate_dps;
    clamped_actuator_angle_command = ClampFloat(
        actuator_angle_command,
        -core->config.actuator.max_beam_angle_deg,
        core->config.actuator.max_beam_angle_deg);
    output->saturated =
        (clamped_beam_angle_command != beam_angle_command) ||
        (clamped_actuator_angle_command != actuator_angle_command);
    output->beam_angle_command_deg = clamped_beam_angle_command;
    output->actuator_angle_command_deg =
        clamped_actuator_angle_command;
    if (!BallControlCore_BeamAngleToMotorUnits(
            &core->config.actuator, clamped_actuator_angle_command,
            &output->motor_target_units)) {
        core->status.state = BALL_CONTROL_STATE_FAULT;
        return false;
    }

    output->valid = true;
    core->status.state = BALL_CONTROL_STATE_ACTIVE;
    core->status.integral_error_mm_s = integral;
    core->status.position_error_mm = error;
    core->status.position_mm = core->kalman.position_mm;
    core->status.velocity_mm_s = core->kalman.velocity_mm_s;
    core->status.beam_angle_command_deg =
        clamped_beam_angle_command;
    core->status.actuator_angle_command_deg =
        clamped_actuator_angle_command;
    core->status.beam_angle_error_deg = beam_angle_error;
    if (output->saturated) {
        core->status.saturated_output_count++;
    }
    return true;
}

void BallControlCore_GetStatus(const BallControl_Core *core,
                               BallControl_Status *status)
{
    if ((core != NULL) && (status != NULL)) {
        *status = core->status;
    }
}
