#include "ball_control_core.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define FLOAT_TOLERANCE 0.05f

static int g_failures;

#define CHECK(condition)                                                   \
    do {                                                                   \
        if (!(condition)) {                                                \
            (void) printf("FAIL line %d: %s\n", __LINE__, #condition);    \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

static bool FloatNear(float actual, float expected, float tolerance)
{
    float difference = actual - expected;

    if (difference < 0.0f) {
        difference = -difference;
    }
    return difference <= tolerance;
}

static BallControl_Config MakeValidConfig(void)
{
    BallControl_Config config;

    BallControlCore_LoadSafeDefaults(&config);
    config.vision.valid = true;
    config.vision.origin_x_px = 640.0f;
    config.vision.origin_y_px = 360.0f;
    config.vision.axis_x = 1.0f;
    config.vision.axis_y = 0.0f;
    config.vision.mm_per_pixel = 0.2f;

    config.actuator.valid = true;
    config.actuator.motor_zero_units = 1000;
    config.actuator.motor_units_per_beam_degree = 100.0f;
    config.actuator.max_beam_angle_deg = 8.0f;
    config.actuator.speed_rpm = 100U;
    config.actuator.acceleration = 10U;
    config.actuator.response_timeout_ms = 20U;
    config.actuator.minimum_command_delta_units = 1U;

    config.gyro.valid = true;
    config.gyro.angle_zero_deg = 0.0f;
    config.gyro.angle_direction = 1.0f;
    config.gyro.rate_direction = 1.0f;
    config.gyro.timeout_ms = 30U;

    config.estimator.valid = true;
    config.estimator.acceleration_noise_mm_s2 = 200.0f;
    config.estimator.measurement_variance_mm2 = 1.0f;
    config.estimator.age_variance_mm2_per_s = 10.0f;
    config.estimator.initial_position_variance_mm2 = 4.0f;
    config.estimator.initial_velocity_variance_mm2_s2 = 400.0f;

    config.feedback.valid = true;
    config.feedback.mode = BALL_CONTROL_MODE_PD;
    config.feedback.position_gain_deg_per_mm = 1.0f;
    config.feedback.velocity_gain_deg_per_mm_s = 0.0f;
    config.feedback.beam_angle_gain = 0.5f;
    config.feedback.beam_rate_gain_s = 0.1f;
    config.feedback.integral_limit_mm_s = 0.0f;
    config.minimum_confidence = 5000U;
    config.maximum_measurement_age_ms = 100U;
    return config;
}

static void TestSafeDefaultsAndValidation(void)
{
    BallControl_Config config;

    BallControlCore_LoadSafeDefaults(&config);
    CHECK(!BallControlCore_ConfigIsReady(&config));
    CHECK(config.maximum_measurement_age_ms == 100U);
    config = MakeValidConfig();
    CHECK(BallControlCore_ConfigIsReady(&config));
    config.feedback.beam_angle_gain = 0.0f;
    CHECK(!BallControlCore_ConfigIsReady(&config));
}

static void TestVisionProjectionAndActuatorConversion(void)
{
    BallControl_Config config = MakeValidConfig();
    float position_mm = 0.0f;
    int32_t motor_units = 0;

    CHECK(BallControlCore_ProjectVisionQ4(
        &config.vision, 650U * 16U, 400U * 16U, &position_mm));
    CHECK(FloatNear(position_mm, 2.0f, FLOAT_TOLERANCE));

    CHECK(BallControlCore_BeamAngleToMotorUnits(
        &config.actuator, 2.25f, &motor_units));
    CHECK(motor_units == 1225);
    CHECK(BallControlCore_BeamAngleToMotorUnits(
        &config.actuator, 20.0f, &motor_units));
    CHECK(motor_units == 1800);
}

static void TestKalmanVelocityEstimate(void)
{
    BallControl_Config config = MakeValidConfig();
    BallControl_Core core;
    BallControl_Status status;
    uint32_t sample;

    BallControlCore_Init(&core, &config);
    CHECK(BallControlCore_AcceptVisionMeasurement(
        &core, 0U, 640U * 16U, 360U * 16U, 0U));
    for (sample = 1U; sample <= 100U; ++sample) {
        uint32_t prediction;
        uint16_t x_q4 = (uint16_t) ((640U + sample * 2U) * 16U);

        for (prediction = 0U; prediction < 4U; ++prediction) {
            BallControlCore_Predict(&core, 0.005f);
        }
        CHECK(BallControlCore_AcceptVisionMeasurement(
            &core, sample, x_q4, 360U * 16U, 0U));
    }

    BallControlCore_GetStatus(&core, &status);
    CHECK(status.estimator_initialized);
    CHECK(status.accepted_measurement_count == 101U);
    CHECK(FloatNear(status.position_mm, 40.0f, 0.5f));
    CHECK(FloatNear(status.velocity_mm_s, 20.0f, 2.0f));
}

static void TestPdOutputAndSafetyGates(void)
{
    BallControl_Config config = MakeValidConfig();
    BallControl_Core core;
    BallControl_SensorInput input = {0};
    BallControl_Output output;
    BallControl_Status status;

    BallControlCore_Init(&core, &config);
    CHECK(BallControlCore_AcceptVisionMeasurement(
        &core, 1U, 650U * 16U, 360U * 16U, 0U));
    CHECK(!BallControlCore_Compute(&core, &input, 0.01f, &output));
    BallControlCore_GetStatus(&core, &status);
    CHECK(status.state == BALL_CONTROL_STATE_DISABLED);

    BallControlCore_SetEnabled(&core, true);
    CHECK(!BallControlCore_Compute(&core, &input, 0.01f, &output));
    BallControlCore_GetStatus(&core, &status);
    CHECK(status.state == BALL_CONTROL_STATE_WAIT_GYRO);

    input.gyro_valid = true;
    CHECK(BallControlCore_Compute(&core, &input, 0.01f, &output));
    CHECK(output.valid);
    CHECK(FloatNear(output.beam_angle_command_deg, -2.0f,
                    FLOAT_TOLERANCE));
    CHECK(FloatNear(output.actuator_angle_command_deg, -3.0f,
                    FLOAT_TOLERANCE));
    CHECK(output.motor_target_units == 700);

    BallControlCore_SetSetpoint(&core, 2.0f);
    input.gyro_angle_deg = 2.0f;
    input.gyro_rate_dps = 1.0f;
    CHECK(BallControlCore_Compute(&core, &input, 0.01f, &output));
    CHECK(FloatNear(output.beam_angle_command_deg, 0.0f,
                    FLOAT_TOLERANCE));
    CHECK(FloatNear(output.actuator_angle_command_deg, -1.1f,
                    FLOAT_TOLERANCE));
    CHECK(output.motor_target_units == 890);

    BallControlCore_SetSetpoint(&core, 100.0f);
    input.gyro_angle_deg = 0.0f;
    input.gyro_rate_dps = 0.0f;
    CHECK(BallControlCore_Compute(&core, &input, 0.01f, &output));
    CHECK(output.saturated);
    CHECK(FloatNear(output.beam_angle_command_deg, 8.0f,
                    FLOAT_TOLERANCE));
    CHECK(FloatNear(output.actuator_angle_command_deg, 8.0f,
                    FLOAT_TOLERANCE));
    CHECK(output.motor_target_units == 1800);
}

static void TestLqiAndImuFeedforward(void)
{
    BallControl_Config config = MakeValidConfig();
    BallControl_Core core;
    BallControl_SensorInput input = {0};
    BallControl_Output output;

    config.feedback.mode = BALL_CONTROL_MODE_LQI;
    config.feedback.position_gain_deg_per_mm = 0.0f;
    config.feedback.integral_gain_deg_per_mm_s = 1.0f;
    config.feedback.integral_limit_mm_s = 2.0f;
    config.imu_feedforward.enabled = true;
    config.imu_feedforward.acceleration_axis = 0U;
    config.imu_feedforward.acceleration_direction = 1.0f;
    config.imu_feedforward.angle_deg_per_g = 10.0f;
    config.imu_feedforward.timeout_ms = 100U;
    CHECK(BallControlCore_ConfigIsReady(&config));

    BallControlCore_Init(&core, &config);
    CHECK(BallControlCore_AcceptVisionMeasurement(
        &core, 1U, 640U * 16U, 360U * 16U, 0U));
    BallControlCore_SetSetpoint(&core, 10.0f);
    BallControlCore_SetEnabled(&core, true);
    input.gyro_valid = true;
    CHECK(!BallControlCore_Compute(&core, &input, 0.01f, &output));

    input.imu_acceleration_valid = true;
    input.imu_acceleration_g = 0.2f;
    CHECK(BallControlCore_Compute(&core, &input, 0.01f, &output));
    CHECK(FloatNear(output.beam_angle_command_deg, 2.1f,
                    FLOAT_TOLERANCE));
    CHECK(FloatNear(output.actuator_angle_command_deg, 3.15f,
                    FLOAT_TOLERANCE));
}

static void TestMeasurementAgeRejection(void)
{
    BallControl_Config config = MakeValidConfig();
    BallControl_Core core;
    BallControl_Status status;

    BallControlCore_Init(&core, &config);
    CHECK(!BallControlCore_AcceptVisionMeasurement(
        &core, 1U, 640U * 16U, 360U * 16U, 101U));
    BallControlCore_GetStatus(&core, &status);
    CHECK(!status.vision_valid);
    CHECK(status.rejected_measurement_count == 1U);
}

static void TestInvalidatedVisionStopsOutput(void)
{
    BallControl_Config config = MakeValidConfig();
    BallControl_Core core;
    BallControl_SensorInput input = {0};
    BallControl_Output output;
    BallControl_Status status;

    BallControlCore_Init(&core, &config);
    CHECK(BallControlCore_AcceptVisionMeasurement(
        &core, 1U, 640U * 16U, 360U * 16U, 0U));
    input.gyro_valid = true;
    BallControlCore_SetEnabled(&core, true);
    CHECK(BallControlCore_Compute(&core, &input, 0.01f, &output));

    BallControlCore_InvalidateVision(&core);
    CHECK(!BallControlCore_Compute(&core, &input, 0.01f, &output));
    CHECK(!output.valid);
    BallControlCore_GetStatus(&core, &status);
    CHECK(status.state == BALL_CONTROL_STATE_WAIT_VISION);
}

int main(void)
{
    TestSafeDefaultsAndValidation();
    TestVisionProjectionAndActuatorConversion();
    TestKalmanVelocityEstimate();
    TestPdOutputAndSafetyGates();
    TestLqiAndImuFeedforward();
    TestMeasurementAgeRejection();
    TestInvalidatedVisionStopsOutput();

    if (g_failures != 0) {
        (void) printf("%d test(s) failed\n", g_failures);
        return 1;
    }

    (void) printf("ball_control_core tests passed\n");
    return 0;
}
