#include "single_axis_gyro.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#define FLOAT_TOLERANCE 0.01f

typedef struct {
    uint8_t writes[8][5];
    uint32_t write_lengths[8];
    uint32_t write_count;
    uint32_t delays[8];
    uint32_t delay_count;
} FakeIO_t;

static int g_failures;

#define CHECK(condition)                                                   \
    do {                                                                   \
        if (!(condition)) {                                                \
            (void) printf("FAIL line %d: %s\n", __LINE__, #condition);    \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

static bool FloatNear(float actual, float expected)
{
    float difference = actual - expected;
    if (difference < 0.0f) {
        difference = -difference;
    }
    return difference <= FLOAT_TOLERANCE;
}

static bool FakeWrite(void *context, const uint8_t *data, uint32_t length)
{
    FakeIO_t *fake = (FakeIO_t *) context;

    if ((fake == NULL) || (data == NULL) ||
        (fake->write_count >= ARRAY_SIZE(fake->writes)) ||
        (length > sizeof(fake->writes[0]))) {
        return false;
    }

    fake->write_lengths[fake->write_count] = length;
    (void) memcpy(fake->writes[fake->write_count], data, length);
    fake->write_count++;
    return true;
}

static void FakeDelay(void *context, uint32_t delay_ms)
{
    FakeIO_t *fake = (FakeIO_t *) context;

    if ((fake != NULL) &&
        (fake->delay_count < ARRAY_SIZE(fake->delays))) {
        fake->delays[fake->delay_count++] = delay_ms;
    }
}

static SingleAxisGyro_FrameResult_t FeedFrame(
    SingleAxisGyro_t *driver, const uint8_t frame[5])
{
    SingleAxisGyro_FrameResult_t result = SINGLE_AXIS_GYRO_FRAME_NONE;
    uint32_t index;

    for (index = 0U; index < 5U; ++index) {
        result = SingleAxisGyro_FeedByte(driver, frame[index]);
    }
    return result;
}

static void TestParserAndConversion(void)
{
    static const uint8_t rate_frame[5] =
        {0x5AU, 0xAAU, 0x00U, 0x40U, 0x44U};
    static const uint8_t yaw_frame[5] =
        {0x5AU, 0xBBU, 0x00U, 0xC0U, 0xD5U};
    SingleAxisGyro_t driver;
    SingleAxisGyro_Sample_t sample;

    SingleAxisGyro_Init(&driver, NULL, 0.0f);
    CHECK(!SingleAxisGyro_GetSample(&driver, &sample));
    CHECK(FeedFrame(&driver, rate_frame) == SINGLE_AXIS_GYRO_FRAME_RATE);
    CHECK(FeedFrame(&driver, yaw_frame) == SINGLE_AXIS_GYRO_FRAME_YAW);
    CHECK(SingleAxisGyro_GetSample(&driver, &sample));
    CHECK(sample.raw_angular_rate == 16384);
    CHECK(sample.raw_yaw == -16384);
    CHECK(FloatNear(sample.angular_rate_dps, 1000.0f));
    CHECK(FloatNear(sample.yaw_deg, -90.0f));
    CHECK(sample.valid_frame_count == 2U);
    CHECK(sample.valid_mask ==
        (SINGLE_AXIS_GYRO_VALID_RATE | SINGLE_AXIS_GYRO_VALID_YAW));
}

static void TestChecksumAndResynchronization(void)
{
    static const uint8_t bad_then_header[5] =
        {0x5AU, 0xAAU, 0x01U, 0x00U, 0x5AU};
    static const uint8_t rate_tail[4] =
        {0xAAU, 0x00U, 0x40U, 0x44U};
    SingleAxisGyro_t driver;
    SingleAxisGyro_Sample_t sample;
    SingleAxisGyro_FrameResult_t result = SINGLE_AXIS_GYRO_FRAME_NONE;
    uint32_t index;

    SingleAxisGyro_Init(&driver, NULL, 400.0f);
    CHECK(FeedFrame(&driver, bad_then_header) ==
        SINGLE_AXIS_GYRO_FRAME_CHECKSUM_ERROR);
    for (index = 0U; index < ARRAY_SIZE(rate_tail); ++index) {
        result = SingleAxisGyro_FeedByte(&driver, rate_tail[index]);
    }
    CHECK(result == SINGLE_AXIS_GYRO_FRAME_RATE);
    CHECK(SingleAxisGyro_GetSample(&driver, &sample));
    CHECK(sample.checksum_error_count == 1U);
    CHECK(sample.valid_frame_count == 1U);
    CHECK(FloatNear(sample.angular_rate_dps, 200.0f));
}

static void TestCommands(void)
{
    static const uint8_t expected_unlock[5] =
        {0x55U, 0xAAU, 0x13U, 0x8EU, 0x5FU};
    static const uint8_t expected_rate[5] =
        {0x55U, 0xAAU, 0x02U, 0x08U, 0x00U};
    static const uint8_t expected_save[5] =
        {0x55U, 0xAAU, 0x00U, 0x00U, 0x00U};
    FakeIO_t fake = {0};
    SingleAxisGyro_IO_t io;
    SingleAxisGyro_t driver;

    io.write = FakeWrite;
    io.delay_ms = FakeDelay;
    io.context = &fake;
    SingleAxisGyro_Init(&driver, &io, 2000.0f);
    CHECK(SingleAxisGyro_SetOutputRate(
        &driver, SINGLE_AXIS_GYRO_RATE_50_HZ) ==
        SINGLE_AXIS_GYRO_STATUS_OK);
    CHECK(fake.write_count == 3U);
    CHECK(fake.delay_count == 2U);
    CHECK(fake.delays[0] == 100U);
    CHECK(fake.delays[1] == 100U);
    CHECK(fake.write_lengths[0] == sizeof(expected_unlock));
    CHECK(memcmp(fake.writes[0], expected_unlock,
        sizeof(expected_unlock)) == 0);
    CHECK(memcmp(fake.writes[1], expected_rate,
        sizeof(expected_rate)) == 0);
    CHECK(memcmp(fake.writes[2], expected_save,
        sizeof(expected_save)) == 0);
}

int main(void)
{
    TestParserAndConversion();
    TestChecksumAndResynchronization();
    TestCommands();

    if (g_failures != 0) {
        (void) printf("%d test(s) failed\n", g_failures);
        return 1;
    }

    (void) printf("single_axis_gyro tests passed\n");
    return 0;
}
