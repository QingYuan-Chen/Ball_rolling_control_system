#include "raspberry_pi_vision_protocol.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#define TARGET_V1_FRAME_SIZE 42U
#define TARGET_V2_FRAME_SIZE 50U
#define HEARTBEAT_V1_FRAME_SIZE 24U
#define HEARTBEAT_V2_FRAME_SIZE 36U

static int g_failures;

#define CHECK(condition)                                                     \
    do {                                                                     \
        if (!(condition)) {                                                  \
            (void) printf("FAIL line %d: %s\n", __LINE__, #condition);      \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

static const uint8_t g_golden_target_v1[TARGET_V1_FRAME_SIZE] = {
    0xAAU, 0x55U, 0x01U, 0x11U, 0x2AU, 0x00U, 0x20U, 0x00U,
    0xD2U, 0x04U, 0x00U, 0x00U, 0x07U, 0x00U, 0x1FU, 0x00U,
    0x00U, 0x00U, 0x12U, 0x24U, 0xECU, 0x27U, 0x78U, 0x16U,
    0x08U, 0x06U, 0x84U, 0x07U, 0x83U, 0xFFU, 0xFFU, 0xFFU,
    0x50U, 0x00U, 0x00U, 0x00U, 0x12U, 0x00U, 0x1EU, 0x00U,
    0x08U, 0xE8U
};

static const uint8_t g_golden_target_v2[TARGET_V2_FRAME_SIZE] = {
    0xAAU, 0x55U, 0x01U, 0x12U, 0x2AU, 0x00U, 0x28U, 0x00U,
    0xD2U, 0x04U, 0x00U, 0x00U, 0x13U, 0x09U, 0x42U, 0x5BU,
    0x98U, 0x01U, 0x00U, 0x00U, 0x07U, 0x00U, 0x1FU, 0x00U,
    0x00U, 0x00U, 0x12U, 0x24U, 0xECU, 0x27U, 0x78U, 0x16U,
    0x08U, 0x06U, 0x84U, 0x07U, 0x83U, 0xFFU, 0xFFU, 0xFFU,
    0x50U, 0x00U, 0x00U, 0x00U, 0x12U, 0x00U, 0x1EU, 0x00U,
    0xA5U, 0x2DU
};

static const uint8_t g_golden_heartbeat_v2[HEARTBEAT_V2_FRAME_SIZE] = {
    0xAAU, 0x55U, 0x01U, 0x02U, 0x2BU, 0x00U, 0x1AU, 0x00U,
    0xE8U, 0x03U, 0x00U, 0x00U, 0x77U, 0x00U, 0x00U, 0x3CU,
    0x00U, 0x3CU, 0x02U, 0x00U, 0x00U, 0x00U, 0xC0U, 0xA8U,
    0x05U, 0x94U, 0x60U, 0x0AU, 0x42U, 0x5BU, 0x98U, 0x01U,
    0x00U, 0x00U, 0x32U, 0x0FU
};

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t) value;
    data[1] = (uint8_t) (value >> 8U);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t) value;
    data[1] = (uint8_t) (value >> 8U);
    data[2] = (uint8_t) (value >> 16U);
    data[3] = (uint8_t) (value >> 24U);
}

static void update_crc(uint8_t *frame, uint32_t frame_length)
{
    uint16_t crc = RPV_Crc16CcittFalse(&frame[2], frame_length - 4U);
    write_u16_le(&frame[frame_length - 2U], crc);
}

static RPV_FeedResult_t feed(const uint8_t *data, uint32_t length,
    RPV_Parser_t *parser, uint32_t now_ms)
{
    RPV_FeedResult_t result = RPV_FEED_NONE;
    uint32_t index;

    for (index = 0U; index < length; ++index) {
        result = RPV_FeedByte(parser, data[index], now_ms);
    }
    return result;
}

static void test_golden_target(void)
{
    static const uint8_t noise[] = {0x00U, 0xAAU, 0x00U, 0x12U};
    RPV_Parser_t parser;
    RPV_TargetSnapshot_t target;
    RPV_Stats_t stats;

    RPV_Init(&parser);
    CHECK(RPV_Crc16CcittFalse(&g_golden_target_v1[2], 38U) == 0xE808U);
    CHECK(feed(noise, ARRAY_SIZE(noise), &parser, 90U) == RPV_FEED_NONE);
    CHECK(feed(g_golden_target_v1, ARRAY_SIZE(g_golden_target_v1), &parser,
              100U) == RPV_FEED_TARGET);
    CHECK(RPV_GetLatestTarget(&parser, &target));
    CHECK(target.packet_sequence == 42U);
    CHECK(target.frame_id == 1234U);
    CHECK(target.capture_unix_ms == 0U);
    CHECK(!target.capture_timestamp_valid);
    CHECK(target.track_id == 7U);
    CHECK(target.status_flags == 0x001FU);
    CHECK(target.class_id == 0U);
    CHECK(target.confidence == 9234U);
    CHECK(target.target_x_q4 == 10220U);
    CHECK(target.target_y_q4 == 5752U);
    CHECK(target.box_width_q4 == 1544U);
    CHECK(target.box_height_q4 == 1924U);
    CHECK(target.yaw_error_mdeg == -125);
    CHECK(target.pitch_error_mdeg == 80);
    CHECK(target.latency_ms == 18U);
    CHECK(target.prediction_ms == 30U);
    CHECK(target.received_at_ms == 100U);
    CHECK(RPV_Q4ToRoundedPixel(target.target_x_q4) == 639U);
    CHECK(RPV_Q4ToRoundedPixelClamped(
              RPV_TARGET_X_Q4_MAX, RPV_IMAGE_WIDTH_PIXELS) == 1279U);
    CHECK(RPV_Q4ToRoundedPixelClamped(
              RPV_TARGET_Y_Q4_MAX, RPV_IMAGE_HEIGHT_PIXELS) == 719U);
    CHECK(RPV_Q4ToRoundedPixelClamped(100U, 0U) == 0U);
    CHECK(RPV_EvaluateTargetState(&parser, 120U, 100U, 300U) ==
          RPV_TARGET_STATE_VALID);

    RPV_GetStats(&parser, &stats);
    CHECK(stats.rx_byte_count == ARRAY_SIZE(noise) +
          ARRAY_SIZE(g_golden_target_v1));
    CHECK(stats.accepted_frame_count == 1U);
    CHECK(stats.target_frame_count == 1U);
    CHECK(stats.crc_error_count == 0U);
}

static void test_v2_golden_vectors(void)
{
    RPV_Parser_t parser;
    RPV_TargetSnapshot_t target;
    RPV_HeartbeatSnapshot_t heartbeat;
    RPV_Stats_t stats;

    RPV_Init(&parser);
    CHECK(RPV_Crc16CcittFalse(
              &g_golden_target_v2[2], TARGET_V2_FRAME_SIZE - 4U) ==
          0x2DA5U);
    CHECK(feed(g_golden_target_v2, TARGET_V2_FRAME_SIZE, &parser,
              200U) == RPV_FEED_TARGET);
    CHECK(RPV_GetLatestTarget(&parser, &target));
    CHECK(target.packet_sequence == 42U);
    CHECK(target.frame_id == 1234U);
    CHECK(target.capture_unix_ms == UINT64_C(1753877711123));
    CHECK(target.capture_timestamp_valid);
    CHECK(target.target_x_q4 == 10220U);
    CHECK(target.target_y_q4 == 5752U);

    CHECK(RPV_Crc16CcittFalse(
              &g_golden_heartbeat_v2[2],
              HEARTBEAT_V2_FRAME_SIZE - 4U) == 0x0F32U);
    CHECK(feed(g_golden_heartbeat_v2, HEARTBEAT_V2_FRAME_SIZE,
              &parser, 201U) == RPV_FEED_HEARTBEAT);
    CHECK(RPV_GetLatestHeartbeat(&parser, &heartbeat));
    CHECK(heartbeat.packet_sequence == 43U);
    CHECK(heartbeat.extended_fields_valid);
    CHECK(heartbeat.ipv4_address[0] == 192U);
    CHECK(heartbeat.ipv4_address[1] == 168U);
    CHECK(heartbeat.ipv4_address[2] == 5U);
    CHECK(heartbeat.ipv4_address[3] == 148U);
    CHECK(heartbeat.system_unix_ms == UINT64_C(1753877711456));

    RPV_GetStats(&parser, &stats);
    CHECK(stats.accepted_frame_count == 2U);
    CHECK(stats.target_frame_count == 1U);
    CHECK(stats.heartbeat_frame_count == 1U);
}

static void test_all_two_part_splits(void)
{
    RPV_Parser_t parser;
    RPV_TargetSnapshot_t target;
    uint32_t split;

    for (split = 0U; split <= TARGET_V1_FRAME_SIZE; ++split) {
        RPV_Init(&parser);
        CHECK(RPV_FeedBytes(&parser, g_golden_target_v1, split,
                  100U) ==
              ((split == TARGET_V1_FRAME_SIZE) ? 1U : 0U));
        CHECK(RPV_FeedBytes(
                  &parser, &g_golden_target_v1[split],
                  TARGET_V1_FRAME_SIZE - split, 101U) ==
              ((split == TARGET_V1_FRAME_SIZE) ? 0U : 1U));
        CHECK(RPV_GetLatestTarget(&parser, &target));
        CHECK(target.packet_sequence == 42U);
        CHECK(target.frame_id == 1234U);
    }
}

static void test_v2_all_two_part_splits(void)
{
    RPV_Parser_t parser;
    RPV_TargetSnapshot_t target;
    uint32_t split;

    for (split = 0U; split <= TARGET_V2_FRAME_SIZE; ++split) {
        RPV_Init(&parser);
        CHECK(RPV_FeedBytes(
                  &parser, g_golden_target_v2, split, 100U) ==
              ((split == TARGET_V2_FRAME_SIZE) ? 1U : 0U));
        CHECK(RPV_FeedBytes(
                  &parser, &g_golden_target_v2[split],
                  TARGET_V2_FRAME_SIZE - split, 101U) ==
              ((split == TARGET_V2_FRAME_SIZE) ? 0U : 1U));
        CHECK(RPV_GetLatestTarget(&parser, &target));
        CHECK(target.capture_unix_ms == UINT64_C(1753877711123));
        CHECK(target.capture_timestamp_valid);
    }
}

static void test_v1_v2_interleaving_and_extension_clearing(void)
{
    uint8_t heartbeat_v1[HEARTBEAT_V1_FRAME_SIZE] = {
        0xAAU, 0x55U, 0x01U, 0x01U, 0x2CU, 0x00U, 0x0EU, 0x00U,
        0xE9U, 0x03U, 0x00U, 0x00U, 0x77U, 0x00U, 0x00U, 0x3CU,
        0x00U, 0x3CU, 0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
    };
    uint8_t stream[
        TARGET_V1_FRAME_SIZE + HEARTBEAT_V2_FRAME_SIZE +
        TARGET_V2_FRAME_SIZE + HEARTBEAT_V1_FRAME_SIZE];
    RPV_Parser_t parser;
    RPV_TargetSnapshot_t target;
    RPV_HeartbeatSnapshot_t heartbeat;
    uint32_t offset = 0U;

    update_crc(heartbeat_v1, sizeof(heartbeat_v1));
    (void) memcpy(&stream[offset], g_golden_target_v1,
        TARGET_V1_FRAME_SIZE);
    offset += TARGET_V1_FRAME_SIZE;
    (void) memcpy(&stream[offset], g_golden_heartbeat_v2,
        HEARTBEAT_V2_FRAME_SIZE);
    offset += HEARTBEAT_V2_FRAME_SIZE;
    (void) memcpy(&stream[offset], g_golden_target_v2,
        TARGET_V2_FRAME_SIZE);
    offset += TARGET_V2_FRAME_SIZE;
    (void) memcpy(&stream[offset], heartbeat_v1,
        HEARTBEAT_V1_FRAME_SIZE);

    RPV_Init(&parser);
    CHECK(RPV_FeedBytes(&parser, stream, sizeof(stream), 300U) == 4U);
    CHECK(RPV_GetLatestTarget(&parser, &target));
    CHECK(target.capture_timestamp_valid);
    CHECK(target.capture_unix_ms == UINT64_C(1753877711123));
    CHECK(RPV_GetLatestHeartbeat(&parser, &heartbeat));
    CHECK(!heartbeat.extended_fields_valid);
    CHECK(heartbeat.ipv4_address[0] == 0U);
    CHECK(heartbeat.ipv4_address[1] == 0U);
    CHECK(heartbeat.ipv4_address[2] == 0U);
    CHECK(heartbeat.ipv4_address[3] == 0U);
    CHECK(heartbeat.system_unix_ms == 0U);

    CHECK(feed(g_golden_target_v1, TARGET_V1_FRAME_SIZE, &parser,
              301U) == RPV_FEED_TARGET);
    CHECK(RPV_GetLatestTarget(&parser, &target));
    CHECK(!target.capture_timestamp_valid);
    CHECK(target.capture_unix_ms == 0U);
}

static void test_v2_wrong_message_length(void)
{
    uint8_t frame[TARGET_V2_FRAME_SIZE];
    RPV_Parser_t parser;
    RPV_Stats_t stats;

    (void) memcpy(frame, g_golden_target_v2, sizeof(frame));
    write_u16_le(&frame[6], RPV_PRIMARY_TARGET_V2_PAYLOAD_SIZE - 1U);
    update_crc(frame, TARGET_V2_FRAME_SIZE - 1U);

    RPV_Init(&parser);
    CHECK(feed(frame, TARGET_V2_FRAME_SIZE - 1U, &parser, 350U) ==
          RPV_FEED_FORMAT_ERROR);
    RPV_GetStats(&parser, &stats);
    CHECK(stats.format_error_count == 1U);
    CHECK(stats.accepted_frame_count == 0U);
}

static void test_crc_error_and_resynchronization(void)
{
    uint8_t stream[TARGET_V1_FRAME_SIZE * 2U];
    RPV_Parser_t parser;
    RPV_TargetSnapshot_t target;
    RPV_Stats_t stats;

    (void) memcpy(stream, g_golden_target_v1, TARGET_V1_FRAME_SIZE);
    stream[20] ^= 0x01U;
    (void) memcpy(&stream[TARGET_V1_FRAME_SIZE], g_golden_target_v1,
        TARGET_V1_FRAME_SIZE);

    RPV_Init(&parser);
    CHECK(RPV_FeedBytes(&parser, stream, sizeof(stream), 200U) == 1U);
    CHECK(RPV_GetLatestTarget(&parser, &target));
    CHECK(target.frame_id == 1234U);
    RPV_GetStats(&parser, &stats);
    CHECK(stats.crc_error_count == 1U);
    CHECK(stats.target_frame_count == 1U);
}

static void test_truncated_frame_resynchronization(void)
{
    uint8_t stream[32U + TARGET_V1_FRAME_SIZE];
    RPV_Parser_t parser;
    RPV_TargetSnapshot_t target;
    RPV_Stats_t stats;

    /* 第一帧少10字节，解析器会先读入第二帧的10字节再发现CRC错误。 */
    (void) memcpy(stream, g_golden_target_v1, 32U);
    (void) memcpy(&stream[32], g_golden_target_v1, TARGET_V1_FRAME_SIZE);

    RPV_Init(&parser);
    CHECK(RPV_FeedBytes(&parser, stream, sizeof(stream), 250U) == 1U);
    CHECK(RPV_GetLatestTarget(&parser, &target));
    CHECK(target.packet_sequence == 42U);
    CHECK(target.frame_id == 1234U);
    RPV_GetStats(&parser, &stats);
    CHECK(stats.crc_error_count == 1U);
    CHECK(stats.target_frame_count == 1U);
}

static void test_semantic_and_format_errors(void)
{
    uint8_t frame[TARGET_V1_FRAME_SIZE];
    RPV_Parser_t parser;
    RPV_Stats_t stats;

    (void) memcpy(frame, g_golden_target_v1, sizeof(frame));
    write_u16_le(&frame[20], RPV_IMAGE_WIDTH_Q4);
    update_crc(frame, sizeof(frame));

    RPV_Init(&parser);
    CHECK(feed(frame, sizeof(frame), &parser, 300U) ==
          RPV_FEED_SEMANTIC_ERROR);
    CHECK(!RPV_GetLatestTarget(&parser, NULL));

    (void) memcpy(frame, g_golden_target_v1, sizeof(frame));
    frame[2] = 0x02U;
    update_crc(frame, sizeof(frame));
    CHECK(feed(frame, sizeof(frame), &parser, 310U) ==
          RPV_FEED_FORMAT_ERROR);

    (void) memcpy(frame, g_golden_target_v1, sizeof(frame));
    write_u16_le(&frame[6], 31U);
    update_crc(frame, 41U);
    CHECK(feed(frame, 41U, &parser, 320U) ==
          RPV_FEED_FORMAT_ERROR);

    CHECK(feed(g_golden_target_v1, sizeof(g_golden_target_v1),
               &parser, 330U) == RPV_FEED_TARGET);

    RPV_GetStats(&parser, &stats);
    CHECK(stats.semantic_error_count == 1U);
    CHECK(stats.format_error_count == 2U);
    CHECK(stats.accepted_frame_count == 1U);
}

static void test_no_target_and_vision_fault(void)
{
    uint8_t frame[TARGET_V1_FRAME_SIZE];
    RPV_Parser_t parser;
    RPV_TargetSnapshot_t target;

    RPV_Init(&parser);
    CHECK(feed(g_golden_target_v1, sizeof(g_golden_target_v1),
               &parser, 390U) == RPV_FEED_TARGET);
    (void) memcpy(frame, g_golden_target_v1, sizeof(frame));
    write_u16_le(&frame[14], RPV_CAMERA_OK | RPV_INFERENCE_OK);
    (void) memset(&frame[18], 0, 22U);
    update_crc(frame, sizeof(frame));

    CHECK(feed(frame, sizeof(frame), &parser, 400U) == RPV_FEED_TARGET);
    CHECK(RPV_EvaluateTargetState(&parser, 410U, 100U, 300U) ==
          RPV_TARGET_STATE_NOT_FOUND);
    CHECK(RPV_GetLatestTarget(&parser, &target));
    CHECK((target.status_flags & RPV_TARGET_VALID) == 0U);
    CHECK(target.target_x_q4 == 0U);
    CHECK(target.target_y_q4 == 0U);

    write_u16_le(&frame[14], RPV_CAMERA_OK);
    update_crc(frame, sizeof(frame));
    CHECK(feed(frame, sizeof(frame), &parser, 420U) == RPV_FEED_TARGET);
    CHECK(RPV_EvaluateTargetState(&parser, 430U, 100U, 300U) ==
          RPV_TARGET_STATE_VISION_FAULT);
}

static void test_heartbeat_and_timeouts(void)
{
    uint8_t heartbeat[24] = {
        0xAAU, 0x55U, 0x01U, 0x01U, 0x2BU, 0x00U, 0x0EU, 0x00U,
        0xE8U, 0x03U, 0x00U, 0x00U, 0x7FU, 0x00U, 0x00U, 0x3CU,
        0x00U, 0x3CU, 0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
    };
    RPV_Parser_t parser;
    RPV_HeartbeatSnapshot_t snapshot;

    update_crc(heartbeat, sizeof(heartbeat));
    RPV_Init(&parser);
    CHECK(feed(heartbeat, sizeof(heartbeat), &parser, 0xFFFFFFF0U) ==
          RPV_FEED_HEARTBEAT);
    CHECK(RPV_GetLatestHeartbeat(&parser, &snapshot));
    CHECK(snapshot.packet_sequence == 43U);
    CHECK(snapshot.uptime_ms == 1000U);
    CHECK(snapshot.system_flags ==
          (RPV_SYSTEM_SERVICE_READY |
           RPV_SYSTEM_CAMERA_RUNNING |
           RPV_SYSTEM_INFERENCE_RUNNING |
           RPV_SYSTEM_CALIBRATION_LOADED |
           RPV_SYSTEM_TRACKER_RUNNING |
           RPV_SYSTEM_WIFI_CONNECTED |
           RPV_SYSTEM_STARTUP_READY));
    CHECK(snapshot.camera_fps_q8 == 15360U);
    CHECK(snapshot.inference_fps_q8 == 15360U);
    CHECK(snapshot.dropped_frames == 2U);
    CHECK(snapshot.last_error == 0U);
    CHECK(RPV_EvaluateTargetState(&parser, 0x00000020U, 100U, 300U) ==
          RPV_TARGET_STATE_STALE);
    CHECK(RPV_EvaluateTargetState(&parser, 0x00000140U, 100U, 300U) ==
          RPV_TARGET_STATE_LINK_LOST);
}

static void test_exact_timeout_boundaries(void)
{
    RPV_Parser_t parser;

    RPV_Init(&parser);
    CHECK(feed(g_golden_target_v1, sizeof(g_golden_target_v1),
               &parser, 1000U) == RPV_FEED_TARGET);
    CHECK(RPV_EvaluateTargetState(&parser, 1099U, 100U, 300U) ==
          RPV_TARGET_STATE_VALID);
    CHECK(RPV_EvaluateTargetState(&parser, 1100U, 100U, 300U) ==
          RPV_TARGET_STATE_STALE);
    CHECK(RPV_EvaluateTargetState(&parser, 1299U, 100U, 300U) ==
          RPV_TARGET_STATE_STALE);
    CHECK(RPV_EvaluateTargetState(&parser, 1300U, 100U, 300U) ==
          RPV_TARGET_STATE_LINK_LOST);
}

static void test_invalid_frame_does_not_refresh_link_time(void)
{
    uint8_t frame[TARGET_V1_FRAME_SIZE];
    RPV_Parser_t parser;
    RPV_Stats_t stats;

    RPV_Init(&parser);
    CHECK(feed(g_golden_target_v1, TARGET_V1_FRAME_SIZE,
               &parser, 100U) == RPV_FEED_TARGET);
    (void) memcpy(frame, g_golden_target_v1, sizeof(frame));
    write_u16_le(&frame[20], RPV_IMAGE_WIDTH_Q4);
    update_crc(frame, sizeof(frame));
    CHECK(feed(frame, sizeof(frame), &parser, 400U) ==
          RPV_FEED_SEMANTIC_ERROR);
    CHECK(RPV_EvaluateTargetState(&parser, 400U, 100U, 300U) ==
          RPV_TARGET_STATE_LINK_LOST);
    RPV_GetStats(&parser, &stats);
    CHECK(stats.last_protocol_frame_ms == 100U);
}

static void test_bulk_latest_wins(void)
{
    uint8_t stream[TARGET_V1_FRAME_SIZE * 2U];
    RPV_Parser_t parser;
    RPV_TargetSnapshot_t target;

    (void) memcpy(stream, g_golden_target_v1, TARGET_V1_FRAME_SIZE);
    (void) memcpy(&stream[TARGET_V1_FRAME_SIZE], g_golden_target_v1,
        TARGET_V1_FRAME_SIZE);
    write_u16_le(&stream[TARGET_V1_FRAME_SIZE + 4U], 43U);
    stream[TARGET_V1_FRAME_SIZE + 8U] = 0xD3U;
    update_crc(&stream[TARGET_V1_FRAME_SIZE], TARGET_V1_FRAME_SIZE);

    RPV_Init(&parser);
    CHECK(RPV_FeedBytes(&parser, stream, sizeof(stream), 500U) == 2U);
    CHECK(RPV_GetLatestTarget(&parser, &target));
    CHECK(target.packet_sequence == 43U);
    CHECK(target.frame_id == 1235U);
}

static void test_ten_second_sixty_hz_stream(void)
{
    uint8_t frame[TARGET_V1_FRAME_SIZE];
    RPV_Parser_t parser;
    RPV_TargetSnapshot_t target;
    RPV_Stats_t stats;
    uint32_t index;

    RPV_Init(&parser);
    for (index = 0U; index < 600U; ++index) {
        (void) memcpy(frame, g_golden_target_v1, sizeof(frame));
        write_u16_le(&frame[4], (uint16_t) index);
        write_u32_le(&frame[8], 1000U + index);
        update_crc(frame, sizeof(frame));
        CHECK(RPV_FeedBytes(&parser, frame, sizeof(frame),
                  index * 17U) == 1U);
    }

    CHECK(RPV_GetLatestTarget(&parser, &target));
    CHECK(target.packet_sequence == 599U);
    CHECK(target.frame_id == 1599U);
    RPV_GetStats(&parser, &stats);
    CHECK(stats.rx_byte_count == 600U * TARGET_V1_FRAME_SIZE);
    CHECK(stats.target_frame_count == 600U);
    CHECK(stats.crc_error_count == 0U);
    CHECK(stats.format_error_count == 0U);
    CHECK(stats.semantic_error_count == 0U);
}

int main(void)
{
    test_golden_target();
    test_v2_golden_vectors();
    test_all_two_part_splits();
    test_v2_all_two_part_splits();
    test_v1_v2_interleaving_and_extension_clearing();
    test_v2_wrong_message_length();
    test_crc_error_and_resynchronization();
    test_truncated_frame_resynchronization();
    test_semantic_and_format_errors();
    test_no_target_and_vision_fault();
    test_heartbeat_and_timeouts();
    test_exact_timeout_boundaries();
    test_invalid_frame_does_not_refresh_link_time();
    test_bulk_latest_wins();
    test_ten_second_sixty_hz_stream();

    if (g_failures != 0) {
        (void) printf("%d test(s) failed\n", g_failures);
        return 1;
    }

    (void) printf("raspberry_pi_vision_protocol tests passed\n");
    return 0;
}
