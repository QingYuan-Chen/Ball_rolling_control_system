#include "raspberry_pi_vision_protocol.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define RPV_SOF_0                    0xAAU
#define RPV_SOF_1                    0x55U
#define RPV_FRAME_PREFIX_SIZE        8U
#define RPV_FRAME_OVERHEAD_SIZE      10U

_Static_assert(sizeof(RPV_Parser_t) <= 224U,
    "RPV parser exceeds the 224-byte MCU RAM budget");

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t) ((uint16_t) data[0] |
                       ((uint16_t) data[1] << 8U));
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t) data[0] |
           ((uint32_t) data[1] << 8U) |
           ((uint32_t) data[2] << 16U) |
           ((uint32_t) data[3] << 24U);
}

static uint64_t read_u64_le(const uint8_t *data)
{
    return (uint64_t) read_u32_le(data) |
           ((uint64_t) read_u32_le(&data[4]) << 32U);
}

static int32_t read_i32_le(const uint8_t *data)
{
    uint32_t raw = read_u32_le(data);

    if (raw <= (uint32_t) INT32_MAX) {
        return (int32_t) raw;
    }
    return -1 - (int32_t) (UINT32_MAX - raw);
}

static void reset_frame(RPV_Parser_t *parser)
{
    parser->frame_index = 0U;
    parser->expected_frame_length = 0U;
}

static void resynchronize_frame(RPV_Parser_t *parser)
{
    uint16_t start;
    uint16_t suffix_length;

    for (start = 1U; (start + 1U) < parser->frame_index; ++start) {
        if ((parser->frame[start] == RPV_SOF_0) &&
            (parser->frame[start + 1U] == RPV_SOF_1)) {
            suffix_length = parser->frame_index - start;
            (void) memmove(parser->frame, &parser->frame[start],
                suffix_length);
            parser->frame_index = suffix_length;
            parser->expected_frame_length = 0U;
            return;
        }
    }

    if ((parser->frame_index != 0U) &&
        (parser->frame[parser->frame_index - 1U] == RPV_SOF_0)) {
        parser->frame[0] = RPV_SOF_0;
        parser->frame_index = 1U;
        parser->expected_frame_length = 0U;
    } else {
        reset_frame(parser);
    }
}

static bool target_semantics_valid(const RPV_TargetSnapshot_t *target)
{
    if (target->confidence > RPV_CONFIDENCE_MAX) {
        return false;
    }

    if ((target->status_flags & RPV_TARGET_VALID) == 0U) {
        return true;
    }

    return (target->target_x_q4 <= RPV_TARGET_X_Q4_MAX) &&
           (target->target_y_q4 <= RPV_TARGET_Y_Q4_MAX) &&
           (target->box_width_q4 != 0U) &&
           (target->box_height_q4 != 0U) &&
           (target->box_width_q4 <= RPV_IMAGE_WIDTH_Q4) &&
           (target->box_height_q4 <= RPV_IMAGE_HEIGHT_Q4);
}

static bool decode_and_publish_target(RPV_Parser_t *parser,
    uint16_t packet_sequence, const uint8_t *payload,
    bool extended_fields, uint32_t received_at_ms)
{
    RPV_TargetSnapshot_t target = {0};
    uint8_t field_offset = extended_fields ? 8U : 0U;

    target.packet_sequence = packet_sequence;
    target.frame_id = read_u32_le(&payload[0]);
    target.capture_unix_ms =
        extended_fields ? read_u64_le(&payload[4]) : 0U;
    target.capture_timestamp_valid =
        extended_fields && (target.capture_unix_ms != 0U);
    target.track_id = read_u16_le(&payload[4U + field_offset]);
    target.status_flags = read_u16_le(&payload[6U + field_offset]);
    target.class_id = read_u16_le(&payload[8U + field_offset]);
    target.confidence = read_u16_le(&payload[10U + field_offset]);
    target.target_x_q4 = read_u16_le(&payload[12U + field_offset]);
    target.target_y_q4 = read_u16_le(&payload[14U + field_offset]);
    target.box_width_q4 = read_u16_le(&payload[16U + field_offset]);
    target.box_height_q4 = read_u16_le(&payload[18U + field_offset]);
    target.yaw_error_mdeg = read_i32_le(&payload[20U + field_offset]);
    target.pitch_error_mdeg = read_i32_le(&payload[24U + field_offset]);
    target.latency_ms = read_u16_le(&payload[28U + field_offset]);
    target.prediction_ms = read_u16_le(&payload[30U + field_offset]);
    target.received_at_ms = received_at_ms;

    if (!target_semantics_valid(&target)) {
        return false;
    }

    parser->target_update_sequence++;
    parser->latest_target.packet_sequence = target.packet_sequence;
    parser->latest_target.frame_id = target.frame_id;
    parser->latest_target.capture_unix_ms = target.capture_unix_ms;
    parser->latest_target.capture_timestamp_valid =
        target.capture_timestamp_valid;
    parser->latest_target.track_id = target.track_id;
    parser->latest_target.status_flags = target.status_flags;
    parser->latest_target.class_id = target.class_id;
    parser->latest_target.confidence = target.confidence;
    parser->latest_target.target_x_q4 = target.target_x_q4;
    parser->latest_target.target_y_q4 = target.target_y_q4;
    parser->latest_target.box_width_q4 = target.box_width_q4;
    parser->latest_target.box_height_q4 = target.box_height_q4;
    parser->latest_target.yaw_error_mdeg = target.yaw_error_mdeg;
    parser->latest_target.pitch_error_mdeg = target.pitch_error_mdeg;
    parser->latest_target.latency_ms = target.latency_ms;
    parser->latest_target.prediction_ms = target.prediction_ms;
    parser->latest_target.received_at_ms = target.received_at_ms;
    parser->target_received = true;
    parser->target_update_sequence++;
    return true;
}

static void publish_heartbeat(RPV_Parser_t *parser,
    uint16_t packet_sequence, const uint8_t *payload,
    bool extended_fields, uint32_t received_at_ms)
{
    parser->heartbeat_update_sequence++;
    parser->latest_heartbeat.packet_sequence = packet_sequence;
    parser->latest_heartbeat.uptime_ms = read_u32_le(&payload[0]);
    parser->latest_heartbeat.system_flags = read_u16_le(&payload[4]);
    parser->latest_heartbeat.camera_fps_q8 = read_u16_le(&payload[6]);
    parser->latest_heartbeat.inference_fps_q8 = read_u16_le(&payload[8]);
    parser->latest_heartbeat.dropped_frames = read_u16_le(&payload[10]);
    parser->latest_heartbeat.last_error = read_u16_le(&payload[12]);
    if (extended_fields) {
        parser->latest_heartbeat.ipv4_address[0] = payload[14];
        parser->latest_heartbeat.ipv4_address[1] = payload[15];
        parser->latest_heartbeat.ipv4_address[2] = payload[16];
        parser->latest_heartbeat.ipv4_address[3] = payload[17];
        parser->latest_heartbeat.system_unix_ms = read_u64_le(&payload[18]);
    } else {
        parser->latest_heartbeat.ipv4_address[0] = 0U;
        parser->latest_heartbeat.ipv4_address[1] = 0U;
        parser->latest_heartbeat.ipv4_address[2] = 0U;
        parser->latest_heartbeat.ipv4_address[3] = 0U;
        parser->latest_heartbeat.system_unix_ms = 0U;
    }
    parser->latest_heartbeat.extended_fields_valid = extended_fields;
    parser->latest_heartbeat.received_at_ms = received_at_ms;
    parser->heartbeat_received = true;
    parser->heartbeat_update_sequence++;
}

static RPV_FeedResult_t process_complete_frame(RPV_Parser_t *parser,
    uint32_t received_at_ms)
{
    uint16_t packet_sequence;
    uint16_t payload_length;
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint8_t message_id;
    RPV_FeedResult_t result;

    payload_length = read_u16_le(&parser->frame[6]);
    received_crc = read_u16_le(
        &parser->frame[parser->expected_frame_length - 2U]);
    calculated_crc = RPV_Crc16CcittFalse(&parser->frame[2],
        parser->expected_frame_length - 4U);

    if (calculated_crc != received_crc) {
        parser->crc_error_count++;
        resynchronize_frame(parser);
        return RPV_FEED_CRC_ERROR;
    }

    if (parser->frame[2] != RPV_PROTOCOL_VERSION) {
        parser->format_error_count++;
        reset_frame(parser);
        return RPV_FEED_FORMAT_ERROR;
    }

    packet_sequence = read_u16_le(&parser->frame[4]);
    message_id = parser->frame[3];

    if ((message_id == RPV_MSG_PRIMARY_TARGET_V1) ||
        (message_id == RPV_MSG_PRIMARY_TARGET_V2)) {
        bool extended_fields = message_id == RPV_MSG_PRIMARY_TARGET_V2;
        uint16_t required_payload_length = extended_fields ?
            RPV_PRIMARY_TARGET_V2_PAYLOAD_SIZE :
            RPV_PRIMARY_TARGET_V1_PAYLOAD_SIZE;

        if (payload_length != required_payload_length) {
            parser->format_error_count++;
            result = RPV_FEED_FORMAT_ERROR;
        } else if (!decode_and_publish_target(parser, packet_sequence,
                       &parser->frame[8], extended_fields,
                       received_at_ms)) {
            parser->semantic_error_count++;
            result = RPV_FEED_SEMANTIC_ERROR;
        } else {
            parser->accepted_frame_count++;
            parser->target_frame_count++;
            parser->has_protocol_frame = true;
            parser->last_protocol_frame_ms = received_at_ms;
            result = RPV_FEED_TARGET;
        }
    } else if ((message_id == RPV_MSG_HEARTBEAT_V1) ||
               (message_id == RPV_MSG_HEARTBEAT_V2)) {
        bool extended_fields = message_id == RPV_MSG_HEARTBEAT_V2;
        uint16_t required_payload_length = extended_fields ?
            RPV_HEARTBEAT_V2_PAYLOAD_SIZE :
            RPV_HEARTBEAT_V1_PAYLOAD_SIZE;

        if (payload_length != required_payload_length) {
            parser->format_error_count++;
            result = RPV_FEED_FORMAT_ERROR;
        } else {
            publish_heartbeat(parser, packet_sequence, &parser->frame[8],
                extended_fields, received_at_ms);
            parser->accepted_frame_count++;
            parser->heartbeat_frame_count++;
            parser->has_protocol_frame = true;
            parser->last_protocol_frame_ms = received_at_ms;
            result = RPV_FEED_HEARTBEAT;
        }
    } else {
        parser->ignored_frame_count++;
        result = RPV_FEED_IGNORED;
    }

    reset_frame(parser);
    return result;
}

void RPV_Init(RPV_Parser_t *parser)
{
    if (parser != NULL) {
        (void) memset(parser, 0, sizeof(*parser));
    }
}

uint16_t RPV_Crc16CcittFalse(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t byte_index;
    uint8_t bit_index;

    if ((data == NULL) && (length != 0U)) {
        return crc;
    }

    for (byte_index = 0U; byte_index < length; ++byte_index) {
        crc ^= (uint16_t) data[byte_index] << 8U;
        for (bit_index = 0U; bit_index < 8U; ++bit_index) {
            crc = ((crc & 0x8000U) != 0U) ?
                      (uint16_t) ((crc << 1U) ^ 0x1021U) :
                      (uint16_t) (crc << 1U);
        }
    }
    return crc;
}

RPV_FeedResult_t RPV_FeedByte(RPV_Parser_t *parser, uint8_t byte,
    uint32_t received_at_ms)
{
    uint16_t payload_length;

    if (parser == NULL) {
        return RPV_FEED_NONE;
    }

    parser->rx_byte_count++;

    if (parser->frame_index == 0U) {
        if (byte == RPV_SOF_0) {
            parser->frame[0] = byte;
            parser->frame_index = 1U;
        }
        return RPV_FEED_NONE;
    }

    if (parser->frame_index == 1U) {
        if (byte == RPV_SOF_1) {
            parser->frame[1] = byte;
            parser->frame_index = 2U;
        } else if (byte != RPV_SOF_0) {
            reset_frame(parser);
        }
        return RPV_FEED_NONE;
    }

    if (parser->frame_index >= RPV_MAX_FRAME_SIZE) {
        parser->format_error_count++;
        resynchronize_frame(parser);
        return RPV_FEED_FORMAT_ERROR;
    }

    parser->frame[parser->frame_index++] = byte;

    /* CRC失败重同步后，缓存中可能已包含新帧8字节以上的前缀。 */
    if ((parser->expected_frame_length == 0U) &&
        (parser->frame_index >= RPV_FRAME_PREFIX_SIZE)) {
        payload_length = read_u16_le(&parser->frame[6]);
        if (payload_length > RPV_MAX_PAYLOAD_SIZE) {
            parser->format_error_count++;
            resynchronize_frame(parser);
            return RPV_FEED_FORMAT_ERROR;
        }
        parser->expected_frame_length =
            (uint16_t) (RPV_FRAME_OVERHEAD_SIZE + payload_length);
    }

    if ((parser->expected_frame_length != 0U) &&
        (parser->frame_index == parser->expected_frame_length)) {
        return process_complete_frame(parser, received_at_ms);
    }

    if ((parser->expected_frame_length != 0U) &&
        (parser->frame_index > parser->expected_frame_length)) {
        parser->format_error_count++;
        resynchronize_frame(parser);
        return RPV_FEED_FORMAT_ERROR;
    }

    return RPV_FEED_NONE;
}

uint32_t RPV_FeedBytes(RPV_Parser_t *parser, const uint8_t *data,
    uint32_t length, uint32_t received_at_ms)
{
    uint32_t index;
    uint32_t accepted = 0U;
    RPV_FeedResult_t result;

    if ((parser == NULL) || ((data == NULL) && (length != 0U))) {
        return 0U;
    }

    for (index = 0U; index < length; ++index) {
        result = RPV_FeedByte(parser, data[index], received_at_ms);
        if ((result == RPV_FEED_TARGET) ||
            (result == RPV_FEED_HEARTBEAT)) {
            accepted++;
        }
    }
    return accepted;
}

bool RPV_GetLatestTarget(const RPV_Parser_t *parser,
    RPV_TargetSnapshot_t *target)
{
    uint32_t sequence_before;
    uint32_t sequence_after;

    if ((parser == NULL) || (target == NULL) || !parser->target_received) {
        return false;
    }

    for (;;) {
        sequence_before = parser->target_update_sequence;
        if ((sequence_before & 1U) != 0U) {
            continue;
        }

        target->packet_sequence = parser->latest_target.packet_sequence;
        target->frame_id = parser->latest_target.frame_id;
        target->capture_unix_ms =
            parser->latest_target.capture_unix_ms;
        target->capture_timestamp_valid =
            parser->latest_target.capture_timestamp_valid;
        target->track_id = parser->latest_target.track_id;
        target->status_flags = parser->latest_target.status_flags;
        target->class_id = parser->latest_target.class_id;
        target->confidence = parser->latest_target.confidence;
        target->target_x_q4 = parser->latest_target.target_x_q4;
        target->target_y_q4 = parser->latest_target.target_y_q4;
        target->box_width_q4 = parser->latest_target.box_width_q4;
        target->box_height_q4 = parser->latest_target.box_height_q4;
        target->yaw_error_mdeg = parser->latest_target.yaw_error_mdeg;
        target->pitch_error_mdeg = parser->latest_target.pitch_error_mdeg;
        target->latency_ms = parser->latest_target.latency_ms;
        target->prediction_ms = parser->latest_target.prediction_ms;
        target->received_at_ms = parser->latest_target.received_at_ms;

        sequence_after = parser->target_update_sequence;
        if ((sequence_before == sequence_after) &&
            ((sequence_after & 1U) == 0U)) {
            return true;
        }
    }
}

bool RPV_GetLatestHeartbeat(const RPV_Parser_t *parser,
    RPV_HeartbeatSnapshot_t *heartbeat)
{
    uint32_t sequence_before;
    uint32_t sequence_after;

    if ((parser == NULL) || (heartbeat == NULL) ||
        !parser->heartbeat_received) {
        return false;
    }

    for (;;) {
        sequence_before = parser->heartbeat_update_sequence;
        if ((sequence_before & 1U) != 0U) {
            continue;
        }

        heartbeat->packet_sequence =
            parser->latest_heartbeat.packet_sequence;
        heartbeat->uptime_ms = parser->latest_heartbeat.uptime_ms;
        heartbeat->system_flags = parser->latest_heartbeat.system_flags;
        heartbeat->camera_fps_q8 =
            parser->latest_heartbeat.camera_fps_q8;
        heartbeat->inference_fps_q8 =
            parser->latest_heartbeat.inference_fps_q8;
        heartbeat->dropped_frames =
            parser->latest_heartbeat.dropped_frames;
        heartbeat->last_error = parser->latest_heartbeat.last_error;
        heartbeat->ipv4_address[0] =
            parser->latest_heartbeat.ipv4_address[0];
        heartbeat->ipv4_address[1] =
            parser->latest_heartbeat.ipv4_address[1];
        heartbeat->ipv4_address[2] =
            parser->latest_heartbeat.ipv4_address[2];
        heartbeat->ipv4_address[3] =
            parser->latest_heartbeat.ipv4_address[3];
        heartbeat->system_unix_ms =
            parser->latest_heartbeat.system_unix_ms;
        heartbeat->extended_fields_valid =
            parser->latest_heartbeat.extended_fields_valid;
        heartbeat->received_at_ms =
            parser->latest_heartbeat.received_at_ms;

        sequence_after = parser->heartbeat_update_sequence;
        if ((sequence_before == sequence_after) &&
            ((sequence_after & 1U) == 0U)) {
            return true;
        }
    }
}

void RPV_GetStats(const RPV_Parser_t *parser, RPV_Stats_t *stats)
{
    if ((parser == NULL) || (stats == NULL)) {
        return;
    }

    stats->rx_byte_count = parser->rx_byte_count;
    stats->accepted_frame_count = parser->accepted_frame_count;
    stats->target_frame_count = parser->target_frame_count;
    stats->heartbeat_frame_count = parser->heartbeat_frame_count;
    stats->crc_error_count = parser->crc_error_count;
    stats->format_error_count = parser->format_error_count;
    stats->semantic_error_count = parser->semantic_error_count;
    stats->ignored_frame_count = parser->ignored_frame_count;
    stats->last_protocol_frame_ms = parser->last_protocol_frame_ms;
    stats->has_protocol_frame = parser->has_protocol_frame;
}

RPV_TargetState_t RPV_EvaluateTargetState(const RPV_Parser_t *parser,
    uint32_t now_ms, uint32_t target_timeout_ms,
    uint32_t link_timeout_ms)
{
    RPV_TargetSnapshot_t target;

    if ((parser == NULL) || !parser->has_protocol_frame) {
        return RPV_TARGET_STATE_NO_DATA;
    }

    if ((uint32_t) (now_ms - parser->last_protocol_frame_ms) >=
        link_timeout_ms) {
        return RPV_TARGET_STATE_LINK_LOST;
    }

    if (!RPV_GetLatestTarget(parser, &target) ||
        ((uint32_t) (now_ms - target.received_at_ms) >=
            target_timeout_ms)) {
        return RPV_TARGET_STATE_STALE;
    }

    if ((target.status_flags & (RPV_CAMERA_OK | RPV_INFERENCE_OK)) !=
        (RPV_CAMERA_OK | RPV_INFERENCE_OK)) {
        return RPV_TARGET_STATE_VISION_FAULT;
    }

    if ((target.status_flags & RPV_TARGET_VALID) == 0U) {
        return RPV_TARGET_STATE_NOT_FOUND;
    }

    return RPV_TARGET_STATE_VALID;
}

uint16_t RPV_Q4ToRoundedPixel(uint16_t value_q4)
{
    return (uint16_t) (((uint32_t) value_q4 + 8U) >> 4U);
}

uint16_t RPV_Q4ToRoundedPixelClamped(
    uint16_t value_q4, uint16_t axis_pixels)
{
    uint16_t pixel;

    if (axis_pixels == 0U) {
        return 0U;
    }
    pixel = RPV_Q4ToRoundedPixel(value_q4);
    return (pixel < axis_pixels) ? pixel : (uint16_t) (axis_pixels - 1U);
}
