/**
 * @file raspberry_pi_vision_protocol.h
 * @brief Raspberry Pi视觉目标到MCU的可移植UART协议解析器。
 *
 * 协议核心不依赖MSPM0 DriverLib。UART中断、DMA或主机测试都可以把收到的
 * 字节交给RPV_FeedByte/RPV_FeedBytes。显示层只读取解码后的目标快照。
 */

#ifndef RASPBERRY_PI_VISION_PROTOCOL_H_
#define RASPBERRY_PI_VISION_PROTOCOL_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RPV_PROTOCOL_VERSION                 0x01U
#define RPV_MSG_HEARTBEAT_V1                 0x01U
#define RPV_MSG_HEARTBEAT_V2                 0x02U
#define RPV_MSG_PRIMARY_TARGET_V1            0x11U
#define RPV_MSG_PRIMARY_TARGET_V2            0x12U

/* 兼容旧调用方。 */
#define RPV_MSG_HEARTBEAT                    RPV_MSG_HEARTBEAT_V1
#define RPV_MSG_PRIMARY_TARGET               RPV_MSG_PRIMARY_TARGET_V1

#define RPV_HEARTBEAT_V1_PAYLOAD_SIZE        14U
#define RPV_HEARTBEAT_V2_PAYLOAD_SIZE        26U
#define RPV_PRIMARY_TARGET_V1_PAYLOAD_SIZE   32U
#define RPV_PRIMARY_TARGET_V2_PAYLOAD_SIZE   40U
#define RPV_MAX_PAYLOAD_SIZE                 40U
#define RPV_MAX_FRAME_SIZE                   50U

/* 兼容旧调用方。 */
#define RPV_HEARTBEAT_PAYLOAD_SIZE           RPV_HEARTBEAT_V1_PAYLOAD_SIZE
#define RPV_PRIMARY_TARGET_PAYLOAD_SIZE      RPV_PRIMARY_TARGET_V1_PAYLOAD_SIZE

#define RPV_IMAGE_WIDTH_PIXELS               1280U
#define RPV_IMAGE_HEIGHT_PIXELS              720U
#define RPV_IMAGE_WIDTH_Q4                   (RPV_IMAGE_WIDTH_PIXELS * 16U)
#define RPV_IMAGE_HEIGHT_Q4                  (RPV_IMAGE_HEIGHT_PIXELS * 16U)
#define RPV_TARGET_X_Q4_MAX                  (RPV_IMAGE_WIDTH_Q4 - 1U)
#define RPV_TARGET_Y_Q4_MAX                  (RPV_IMAGE_HEIGHT_Q4 - 1U)
#define RPV_CONFIDENCE_MAX                   10000U

#define RPV_TARGET_VALID                     (1U << 0)
#define RPV_TARGET_PREDICTED                 (1U << 1)
#define RPV_ANGLE_VALID                      (1U << 2)
#define RPV_CAMERA_OK                        (1U << 3)
#define RPV_INFERENCE_OK                     (1U << 4)
#define RPV_TARGET_CHANGED                   (1U << 5)

#define RPV_SYSTEM_SERVICE_READY             (1U << 0)
#define RPV_SYSTEM_CAMERA_RUNNING            (1U << 1)
#define RPV_SYSTEM_INFERENCE_RUNNING         (1U << 2)
#define RPV_SYSTEM_CALIBRATION_LOADED        (1U << 3)
#define RPV_SYSTEM_TRACKER_RUNNING           (1U << 4)
#define RPV_SYSTEM_WIFI_CONNECTED            (1U << 5)
#define RPV_SYSTEM_STARTUP_READY              (1U << 6)

typedef enum {
    RPV_FEED_NONE = 0,
    RPV_FEED_TARGET = 1,
    RPV_FEED_HEARTBEAT = 2,
    RPV_FEED_IGNORED = 3,
    RPV_FEED_CRC_ERROR = -1,
    RPV_FEED_FORMAT_ERROR = -2,
    RPV_FEED_SEMANTIC_ERROR = -3
} RPV_FeedResult_t;

typedef enum {
    RPV_TARGET_STATE_NO_DATA = 0,
    RPV_TARGET_STATE_LINK_LOST,
    RPV_TARGET_STATE_STALE,
    RPV_TARGET_STATE_VISION_FAULT,
    RPV_TARGET_STATE_NOT_FOUND,
    RPV_TARGET_STATE_VALID
} RPV_TargetState_t;

typedef struct {
    uint16_t packet_sequence;
    uint32_t frame_id;
    uint64_t capture_unix_ms;
    bool capture_timestamp_valid;
    uint16_t track_id;
    uint16_t status_flags;
    uint16_t class_id;
    uint16_t confidence;
    uint16_t target_x_q4;
    uint16_t target_y_q4;
    uint16_t box_width_q4;
    uint16_t box_height_q4;
    int32_t yaw_error_mdeg;
    int32_t pitch_error_mdeg;
    uint16_t latency_ms;
    uint16_t prediction_ms;
    uint32_t received_at_ms;
} RPV_TargetSnapshot_t;

typedef struct {
    uint16_t packet_sequence;
    uint32_t uptime_ms;
    uint16_t system_flags;
    uint16_t camera_fps_q8;
    uint16_t inference_fps_q8;
    uint16_t dropped_frames;
    uint16_t last_error;
    uint8_t ipv4_address[4];
    uint64_t system_unix_ms;
    bool extended_fields_valid;
    uint32_t received_at_ms;
} RPV_HeartbeatSnapshot_t;

typedef struct {
    uint32_t rx_byte_count;
    uint32_t accepted_frame_count;
    uint32_t target_frame_count;
    uint32_t heartbeat_frame_count;
    uint32_t crc_error_count;
    uint32_t format_error_count;
    uint32_t semantic_error_count;
    uint32_t ignored_frame_count;
    uint32_t last_protocol_frame_ms;
    bool has_protocol_frame;
} RPV_Stats_t;

typedef struct {
    uint8_t frame[RPV_MAX_FRAME_SIZE];
    uint16_t frame_index;
    uint16_t expected_frame_length;

    volatile uint32_t target_update_sequence;
    volatile bool target_received;
    volatile RPV_TargetSnapshot_t latest_target;

    volatile uint32_t heartbeat_update_sequence;
    volatile bool heartbeat_received;
    volatile RPV_HeartbeatSnapshot_t latest_heartbeat;

    volatile uint32_t rx_byte_count;
    volatile uint32_t accepted_frame_count;
    volatile uint32_t target_frame_count;
    volatile uint32_t heartbeat_frame_count;
    volatile uint32_t crc_error_count;
    volatile uint32_t format_error_count;
    volatile uint32_t semantic_error_count;
    volatile uint32_t ignored_frame_count;
    volatile uint32_t last_protocol_frame_ms;
    volatile bool has_protocol_frame;
} RPV_Parser_t;

/** 初始化解析器和全部统计量。 */
void RPV_Init(RPV_Parser_t *parser);

/** CRC-16/CCITT-FALSE：poly=0x1021、init=0xFFFF。 */
uint16_t RPV_Crc16CcittFalse(const uint8_t *data, uint32_t length);

/**
 * 输入一个UART字节。
 *
 * @param parser 解析器。
 * @param byte 收到的字节。
 * @param received_at_ms MCU单调毫秒时钟；同一批FIFO/DMA字节可使用同一值。
 */
RPV_FeedResult_t RPV_FeedByte(RPV_Parser_t *parser, uint8_t byte,
    uint32_t received_at_ms);

/** 输入一批UART字节，返回其中成功接受的目标包和心跳包总数。 */
uint32_t RPV_FeedBytes(RPV_Parser_t *parser, const uint8_t *data,
    uint32_t length, uint32_t received_at_ms);

/** 获取一份可与中断FeedByte并发使用的一致目标快照。 */
bool RPV_GetLatestTarget(const RPV_Parser_t *parser,
    RPV_TargetSnapshot_t *target);

/** 获取一份一致的心跳快照。 */
bool RPV_GetLatestHeartbeat(const RPV_Parser_t *parser,
    RPV_HeartbeatSnapshot_t *heartbeat);

/** 获取协议统计量。 */
void RPV_GetStats(const RPV_Parser_t *parser, RPV_Stats_t *stats);

/**
 * 根据本地时间评估目标状态。建议target_timeout_ms=100、
 * link_timeout_ms=300。函数不修改解析器。
 */
RPV_TargetState_t RPV_EvaluateTargetState(const RPV_Parser_t *parser,
    uint32_t now_ms, uint32_t target_timeout_ms,
    uint32_t link_timeout_ms);

/** Q12.4坐标四舍五入为整数像素，供简单显示使用。 */
uint16_t RPV_Q4ToRoundedPixel(uint16_t value_q4);

/** 四舍五入并钳制到[0, axis_pixels-1]，供图像坐标显示使用。 */
uint16_t RPV_Q4ToRoundedPixelClamped(
    uint16_t value_q4, uint16_t axis_pixels);

#ifdef __cplusplus
}
#endif

#endif /* RASPBERRY_PI_VISION_PROTOCOL_H_ */
