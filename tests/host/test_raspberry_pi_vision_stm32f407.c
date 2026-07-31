#include "raspberry_pi_vision_stm32f407.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TARGET_V1_FRAME_SIZE 42U

static int g_failures;

#define CHECK(condition)                                                     \
    do {                                                                     \
        if (!(condition)) {                                                  \
            (void) printf("FAIL line %d: %s\n", __LINE__, #condition);      \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

static const uint8_t g_golden_target[TARGET_V1_FRAME_SIZE] = {
    0xAAU, 0x55U, 0x01U, 0x11U, 0x2AU, 0x00U, 0x20U, 0x00U,
    0xD2U, 0x04U, 0x00U, 0x00U, 0x07U, 0x00U, 0x1FU, 0x00U,
    0x00U, 0x00U, 0x12U, 0x24U, 0xECU, 0x27U, 0x78U, 0x16U,
    0x08U, 0x06U, 0x84U, 0x07U, 0x83U, 0xFFU, 0xFFU, 0xFFU,
    0x50U, 0x00U, 0x00U, 0x00U, 0x12U, 0x00U, 0x1EU, 0x00U,
    0x08U, 0xE8U
};

static void test_callback_only_enqueues(void)
{
    RaspberryPiVision_STM32F407_Receiver_t receiver;
    RaspberryPiVision_STM32F407_Stats_t stats;
    RPV_TargetSnapshot_t target;

    RaspberryPiVision_STM32F407_ReceiverInit(&receiver);
    CHECK(!RaspberryPiVision_STM32F407_ReceiverIsUsbConnected(
        &receiver));
    RaspberryPiVision_STM32F407_ReceiverSetUsbConnected(
        &receiver, true);
    CHECK(RaspberryPiVision_STM32F407_ReceiverProcess(
        &receiver, 90U) == 0U);
    CHECK(RaspberryPiVision_STM32F407_ReceiverEnqueueUsbData(
        &receiver, g_golden_target, 13U));
    CHECK(RaspberryPiVision_STM32F407_ReceiverEnqueueUsbData(
        &receiver, &g_golden_target[13], 29U));

    RaspberryPiVision_STM32F407_ReceiverGetStats(
        &receiver, &stats);
    CHECK(stats.protocol.rx_byte_count == 0U);
    CHECK(stats.queued_byte_count == TARGET_V1_FRAME_SIZE);
    CHECK(stats.usb_packet_count == 2U);
    CHECK(!RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
        &receiver, &target));

    CHECK(RaspberryPiVision_STM32F407_ReceiverProcess(
        &receiver, 100U) == 1U);
    CHECK(RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
        &receiver, &target));
    CHECK(target.packet_sequence == 42U);
    CHECK(target.frame_id == 1234U);
    CHECK(target.received_at_ms == 100U);
    CHECK(RaspberryPiVision_STM32F407_ReceiverGetTargetState(
        &receiver, 120U) == RPV_TARGET_STATE_VALID);
}

static void test_ring_wrap_and_latest_snapshot(void)
{
    RaspberryPiVision_STM32F407_Receiver_t receiver;
    RaspberryPiVision_STM32F407_Stats_t stats;
    RPV_TargetSnapshot_t target;
    uint32_t index;

    RaspberryPiVision_STM32F407_ReceiverInit(&receiver);
    for (index = 0U; index < 600U; ++index) {
        CHECK(RaspberryPiVision_STM32F407_ReceiverEnqueueUsbData(
            &receiver, g_golden_target,
            TARGET_V1_FRAME_SIZE));
        CHECK(RaspberryPiVision_STM32F407_ReceiverProcess(
            &receiver, index * 17U) == 1U);
    }

    RaspberryPiVision_STM32F407_ReceiverGetStats(
        &receiver, &stats);
    CHECK(stats.protocol.rx_byte_count ==
          600U * TARGET_V1_FRAME_SIZE);
    CHECK(stats.protocol.target_frame_count == 600U);
    CHECK(stats.protocol.crc_error_count == 0U);
    CHECK(stats.protocol.format_error_count == 0U);
    CHECK(stats.protocol.semantic_error_count == 0U);
    CHECK(stats.usb_rx_overflow_count == 0U);
    CHECK(stats.queued_byte_count == 0U);
    CHECK(RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
        &receiver, &target));
    CHECK(target.frame_id == 1234U);
}

static void test_overflow_is_atomic_and_recovers(void)
{
    RaspberryPiVision_STM32F407_Receiver_t receiver;
    RaspberryPiVision_STM32F407_Stats_t stats;
    RPV_TargetSnapshot_t target;
    uint8_t noise[1000];

    (void) memset(noise, 0x33, sizeof(noise));
    RaspberryPiVision_STM32F407_ReceiverInit(&receiver);
    CHECK(RaspberryPiVision_STM32F407_ReceiverEnqueueUsbData(
        &receiver, noise, sizeof(noise)));
    CHECK(!RaspberryPiVision_STM32F407_ReceiverEnqueueUsbData(
        &receiver, g_golden_target, TARGET_V1_FRAME_SIZE));
    CHECK(RaspberryPiVision_STM32F407_ReceiverProcess(
        &receiver, 200U) == 0U);
    CHECK(RaspberryPiVision_STM32F407_ReceiverEnqueueUsbData(
        &receiver, g_golden_target, TARGET_V1_FRAME_SIZE));
    CHECK(RaspberryPiVision_STM32F407_ReceiverProcess(
        &receiver, 210U) == 1U);
    CHECK(RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
        &receiver, &target));

    RaspberryPiVision_STM32F407_ReceiverGetStats(
        &receiver, &stats);
    CHECK(stats.usb_rx_overflow_count == 1U);
    CHECK(stats.usb_dropped_byte_count == TARGET_V1_FRAME_SIZE);
    CHECK(stats.protocol.target_frame_count == 1U);
}

static void test_new_usb_session_invalidates_old_snapshot(void)
{
    RaspberryPiVision_STM32F407_Receiver_t receiver;
    RaspberryPiVision_STM32F407_Stats_t stats;
    RPV_TargetSnapshot_t target;

    RaspberryPiVision_STM32F407_ReceiverInit(&receiver);
    RaspberryPiVision_STM32F407_ReceiverSetUsbConnected(
        &receiver, true);
    CHECK(RaspberryPiVision_STM32F407_ReceiverProcess(
        &receiver, 10U) == 0U);
    CHECK(RaspberryPiVision_STM32F407_ReceiverEnqueueUsbData(
        &receiver, g_golden_target, TARGET_V1_FRAME_SIZE));
    CHECK(RaspberryPiVision_STM32F407_ReceiverProcess(
        &receiver, 20U) == 1U);
    CHECK(RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
        &receiver, &target));

    RaspberryPiVision_STM32F407_ReceiverSetUsbConnected(
        &receiver, false);
    CHECK(RaspberryPiVision_STM32F407_ReceiverProcess(
        &receiver, 21U) == 0U);
    CHECK(!RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
        &receiver, &target));
    RaspberryPiVision_STM32F407_ReceiverGetStats(
        &receiver, &stats);
    CHECK(!stats.protocol.has_protocol_frame);

    RaspberryPiVision_STM32F407_ReceiverSetUsbConnected(
        &receiver, true);
    CHECK(RaspberryPiVision_STM32F407_ReceiverEnqueueUsbData(
        &receiver, g_golden_target, TARGET_V1_FRAME_SIZE));
    CHECK(RaspberryPiVision_STM32F407_ReceiverProcess(
        &receiver, 22U) == 0U);
    CHECK(!RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
        &receiver, &target));
    RaspberryPiVision_STM32F407_ReceiverGetStats(
        &receiver, &stats);
    CHECK(!stats.protocol.has_protocol_frame);

    CHECK(RaspberryPiVision_STM32F407_ReceiverEnqueueUsbData(
        &receiver, g_golden_target, TARGET_V1_FRAME_SIZE));
    CHECK(RaspberryPiVision_STM32F407_ReceiverProcess(
        &receiver, 30U) == 1U);
    CHECK(RaspberryPiVision_STM32F407_ReceiverGetLatestTarget(
        &receiver, &target));
}

int main(void)
{
    test_callback_only_enqueues();
    test_ring_wrap_and_latest_snapshot();
    test_overflow_is_atomic_and_recovers();
    test_new_usb_session_invalidates_old_snapshot();

    if (g_failures != 0) {
        (void) printf("%d STM32 adapter test(s) failed\n",
                      g_failures);
        return 1;
    }

    (void) printf(
        "raspberry_pi_vision_stm32f407 tests passed\n");
    return 0;
}
