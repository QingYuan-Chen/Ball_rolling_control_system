#include "tianmengxing_wireless_serial.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

typedef struct {
    uint8_t data[512];
    uint32_t length;
    bool fail;
} FakeWrite_t;

static int g_failures;

#define CHECK(condition)                                                     \
    do {                                                                     \
        if (!(condition)) {                                                  \
            (void) printf("FAIL line %d: %s\n", __LINE__, #condition);       \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

static bool fake_write(void *context, const uint8_t *data, uint32_t length)
{
    FakeWrite_t *fake = (FakeWrite_t *) context;

    if ((fake == NULL) || (data == NULL) || fake->fail ||
        (length > (ARRAY_SIZE(fake->data) - fake->length))) {
        return false;
    }

    (void) memcpy(&fake->data[fake->length], data, length);
    fake->length += length;
    return true;
}

static void test_receive_and_wrap(void)
{
    TianmengxingWirelessSerial_t serial;
    TianmengxingWirelessSerial_Stats_t stats;
    uint8_t output[300];
    uint32_t index;

    TianmengxingWirelessSerial_Init(&serial, NULL);
    CHECK(TianmengxingWirelessSerial_Available(&serial) == 0U);

    for (index = 0U;
         index < (TIANMENGXING_WIRELESS_SERIAL_RX_BUFFER_SIZE - 1U);
         ++index) {
        CHECK(TianmengxingWirelessSerial_FeedRxByte(
            &serial, (uint8_t) index));
    }
    CHECK(!TianmengxingWirelessSerial_FeedRxByte(&serial, 0xEEU));
    CHECK(TianmengxingWirelessSerial_Available(&serial) ==
          (TIANMENGXING_WIRELESS_SERIAL_RX_BUFFER_SIZE - 1U));

    CHECK(TianmengxingWirelessSerial_Read(&serial, output, 128U) == 128U);
    for (index = 0U; index < 128U; ++index) {
        CHECK(output[index] == (uint8_t) index);
    }

    for (index = 0U; index < 100U; ++index) {
        CHECK(TianmengxingWirelessSerial_FeedRxByte(
            &serial, (uint8_t) (0x80U + index)));
    }
    CHECK(TianmengxingWirelessSerial_Read(
              &serial, output, ARRAY_SIZE(output)) == 227U);
    CHECK(TianmengxingWirelessSerial_Available(&serial) == 0U);

    CHECK(TianmengxingWirelessSerial_GetStats(&serial, &stats));
    CHECK(stats.rx_received_bytes == 355U);
    CHECK(stats.rx_dropped_bytes == 1U);
    CHECK(stats.rx_available == 0U);
}

static void test_write_and_errors(void)
{
    FakeWrite_t fake = {0};
    TianmengxingWirelessSerial_IO_t io = {
        .write = fake_write,
        .context = &fake
    };
    TianmengxingWirelessSerial_t serial;
    TianmengxingWirelessSerial_Stats_t stats;
    static const uint8_t binary[] = {0x00U, 0x5AU, 0xFFU};

    TianmengxingWirelessSerial_Init(&serial, &io);
    CHECK(TianmengxingWirelessSerial_Write(
              &serial, binary, ARRAY_SIZE(binary)) ==
          TIANMENGXING_WIRELESS_SERIAL_STATUS_OK);
    CHECK(TianmengxingWirelessSerial_WriteString(&serial, "hello") ==
          TIANMENGXING_WIRELESS_SERIAL_STATUS_OK);
    CHECK(fake.length == 8U);
    CHECK(memcmp(fake.data, binary, ARRAY_SIZE(binary)) == 0);
    CHECK(memcmp(&fake.data[3], "hello", 5U) == 0);

    CHECK(TianmengxingWirelessSerial_GetStats(&serial, &stats));
    CHECK(stats.tx_bytes == 8U);

    fake.fail = true;
    CHECK(TianmengxingWirelessSerial_WriteString(&serial, "x") ==
          TIANMENGXING_WIRELESS_SERIAL_STATUS_IO_ERROR);
    CHECK(TianmengxingWirelessSerial_Write(NULL, binary, 1U) ==
          TIANMENGXING_WIRELESS_SERIAL_STATUS_INVALID_ARGUMENT);
    CHECK(TianmengxingWirelessSerial_WriteString(&serial, NULL) ==
          TIANMENGXING_WIRELESS_SERIAL_STATUS_INVALID_ARGUMENT);
}

static void test_clear_and_arguments(void)
{
    TianmengxingWirelessSerial_t serial;
    uint8_t byte;

    TianmengxingWirelessSerial_Init(&serial, NULL);
    CHECK(!TianmengxingWirelessSerial_FeedRxByte(NULL, 0U));
    CHECK(TianmengxingWirelessSerial_Read(NULL, &byte, 1U) == 0U);
    CHECK(!TianmengxingWirelessSerial_GetStats(NULL, NULL));

    CHECK(TianmengxingWirelessSerial_FeedRxByte(&serial, 0x42U));
    TianmengxingWirelessSerial_ClearRx(&serial);
    CHECK(TianmengxingWirelessSerial_Available(&serial) == 0U);
    CHECK(TianmengxingWirelessSerial_Read(&serial, &byte, 1U) == 0U);
}

int main(void)
{
    test_receive_and_wrap();
    test_write_and_errors();
    test_clear_and_arguments();

    if (g_failures != 0) {
        (void) printf("%d test(s) failed\n", g_failures);
        return 1;
    }

    (void) printf("tianmengxing wireless serial tests passed\n");
    return 0;
}
