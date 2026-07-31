#include "oled_ssd1306.h"

#include <stddef.h>
#include <string.h>

#define OLED_ADDRESS_7BIT 0x3CU
#define OLED_DMA_PHASE_COMMAND 1U
#define OLED_DMA_PHASE_DATA    2U

static bool OledSsd1306_StartDmaCommand(OledSsd1306 *oled)
{
    oled->dma_command[0] = 0x00U;
    oled->dma_command[1] = (uint8_t) (0xB0U + oled->dma_page);
    oled->dma_command[2] = 0x02U;
    oled->dma_command[3] = 0x10U;
    oled->dma_phase = OLED_DMA_PHASE_COMMAND;
    return HAL_I2C_Master_Transmit_DMA(
        oled->i2c, OLED_ADDRESS_7BIT << 1U,
        oled->dma_command, sizeof(oled->dma_command)) == HAL_OK;
}

static void OledSsd1306_StopDmaWithError(OledSsd1306 *oled)
{
    oled->error_count++;
    oled->dma_phase = 0U;
    oled->dma_busy = false;
}

static bool OledSsd1306_Transmit(OledSsd1306 *oled, uint8_t *data,
                                 uint16_t length)
{
    if ((oled == NULL) || (oled->i2c == NULL) || !oled->online) {
        return false;
    }
    if (HAL_I2C_Master_Transmit(oled->i2c, OLED_ADDRESS_7BIT << 1U,
                                data, length, 50U) != HAL_OK) {
        oled->error_count++;
        return false;
    }
    return true;
}

static bool OledSsd1306_Command(OledSsd1306 *oled, uint8_t command)
{
    uint8_t buffer[2] = {0x00U, command};
    return OledSsd1306_Transmit(oled, buffer, sizeof(buffer));
}

static void OledSsd1306_SetPixel(OledSsd1306 *oled, uint8_t x,
                                 uint8_t y)
{
    if ((oled == NULL) || (x >= OLED_SSD1306_WIDTH) ||
        (y >= OLED_SSD1306_HEIGHT)) {
        return;
    }
    oled->framebuffer[y / 8U][x] |=
        (uint8_t) (1U << (y % 8U));
}

static void OledSsd1306_DrawCharacter(
    OledSsd1306 *oled, uint8_t x, uint8_t y, char character,
    const ASCIIFont *font)
{
    uint8_t column;
    uint8_t row;
    uint8_t bytes_per_column;
    const uint8_t *glyph;

    if ((font == NULL) || (character < ' ') || (character > '~')) {
        return;
    }

    bytes_per_column = (uint8_t) ((font->h + 7U) / 8U);
    glyph = font->chars +
        (uint32_t) (character - ' ') * font->w * bytes_per_column;

    for (column = 0U; column < font->w; ++column) {
        for (row = 0U; row < font->h; ++row) {
            uint8_t data = glyph[
                (uint32_t) (row / 8U) * font->w + column];
            if ((data & (uint8_t) (1U << (row % 8U))) != 0U) {
                OledSsd1306_SetPixel(
                    oled, (uint8_t) (x + column),
                    (uint8_t) (y + row));
            }
        }
    }
}

bool OledSsd1306_Init(OledSsd1306 *oled, I2C_HandleTypeDef *i2c)
{
    static const uint8_t init_commands[] = {
        0xAEU, 0x20U, 0x10U, 0xB0U, 0xC8U, 0x00U, 0x10U, 0x40U,
        0x81U, 0xFFU, 0xA1U, 0xA6U, 0xA8U, 0x3FU, 0xA4U, 0xD3U,
        0x00U, 0xD5U, 0xF0U, 0xD9U, 0x22U, 0xDAU, 0x12U, 0xDBU,
        0x20U, 0x8DU, 0x14U
    };
    uint8_t index;

    if ((oled == NULL) || (i2c == NULL)) {
        return false;
    }

    (void) memset(oled, 0, sizeof(*oled));
    oled->i2c = i2c;
    oled->online =
        HAL_I2C_IsDeviceReady(i2c, OLED_ADDRESS_7BIT << 1U, 2U, 20U) ==
        HAL_OK;
    if (!oled->online) {
        oled->error_count++;
        return false;
    }

    HAL_Delay(100U);
    for (index = 0U; index < sizeof(init_commands); ++index) {
        if (!OledSsd1306_Command(oled, init_commands[index])) {
            oled->online = false;
            return false;
        }
    }

    OledSsd1306_Clear(oled);
    if (!OledSsd1306_Refresh(oled) ||
        !OledSsd1306_Command(oled, 0xAFU)) {
        oled->online = false;
        return false;
    }
    return true;
}

void OledSsd1306_Clear(OledSsd1306 *oled)
{
    if (oled != NULL) {
        (void) memset(oled->framebuffer, 0, sizeof(oled->framebuffer));
    }
}

void OledSsd1306_DrawText(OledSsd1306 *oled, uint8_t x, uint8_t y,
                          const char *text, const ASCIIFont *font)
{
    if ((oled == NULL) || (text == NULL) || (font == NULL)) {
        return;
    }

    while ((*text != '\0') && (x < OLED_SSD1306_WIDTH)) {
        OledSsd1306_DrawCharacter(oled, x, y, *text, font);
        x = (uint8_t) (x + font->w);
        ++text;
    }
}

bool OledSsd1306_Refresh(OledSsd1306 *oled)
{
    uint8_t page;
    uint8_t buffer[OLED_SSD1306_WIDTH + 1U];

    if ((oled == NULL) || !oled->online || oled->dma_busy) {
        return false;
    }
    buffer[0] = 0x40U;
    for (page = 0U; page < OLED_SSD1306_PAGES; ++page) {
        if (!OledSsd1306_Command(oled, (uint8_t) (0xB0U + page)) ||
            !OledSsd1306_Command(oled, 0x02U) ||
            !OledSsd1306_Command(oled, 0x10U)) {
            return false;
        }
        (void) memcpy(&buffer[1], oled->framebuffer[page],
                      OLED_SSD1306_WIDTH);
        if (!OledSsd1306_Transmit(oled, buffer, sizeof(buffer))) {
            return false;
        }
    }
    return true;
}

bool OledSsd1306_RefreshAsync(OledSsd1306 *oled)
{
    uint8_t page;

    if ((oled == NULL) || (oled->i2c == NULL) || !oled->online ||
        oled->dma_busy) {
        return false;
    }

    for (page = 0U; page < OLED_SSD1306_PAGES; ++page) {
        oled->dma_pages[page][0] = 0x40U;
        (void) memcpy(&oled->dma_pages[page][1],
                      oled->framebuffer[page], OLED_SSD1306_WIDTH);
    }

    oled->dma_page = 0U;
    oled->dma_busy = true;
    if (!OledSsd1306_StartDmaCommand(oled)) {
        OledSsd1306_StopDmaWithError(oled);
        return false;
    }
    return true;
}

bool OledSsd1306_IsBusy(const OledSsd1306 *oled)
{
    return (oled != NULL) && oled->dma_busy;
}

void OledSsd1306_TxCompleteCallback(OledSsd1306 *oled,
                                    I2C_HandleTypeDef *i2c)
{
    if ((oled == NULL) || (i2c != oled->i2c) || !oled->dma_busy) {
        return;
    }

    if (oled->dma_phase == OLED_DMA_PHASE_COMMAND) {
        oled->dma_phase = OLED_DMA_PHASE_DATA;
        if (HAL_I2C_Master_Transmit_DMA(
                oled->i2c, OLED_ADDRESS_7BIT << 1U,
                oled->dma_pages[oled->dma_page],
                OLED_SSD1306_WIDTH + 1U) != HAL_OK) {
            OledSsd1306_StopDmaWithError(oled);
        }
        return;
    }

    if (oled->dma_phase != OLED_DMA_PHASE_DATA) {
        OledSsd1306_StopDmaWithError(oled);
        return;
    }

    oled->dma_page++;
    if (oled->dma_page >= OLED_SSD1306_PAGES) {
        oled->dma_phase = 0U;
        oled->dma_busy = false;
    } else if (!OledSsd1306_StartDmaCommand(oled)) {
        OledSsd1306_StopDmaWithError(oled);
    }
}

void OledSsd1306_ErrorCallback(OledSsd1306 *oled,
                               I2C_HandleTypeDef *i2c)
{
    if ((oled != NULL) && (i2c == oled->i2c) && oled->dma_busy) {
        OledSsd1306_StopDmaWithError(oled);
    }
}
