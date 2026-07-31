#ifndef BALLCONTROL_OLED_SSD1306_H
#define BALLCONTROL_OLED_SSD1306_H

#include <stdbool.h>
#include <stdint.h>

#include "font.h"
#include "stm32f4xx_hal.h"

#define OLED_SSD1306_WIDTH  128U
#define OLED_SSD1306_HEIGHT 64U
#define OLED_SSD1306_PAGES  8U

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t framebuffer[OLED_SSD1306_PAGES][OLED_SSD1306_WIDTH];
    uint8_t dma_pages[OLED_SSD1306_PAGES][OLED_SSD1306_WIDTH + 1U];
    uint8_t dma_command[4];
    uint32_t error_count;
    volatile uint8_t dma_page;
    volatile uint8_t dma_phase;
    volatile bool dma_busy;
    bool online;
} OledSsd1306;

bool OledSsd1306_Init(OledSsd1306 *oled, I2C_HandleTypeDef *i2c);
void OledSsd1306_Clear(OledSsd1306 *oled);
void OledSsd1306_DrawText(OledSsd1306 *oled, uint8_t x, uint8_t y,
                          const char *text, const ASCIIFont *font);
bool OledSsd1306_Refresh(OledSsd1306 *oled);
bool OledSsd1306_RefreshAsync(OledSsd1306 *oled);
bool OledSsd1306_IsBusy(const OledSsd1306 *oled);
void OledSsd1306_TxCompleteCallback(OledSsd1306 *oled,
                                    I2C_HandleTypeDef *i2c);
void OledSsd1306_ErrorCallback(OledSsd1306 *oled,
                               I2C_HandleTypeDef *i2c);

#endif
