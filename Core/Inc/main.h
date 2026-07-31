#ifndef BALLCONTROL_MAIN_H
#define BALLCONTROL_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern PCD_HandleTypeDef hpcd;

#define CORE_LED_GPIO_PORT GPIOB
#define CORE_LED_PIN       GPIO_PIN_2

#define CORE_USER_KEY_GPIO_PORT GPIOA
#define CORE_USER_KEY_PIN       GPIO_PIN_0

#define KEY0_GPIO_PORT GPIOE
#define KEY0_PIN       GPIO_PIN_1
#define KEY1_GPIO_PORT GPIOC
#define KEY1_PIN       GPIO_PIN_0
#define KEY2_GPIO_PORT GPIOE
#define KEY2_PIN       GPIO_PIN_2
#define KEY3_GPIO_PORT GPIOE
#define KEY3_PIN       GPIO_PIN_3
#define KEY4_GPIO_PORT GPIOE
#define KEY4_PIN       GPIO_PIN_4

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
