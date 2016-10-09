#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_rcc_ex.h"
#include "stm32f0xx_it.h"

/* Private macro -------------------------------------------------------------*/

#define LED_GPIO_PORT GPIOB
#define LED_GPIO_PIN GPIO_PIN_1

#define BTN_GPIO_PORT GPIOA
#define BTN_GPIO_PIN GPIO_PIN_0

#endif /* __MAIN_H */
