/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define AUTO_CMD_Pin GPIO_PIN_0
#define AUTO_CMD_GPIO_Port GPIOA
#define MANUAL_CMD_Pin GPIO_PIN_1
#define MANUAL_CMD_GPIO_Port GPIOA
#define ADC_24VSENSE_Pin GPIO_PIN_4
#define ADC_24VSENSE_GPIO_Port GPIOA
#define ADC_POTEN_SIG_Pin GPIO_PIN_5
#define ADC_POTEN_SIG_GPIO_Port GPIOA
#define ADC_48VSENSE_Pin GPIO_PIN_6
#define ADC_48VSENSE_GPIO_Port GPIOA
#define ADC_BRAKE_CUR_Pin GPIO_PIN_7
#define ADC_BRAKE_CUR_GPIO_Port GPIOA
#define EMER_SIG_Pin GPIO_PIN_0
#define EMER_SIG_GPIO_Port GPIOB
#define DIR_SIG_Pin GPIO_PIN_1
#define DIR_SIG_GPIO_Port GPIOB
#define Gear_P_Pin GPIO_PIN_12
#define Gear_P_GPIO_Port GPIOB
#define Gear_R_Pin GPIO_PIN_13
#define Gear_R_GPIO_Port GPIOB
#define Gear_N_Pin GPIO_PIN_14
#define Gear_N_GPIO_Port GPIOB
#define Gear_D_Pin GPIO_PIN_15
#define Gear_D_GPIO_Port GPIOB
#define RGB_WS2812_Pin GPIO_PIN_8
#define RGB_WS2812_GPIO_Port GPIOA
#define BRAKE_PWM_Pin GPIO_PIN_4
#define BRAKE_PWM_GPIO_Port GPIOB
#define BRAKE_DIR_Pin GPIO_PIN_5
#define BRAKE_DIR_GPIO_Port GPIOB
#define ENCODER_A_Pin GPIO_PIN_6
#define ENCODER_A_GPIO_Port GPIOB
#define ENCODER_B_Pin GPIO_PIN_7
#define ENCODER_B_GPIO_Port GPIOB
#define STEERING_RELAY_Pin GPIO_PIN_8
#define STEERING_RELAY_GPIO_Port GPIOB
#define BRAKE_SIG_Pin GPIO_PIN_9
#define BRAKE_SIG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
