/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define mixer_op_filtered_Pin GPIO_PIN_0
#define mixer_op_filtered_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_14
#define LED4_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_15
#define LED5_GPIO_Port GPIOB
#define I2C2_SDA_digital_pot_Pin GPIO_PIN_8
#define I2C2_SDA_digital_pot_GPIO_Port GPIOA
#define I2C2_SCL_digital_pot_Pin GPIO_PIN_9
#define I2C2_SCL_digital_pot_GPIO_Port GPIOA
#define TIM1_CH3_LED_Pin GPIO_PIN_10
#define TIM1_CH3_LED_GPIO_Port GPIOA
#define LSM6DSL_ncs_Pin GPIO_PIN_15
#define LSM6DSL_ncs_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
