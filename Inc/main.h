/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l4xx_hal.h"

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
#define TestPoint_output1_Pin GPIO_PIN_1
#define TestPoint_output1_GPIO_Port GPIOA
#define TestPoint_output2_Pin GPIO_PIN_2
#define TestPoint_output2_GPIO_Port GPIOA
#define INT_PPG_Pin GPIO_PIN_6
#define INT_PPG_GPIO_Port GPIOA
#define EnableLIN_1V8_Pin GPIO_PIN_0
#define EnableLIN_1V8_GPIO_Port GPIOB
#define EnableSW_5V_Pin GPIO_PIN_1
#define EnableSW_5V_GPIO_Port GPIOB
#define MotorON_OFF_Pin GPIO_PIN_8
#define MotorON_OFF_GPIO_Port GPIOA
#define TestPoint_input_Pin GPIO_PIN_15
#define TestPoint_input_GPIO_Port GPIOA
#define LED_test_Pin GPIO_PIN_3
#define LED_test_GPIO_Port GPIOB
#define HeaterON_OFF_Pin GPIO_PIN_5
#define HeaterON_OFF_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
