/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f0xx_hal.h"

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
#define LEDR_Pin GPIO_PIN_0
#define LEDR_GPIO_Port GPIOA
#define LEDG_Pin GPIO_PIN_1
#define LEDG_GPIO_Port GPIOA
#define LEDB_Pin GPIO_PIN_2
#define LEDB_GPIO_Port GPIOA
#define AMP_EN_Pin GPIO_PIN_5
#define AMP_EN_GPIO_Port GPIOA
#define I_G0_Pin GPIO_PIN_14
#define I_G0_GPIO_Port GPIOB
#define I_G1_Pin GPIO_PIN_15
#define I_G1_GPIO_Port GPIOB
#define I_G4_Pin GPIO_PIN_8
#define I_G4_GPIO_Port GPIOA
#define I_G3_Pin GPIO_PIN_9
#define I_G3_GPIO_Port GPIOA
#define I_G2_Pin GPIO_PIN_10
#define I_G2_GPIO_Port GPIOA
#define ADC_Alert_Pin GPIO_PIN_9
#define ADC_Alert_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
