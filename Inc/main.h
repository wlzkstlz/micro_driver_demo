/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#define PWM_AN_Pin GPIO_PIN_13
#define PWM_AN_GPIO_Port GPIOB
#define PWM_BN_Pin GPIO_PIN_14
#define PWM_BN_GPIO_Port GPIOB
#define PWM_CN_Pin GPIO_PIN_15
#define PWM_CN_GPIO_Port GPIOB
#define PWM_AP_Pin GPIO_PIN_8
#define PWM_AP_GPIO_Port GPIOA
#define PWM_BP_Pin GPIO_PIN_9
#define PWM_BP_GPIO_Port GPIOA
#define PWM_CP_Pin GPIO_PIN_10
#define PWM_CP_GPIO_Port GPIOA
#define HSENSOR_A_Pin GPIO_PIN_15
#define HSENSOR_A_GPIO_Port GPIOA
#define HSENSOR_B_Pin GPIO_PIN_3
#define HSENSOR_B_GPIO_Port GPIOB
#define HSENSOR_C_Pin GPIO_PIN_4
#define HSENSOR_C_GPIO_Port GPIOB
#define HSENSOR_C_EXTI_IRQn EXTI4_IRQn
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
