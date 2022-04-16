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
#include "stm32wbxx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IASO_SMART_Function.h"
#include "MAX30102.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/**LED**/
#define LED_MD1_ON HAL_GPIO_WritePin(LED_MD1_GPIO_Port,LED_MD1_Pin,RESET)
#define LED_MD1_OFF HAL_GPIO_WritePin(LED_MD1_GPIO_Port,LED_MD1_Pin,SET)
#define LED_MD2_ON HAL_GPIO_WritePin(LED_MD2_GPIO_Port,LED_MD2_Pin,RESET)
#define LED_MD2_OFF HAL_GPIO_WritePin(LED_MD2_GPIO_Port,LED_MD2_Pin,SET)
#define LED_MD3_ON HAL_GPIO_WritePin(LED_MD3_GPIO_Port,LED_MD3_Pin,RESET)
#define LED_MD3_OFF HAL_GPIO_WritePin(LED_MD3_GPIO_Port,LED_MD3_Pin,SET)

/**RGB_LED**/
#define LED_COLOR_GR_ON HAL_GPIO_WritePin(LED_GR_GPIO_Port,LED_GR_Pin,RESET)
#define LED_COLOR_GR_OFF HAL_GPIO_WritePin(LED_GR_GPIO_Port,LED_GR_Pin,SET)
#define LED_COLOR_RD_ON HAL_GPIO_WritePin(LED_RD_GPIO_Port,LED_RD_Pin,RESET)
#define LED_COLOR_RD_OFF HAL_GPIO_WritePin(LED_RD_GPIO_Port,LED_RD_Pin,SET)
#define LED_COLOR_BL_ON HAL_GPIO_WritePin(LED_BL_GPIO_Port,LED_BL_Pin,RESET)
#define LED_COLOR_BL_OFF HAL_GPIO_WritePin(LED_BL_GPIO_Port,LED_BL_Pin,SET)
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
#define BTN_DIAGO_Pin GPIO_PIN_8
#define BTN_DIAGO_GPIO_Port GPIOB
#define TIM2_BEEP_ON_Pin GPIO_PIN_0
#define TIM2_BEEP_ON_GPIO_Port GPIOA
#define ON_DDS_PWR_Pin GPIO_PIN_1
#define ON_DDS_PWR_GPIO_Port GPIOA
#define MAX30102_INT_Pin GPIO_PIN_8
#define MAX30102_INT_GPIO_Port GPIOA
#define MAX30102_INT_EXTI_IRQn EXTI9_5_IRQn
#define PPG_PWR_EN_Pin GPIO_PIN_4
#define PPG_PWR_EN_GPIO_Port GPIOC
#define MAX30102_CLK_Pin GPIO_PIN_10
#define MAX30102_CLK_GPIO_Port GPIOB
#define MAX30102_SDA_Pin GPIO_PIN_11
#define MAX30102_SDA_GPIO_Port GPIOB
#define CHG_ST_Pin GPIO_PIN_0
#define CHG_ST_GPIO_Port GPIOB
#define ON_LD_PWR_Pin GPIO_PIN_4
#define ON_LD_PWR_GPIO_Port GPIOE
#define LED_MD1_Pin GPIO_PIN_14
#define LED_MD1_GPIO_Port GPIOB
#define LED_MD2_Pin GPIO_PIN_15
#define LED_MD2_GPIO_Port GPIOB
#define LED_MD3_Pin GPIO_PIN_6
#define LED_MD3_GPIO_Port GPIOC
#define BTN_MODE_Pin GPIO_PIN_10
#define BTN_MODE_GPIO_Port GPIOA
#define ON_SOUND_PWR_Pin GPIO_PIN_15
#define ON_SOUND_PWR_GPIO_Port GPIOA
#define LED_RD_Pin GPIO_PIN_10
#define LED_RD_GPIO_Port GPIOC
#define LED_GR_Pin GPIO_PIN_11
#define LED_GR_GPIO_Port GPIOC
#define LED_BL_Pin GPIO_PIN_12
#define LED_BL_GPIO_Port GPIOC
#define ON_SYSTEM_Pin GPIO_PIN_1
#define ON_SYSTEM_GPIO_Port GPIOD
#define ON_LD650_Pin GPIO_PIN_4
#define ON_LD650_GPIO_Port GPIOB
#define ON_LD830_Pin GPIO_PIN_5
#define ON_LD830_GPIO_Port GPIOB
#define SYS_BTN_Pin GPIO_PIN_7
#define SYS_BTN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
