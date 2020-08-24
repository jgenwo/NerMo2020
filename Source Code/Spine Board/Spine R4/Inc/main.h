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
#include "stm32f7xx_hal.h"

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
#define ADC1_IN10_VBatt_Pin GPIO_PIN_0
#define ADC1_IN10_VBatt_GPIO_Port GPIOC
#define ADC1_IN11_VBatt_2_Pin GPIO_PIN_1
#define ADC1_IN11_VBatt_2_GPIO_Port GPIOC
#define GPIO_ADC_Supply_GND_Pin GPIO_PIN_2
#define GPIO_ADC_Supply_GND_GPIO_Port GPIOC
#define GPIO_ADC_2_GND_Pin GPIO_PIN_3
#define GPIO_ADC_2_GND_GPIO_Port GPIOC
#define GPIO_out_LED_Pin GPIO_PIN_1
#define GPIO_out_LED_GPIO_Port GPIOA
#define GPIO_Nose_Bumper_Pin GPIO_PIN_5
#define GPIO_Nose_Bumper_GPIO_Port GPIOA
#define GPIO_Head_Padding_Pin GPIO_PIN_7
#define GPIO_Head_Padding_GPIO_Port GPIOA
#define GPIO_PCB_routing_dummy_Pin GPIO_PIN_0
#define GPIO_PCB_routing_dummy_GPIO_Port GPIOB
#define GPIO_input_routing_dummy_Pin GPIO_PIN_12
#define GPIO_input_routing_dummy_GPIO_Port GPIOB
#define GPIO____U3_RTS_Pin GPIO_PIN_14
#define GPIO____U3_RTS_GPIO_Port GPIOB
#define GPIO_enable_Vreg_Servo_Pin GPIO_PIN_15
#define GPIO_enable_Vreg_Servo_GPIO_Port GPIOB
#define GPIO_enable_Vreg_RPIZW_Pin GPIO_PIN_8
#define GPIO_enable_Vreg_RPIZW_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
