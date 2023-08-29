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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
#define Acc_EXTI_Pin GPIO_PIN_1
#define Acc_EXTI_GPIO_Port GPIOB
#define Acc_EXTI_EXTI_IRQn EXTI1_IRQn
#define Gyro_EXTI_Pin GPIO_PIN_2
#define Gyro_EXTI_GPIO_Port GPIOB
#define Gyro_EXTI_EXTI_IRQn EXTI2_TSC_IRQn
#define KEY2_Pin GPIO_PIN_10
#define KEY2_GPIO_Port GPIOB
#define KEY2_EXTI_IRQn EXTI15_10_IRQn
#define KEY1_Pin GPIO_PIN_11
#define KEY1_GPIO_Port GPIOB
#define KEY1_EXTI_IRQn EXTI15_10_IRQn
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define KEY3_Pin GPIO_PIN_8
#define KEY3_GPIO_Port GPIOA
#define KEY3_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
typedef struct 
{
  bool flag1;
  bool flag2;
  bool flag3;
  bool flag4;
  bool flag5;
  bool flag6;

}LSM6DS3_FLAG_TypeDef;
typedef struct
{
  float ACC_DATA_INT[3];
  float GYRO_DATA_INT[3];
  uint8_t Pitch;
  uint8_t Roll;
}LSM6DS3_DATA_TypeDef;
typedef struct
{
  uint8_t Change1;

}LSM6DS3_CHANGE_TypeDef;



void run(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
