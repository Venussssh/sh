/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */

#define _CS_ON()   HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
#define _CS_OFF()   HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);


/* USER CODE END Private defines */

void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */
/*
 uint8_t SPI2_Read(uint8_t address);
 void SPI2_Write(uint8_t address, uint8_t text);
 uint8_t SPI_Read_Mul_Bite(uint8_t address_start);

void SPI2_Read(uint8_t reg_addr, uint8_t *rxBuf)
*/
uint8_t SPI2_Read(uint8_t address);
void SPI2_ReadCommand(uint8_t reg_addr);
void SPI2_WriteCommand(uint8_t reg_addr, uint8_t send_text);
void SPI_Read_Mul_Bite(uint8_t address_start, uint8_t *rev_data);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

