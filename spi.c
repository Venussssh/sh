/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


//7.31修改代码
void SPI2_ReadCommand(uint8_t reg_addr)
{
    reg_addr |= 0x80;
    uint8_t txBuf[2] = { reg_addr, 0x00};
    uint8_t rxBuf[2] = {0};

    _CS_ON();
    HAL_SPI_TransmitReceive(&hspi2,txBuf,rxBuf,sizeof(txBuf), 100);
    _CS_OFF();
}
// useless


void SPI2_WriteCommand(uint8_t reg_addr, uint8_t send_text)
{
reg_addr &= 0x7F;                               // write mode : MSB must be 0
  uint8_t txBuf[2] = { reg_addr, send_text };

  _CS_ON();
  HAL_SPI_Transmit(&hspi2,txBuf,sizeof(txBuf), 100);
  _CS_OFF();
}

void SPI2_Write(uint8_t *send_text, uint8_t reg_addr)
{
  reg_addr &= 0x7F;                               // write mode : MSB must be 0
  _CS_ON();
  HAL_SPI_Transmit(&hspi2,&reg_addr,1, 100);
  HAL_SPI_Transmit(&hspi2,(uint8_t *)send_text,1, 100);
  _CS_OFF();
}


 uint8_t SPI2_Read(uint8_t address)
 {
    address |= 0x80;                            // read mode : MSB must be 1
    uint8_t txBuf[2] = { address, 0x00};
    uint8_t rxBuf[2] = {0};

    _CS_ON();
    HAL_SPI_TransmitReceive(&hspi2,txBuf,rxBuf,sizeof(txBuf), 100);
    _CS_OFF();

    return rxBuf[1];                            // data in the buffer

     
 }


void SPI_Read_Mul_Bite(uint8_t *data, uint8_t address_start,uint16_t RxSize)
{
  address_start |= 0x80; // when reading, bit0 should be 1  
  _CS_ON();
  HAL_SPI_Transmit(&hspi2, &address_start, 1, 100);
  HAL_SPI_Receive(&hspi2, (uint8_t*)data, RxSize, 100); 
  _CS_OFF();
}

/* USER CODE END 1 */
