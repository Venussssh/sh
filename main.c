/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "LSM6DS3.h"
#include "OLED.h"
#include "stm32f3xx_it.h"
#include "vqf.h"
#include "math.h"
// #include "basicvqf.h"
LSM6DS3_FLAG_TypeDef LSM6DS3_READWRITE;
LSM6DS3_DATA_TypeDef LSM6DS3_AG_DATA;
LSM6DS3_CHANGE_TypeDef LSM6DS3_CHANGE;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
SSD1306_Init();
char finish[] = {'I','n','i','t',' ','F','i','n','i','s','h','e','d','!','!'};
OLED_ShowString(1, 1, &finish);
LSM6DS3_init();

HAL_Delay(500);
float Gyro_Tempout[3] = {0};
Get_Gyro_Compensate(&Gyro_Tempout);
float Acc_Tempout[3] = {0};
Get_Acc_Compensate(&Acc_Tempout);

HAL_Delay(500);

HAL_TIM_Base_Start_IT(&htim2);







  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(LSM6DS3_READWRITE.flag6 == true)
    {
      // update(LSM6DS3_AG_DATA.GYRO_DATA_INT, LSM6DS3_AG_DATA.ACC_DATA_INT);
      // getQuat6D(quat);
      // vqf_real_t angle[3] = {0, 0, 0};
      // RunBasicVQF(LSM6DS3_AG_DATA.GYRO_DATA_INT[0] - Gyro_Tempout[0], LSM6DS3_AG_DATA.GYRO_DATA_INT[1] - Gyro_Tempout[1], LSM6DS3_AG_DATA.GYRO_DATA_INT[2] - Gyro_Tempout[2], LSM6DS3_AG_DATA.ACC_DATA_INT[0] - Acc_Tempout[0], LSM6DS3_AG_DATA.ACC_DATA_INT[1] - Acc_Tempout[1], LSM6DS3_AG_DATA.ACC_DATA_INT[2] - Acc_Tempout[2]+0.98, &angle);
      // printf("\raccX:%.2f,\taccY:%.2f,\taccZ:%.2f,\tgyroX:%.2f,\tgyroY:%.2f,\tgyroZ:%.2f, \tPitch:%.2f, \tRoll:%.2f, \tYaw:%.2f\r", LSM6DS3_AG_DATA.ACC_DATA_INT[0] - Acc_Tempout[0], LSM6DS3_AG_DATA.ACC_DATA_INT[1] - Acc_Tempout[1], LSM6DS3_AG_DATA.ACC_DATA_INT[2] - Acc_Tempout[2] + 0.98, LSM6DS3_AG_DATA.GYRO_DATA_INT[0] - Gyro_Tempout[0], LSM6DS3_AG_DATA.GYRO_DATA_INT[1] - Gyro_Tempout[1], LSM6DS3_AG_DATA.GYRO_DATA_INT[2] - Gyro_Tempout[2], angle[0], angle[1], angle[2]);
      // printf("\n");
      vqf_real_t angle[3] = {0, 0, 0};
      //VQF
      RunVQF(LSM6DS3_AG_DATA.GYRO_DATA_INT[0] - Gyro_Tempout[0], LSM6DS3_AG_DATA.GYRO_DATA_INT[1] - Gyro_Tempout[1], LSM6DS3_AG_DATA.GYRO_DATA_INT[2] - Gyro_Tempout[2], LSM6DS3_AG_DATA.ACC_DATA_INT[0] - Acc_Tempout[0], LSM6DS3_AG_DATA.ACC_DATA_INT[1] - Acc_Tempout[1], LSM6DS3_AG_DATA.ACC_DATA_INT[2] - Acc_Tempout[2]+0.98, &angle);
      // printf("\raccX:%.2f,\taccY:%.2f,\taccZ:%.2f,\tgyroX:%.2f,\tgyroY:%.2f,\tgyroZ:%.2f, \tPitch:%.2f, \tRoll:%.2f\r", LSM6DS3_AG_DATA.ACC_DATA_INT[0] - Acc_Tempout[0], LSM6DS3_AG_DATA.ACC_DATA_INT[1] - Acc_Tempout[1], LSM6DS3_AG_DATA.ACC_DATA_INT[2] - Acc_Tempout[2] + 0.98, LSM6DS3_AG_DATA.GYRO_DATA_INT[0] - Gyro_Tempout[0], LSM6DS3_AG_DATA.GYRO_DATA_INT[1] - Gyro_Tempout[1], LSM6DS3_AG_DATA.GYRO_DATA_INT[2] - Gyro_Tempout[2], Pitch, Roll);
      printf("\raccX:%.2f,\taccY:%.2f,\taccZ:%.2f,\tgyroX:%.2f,\tgyroY:%.2f,\tgyroZ:%.2f, \tPitch:%.2f, \tRoll:%.2f, \tYaw:%.2f\r", LSM6DS3_AG_DATA.ACC_DATA_INT[0] - Acc_Tempout[0], LSM6DS3_AG_DATA.ACC_DATA_INT[1] - Acc_Tempout[1], LSM6DS3_AG_DATA.ACC_DATA_INT[2] - Acc_Tempout[2] + 0.98, LSM6DS3_AG_DATA.GYRO_DATA_INT[0] - Gyro_Tempout[0], LSM6DS3_AG_DATA.GYRO_DATA_INT[1] - Gyro_Tempout[1], LSM6DS3_AG_DATA.GYRO_DATA_INT[2] - Gyro_Tempout[2], angle[0], angle[1], angle[2]);
      printf("\n");
      LSM6DS3_READWRITE.flag6 = false;
    }
    


    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
  if(GPIO_Pin==KEY1_Pin)
  {
    LSM6DS3_READWRITE.flag1 = true;

  }
  

  if(GPIO_Pin==KEY2_Pin)
  {
    LSM6DS3_READWRITE.flag2 = true;
  }



  if(GPIO_Pin==KEY3_Pin)
  {
    LSM6DS3_READWRITE.flag3 = true;
  }
  //Acc中断
  if(GPIO_Pin==Acc_EXTI_Pin)
	
  {
    LSM6DS3_READWRITE.flag4 = true;
    
    Get_Acceleration(LSM6DS3_AG_DATA.ACC_DATA_INT);

	}
  //Gyro中断
  if(GPIO_Pin==Gyro_EXTI_Pin)
  {
    LSM6DS3_READWRITE.flag5 = true;
    
    Get_Gyroscope(LSM6DS3_AG_DATA.GYRO_DATA_INT);
    
  }

}

//定时器回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  vqf_real_t quat[4] = {0, 0, 0, 0};
  Get_Acceleration(LSM6DS3_AG_DATA.ACC_DATA_INT);
  Get_Gyroscope(LSM6DS3_AG_DATA.GYRO_DATA_INT);
  LSM6DS3_READWRITE.flag6 = true;
  
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
