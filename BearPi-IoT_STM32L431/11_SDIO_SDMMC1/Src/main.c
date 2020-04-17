
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "sdmmc.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
HAL_SD_CardCIDTypedef SD_CardCID;
#define BLOCK_SIZE            512         // 块大小     
#define NUMBER_OF_BLOCKS      8           // 块数量
#define WRITE_READ_ADDRESS    0x00002000  // 读写地址
#define WRITE_READ_ADDRESS2    0x00004000  // 读写地址
uint32_t Buffer_Block_Tx[BLOCK_SIZE*NUMBER_OF_BLOCKS]; // 据缓存
uint32_t Buffer_Block_Rx[BLOCK_SIZE*NUMBER_OF_BLOCKS] = {1,2}; // 数据缓存
HAL_StatusTypeDef sd_status;    // HAL返回值
void ReadSD(void);
void SD_Write_Read_Test(int32_t addr,uint8_t* temp);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
uint8_t temp[]="欢迎来到物联网俱乐部";
uint8_t temp1[]="Welcome to IoT";

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SDMMC1_SD_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ReadSD();//读取SD卡初始化参数
	HAL_SD_ReadBlocks(&hsd1,(uint8_t *)Buffer_Block_Rx,WRITE_READ_ADDRESS,NUMBER_OF_BLOCKS,0xffff);
	printf("Read SD old data1:%s,address:%x\r\n",(char*)Buffer_Block_Rx,WRITE_READ_ADDRESS); 
	memset(Buffer_Block_Rx,0x00,sizeof(Buffer_Block_Rx));
  HAL_SD_ReadBlocks(&hsd1,(uint8_t *)Buffer_Block_Rx,WRITE_READ_ADDRESS2,NUMBER_OF_BLOCKS,0xffff);
	printf("Read SD old data2:%s,address:%x\r\n",(char*)Buffer_Block_Rx,WRITE_READ_ADDRESS2); 
	memset(Buffer_Block_Rx,0x00,sizeof(Buffer_Block_Rx));
  SD_Write_Read_Test(WRITE_READ_ADDRESS,temp);
	memset(Buffer_Block_Rx,0x00,sizeof(Buffer_Block_Rx));
	SD_Write_Read_Test(WRITE_READ_ADDRESS2,temp1);
	memset(Buffer_Block_Rx,0x00,sizeof(Buffer_Block_Rx));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */
  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
	HAL_Delay(1000);
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SDMMC1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void ReadSD(void)//读取SD出厂参数
{
      int State=0;
      State = HAL_SD_GetCardState(&hsd1);
	if(State == HAL_SD_CARD_TRANSFER)
	{
	HAL_SD_GetCardCID(&hsd1,&SD_CardCID);
	printf("\r\n Initialize SD card successfully!\r\n\r\n");
	printf(" SD card information! \r\n");
	printf(" CardCapacity  : %llu \r\n",((unsigned long long)hsd1.SdCard.BlockSize*hsd1.SdCard.BlockNbr));
	printf(" CardBlockSize : %d \r\n",hsd1.SdCard.BlockSize);
	printf(" RCA           : %d \r\n",hsd1.SdCard.RelCardAdd);
	printf(" CardType      : %d \r\n",hsd1.SdCard.CardType);
	printf(" ManufacturerID: %d \r\n",SD_CardCID.ManufacturerID);
	}
	else
	{
  	printf("SD card init fail!\r\n" );
		while(1); // 停机
	}
}


void SD_Write_Read_Test(int32_t addr,uint8_t* temp)//写入SD卡数据并读出数据
{  
  
  printf("\r\n\r\n*************Write SD card data *************\r\n");
  sd_status = HAL_SD_WriteBlocks(&hsd1,(uint8_t *)temp,addr,NUMBER_OF_BLOCKS,0xffff);
  printf("Write data:%s,address:%x\r\n",temp,addr);
  HAL_Delay(480);
  if(sd_status == 0)
   {
     printf("Write data test is OK\r\n" );
   }	
  else  
     printf("Write data test is fail!\r\n " );  
   printf("*************Read SD card data *************\r\n");
   sd_status = HAL_SD_ReadBlocks(&hsd1,(uint8_t *)Buffer_Block_Rx,addr,NUMBER_OF_BLOCKS,0xffff);
   printf("ReadSDData:%s,address:%x\r\n",(char*)Buffer_Block_Rx,addr); 
  if(sd_status == 0)
   {
     printf("Read data test is OK\r\n" );
   }	
  else  
  printf("Read data test is fail!\r\n " );  
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
