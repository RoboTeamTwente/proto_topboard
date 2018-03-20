/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include <roboNRF24.h>
#include "main.h"
#include "stm32f3xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "PuttyInterface/PuttyInterface.h"
#include "myNRF24.h"
#include "roboNRF24.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//values for puttyinterface.h
uint8_t rec_buf[8];
char small_buf;
volatile bool huart2_Rx_flag = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HandleCommand(char * input);
int ReadAddress();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */

  //setup code for using the nRF24 module
  uint8_t robotID = ReadAddress(); //usually that should be the RobotID
  nrf24nssHigh(); //I think we need that, but I can't really say, yet, why we would need to call low-level functions in main()

  while(initRobo(&hspi2, RADIO_CHANNEL, robotID) != 0) {
	  uprintf("Error while initializing nRF wireless module. Check connections.\n");
  }
  dataPacket dataStruct;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char * startmessage = "---------------------\n\r";
  uprintf(startmessage);
  uprintf("Build: %s %s\n", __DATE__, __TIME__);

  HAL_UART_Receive_IT(&huart1, rec_buf, 1); //This is needed for debugging input/output on serial

  //int tick = 0;
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
  //HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
//  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);

  while (1)
  {
	  //HAL_Delay(10); //10ms delay
	  if(huart2_Rx_flag){
		  //handle debug input/output over UART (connect an ST-Link to it for easy debugging over USB)
		  huart2_Rx_flag = false;
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED3_Pin);
		  HandlePcInput(&small_buf, 1, HandleCommand);
		  HAL_UART_Receive_IT(&huart1, rec_buf, 1);
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED3_Pin);
	  }

	  if(irqRead()){
		  uprintf("Interrupt up..\n");
		  uint8_t status_reg = readReg(STATUS);
		  uint8_t rx_dr = (status_reg & RX_DR) > 0;
		  uint8_t tx_ds = (status_reg & TX_DS) > 0;
		  uint8_t max_rt = (status_reg & MAX_RT) > 0;
		  uint8_t tx_full = (status_reg & STATUS_TX_FULL) > 0;
		  uprintf("Interrupts:\n rx_dr: %i, tx_ds: %i, max_rt: %i, tx_full: %i\n", rx_dr, tx_ds, max_rt, tx_full);
		  roboCallback(&dataStruct);
		  if(dataStruct.robotID == ReadAddress()){
			  uprintf("It's for me!\n");
			  //blink
			  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

		  }
		  else {
			  uprintf("It was for: %i", dataStruct.robotID);
		  }
		  //HAL_Delay(10);
		  //flushRX(&hspi2);

		  //clearing interrupts without checking why the interrupt was there
		  //not a good idea in general
		  //clearInterrupts();
	  }
	  /* END nRF24 polling */



  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
void HandleCommand(char * input){
	if(!strcmp(input, "start")){
		uprintf("started\n\r");
	}else if(!strcmp(input, "address")){
		uprintf("Address = [%d]\n\r", ReadAddress());
	}else if(!strcmp(input, "reg")) {
		uprintf("You are trying to read registers.\n");
		uprintf("This feature is under construction.\n");
		uint8_t registerOutput = readReg(RX_ADDR_P0); //0x0A = RX_ADDR_P0 -- Receiving Address, Data Pipe 0.
		uprintf("RX_ADDR_P0: 0x%x\n", registerOutput);
		uprintf("This output will be more explanatory in the future.\n");
	}else if(!strcmp(input, "help")) {
		uprintf("----HELP----\n");
		uprintf("Build: %s %s\n", __DATE__, __TIME__);
		uprintf("Hardware Version: 2017 Rev. A\n");
		uprintf("List of available commands: \n");
		uprintf("help -- Prints this help message\n");
		uprintf("start -- Not implemented\n");
		uprintf("address -- Prints the address as read from the DIP switches on the board.\n");
		uprintf("reg -- Print the values of some registers. Call without Parameters.\n");
		uprintf("(this list may be incomplete)\n");
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED4_Pin);
	if(huart->Instance == huart1.Instance){
		huart2_Rx_flag = true;
		small_buf = *(huart->pRxBuffPtr-1);
	}
}
int ReadAddress(){
	return (HAL_GPIO_ReadPin(JD0_GPIO_Port, JD0_Pin) << 0 | HAL_GPIO_ReadPin(JD1_GPIO_Port, JD1_Pin)  << 1 | HAL_GPIO_ReadPin(JD2_GPIO_Port, JD2_Pin) << 2 | HAL_GPIO_ReadPin(JD3_GPIO_Port, JD3_Pin)  << 3);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	uprintf("ERROR in file %s in line %i\n", file, line);
	uprintf("Stopping execution. Please reset.\n");
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
