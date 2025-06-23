
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#define DATA_SIZE 1000
#define BREAK_DELAY 100
#define SETUP_DELAY 500
typedef enum { false, true } bool;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//uint16_t mdb_tx_buf[DATA_SIZE] = {0x60, 0x60, 0x40, 0x18, 0x18, 0x00, 0x26, 0x40, 0x80, 0x34, 0xFF, 0xFF, 0xFF, 0x80, 0xA0, 0x50, 0x28, 0x4C, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

//uint16_t mdb_tx_buf[DATA_SIZE] = {0x40, 0x18, 0x18, 0x00, 0x26, 0x40, 0x80, 0x34, 0xFF, 0xFF, 0xFF, 0x80, 0xA0, 0x50, 0x28, 0x4C, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E8};

//uint16_t mdb_tx_buf[DATA_SIZE] = {0x02, 0x18, 0x18, 0x00, 0x64, 0x02, 0x01, 0x2c, 0xff, 0xff, 0xff, 0x01, 0x05, 0x0a,
		//, 0x32, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17c};

//uint8_t printing_mdb_tx_bufu8[DATA_SIZE] = {0x6, 0x0, 0x6, 0x0, 0x4, 0x0, 0x1, 0x8, 0x1, 0x8, 0x0, 0x0, 0x2, 0x6, 0x4, 0x0, 0x8, 0x0, 0x3, 0x4, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0x8, 0x0, 0xA, 0x0, 0x5, 0x0, 0x2, 0x8, 0x4, 0xC, 0x2, 0x6, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

uint16_t adding10[] = {0x0000, 0x0006, 0x0106, 0x0002, 0x0018, 0x0018, 0x0000, 0x0064,
0x0002, 0x0001, 0x002c, 0x00ff, 0x00ff, 0x00ff, 0x0001, 0x0005,
0x000a, 0x0014, 0x0032, 0x0064, 0x0000, 0x0000, 0x0000, 0x0000,
0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x017c, 0x0000,
0x0000, 0x0100, 0x0049, 0x0054, 0x004c, 0x0030, 0x0030, 0x0030,
0x0030, 0x0030, 0x0035, 0x0030, 0x0031, 0x0034, 0x0032, 0x0034,
0x0034, 0x004e, 0x0056, 0x0039, 0x0030, 0x0020, 0x0033, 0x0038,
0x0033, 0x0020, 0x0030, 0x0030, 0x0030, 0x0003, 0x0025, 0x01e0,
0x0100, 0x0000, 0x0000, 0x0100, 0x0100, 0x0000, 0x0000, 0x0100,
0x0100, 0x0082, 0x0182, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100};



uint16_t ack = 0x0100;
uint16_t  storing[DATA_SIZE];
uint16_t mdb_tx_buf[DATA_SIZE];
uint8_t  usb_tx_buf[DATA_SIZE];
uint16_t mdb_rx_buf[1];
uint16_t uart_tx_pointer = 0;
uint16_t storing_pointer = 0;
uint32_t usb_tx_ready = 0;
uint32_t mdb_tx_ready = 0;
uint16_t temp_size = 96;
uint32_t startMonit = 0;
extern volatile uint16_t break_timer;
extern volatile uint16_t usb_break_timer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void CDC_Break_Callback() {
  break_timer = BREAK_DELAY;
}

void CDC_Receive_Callback(uint8_t* Buf, uint32_t Len) {
  for (uint16_t i = 0; i < Len; i++) {
    mdb_tx_buf[i] = Buf[i];
  }
  mdb_tx_buf[0] |= 0x0100; //Set mode bit for command;
  mdb_tx_ready = Len;
}
int SnapShotFlag = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
	  uint16_t word = mdb_rx_buf[0]&(0x1FF);
	  storing[storing_pointer++] = mdb_rx_buf[0];
//	  usb_tx_buf[uart_tx_pointer++] = word;
	  uint8_t data = word & 0xFF;
	  uint8_t mode = (word>>8) & (0xFF);//(word & 0x100) ? 1 : 0;
	  uint32_t TimeStamp = HAL_GetTick()- startMonit;
	  usb_tx_buf[uart_tx_pointer++] = mode;  // 9th bit
	  usb_tx_buf[uart_tx_pointer++] = data;  // 8-bit data
	   usb_tx_buf[uart_tx_pointer++] = (TimeStamp>>24) & (0xFF);
	   usb_tx_buf[uart_tx_pointer++] = (TimeStamp>>16) & (0xFF);
	   usb_tx_buf[uart_tx_pointer++] = (TimeStamp>>8) & (0xFF);
	   usb_tx_buf[uart_tx_pointer++] = (TimeStamp>>0) & (0xFF);
//    if (!SnapShotFlag){
//      usb_tx_buf[uart_tx_pointer++] = 0xFF;
//      usb_tx_buf[uart_tx_pointer++] = (startMonit>>24) & (0xFF);
//     	usb_tx_buf[uart_tx_pointer++] = (startMonit>>16) & (0xFF);
//     	usb_tx_buf[uart_tx_pointer++] = (startMonit>>8) & (0xFF);
//     	usb_tx_buf[uart_tx_pointer++] = (startMonit>>0) & (0xFF);
//      usb_tx_buf[uart_tx_pointer++] = 0xFF;
//      SnapShotFlag =1;
//    }


	   usb_tx_buf[uart_tx_pointer++] = 0xAA;


//    uart_tx_pointer++;
    usb_tx_ready = uart_tx_pointer;

    uart_tx_pointer = 0;
//    if (mdb_rx_buf[0] == 0x0060) {
//      mdb_tx_ready = 1;
//    }
    HAL_UART_Receive_IT(huart, (uint8_t *) mdb_rx_buf, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

  __HAL_UART_DISABLE_IT(huart, UART_IT_TC);
  __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);

  /* Check if a receive process is ongoing or not */
  if(huart->gState == HAL_UART_STATE_BUSY_TX_RX)
  {
    huart->gState = HAL_UART_STATE_BUSY_RX;
  }
  else
  {
    huart->gState = HAL_UART_STATE_READY;
  }
//  HAL_UART_RxCpltCallback(huart);
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
uint8_t char_arr[] = "We are in the condition\n";
bool checking_response(uint16_t *buf, uint16_t buf_size)
{
	if(buf_size < 10) return false;
	for(int i = buf_size - 1; i > buf_size - 11; i--)
	{
		if(buf[i] != 0x0100)
		{
			return false;
		}
	}
	return true;
}

void SendData(uint16_t * byte)
{

}

void sendMDBByte(uint8_t data)
{
  // Send start bit (LOW)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  delayBit();

  // Send 8 data bits, LSB first
  for (uint8_t i = 0; i < 8; i++)
  {
    if (data & (1 << i))
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);  // Release bus
    else
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // Pull low

    delayBit();
  }

  // Send stop bit (HIGH)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  delayBit();
}

void delayBit(void)
{
  // For 9600 baud, bit time = 1/9600 = ~104 microseconds
  // We'll use ~100 us to match this
  uint32_t bitTime_us = 104;
  HAL_Delay(0); // Ensure context switch
  for (volatile uint32_t i = 0; i < (bitTime_us * 10); i++) {
    __NOP(); // Crude micro delay loop
  }
}

void send_9bit_data(UART_HandleTypeDef *huart, uint16_t data)
{
    // Write directly using DR register for 9-bit data
    while (!(huart->Instance->SR & USART_SR_TXE)); // Wait for TXE
    huart->Instance->DR = (data & 0x1FF); // Send lower 9 bits
}
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
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  usb_break_timer = SETUP_DELAY;
  int i = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
  int flag_entered = 0;
  uint32_t startMonit = 0;
  uint32_t curTime = 0;
  HAL_UART_Receive_IT(&huart1, (uint8_t *) mdb_rx_buf, 1);
  uint16_t temp[5]  = {0x00E1,0x00E2, 0x00E3, 0x00E4, 0x00E5};

  while (1)
  {
//    if (mdb_tx_ready > 0 && i < temp_size && HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX &&
//            HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX_RX) {
//    	//HAL_Delay(2);
//    	CDC_Transmit_FS((uint8_t *)(adding10 + i), 1);
//    	HAL_UART_Transmit_IT(&huart1, (uint8_t *)(adding10 + i), 1);
//    	i++;
//        //mdb_tx_ready = 0;
//
//    }
    //else if(i == temp_size) i = 0;
    //if(storing_pointer >= 10 && mdb_tx_ready == 0 && usb_tx_ready > 0)
    //{

    //}

    if (usb_tx_ready > 0) {
      CDC_Transmit_FS(usb_tx_buf, usb_tx_ready);
      usb_tx_ready = 0;
    }

//    if (break_timer > 0) {
//    	//HAL_GPIO_WritePin(BREAK_GPIO_Port, BREAK_Pin, GPIO_PIN_RESET);
//    } else {
//      HAL_GPIO_WritePin(BREAK_GPIO_Port, BREAK_Pin, GPIO_PIN_SET);
//    }
//
    if (usb_break_timer > 0) {
      HAL_GPIO_WritePin(USB_ON_GPIO_Port, USB_ON_Pin, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(USB_ON_GPIO_Port, USB_ON_Pin, GPIO_PIN_SET);
    }
     if (!flag_entered && usb_break_timer == 0)
     {
     	//HAL_Delay(8000);
     	startMonit = HAL_GetTick();
     	usb_tx_buf[0] = 0xFF;
     	usb_tx_buf[1] = (startMonit>>24) & (0xFF);
     	usb_tx_buf[2] = (startMonit>>16) & (0xFF);
     	usb_tx_buf[3] = (startMonit>>8) & (0xFF);
     	usb_tx_buf[4] = (startMonit>>0) & (0xFF);
		usb_tx_buf[5] = 0xFF;
     	CDC_Transmit_FS(usb_tx_buf, 8);
     	flag_entered = 1;
     	curTime = HAL_GetTick();
     }
    // if (HAL_GetTick() - curTime > 20000 && usb_break_timer == 0)
    // {
    // 	usb_tx_buf[0] = 0x77;
    // 	usb_tx_buf[1] = 0x77;
    // 	usb_tx_buf[2] = 0x77;
    // 	usb_tx_buf[3] = 0x77;
    // 	CDC_Transmit_FS(usb_tx_buf, 4);
    // 	HAL_GPIO_WritePin(BREAK_GPIO_Port, BREAK_Pin, GPIO_PIN_RESET);
    // 	HAL_Delay(1000);
    // 	curTime = HAL_GetTick();
    // }
  /* USER CODE END WHILE */

//	  HAL_UART_Transmit_IT(&huart1, (uint8_t *)(temp), 2);
//	  HAL_Delay(1000);
//	  send_9bit_data(&huart1, 0x1EF);
//	      send_9bit_data(&huart1, 0x1A3);
	      //HAL_Delay(1000);
// for (int i =0; i< 5; i++)
// {
//	  HAL_UART_Transmit(&huart1, (uint8_t *) (temp+i), 1, 1000);
////	  HAL_Delay(100);
// }
//
//
// HAL_Delay(2000);

	 // HAL_UART_Receive_IT(&huart1, (uint8_t *) mdb_rx_buf, 1);
  /* USER CODE BEGIN 3 */
  }
#pragma clang diagnostic pop
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
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
