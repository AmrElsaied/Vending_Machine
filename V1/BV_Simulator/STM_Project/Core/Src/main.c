
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
#include "bill_validator.h"
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#define DATA_SIZE 1000
#define BREAK_DELAY 100
#define SETUP_DELAY 500
#define ENABLE_USB_LOGGING  0
#define ENABLE_BV_TX        1
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t mdb_tx_buf[DATA_SIZE];
uint8_t  usb_tx_buf[DATA_SIZE];
uint16_t mdb_rx_buf[1];
uint16_t USB_tx_pointer = 0;
extern volatile uint16_t break_timer;
extern volatile uint16_t usb_break_timer;
bool Vending_EN = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MDB_Peripheral_Init(void);
void MDB_HandleCommand(uint16_t *data, uint8_t dataLength);
void MDB_SendResponseWithModeBit(uint16_t *data, uint8_t dataLength);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void CDC_Break_Callback() {
  break_timer = BREAK_DELAY;
}

void CDC_Receive_Callback(uint8_t* Buf, uint32_t Len) {
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    uint16_t word = mdb_rx_buf[0]&0x1FF;
    // check if the BV_CMD_RXhandler is ready to recieve a new command
    switch (BV_StateManager.BV_CMD_RX_StateHandler)
    {
    case CMD_RX_READY:
      // Check if the first byte is a valid VMC command
      for (uint8_t i = 0; i < VMC_CMD_MAX_NUMBER; i++)
      {
        if( word == VMC_CMDs[i].CMD[0])
        {
          // Command found, recieve rest of the command
          BV_StateManager.BV_CMD_RX_StateHandler = CMD_RX_INPROGRESS;
          BV_MDB_BusManager.MDB_RX_CMD_Index = i; // Store the Rx command index
          BV_MDB_BusManager.MDB_RXbuffer[BV_MDB_BusManager.RXBuffer_index++] = word;
          break;
        }
      }
      break;
    case CMD_RX_INPROGRESS:
      // Store the received word in the RX buffer
      BV_MDB_BusManager.MDB_RXbuffer[BV_MDB_BusManager.RXBuffer_index++] = word;
      // Check if the command is fully received
      if (BV_MDB_BusManager.RXBuffer_index >= VMC_CMDs[BV_MDB_BusManager.MDB_RX_CMD_Index].CMD_Length)
      {
        // Command fully received, process it
        BV_StateManager.BV_CMD_RX_StateHandler = CMD_RX_DONE;
        // MDB_HandleCommand(BV_MDB_BusManager.MDB_RXbuffer, BV_MDB_BusManager.RXBuffer_index);
      }
      break;
    case CMD_RX_DONE:
      //TODO handle the state of processing at this time
      break;
    case CMD_RX_BUSY:
      //TODO handle the state of processing at this time
      break;
    default:
      break;
    }
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
   HAL_UART_RxCpltCallback(huart);
}


// MDB Peripheral Initialization Function
void MDB_Peripheral_Init(void)
{
  BV_StateManager.BV_StateHnadler = STATE_RESTART; // Set initial state to RESTART
  BV_StateManager.BV_CMD_RX_StateHandler = CMD_RX_READY; // Set command reception state to READY
}

void MDB_SendResponseWithModeBit(uint16_t *data, uint8_t dataLength)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)data, dataLength);
}
// void MDB_HandleTXCommand(uint8_t VMC_CMD_Index)
// {
//     if (BV_StateManager.BV_CMD_TX_StateHandler == CMD_TX_READY)
//     {
//         BV_StateManager.BV_CMD_TX_StateHandler = CMD_TX_BUSY; // Set the state to BUSY
//         BV_MDB_BusManager.MDB_TX_CMD_Index = VMC_CMD_Index; // Set the command index to the command being processed
//         BV_MDB_BusManager.TXBuffer_index = 0; // Reset the TX buffer index
//         uint8_t length = VMC_CMDs[VMC_CMD_Index].CMD_Length;
//         for (uint8_t i = 0; i < length; i++)
//         {
//             mdb_tx_buf[BV_MDB_BusManager.TXBuffer_index++] = VMC_CMDs[VMC_CMD_Index].CMD[i];
//         }
//         MDB_SendResponseWithModeBit(mdb_tx_buf, BV_MDB_BusManager.TXBuffer_index);
//         BV_StateManager.BV_CMD_TX_StateHandler = CMD_TX_DONE; // Set the state to DONE after transmission
//     }
// }
void MDB_HandleCommand(uint16_t *BV_RxBuffer, uint8_t length)
{
  switch (BV_StateManager.BV_CMD_Process_StateHandler)
  {
    case CMD_PROCESS_READY:
      if (BV_StateManager.BV_CMD_RX_StateHandler == CMD_RX_DONE)
      {
        BV_StateManager.BV_CMD_RX_StateHandler = CMD_RX_BUSY; // Set the state to BUSY
        BV_StateManager.BV_CMD_Process_StateHandler = CMD_PROCESS_INPROGRESS; // Set the command processing state to INPROGRESS
        // Command reception is done, process the command
        int temp_index = BV_MDB_BusManager.MDB_RX_CMD_Index;
        int temp_length = VMC_CMDs[temp_index].CMD_Length;
        if (BV_MDB_BusManager.RXBuffer_index != VMC_CMDs[temp_index].CMD_Length)
        {
          // Error: Received command length does not match expected length
          // TODO Handle error appropriately
        }
        else
        {
          if (BV_RxBuffer[(BV_MDB_BusManager.RXBuffer_index)-1] == VMC_CMDs[temp_index].CMD[temp_length-1])
          {
              // Command is valid, process it
            BV_MDB_BusManager.MDB_Process_CMD_Index = temp_index; // Set the command index to the command being processed
            if (VMC_CMDs[temp_index].CMD[0] == VMC_CMDs[VMC_CMD_0x0066].CMD[0])
            {
              switch (BV_StateManager.BV_StateHnadler)
              {
              case STATE_RESTART:
                VMC_CMDs[VMC_CMD_0x0066].CMD_Response[0] = 0x0006;
                VMC_CMDs[VMC_CMD_0x0066].CMD_Response[1] = 0x0106;
                VMC_CMDs[VMC_CMD_0x0066].CMD_Response_Length = 2;
                BV_StateManager.BV_StateHnadler = STATE_DISABLED; // Set the system state to disabled
                break;
              case STATE_DISABLED:
                VMC_CMDs[VMC_CMD_0x0066].CMD_Response[0] = 0x0009;
                VMC_CMDs[VMC_CMD_0x0066].CMD_Response[1] = 0x0109;
                VMC_CMDs[VMC_CMD_0x0066].CMD_Response_Length = 2;
                break;
              case STATE_READY:
                if (HAL_GPIO_ReadPin(VENDING_EN_GPIO_Port, VENDING_EN_Pin) == GPIO_PIN_RESET
                                     && Vending_EN == false)
                  {
                    VMC_CMDs[VMC_CMD_0x0066].CMD_Response[0] = 0x0083;
                    VMC_CMDs[VMC_CMD_0x0066].CMD_Response[1] = 0x0183;
                    VMC_CMDs[VMC_CMD_0x0066].CMD_Response_Length = 2;
                    Vending_EN = true;
                  }
                  else
                  {
                    VMC_CMDs[VMC_CMD_0x0066].CMD_Response[0] = 0x0100;
                    VMC_CMDs[VMC_CMD_0x0066].CMD_Response_Length = 1;
                  }
                  if (HAL_GPIO_ReadPin(VENDING_EN_GPIO_Port, VENDING_EN_Pin) == GPIO_PIN_SET
                                     && Vending_EN == true)
                  {
                    Vending_EN = false;
                  }
                break;
              default:
                BV_StateManager.BV_StateHnadler = STATE_ERROR; // Set the system state to error
                break;
              }
            }
            else if (VMC_CMDs[temp_index].CMD[0] == VMC_CMDs[VMC_CMD_0x009D].CMD[0])
            {
              BV_StateManager.BV_StateHnadler = STATE_READY; // Set the system state to ready
            }
            // Send the response
            #if ENABLE_BV_TX == 1
            MDB_SendResponseWithModeBit(VMC_CMDs[BV_MDB_BusManager.MDB_Process_CMD_Index].CMD_Response,
                                        VMC_CMDs[BV_MDB_BusManager.MDB_Process_CMD_Index].CMD_Response_Length);
            #endif
          }
          else
          {
              // Error: Received command does not match expected command
              // TODO Handle error appropriately
          }
        }
        // Reset the RX for a new command
        BV_StateManager.BV_CMD_RX_StateHandler = CMD_RX_READY; // Set the state to READY for the next command
        BV_StateManager.BV_CMD_Process_StateHandler = CMD_PROCESS_READY; // Set the command processing state to DONE
        BV_MDB_BusManager.RXBuffer_index = 0; // Reset the RX buffer index
      }
      else
      {
        // No CMD to be processed
        // This means we are still waiting for the command to be fully received
        return;
      }
      break;
    case CMD_PROCESS_INPROGRESS:
      // Error: Command processing state is already in progress
      // TODO Handle error appropriately
      return;
    case CMD_PROCESS_DONE:
      // Command processing is done, reset the state
      BV_StateManager.BV_CMD_Process_StateHandler = CMD_PROCESS_READY; // Set the command processing state to READY
      BV_MDB_BusManager.RXBuffer_index = 0; // Reset the RX buffer index for the next command
      break;
    default:
      // Error: Command processing state is not ready or in progress
      // TODO Handle error appropriately
      return;
  }
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
  /* USER CODE END 2 */

  // Initialize MDB Peripheral
  MDB_Peripheral_Init();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
  HAL_UART_Receive_IT(&huart1, (uint8_t *) mdb_rx_buf, 1);
  while (1)
  {

    // if (break_timer > 0) {
    //   // HAL_GPIO_WritePin(BREAK_GPIO_Port, BREAK_Pin, GPIO_PIN_RESET);
    // } else {
    //   HAL_GPIO_WritePin(BREAK_GPIO_Port, BREAK_Pin, GPIO_PIN_SET);
    // }

    // if (usb_break_timer > 0) {
    //   HAL_GPIO_WritePin(USB_ON_GPIO_Port, USB_ON_Pin, GPIO_PIN_RESET);
    // } else {
    //   HAL_GPIO_WritePin(USB_ON_GPIO_Port, USB_ON_Pin, GPIO_PIN_SET);
    // }

    MDB_HandleCommand(BV_MDB_BusManager.MDB_RXbuffer, BV_MDB_BusManager.RXBuffer_index);
  /* USER CODE END WHILE */

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
