/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bill_validator.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENABLE_USB_LOGGING  0
#define ENABLE_BV_TX        1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
bool Vending_EN = false;
uint16_t mdb_rx_buf[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void MDB_HandleCommand(uint16_t *data, uint8_t dataLength);
void MDB_SendResponseWithModeBit(uint16_t *data, uint8_t dataLength);
void MDB_Peripheral_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
                if (HAL_GPIO_ReadPin(VENDING_GPIO_Port, VENDING_Pin) == GPIO_PIN_RESET
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
                  if (HAL_GPIO_ReadPin(VENDING_GPIO_Port, VENDING_Pin) == GPIO_PIN_SET
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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // Initialize MDB Peripheral
  MDB_Peripheral_Init();
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
  HAL_UART_Receive_IT(&huart1, (uint8_t *) mdb_rx_buf, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : VENDING_Pin */
  GPIO_InitStruct.Pin = VENDING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(VENDING_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
