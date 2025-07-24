/*******************************************************************************
 * @file    MDB_Handler.c
 * @author  Amr Mohamed
 * @brief   Source file for the MDB handler module
 *
 * @details
 * Implements functionality for sending and receiving data over the MDB bus,
 * managing frame parsing, state control, and communication protocol handling.
 *
 * @version 1.0
 * @date    2025-07-06
 ******************************************************************************/

/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/
#include "MDB_Handler.h"
#include "main.h"
#include "VMC_Config.h"
/* FreeRTOS core */
#include "FreeRTOS.h"
#include "task.h"
/******************************************************************************
 *                             Module Config                                  *
 ******************************************************************************/

/******************************************************************************
 *                            Private Macros                                  *
 ******************************************************************************/

/******************************************************************************
 *                         Private Data Types                                 *
 ******************************************************************************/

/******************************************************************************
 *                          Private Variables                                 *
 ******************************************************************************/

/******************************************************************************
 *                           Public Variables                                 *
 ******************************************************************************/
mdb_ring_t rxRing;                   /* our ring instance */
uint16_t mdb_rx_buf[1];
TaskHandle_t mdbRxTaskHandle = NULL;
TaskHandle_t mdbCMDProcessTaskHandle = NULL;

MDB_StateManager_t MDB_StateManager = {
    .BV_StateHnadler = STATE_RESTART,               /* Initialize the state to RESTART */
    .CMD_RX_StateHandler = CMD_RX_READY,            /* Initialize the command reception state to READY */
    .CMD_TX_StateHandler = CMD_TX_READY,             /* Initialize the command transmission state to BUSY */
    .CMD_Process_StateHandler = CMD_PROCESS_READY    /* Initialize the command processing state to BUSY */
};

MDB_BusManager_t MDB_BusManager = {
	.MDB_RXbuffer = {0},
	.RXBuffer_index = 0,
    .MDB_RX_CMD_Index = VMC_CMD_MAX_NUMBER, /* Initialize to a default value */
    .MDB_TX_CMD_Index = VMC_CMD_MAX_NUMBER, /* Initialize to a default value */
    .MDB_Process_CMD_Index = VMC_CMD_MAX_NUMBER /* Initialize to a default value */
};

bool Vending_EN = false;
/******************************************************************************
 *                         Private Prototypes                                 *
 ******************************************************************************/
static void mdbRxTask(void *argument);
static void mdbCMDProcessTask(void *argument);
static void MDB_HandleCommand(uint16_t *RxBuffer, uint8_t length);
static void MDB_SendResponseWithModeBit(uint16_t *data, uint8_t dataLength);
/******************************************************************************
 *                          Public Functions                                  *
 ******************************************************************************/
/* ------------------------------------------------------------
 * HAL_UART_RxCpltCallback
 *
 * Called on every RXNE (UART receive complete).
 * Copies the 9-bit MDB word to the ring buffer,
 * notifies rxTask, and re-arms the interrupt.
 * ---------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
    uint16_t word = mdb_rx_buf[0]&0x1FF;

    /* ---- 2. Store in ring buffer (overflow returns false) --------- */
	(void)mdbRing_push(&rxRing, word);   /* ignore overflow for now   */

	/* ---- 3. Notify mdbTask that data is ready --------------------- */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(mdbRxTaskHandle, &xHigherPriorityTaskWoken);

	/* ---- 4. Context‑switch immediately if mdbTask has higher prio  */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	/* ---- 5. Re‑arm reception of the NEXT byte --------------------- */
    HAL_UART_Receive_IT(huart, (uint8_t *) mdb_rx_buf, 1);
  }
}

/* ----------------------------------------------------------------------
 * mdbRing_init
 * ----------------------------------------------------------------------
 * Clear the read and write indices, effectively emptying the ring.
 * Caller: mdbTask() once at startup.
 * -------------------------------------------------------------------- */
void mdbRing_init(mdb_ring_t *r)
{
    r->wr = r->rd = 0U;
}

/* ----------------------------------------------------------------------
 * mdbRing_push
 * ----------------------------------------------------------------------
 * Put one 16‑bit word into the ring.
 * • Returns true  if the word was stored.
 * • Returns false if the ring is full (caller may drop byte or count error).
 *
 * Called ONLY from the USART RX ISR (single producer).
 * -------------------------------------------------------------------- */
bool mdbRing_push(mdb_ring_t *r, uint16_t word)
{
    /* Calculate the index that WRITER would have AFTER this push.
     * Because MDB_RING_LEN is a power of two (e.g. 256 = 0x100), we can
     * wrap the counter with a bit‑mask instead of an expensive “% len”.
     *
     * Example (LEN = 256):
     *     r->wr = 255
     *     next  = (255 + 1) & 255  -> 0   (wrap around)
     */
    uint16_t next = (r->wr + 1U) & (MDB_RING_LEN - 1U);

    /* If next equals the reader index, buffer is ALMOST full
     * (one slot must stay empty to differentiate full vs empty).
     */
    if (next == r->rd)
        return false;                       /* overflow */

    r->buf[r->wr] = word;                   /* store new sample */
    r->wr = next;                           /* advance write pointer */
    return true;
}

/* ----------------------------------------------------------------------
 * mdbRing_pop
 * ----------------------------------------------------------------------
 * Retrieve one 16‑bit word from the ring.
 * • Returns true  and sets *word when data available.
 * • Returns false when ring is empty.
 *
 * Called ONLY from mdbTask() (single consumer).
 * -------------------------------------------------------------------- */
bool mdbRing_pop(mdb_ring_t *r, uint16_t *word)
{
    if (r->rd == r->wr)
        return false;                       /* empty */

    *word = r->buf[r->rd];                  /* fetch oldest entry */
    r->rd = (r->rd + 1U) & (MDB_RING_LEN - 1U);
    return true;
}

void MDB_TaskCreate(void)
{
    /* Stack size is in WORDS (not bytes) for xTaskCreate.     */

    xTaskCreate(
    	mdbRxTask,                 									/* task function                   */
        "mdbRxTask",               									/* name (for trace)                */
        384,              									/* stack size in WORDS             */
        NULL,                    									/* no pvParameters                 */
        configMAX_PRIORITIES-3,  									/* priority (just below max)       */
        &mdbRxTaskHandle);         									/* return handle                   */

    xTaskCreate(
		mdbCMDProcessTask,                 							/* task function                   */
        "mdbCMDProcessTask",               							/* name (for trace)                */
        256,              									    	/* stack size in WORDS             */
        NULL,                    									/* no pvParameters                 */
        configMAX_PRIORITIES-2,  									/* priority (just below max)       */
        &mdbCMDProcessTaskHandle);         							/* return handle                   */

}

/******************************************************************************
 *                          Private Functions                                 *
 ******************************************************************************/

/*************************** Private Functions *******************************/

static void mdbRxTask(void *argument)
{
	mdbRing_init(&rxRing);
    HAL_UART_Receive_IT(&huart1, (uint8_t*)mdb_rx_buf, 1);
    uint16_t word;
    uint8_t CMD_expectedLength =0;
    for (;;)
    {
        /* Wait until ISR “gives” a token (see ISR code). */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (mdbRing_pop(&rxRing, &word))
        {
            switch (MDB_StateManager.CMD_RX_StateHandler)
            {
            case CMD_RX_READY:
                for (uint8_t i = 0; i < VMC_CMD_MAX_NUMBER; ++i)
                {
                    if (word == VMC_CMDs[i].CMD[0])
                    {
                        MDB_StateManager.CMD_RX_StateHandler = CMD_RX_INPROGRESS;
                        MDB_BusManager.MDB_RX_CMD_Index      = i;
                        MDB_BusManager.MDB_RXbuffer[0]       = word;
                        MDB_BusManager.RXBuffer_index        = 1;
                        break;
                    }
                }
                break;

            case CMD_RX_INPROGRESS:
            {
                MDB_BusManager
                   .MDB_RXbuffer[MDB_BusManager.RXBuffer_index++] = word;

                CMD_expectedLength =
                    VMC_CMDs[MDB_BusManager.MDB_RX_CMD_Index].CMD_Length;

                if (MDB_BusManager.RXBuffer_index >= CMD_expectedLength)
                {
                    MDB_BusManager.RXBuffer_index = 0;
                    MDB_StateManager.CMD_RX_StateHandler = CMD_RX_DONE;
					/* ---- Notify mdbTask that data is ready --------------------- */
                    xTaskNotify(mdbCMDProcessTaskHandle,
											(uint32_t)MDB_BusManager.MDB_RX_CMD_Index,
											eSetValueWithOverwrite);
					/* ---- Context‑switch immediately if mdbTask has higher prio  */
					taskYIELD();
                }
                break;
            }

            default:

                break;
            }
        }
    }
}


static void mdbCMDProcessTask(void *argument)
{
    uint32_t CMD_Index;                       /* 32‑bit matches notify type   */

    for (;;)
    {
        /* Block until rxTask notifies us with an index value */
        xTaskNotifyWait(0,                		/* don’t clear bits          */
			        UINT32_MAX,                	/* clear all bits          */
                    &CMD_Index,             	/* returns the cmd index     */
                    portMAX_DELAY);

        /* Handle & respond */
        MDB_HandleCommand(MDB_BusManager.MDB_RXbuffer,
                          CMD_Index);
        /* notification value auto‑overwritten next time; nothing to clear */
    }
}

static void MDB_HandleCommand(uint16_t *RxBuffer, uint8_t cmd_index)
{
 switch (MDB_StateManager.CMD_Process_StateHandler)
 {
   case CMD_PROCESS_READY:
     MDB_StateManager.CMD_Process_StateHandler = CMD_PROCESS_INPROGRESS; // Set the command processing state to INPROGRESS
     int temp_length = VMC_CMDs[cmd_index].CMD_Length;
     if (MDB_StateManager.CMD_RX_StateHandler == CMD_RX_DONE)
     {
        MDB_StateManager.CMD_RX_StateHandler = CMD_RX_BUSY; // Set the state to BUSY
        // Command reception is done, process the command
        if (RxBuffer[temp_length-1] == VMC_CMDs[cmd_index].CMD[temp_length-1])
        {
            // Command is valid, process it
            MDB_BusManager.MDB_Process_CMD_Index = cmd_index; // Set the command index to the command being processed
            if (VMC_CMDs[cmd_index].CMD[0] == VMC_CMDs[VMC_CMD_0x0066].CMD[0])
            {
                switch (MDB_StateManager.BV_StateHnadler)
                {
                case STATE_RESTART:
                    VMC_CMDs[VMC_CMD_0x0066].CMD_Response[0] = 0x0006;
                    VMC_CMDs[VMC_CMD_0x0066].CMD_Response[1] = 0x0106;
                    VMC_CMDs[VMC_CMD_0x0066].CMD_Response_Length = 2;
                    MDB_StateManager.BV_StateHnadler = STATE_DISABLED; // Set the system state to disabled
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
                MDB_StateManager.BV_StateHnadler = STATE_ERROR; // Set the system state to error
                break;
                }
            }
            else if (VMC_CMDs[cmd_index].CMD[0] == VMC_CMDs[VMC_CMD_0x009D].CMD[0])
            {
                MDB_StateManager.BV_StateHnadler = STATE_READY; // Set the system state to ready
            }
        // Send the response
        #if ENABLE_BV_TX == 1
        MDB_SendResponseWithModeBit(VMC_CMDs[MDB_BusManager.MDB_Process_CMD_Index].CMD_Response,
                                    VMC_CMDs[MDB_BusManager.MDB_Process_CMD_Index].CMD_Response_Length);
        #endif
        }
        else
        {
            // Error: Received command does not match expected command
            // TODO Handle error appropriately
        }
       // Reset the RX for a new command
       MDB_StateManager.CMD_RX_StateHandler = CMD_RX_READY; // Set the state to READY for the next command
       MDB_StateManager.CMD_Process_StateHandler = CMD_PROCESS_READY; // Set the command processing state to DONE
     }
     else
     {
       // No CMD to be processed
       // This means we are still waiting for the command to be fully received
       return;
     }
     break;
   case CMD_PROCESS_INPROGRESS:
     // TODO Handle error appropriately
     return;
   case CMD_PROCESS_DONE:
     // TODO Handle error appropriately
     break;
   default:
     // Error: Command processing state is not ready or in progress
     // TODO Handle error appropriately
     return;
 }
}

static void MDB_SendResponseWithModeBit(uint16_t *data, uint8_t dataLength)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)data, dataLength);
}
