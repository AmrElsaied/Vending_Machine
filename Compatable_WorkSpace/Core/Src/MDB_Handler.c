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

/******************************************************************************
 *                         Private Prototypes                                 *
 ******************************************************************************/
static void mdbRxTask(void *argument);
static void mdbCMDProcessTask(void *argument);
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
    const uint16_t stackWords = 512;                  				/* 512 × 4 B = 2 kB */

    xTaskCreate(
    	mdbRxTask,                 									/* task function                   */
        "mdbRxTask",               									/* name (for trace)                */
        stackWords,              									/* stack size in WORDS             */
        NULL,                    									/* no pvParameters                 */
        configMAX_PRIORITIES-3,  									/* priority (just below max)       */
        &mdbRxTaskHandle);         									/* return handle                   */

    xTaskCreate(
		mdbCMDProcessTask,                 									/* task function                   */
        "mdbCMDProcessTask",               									/* name (for trace)                */
        stackWords,              									/* stack size in WORDS             */
        NULL,                    									/* no pvParameters                 */
        configMAX_PRIORITIES-2,  									/* priority (just below max)       */
        &mdbCMDProcessTaskHandle);         									/* return handle                   */

}

/******************************************************************************
 *                          Private Functions                                 *
 ******************************************************************************/

/*************************** Private Functions *******************************/

static void mdbRxTask(void *argument)
{
	mdbRing_init(&rxRing);
    HAL_UART_Receive_IT(&huart1, (uint8_t*)mdb_rx_buf, 1);

    for (;;)
    {
        /* Wait until ISR “gives” a token (see ISR code). */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint16_t word;
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

                uint8_t CMD_expectedLength =
                    VMC_CMDs[MDB_BusManager.MDB_RX_CMD_Index].CMD_Length;

                if (MDB_BusManager.RXBuffer_index >= CMD_expectedLength)
                {
                    MDB_BusManager.RXBuffer_index = 0;
                    MDB_StateManager.CMD_RX_StateHandler = CMD_RX_READY;
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
        xTaskNotifyWait(0,                		  /* don’t clear bits          */
                        0,                		  /* don’t clear bits          */
                        &CMD_Index,             /* returns the cmd index     */
                        portMAX_DELAY);

        /* Handle & respond */
        MDB_HandleCommand(VMC_CMDs[CMD_Index].CMD,
                          VMC_CMDs[CMD_Index].CMD_Length);

        uint16_t *resp = VMC_CMDs[CMD_Index].CMD_Response;
        uint8_t   len  = VMC_CMDs[CMD_Index].CMD_Response_Length;

        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)resp, len * 2);
        /* notification value auto‑overwritten next time; nothing to clear */
    }
}
