/*******************************************************************************
 * @file    system_tasks.c
 * @author  Amr Mohamed
 * @brief   Source file for the system tasks module
 *
 * @details
 * Implements task management and scheduling functionality for the
 * vending machine control system, handling system-level operations.
 *
 * @version 1.0
 * @date    2025-08-08
 ******************************************************************************/

/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/
#include "system_tasks.h"
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
TaskHandle_t mdbRxTaskHandle = NULL;            /* Handle for MDB receive task */
TaskHandle_t mdbCMDProcessTaskHandle = NULL;    /* Handle for MDB command processing task */
/******************************************************************************
 *                      Private Function Prototypes                           *
 ******************************************************************************/
static void mdbRxTask(void *argument);
static void mdbCMDProcessTask(void *argument);
/******************************************************************************
 *                      Public Function Definitions                           *
 ******************************************************************************/

/**
 * @brief Creates all system tasks for the vending machine operation
 * 
 * See system_tasks.h for detailed documentation
 */
void System_TaskCreate(void)
{
    /* Stack size is in WORDS (not bytes) for xTaskCreate.     */

    xTaskCreate(
        mdbRxTask,                 									/* task function                   */
        "mdbRxTask",               									/* name (for trace)                */
        384,              									        /* stack size in WORDS             */
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
 *                      Private Function Definitions                          *
 ******************************************************************************/
/**
 * @brief MDB receive task that processes incoming MDB communication data
 * 
 * @details This task is responsible for handling all incoming MDB protocol 
 *          communications. It waits for notifications from the UART ISR,
 *          checks for timeout conditions, and processes received data words.
 *          The task implements timeout detection to reset the communication
 *          state if no messages are received within the MDB_BUS_TIMEOUT period.
 *
 * @param argument Task parameter (unused in this implementation)
 *
 * @note This task has a high priority (configMAX_PRIORITIES-3) and uses 384 words
 *       of stack space. It operates by waiting for notifications from the UART
 *       interrupt service routine, then reading and processing data from the 
 *       MDB ring buffer.
 *
 * @warning This task must not be blocked for extended periods as it would
 *          cause MDB communication failures. The timeout detection mechanism
 *          helps recover from communication errors by resetting the state machine
 *          when messages are interrupted.
 */
static void mdbRxTask(void *argument)
{
    uint16_t word;
    TickType_t lastCallTick = xTaskGetTickCount();
    for (;;)
    {
        /* Wait until ISR "gives" a token (see ISR code). */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        TickType_t nowTick = xTaskGetTickCount();
        if ((nowTick - lastCallTick) > MDB_BUS_TIMEOUT) // 10 ms interval exceeded
        {
            MDB_BusManager.RXBuffer_index = 0;
            MDB_StateManager.CMD_RX_StateHandler = CMD_RX_READY;           
            MDB_StateManager.CMD_Process_StateHandler = CMD_PROCESS_READY; 
        }
        lastCallTick = nowTick;

        while (mdbRing_pop(&rxRing, &word))
        {
            MDB_ReceiveCommand(word);
        }
    }
}

/**
 * @brief MDB command processing task that handles received commands
 * 
 * @details This task is responsible for processing MDB commands once they have
 *          been fully received by the mdbRxTask. It waits for notifications
 *          containing the command index, then calls MDB_HandleCommand to process
 *          the command and generate appropriate responses. This separation of
 *          reception and processing allows for better task prioritization and
 *          prevents blocking the reception of new commands during processing.
 *
 * @param argument Task parameter (unused in this implementation)
 *
 * @note This task has a higher priority than the mdbRxTask (configMAX_PRIORITIES-2)
 *       but uses less stack space (256 words). The higher priority ensures command
 *       processing takes precedence once a complete command has been received.
 *       The task blocks indefinitely until notified by the mdbRxTask with a
 *       command index.
 *
 * @warning Command processing must be efficient to avoid missing subsequent
 *          commands. The MDB_HandleCommand function should not block or perform
 *          excessively long operations.
 */
static void mdbCMDProcessTask(void *argument)
{
    uint32_t CMD_Index;                       /* 32‑bit matches notify type   */

    for (;;)
    {
        /* Block until rxTask notifies us with an index value */
        xTaskNotifyWait(0,                		/* don't clear bits          */
                    UINT32_MAX,                	/* clear all bits          */
                    &CMD_Index,             	/* returns the cmd index     */
                    portMAX_DELAY);

        /* Handle & respond */
        MDB_HandleCommand(MDB_BusManager.MDB_RXbuffer,
                          CMD_Index);
        /* notification value auto‑overwritten next time; nothing to clear */
    }

}