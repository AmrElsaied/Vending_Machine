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
TaskHandle_t mdbRxTaskHandle = NULL;
TaskHandle_t mdbCMDProcessTaskHandle = NULL;
extern MDB_BusManager_t MDB_BusManager;
/******************************************************************************
 *                      Private Function Prototypes                           *
 ******************************************************************************/
static void mdbRxTask(void *argument);
static void mdbCMDProcessTask(void *argument);
/******************************************************************************
 *                      Public Function Definitions                           *
 ******************************************************************************/

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
static void mdbRxTask(void *argument)
{
    uint16_t word;
    for (;;)
    {
        /* Wait until ISR “gives” a token (see ISR code). */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (mdbRing_pop(&rxRing, &word))
        {
            MDB_ReceiveCommand(word);
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