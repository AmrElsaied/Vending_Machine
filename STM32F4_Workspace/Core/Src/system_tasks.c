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
#include "queue.h"
#include "ESP8266_Handler.h"
#include <string.h>
#include "SYS_Logger.h"
/******************************************************************************
 *                             Module Config                                  *
 ******************************************************************************/
#define ESP_PUBLISH_QUEUE_SIZE          (10)        /* Max pending publish requests */

/******************************************************************************
 *                            Private Macros                                  *
 ******************************************************************************/

/******************************************************************************
 *                         Private Data Types                                 *
 ******************************************************************************/

/******************************************************************************
 *                          Private Variables                                 *
 ******************************************************************************/
static QueueHandle_t espPublishQueue = NULL;       /* Queue for publish requests */

/******************************************************************************
 *                           Public Variables                                 *
 ******************************************************************************/
TaskHandle_t mdbRxTaskHandle = NULL;                /* Handle for MDB receive task */
TaskHandle_t mdbCMDProcessTaskHandle = NULL;        /* Handle for MDB command processing task */
TaskHandle_t espCommunicationTaskHandle = NULL;           /* Handle for ESP publish task */
/******************************************************************************
 *                      Private Function Prototypes                           *
 ******************************************************************************/
static void mdbRxTask(void *argument);
static void mdbCMDProcessTask(void *argument);
static void espCommunicationTask(void *argument);
static uint8_t safe_string_copy(char* dest, const char* src, uint8_t dest_size);
static uint8_t safe_strlen(const char* str, uint8_t max_len);
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
    /* Create ESP publish request queue */
    espPublishQueue = xQueueCreate(ESP_PUBLISH_QUEUE_SIZE, sizeof(ESP_PublishRequest_t));
    if (espPublishQueue == NULL) {
        /* Queue creation failed - handle error */
        ESP_LOG_CRITICAL_ERROR(SYSTASK_ERROR_QUEUE_CREATION_FAILED, NO_DATA_PRESENT, NO_CONTEXT_PRESENT);
    }
    /* Stack size is in WORDS (not bytes) for xTaskCreate.     */

    xTaskCreate(
        mdbRxTask,                 									/* task function                   */
        "mdbRxTask",               									/* name (for trace)                */
        384,              									        /* stack size in WORDS             */
        NULL,                    									/* no pvParameters                 */
        configMAX_PRIORITIES-3,  									/* priority (just below max)       */
        &mdbRxTaskHandle);         									/* return handle                   */

    if(mdbRxTaskHandle == NULL) {
        /* Task creation failed - handle error */
        ESP_LOG_CRITICAL_ERROR(SYSTASK_ERROR_TASK_CREATION_FAILED, configMAX_PRIORITIES-3, NO_CONTEXT_PRESENT);
    }

    xTaskCreate(
        mdbCMDProcessTask,                 							/* task function                   */
        "mdbCMDProcessTask",               							/* name (for trace)                */
        384,              									    	/* stack size in WORDS             */
        NULL,                    									/* no pvParameters                 */
        configMAX_PRIORITIES-2,  									/* priority (just below max)       */
        &mdbCMDProcessTaskHandle);         							/* return handle                   */

    if(mdbCMDProcessTaskHandle == NULL) {
        /* Task creation failed - handle error */
        ESP_LOG_CRITICAL_ERROR(SYSTASK_ERROR_TASK_CREATION_FAILED, configMAX_PRIORITIES-2, NO_CONTEXT_PRESENT);
    }

    xTaskCreate(
        espCommunicationTask,
        "espCommunicationTask",
        600,                                                       /* Increased stack for combined functionality */
        NULL,
        configMAX_PRIORITIES-4,
        &espCommunicationTaskHandle);       				        /* return handle                   */
        
    if(espCommunicationTaskHandle == NULL) {
        /* Task creation failed - handle error */
        ESP_LOG_CRITICAL_ERROR(SYSTASK_ERROR_TASK_CREATION_FAILED, configMAX_PRIORITIES-4, NO_CONTEXT_PRESENT);

    }
}

/**
 * @brief Request ESP publish operation from other tasks (optimized with memcpy)
 */
bool ESP_RequestPublish(const char* topic, const char* message, uint8_t qos)
{
    if (topic == NULL || message == NULL || espPublishQueue == NULL) {
        return false;
    }
    
    ESP_PublishRequest_t request;
    
    /* Fast string copying with memcpy - much better than strncpy */
    safe_string_copy(request.topic, topic, ESP_TASK_MAX_TOPIC_LEN);
    safe_string_copy(request.message, message, ESP_TASK_MAX_MESSAGE_LEN);
    
    request.qos = qos;
    
    /* Send to queue (non-blocking) */
    return (xQueueSend(espPublishQueue, &request, 0) == pdPASS);
}
/**
 * @brief Get number of pending publish requests
 */
uint32_t ESP_GetPendingPublishCount(void)
{
    if (espPublishQueue == NULL) {
        return 0;
    }
    
    return (uint32_t)uxQueueMessagesWaiting(espPublishQueue);
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
    MDB_StartUARTReceive();   /* Start MDB UART reception */
    for (;;)
    {
        /* Wait until ISR "gives" a token (see ISR code). */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        TickType_t nowTick = xTaskGetTickCount();
        if ((nowTick - lastCallTick) > mdb_config.bus_timeout) // 10 ms interval exceeded
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
/**
 * @brief Combined ESP communication task handling both publish requests and ping
 * 
 * @details This task handles two types of operations:
 *          1. Process publish requests from other tasks via queue
 *          2. Send ping requests if no activity for 40 seconds
 *          
 *          The task waits for publish requests with a 40-second timeout.
 *          If timeout occurs (no publish activity), it sends a ping.
 *          If a publish request is received, it processes the request immediately.
 *
 * @param argument Task parameter (unused)
 *
 * @note Priority: configMAX_PRIORITIES-4 (lower than MDB tasks)
 *       Stack: 512 words to handle queue operations and ESP communication
 *       Timeout: 40 seconds - triggers ping if no publish activity
 *
 * @warning Ensure ESP_Publish and PingREQ functions are non-blocking or have
 *          appropriate timeouts to prevent blocking other operations.
 */
static void espCommunicationTask(void *argument)
{
    ESP_PublishRequest_t publishRequest;
    const TickType_t xTimeout = pdMS_TO_TICKS(ESP_PING_TIMEOUT_MS); /* 40 seconds */
    
    for (;;)
    {
        /* Wait for publish request or timeout after 40 seconds */
        if (xQueueReceive(espPublishQueue, &publishRequest, xTimeout) == pdPASS)
        {
            
            /* Call ESP publish function */
            ESP_Publish(publishRequest.topic, publishRequest.message, publishRequest.qos);
            
        }
        else
        {
            /* Timeout occurred - no publish requests for 40 seconds */
            PingREQ();
            
        }
        
        /* Optional: Yield to other tasks */
        taskYIELD();
    }
}
/**
 * @brief Calculate string length with maximum limit (optimized)
 */
static uint8_t safe_strlen(const char* str, uint8_t max_len)
{
    uint8_t len = 0;
    while (len < max_len && str[len] != '\0') {
        len++;
    }
    return len;
}
/**
 * @brief Fast and safe string copy using memcpy with length calculation
 * @param dest Destination buffer
 * @param src Source string
 * @param dest_size Maximum destination buffer size (including null terminator)
 * @return Actual length copied (excluding null terminator)
 */
static uint8_t safe_string_copy(char* dest, const char* src, uint8_t dest_size)
{
    if (dest_size == 0) return 0;
    
    // Calculate actual source length (bounded by destination size)
    uint8_t src_len = safe_strlen(src, dest_size - 1);
    
    // Copy exact number of bytes using memcpy (fast!)
    if (src_len > 0) {
        memcpy(dest, src, src_len);
    }
    
    // Ensure null termination
    dest[src_len] = '\0';
    
    return src_len;
}