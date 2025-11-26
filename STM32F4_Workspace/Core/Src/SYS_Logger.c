/*******************************************************************************
 * @file    SYS_Logger.c
 * @author  Amr Mohamed
 * @brief   Source file for the system logger module
 *
 * @details
 * Implements system logging functionality including error logging, debug output,
 * event tracking, and system monitoring capabilities. Provides structured
 * logging with timestamps, severity levels, and module identification.
 *
 * @version 1.0
 * @date    2025-10-17
 ******************************************************************************/

/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/
#include "SYS_Logger.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "LED_Controller.h"
#include "ESP8266_Handler.h"
/******************************************************************************
 *                             Module Config                                  *
 ******************************************************************************/

/******************************************************************************
 *                            Private Macros                                  *
 ******************************************************************************/

/******************************************************************************
 *                         Private Data Types                                 *
 ******************************************************************************/
typedef struct {
    Error_code_t error_code;
    const char* error_string;
} Error_String_Map_t;

 /******************************************************************************
 *                          Private Variables                                 *
 ******************************************************************************/
static Error_Logger_t Error_Logger = {.head = 0, .count = 0, .total_errors = 0};
static bool logger_initialized = false;
Critical_Error_logger_t Critical_Error_Logger[MAX_ERROR_CODE_NUMBER] = {
    {MDB_ERROR_UART_NOT_CONFIGURED, 0, 3},
    {MDB_ERROR_SUB_COMMAND_NOT_RECOGNIZED, 0, 3},
    {MDB_ERROR_INVALID_RX_STATE, 0, 3},
    {MDB_ERROR_INVALID_PROCESS_STATE, 0, 3},
    {MDB_ERROR_COMMAND_NOT_VALID_IN_STATE, 0, 3},
    {MDB_ERROR_INVALID_STATE_FOR_RESET, 0, 3},
    {MDB_ERROR_UNEXPECTED_ACK_WORD, 0, 3},
    {MDB_ERROR_BALANCE_EXCEEDS_MAXIMUM, 0, 3},
    {SYSTASK_ERROR_TASK_CREATION_FAILED, 0, 1},
    {SYSTASK_ERROR_QUEUE_CREATION_FAILED, 0, 1},
    {ESP_WIFI_ERROR_CONNECTION_FAILED, 0, 1},
    {ESP_TCP_ERROR_CONNECTION_FAILED, 0, 1},
    {ESP_MQTT_ERROR_CONNECTION_FAILED, 0, 1},
    {ESP_SERVER_ERROR_LOSE_CONNECTION, 0, 4}
};
/******************************************************************************
 *                         Private Prototypes                                 *
 ******************************************************************************/
static void SYS_WriteErrorToLog(Error_code_t error_code, uint16_t error_data, uint8_t command_context);
static uint32_t SYS_GetTimestamp(void);
static uint8_t SYS_GetCurrentState(void);
static void SYS_HandleCriticalErrorAction(Error_code_t error_code);
static HAL_StatusTypeDef SYS_TransmitUART(const char* message);

/******************************************************************************
 *                           Public Variables                                 *
 ******************************************************************************/
extern UART_HandleTypeDef huart6;  /* Debug output UART handle */
Error_Logger_Config_t Logger_Config = {
    .Logger_uart = &huart6
};
/******************************************************************************
 *                          Public Functions                                  *
 ******************************************************************************/

/**
 * @brief Initialize the system logger
 * 
 * @details Initializes the error logging system, clears any previous log entries,
 *          and sets up the necessary data structures for error tracking. This
 *          function should be called during system initialization before any
 *          error logging operations are performed.
 *
 * @note This function is thread-safe and can be called multiple times without
 *       adverse effects. Subsequent calls will reset the error log.
 */
void SYS_InitLogger(void) {
    // Clear error log
    memset(&Error_Logger, 0, sizeof(Error_Logger_t));
    Error_Logger.head = 0;
    Error_Logger.count = 0;
    Error_Logger.total_errors = 0;
    
    // Mark logger as initialized
    logger_initialized = true;
}

/**
 * @brief Log a standard error with context information
 * 
 * @details Records an error in the system error log with timestamp, error code,
 *          additional data, and context information. The error is stored in a
 *          circular buffer for later retrieval and analysis. This function
 *          provides comprehensive error tracking for debugging and system monitoring.
 *
 * @param error_code     Error code identifying the type of error
 * @param error_data     Additional error-specific data (optional)
 * @param command_context Command being processed when error occurred (optional)
 *
 * @note The function automatically captures timestamp and current system state.
 *       If the error log buffer is full, the oldest entry will be overwritten.
 *
 * @warning Ensure SYS_InitLogger() has been called before using this function.
 */
void SYS_LogError(Error_code_t error_code, uint16_t error_data, uint8_t command_context) {
    if (!logger_initialized) {
        return; // Logger not initialized, silently return
    }
    
    // Write error to log
    SYS_WriteErrorToLog(error_code, error_data, command_context);
}

/**
 * @brief Log a critical error that requires immediate attention
 * 
 * @details Records a critical error and triggers additional error handling actions
 *          such as system state reset or safety measures. Critical errors are
 *          tracked separately and may trigger system recovery procedures if the
 *          critical error threshold is exceeded.
 *
 * @param error_code     Critical error code
 * @param error_data     Additional error-specific data
 * @param command_context Command being processed when error occurred
 *
 * @note Critical errors increment a separate counter and may trigger system
 *       recovery actions. The system may reset or enter safe mode if too many
 *       critical errors occur within a short time period.
 *
 * @warning This function may trigger system state changes for error recovery.
 */
void SYS_LogCriticalError(Error_code_t error_code, uint16_t error_data, uint8_t command_context) {
    if (!logger_initialized) {
        return;
    }
    
    // Log the error normally
    SYS_WriteErrorToLog(error_code, error_data, command_context);
    
    // increment the error count for this critical error
    Critical_Error_Logger[error_code].error_count++;
    
    
    // Check if critical error threshold exceeded
    if (Critical_Error_Logger[error_code].error_count >= Critical_Error_Logger[error_code].error_threshold) {
        // Handle critical error actions
        SYS_HandleCriticalErrorAction(error_code);
    }
}


/**
 * @brief Get error information by index
 * 
 * @details Retrieves error information from the log buffer by index. Index 0
 *          represents the oldest error still in the buffer, while higher indices
 *          represent more recent errors. This function enables sequential access
 *          to the error log for analysis and reporting.
 *
 * @param index      Index of error to retrieve (0 = oldest, count-1 = newest)
 * @param error_info Pointer to structure to receive error information
 * @return true if error retrieved successfully, false if invalid index or pointer
 *
 * @note The index is relative to the current buffer contents. If the buffer
 *       has wrapped around, index 0 may not be the first error ever logged.
 */
bool SYS_GetErrorByIndex(uint8_t index, Error_Info_t *error_info) {
    if (!logger_initialized || error_info == NULL || index >= Error_Logger.count) {
        return false;
    }
    
    // Calculate actual buffer index
    uint8_t buffer_index;
    if (Error_Logger.count < ERROR_LOGS_BUFFER_SIZE) {
        buffer_index = index;
    } else {
        buffer_index = (Error_Logger.head + index) % ERROR_LOGS_BUFFER_SIZE;
    }
    
    *error_info = Error_Logger.error_logs[buffer_index];
    return true;
}

/**
 * @brief Clear the error log buffer
 * 
 * @details Clears all error entries from the log buffer and resets error
 *          counters. This function can be used to start fresh error logging
 *          after resolving system issues or for periodic log maintenance.
 *
 * @note This function preserves the logger initialization state but resets
 *       all error data including the critical error count.
 */
void SYS_ClearErrorLog(void) {
    if (!logger_initialized) {
        return;
    }
    
    memset(&Error_Logger.error_logs, 0, sizeof(Error_Logger.error_logs));
    Error_Logger.head = 0;
    Error_Logger.count = 0;
    Error_Logger.total_errors = 0;
}


/**
 * @brief Log formatted information message
 * 
 * @details Outputs a formatted information message using printf-style formatting.
 *          This function provides flexible message formatting for system status
 *          reporting and general information logging.
 *
 * @param format Printf-style format string
 * @param ...    Variable arguments for format string
 *
 * @note This function uses variable arguments and requires adequate stack space
 *       for the formatted message buffer.
 */
void SYS_LogInfo(const char* format, ...) {
    if (format != NULL && Logger_Config.Logger_uart != NULL) {
        char buffer[DEBUG_MESSAGE_MAX_LENGTH];
        va_list args;
        
        // Add INFO prefix first
        int prefix_len = snprintf(buffer, sizeof(buffer), "[INFO] ");
        
        // Append formatted message after prefix
        va_start(args, format);
        vsnprintf(buffer + prefix_len, sizeof(buffer) - prefix_len, format, args);
        va_end(args);
        
        // Append newline
        int len = strlen(buffer);
        if (len < sizeof(buffer) - 2) {
            buffer[len] = '\r';
            buffer[len + 1] = '\n';
            buffer[len + 2] = '\0';
        }
        
        // Transmit via UART
        SYS_TransmitUART(buffer);
    }
}

/******************************************************************************
 *                          Private Functions                                 *
 ******************************************************************************/

/**
 * @brief Transmit message via UART
 * 
 * @param message Null-terminated string to transmit
 * @return HAL_StatusTypeDef HAL transmission status
 */
static HAL_StatusTypeDef SYS_TransmitUART(const char* message) {
    if (message == NULL || Logger_Config.Logger_uart == NULL) {
        return HAL_ERROR;
    }
    
    uint16_t message_length = strlen(message);
    return HAL_UART_Transmit_IT(Logger_Config.Logger_uart, 
                            (uint8_t*)message, 
                            message_length);
}

/**
 * @brief Internal function to write error to log buffer
 * 
 * @param error_code Error code to log
 * @param error_data Additional error data
 * @param command_context Command context
 */
static void SYS_WriteErrorToLog(Error_code_t error_code, uint16_t error_data, uint8_t command_context) {
    // Get current error entry
    Error_Info_t *error_entry = &Error_Logger.error_logs[Error_Logger.head];
    
    // Fill error information
    error_entry->error_code = error_code;
    error_entry->error_data = error_data;
    error_entry->timestamp = SYS_GetTimestamp();
    error_entry->state_context = SYS_GetCurrentState();
    error_entry->command_context = command_context;
    
    // Update circular buffer pointers
    Error_Logger.head = (Error_Logger.head + 1) % ERROR_LOGS_BUFFER_SIZE;
    
    // Update count (max is buffer size)
    if (Error_Logger.count < ERROR_LOGS_BUFFER_SIZE) {
        Error_Logger.count++;
    }
    
    // Always increment total errors
    Error_Logger.total_errors++;
}

/**
 * @brief Get current timestamp for error logging
 * 
 * @return Current timestamp in milliseconds
 */
static uint32_t SYS_GetTimestamp(void) {
    #if ERROR_LOGGER_TIMESTAMP_ENABLED
    return HAL_GetTick(); // Use HAL tick counter
    #else
    return 0;
    #endif
}

/**
 * @brief Get current system state for context
 * 
 * @return Current system state value
 */
static uint8_t SYS_GetCurrentState(void) {
    // This would typically access the MDB state manager
    return (uint8_t)MDB_StateManager.Cashless_StateHandler;
}

/**
 * @brief Handle actions for critical errors
 * 
 * @param error_code Critical error code that triggered the action
 */
static void SYS_HandleCriticalErrorAction(Error_code_t error_code) {
    // Handle specific critical error actions
    switch (error_code) {
        case SYSTASK_ERROR_TASK_CREATION_FAILED:
        case SYSTASK_ERROR_QUEUE_CREATION_FAILED:
            while(1){ // Halt system
                LED_Toggle(LED_CHANNEL_DIAG);
                HAL_Delay(500);
            }
            break;
        case ESP_WIFI_ERROR_CONNECTION_FAILED:
            // Attempt to reset WiFi module or reinitialize
            if (esp_setup_wifi_connection() != HAL_OK) {
                while(1){ // Halt if still failing
                    LED_Toggle(LED_CHANNEL_DIAG);
                    HAL_Delay(500);
                }
            }
            break;
        case ESP_TCP_ERROR_CONNECTION_FAILED:
            // Attempt to reset TCP connection
            if (esp_setup_tcp_connection() != HAL_OK) {
                while(1){ // Halt if still failing
                    LED_Toggle(LED_CHANNEL_DIAG);
                    HAL_Delay(500);
                }
            }
            break;
        case ESP_MQTT_ERROR_CONNECTION_FAILED:
            // Attempt to resend MQTT connect packet
            if (esp_send_mqtt_connect_packet() != HAL_OK) {
                while(1){ // Halt if still failing
                    LED_Toggle(LED_CHANNEL_DIAG);
                    HAL_Delay(500);
                }
            }
            break;
        case ESP_SERVER_ERROR_LOSE_CONNECTION:
            // Attempt to reconnect to MQTT server
            ESP_Init();
            if (esp_current_status != ESP_STATUS_MQTT_CONNECTED) {
                while(1){ // Halt if still failing
                    LED_Toggle(LED_CHANNEL_DIAG);
                    HAL_Delay(500);
                }
            }
            break;
        default:
            // General critical error handling
            break;
    }
    
    // Additional critical error actions could be added here:
    // - LED indicators
    // - System state changes
    // - Emergency shutdowns
    // - Watchdog resets
}
