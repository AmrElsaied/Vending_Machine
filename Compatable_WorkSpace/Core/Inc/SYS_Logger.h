/*******************************************************************************
 * @file    SYS_Logger.h
 * @author  Amr Mohamed
 * @brief   Header file for the system logger module
 *
 * @details
 * This file contains the declarations and definitions for the system logging
 * functionality, including error logging, debug output, and system event tracking.
 * Provides structured logging capabilities for debugging and system monitoring.
 *
 * @version 1.0
 * @date    2025-10-17
 ******************************************************************************/

#ifndef __SYS_LOGGER_H
#define __SYS_LOGGER_H


/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "MDB_Handler.h"
/******************************************************************************
 *                             Module Config                                  *
 ******************************************************************************/
#define ERROR_LOGS_BUFFER_SIZE          32      /* Number of error entries to store */
#define ERROR_MESSAGE_MAX_LENGTH        64      /* Maximum error message length */
#define DEBUG_MESSAGE_MAX_LENGTH        128     /* Maximum debug message length */

/******************************************************************************
 *                            Private Macros                                  *
 ******************************************************************************/
#define MDB_LOGGER_ERROR_ENABLED            1       /* Enable/disable error logging */
#define ESP_LOGGER_ERROR_ENABLED            1       /* Enable/disable error logging */
#define MDB_LOGGER_DEBUG_ENABLED            0       /* Enable/disable debug output */
#define ESP_LOGGER_DEBUG_ENABLED            0       /* Enable/disable debug output */
#define ERROR_LOGGER_TIMESTAMP_ENABLED      1       /* Enable/disable timestamp logging */

/******************************************************************************
 *                         Private Data Types                                 *
 ******************************************************************************/
typedef enum {
    MDB_ERROR_NONE = 0x00,                    /* No error - successful operation */
    // Communication Errors (0x10-0x1F)
    MDB_ERROR_UART_NOT_CONFIGURED = 0x10,     /* UART handle not configured in mdb_config */    
    // Buffer Management Errors (0x20-0x2F)
    // Command Processing Errors (0x30-0x3F)
    MDB_ERROR_SUB_COMMAND_NOT_RECOGNIZED = 0x30,  /* Subcommand not recognized in 0x0076/0x0077 */
    MDB_ERROR_INVALID_COMMAND_STRUCTURE = 0x31, /* Command structure validation failed */
    MDB_ERROR_COMMAND_LENGTH_MISMATCH = 0x32, /* Received command length doesn't match expected */
    MDB_ERROR_INCOMPLETE_COMMAND = 0x33,      /* Command reception incomplete */

    // State Management Errors (0x40-0x4F)
    MDB_ERROR_INVALID_RX_STATE = 0x40,        /* Invalid CMD_RX_StateHandler value */
    MDB_ERROR_INVALID_PROCESS_STATE = 0x41,   /* Invalid CMD_Process_StateHandler value */
    MDB_ERROR_COMMAND_NOT_VALID_IN_STATE = 0x42, /* Command not valid in current state */
    MDB_ERROR_INVALID_STATE_FOR_RESET = 0x43,   /* RESET command received in invalid state */

    // ACKnowledgment Errors (0x50-0x5F)
    MDB_ERROR_UNEXPECTED_ACK_WORD = 0x50,     /* Unexpected word received while waiting for ACK */
    MDB_ERROR_ACK_TIMEOUT = 0x51,             /* ACK not received within timeout */

    // Balance and Transaction Errors (0x60-0x6F)
    MDB_ERROR_BALANCE_EXCEEDS_MAXIMUM = 0x60, /* Balance exceeds Max_balance limit */

    // Vending Operation Errors (0x70-0x7F)
    MDB_ERROR_VENDING_OPERATION_FAILED = 0x70, /* Vending operation failed */

    // Configuration and Initialization Errors (0x80-0x8F)
    MDB_ERROR_INITIALIZATION_FAILED = 0x80,   /* MDB system initialization failed */
    MDB_ERROR_CONFIG_INVALID = 0x81,          /* Invalid configuration parameters */

    // System and Resource Errors (0x90-0x9F)
    MDB_ERROR_RESOURCE_BUSY = 0x90,           /* Resource currently busy */

    // Protocol Specific Errors (0xA0-0xAF)

    // Hardware Errors (0xB0-0xBF)

    // Generic Errors (0xC0-0xFF)

 }Error_code_t;
/******************************************************************************
 *                          Private Variables                                 *
 ******************************************************************************/

/******************************************************************************
 *                         Private Prototypes                                 *
 ******************************************************************************/

/******************************************************************************
 *                           Public Variables                                 *
 ******************************************************************************/

/******************************************************************************
 *                         Public Data Types                                  *
 ******************************************************************************/
typedef struct {
    Error_code_t error_code;        /* Error code */
    uint8_t state_context;          /* State when error occurred */
    uint8_t command_context;        /* Command being processed when error occurred */
    uint16_t error_data;            /* Additional error data */
    uint32_t timestamp;             /* Timestamp when error occurred */
} Error_Info_t;

typedef struct {
    Error_Info_t error_logs[ERROR_LOGS_BUFFER_SIZE];
    uint8_t head;                  /* Write pointer */
    uint8_t count;                 /* Number of errors logged */
    uint32_t total_errors;         /* Total errors since startup */
} Error_Logger_t;

typedef struct{
    UART_HandleTypeDef *Logger_uart;        /* Debug output UART handle pointer */
}Error_Logger_Config_t;
/******************************************************************************
 *                      Public Function Prototypes                           *
 ******************************************************************************/
/* Error Logging Functions */
void SYS_LogError(Error_code_t error_code, uint16_t error_data, uint8_t command_context);
void SYS_LogCriticalError(Error_code_t error_code, uint16_t error_data, uint8_t command_context);
bool SYS_GetErrorByIndex(uint8_t index, Error_Info_t *error_info);
void SYS_ClearErrorLog(void);

/* Debug and Information Functions */
void SYS_LogInfo(const char* format, ...);
const char* SYS_GetErrorString(Error_code_t error_code);

/* Logger System Functions */
void SYS_InitLogger(void);
void SYS_PrintErrorReport(void);

/******************************************************************************
 *                            Public Macros                                   *
 ******************************************************************************/
/* Conditional logging macros */
#if MDB_LOGGER_ERROR_ENABLED
    #define MDB_LOG_ERROR(code, data, context)          SYS_LogError(code, data, context)
    #define MDB_LOG_CRITICAL_ERROR(code, data, context) SYS_LogCriticalError(code, data, context)
#else
    #define MDB_LOG_ERROR(code, data, context)          do { } while(0)
    #define MDB_LOG_CRITICAL_ERROR(code, data, context) do { } while(0)
#endif

#if ESP_LOGGER_ERROR_ENABLED
    #define ESP_LOG_ERROR(code, data, context)          SYS_LogError(code, data, context)
    #define ESP_LOG_CRITICAL_ERROR(code, data, context) SYS_LogCriticalError(code, data, context)
#else
    #define ESP_LOG_ERROR(code, data, context)          do { } while(0)
    #define ESP_LOG_CRITICAL_ERROR(code, data, context) do { } while(0)
#endif

#if MDB_LOGGER_DEBUG_ENABLED
    #define MDB_PRINT_INFO(format, ...)                   SYS_LogInfo(format, ##__VA_ARGS__)
#else
    #define MDB_PRINT_INFO(format, ...)                   do { } while(0)
#endif
#if ESP_LOGGER_DEBUG_ENABLED
    #define ESP_PRINT_INFO(format, ...)                   SYS_LogInfo(format, ##__VA_ARGS__)
#else
    #define ESP_PRINT_INFO(format, ...)                   do { } while(0)
#endif

#define NO_DATA_PRESENT           (uint16_t)0
#define NO_CONTEXT_PRESENT        (uint8_t)0
#endif /* __SYS_LOGGER_H */