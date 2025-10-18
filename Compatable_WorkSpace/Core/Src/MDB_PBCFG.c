/*******************************************************************************
 * @file    MDB_PBCFG.c
 * @author  Amr Mohamed
 * @brief   Post-Build Configuration for MDB module
 *
 * @details
 * This file contains the post-build configuration variables for the MDB
 * (Multi-Drop Bus) protocol handler, including default settings for
 * UART assignments and timeout configurations.
 *
 * @version 1.0
 * @date    2025-09-19
 ******************************************************************************/

/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/
#include "MDB_Handler.h"
#include "stm32f1xx_hal.h"
#include "system_tasks.h"

/******************************************************************************
 *                          External UART Handles                             *
 ******************************************************************************/
/* These UART handles should be defined in main.c or respective UART init files */
extern UART_HandleTypeDef huart1;
/******************************************************************************
 *                        MDB Configuration Variable                          *
 ******************************************************************************/

/**
 * @brief Default MDB configuration structure
 * 
 * @details This configuration variable contains the default settings for the
 *          MDB module including UART handle assignments and timeout settings.
 *          This variable can be modified at runtime or used as-is for default
 *          operation.
 *
 * @note UART Configuration:
 *       - MDB Bus Communication: UART1 (huart1)
 *       - Debug Output: UART2 (huart2)
 *
 * @note Task Configuration:
 *       - MDB Task Handle: Set to mdbRxTaskHandle from system_tasks
 *
 * @note Timeout Configuration:
 *       - Bus Timeout: 10ms (default MDB specification)
 *
 * @note Debug Configuration:
 *       - Debug Output: Enabled by default
 */
MDB_Config_t mdb_config = {
    .mdb_uart = &huart1,                /* MDB bus communication UART */
    .bus_timeout = MDB_BUS_TIMEOUT,     /* MDB bus timeout in milliseconds */
    .debug_enabled = false              /* Enable debug output by default */
};
