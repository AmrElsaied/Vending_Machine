/*******************************************************************************
 * @file    system_tasks.h
 * @author  Amr Mohamed
 * @brief   Header file for the system tasks module
 *
 * @details
 * This module provides task management and scheduling functionality for the
 * vending machine control system, handling system-level operations.
 *
 * @version 1.0
 * @date    2025-08-08
 ******************************************************************************/

#ifndef SYSTEM_TASKS_H_
#define SYSTEM_TASKS_H_

/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "MDB_Handler.h"
/******************************************************************************
 *                             Module Config                                  *
 ******************************************************************************/

/******************************************************************************
 *                          Macros & Constants                                *
 ******************************************************************************/

/******************************************************************************
 *                          Type Definitions                                  *
 ******************************************************************************/

/******************************************************************************
 *                     Global Variables (extern)                              *
 ******************************************************************************/
extern TaskHandle_t mdbRxTaskHandle ;           /* Handle for MDB receive task */ 
extern TaskHandle_t mdbCMDProcessTaskHandle ;   /* Handle for MDB command processing task */
extern mdb_ring_t rxRing;                       /* Ring buffer for MDB received data */
/******************************************************************************
 *                        Function Declarations                               *
 ******************************************************************************/

/**
 * @brief Creates and initializes all system tasks
 * 
 * @details This function creates the FreeRTOS tasks required for the vending machine 
 *          system operation. It initializes the MDB communication tasks (including
 *          the MDB receive task and command processing task) and configures their
 *          priorities, stack sizes, and parameters. The function must be called
 *          during system initialization before starting the FreeRTOS scheduler.
 *
 * @note The function creates the following tasks:
 *       - mdbRxTask: Handles incoming MDB communication data
 *       - mdbCMDProcessTask: Processes MDB commands in a separate thread
 *       Task handles are stored in global variables for later reference.
 *
 * @warning This function must be called before vTaskStartScheduler() and after 
 *          hardware peripherals have been initialized. Task creation failures 
 *          will trigger error handling procedures.
 *
 * @example
 * // System initialization
 * void SystemInit(void)
 * {
 *     // Initialize hardware peripherals
 *     InitPeripherals();
 *     
 *     // Create system tasks
 *     System_TaskCreate();
 *     
 *     // Start the FreeRTOS scheduler
 *     vTaskStartScheduler();
 * }
 */
 void System_TaskCreate(void);

#endif /* SYSTEM_TASKS_H_ */
