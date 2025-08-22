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
extern TaskHandle_t mdbRxTaskHandle ;
extern TaskHandle_t mdbCMDProcessTaskHandle ;
extern mdb_ring_t rxRing;
/******************************************************************************
 *                        Function Declarations                               *
 ******************************************************************************/
void System_TaskCreate(void);

#endif /* SYSTEM_TASKS_H_ */
