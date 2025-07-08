/*******************************************************************************
 * @file    VMC_Config.h
 * @author  Amr Mohamed
 * @brief   Header file for the VMC configuration module
 *
 * @details
 * This module provides configuration structures and interface functions
 * for initializing and managing VMC (Vending Machine Controller) settings.
 *
 * @version 1.0
 * @date    2025-07-06
 ******************************************************************************/

#ifndef VMC_CONFIG_H_
#define VMC_CONFIG_H_

/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
/******************************************************************************
 *                             Module Config                                  *
 ******************************************************************************/
#define ENABLE_USB_LOGGING  0
#define ENABLE_BV_TX        1
/******************************************************************************
 *                          Macros & Constants                                *
 ******************************************************************************/

/******************************************************************************
 *                          Type Definitions                                  *
 ******************************************************************************/
typedef struct{
    uint16_t CMD[36];   /* Command buffer for MDB commands */
    uint16_t CMD_Length; /* Length of the command buffer */
    uint16_t CMD_Response[36]; /* Response buffer for MDB commands */
    uint16_t CMD_Response_Length; /* Length of the response buffer */
}CMD_Type;

typedef enum {
    VMC_CMD_0x01E6,
    VMC_CMD_0x01E7,
    VMC_CMD_0x0067,
    VMC_CMD_0x01F9,
    VMC_CMD_0x0133,
    VMC_CMD_0x0199,
    VMC_CMD_0x013B,
    VMC_CMD_0x0132,
    VMC_CMD_0x0077,
    VMC_CMD_0x0066,
    VMC_CMD_0x0074,
    VMC_CMD_0x0064,
    VMC_CMD_0x0099,
    VMC_CMD_0x009D,
    VMC_CMD_MAX_NUMBER
} CMD_Data;
/******************************************************************************
 *                     Global Variables (extern)                              *
 ******************************************************************************/
extern CMD_Type VMC_CMDs[VMC_CMD_MAX_NUMBER];
/******************************************************************************
 *                              Public API                                    *
 ******************************************************************************/

#endif /* VMC_CONFIG_H_ */
