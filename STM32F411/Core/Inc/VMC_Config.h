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
    uint16_t CMD[5];   /* Command buffer for MDB commands */
    uint16_t CMD_Length; /* Length of the command buffer */
    uint16_t CMD_Response[36]; /* Response buffer for MDB commands */
    uint16_t CMD_Response_Length; /* Length of the response buffer */
}CMD_Type;

typedef enum {
    VMC_CMD_0x01E7,  /* First command in the array - 0x01E7 */
    VMC_CMD_0x013B,  /* Second command in the array - 0x013B */
    VMC_CMD_0x01D5,  /* Third command in the array - 0x01D5 */
    VMC_CMD_0x0074,  /* Fourth command in the array - 0x0074 */
    VMC_CMD_0x0077,  /* Fifth command in the array - 0x0077 */
    VMC_CMD_0x0075,  /* Sixth command in the array - 0x0075 */
    VMC_CMD_0x0076,  /* Seventh command in the array - 0x0076 */
	VMC_CMD_0x017D,
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
