/*******************************************************************************
 * @file    MDB_Handler.h
 * @author  Amr Mohamed
 * @brief   Header file for the MDB handler module
 *
 * @details
 * This module provides the interface for managing communication over the
 * MDB (Multi-Drop Bus) protocol used in vending machines.
 *
 * @version 1.0
 * @date    2025-07-06
 ******************************************************************************/

#ifndef MDB_HANDLER_H_
#define MDB_HANDLER_H_

/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
/******************************************************************************
 *                             Module Config                                  *
 ******************************************************************************/

/******************************************************************************
 *                          Macros & Constants                                *
 ******************************************************************************/
#define MDB_RING_LEN  256U           /* power‑of‑two for cheap wrap   */

/******************************************************************************
 *                          Type Definitions                                  *
 ******************************************************************************/

 typedef enum {
    STATE_DISABLED,             /* Cashless device is disabled */
    STATE_IDLE,                 /* Cashless device is idle */
    STATE_ACTIVE,               /* Cashless device is active */
    STATE_INIT,                 /* Cashless device is initializing */
    STATE_RESTART,              /* Cashless device is restarting */
    STATE_START_SESSION,          /* Cashless device Card is inserted */
    STATE_CANCEL_SESSION,          /* Cashless device Card is removed */
    STATE_VEND_REQ,             /* Cashless device Vend request */
    STATE_VEND_PROCESS,        /* Cashless device Vend process */
    STATE_ERROR                 /* Cashless device encountered an error */
} Peripheral_State_t;

typedef enum {
    CMD_TX_READY = 0,       /* Command is ready to be transmitted */
    CMD_TX_INPROGRESS,      /* Command is being transmitted */
    CMD_TX_DONE,            /* Command has been fully transmitted */
    CMD_TX_BUSY             /* Command is busy being processed */
} CMD_TX_State;

typedef enum {
    CMD_PROCESS_READY,      /* Command is ready to be processed */
    CMD_PROCESS_INPROGRESS, /* Command is being processed */
    CMD_PROCESS_DONE,       /* Command has been fully processed */
    CMD_PROCESS_BUSY        /* Command is busy being processed */
} CMD_Process_State;

typedef enum {
    CMD_RX_READY = 0, /* Command is ready to be received */
    CMD_RX_INPROGRESS, /* Command is being received */
    CMD_RX_DONE, /* Command has been fully received */
    CMD_RX_BUSY /* Command is busy being processed */
} CMD_RX_State;

typedef struct {
    uint16_t buf[MDB_RING_LEN];      /* storage                       */
    volatile uint16_t wr;            /* write index  (ISR)            */
    volatile uint16_t rd;            /* read index   (task)           */
} mdb_ring_t;


typedef struct{
    Peripheral_State_t Cashless_StateHandler;                   /* State of the Cashless device */
    CMD_RX_State CMD_RX_StateHandler;           /* State of the command reception */
    CMD_TX_State CMD_TX_StateHandler;           /* State of the command transmission */
    CMD_Process_State CMD_Process_StateHandler; /* State of the command processing */
}MDB_StateManager_t;


typedef struct {
	uint16_t MDB_RXbuffer[36];
	uint8_t RXBuffer_index;
    uint8_t MDB_RX_CMD_Index;
    uint8_t MDB_TX_CMD_Index;
    uint8_t MDB_Process_CMD_Index;
} MDB_BusManager_t;

typedef void (*CmdHandlerFn)(uint16_t *RxBuffer, uint8_t cmd_length);

typedef struct {
    CmdHandlerFn handler;
} CommandEntry_t;
/******************************************************************************
 *                     Global Variables (extern)                              *
 ******************************************************************************/
extern UART_HandleTypeDef huart1;

/******************************************************************************
 *                              Public API                                    *
 ******************************************************************************/
void     mdbRing_init(mdb_ring_t *r);                  /* empty the ring    */
bool     mdbRing_push(mdb_ring_t *r, uint16_t word);   /* ISR context       */
bool     mdbRing_pop (mdb_ring_t *r, uint16_t *word);  /* Task context      */

static inline uint16_t mdbRing_count(const mdb_ring_t *r)
{ return (r->wr - r->rd) & (MDB_RING_LEN - 1U); }

static inline uint16_t mdbRing_free(const mdb_ring_t *r)
{ return (MDB_RING_LEN - 1U) - mdbRing_count(r); }


void MDB_HandleCommand(uint16_t *RxBuffer, uint8_t length);
void MDB_SendResponseWithModeBit(uint16_t *data, uint8_t dataLength);
void MDB_ReceiveCommand(uint16_t word);

#endif /* MDB_HANDLER_H_ */

