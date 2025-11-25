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
#include "stm32f4xx_hal.h"
/******************************************************************************
 *                             Module Config                                  *
 ******************************************************************************/
#define MDB_RING_LEN  40U                   /* power‑of‑two for cheap wrap   */
#define MDB_BUS_TIMEOUT 10                  /* MDB bus timeout in ticks   */
/******************************************************************************
 *                          Macros & Constants                                *
 ******************************************************************************/

/******************************************************************************
 *                          Type Definitions                                  *
 ******************************************************************************/

 typedef enum {
    STATE_INACTIVE,                 /* Cashless device is inactive */
    STATE_RESET,                    /* Cashless device is resetting */
    STATE_DISABLED,                 /* Cashless device is disabled */
    STATE_ENABLED,                  /* Cashless device is enabled */
    STATE_SESSION_IDLE,             /* Cashless device is Session idle */
    STATE_INIT,                     /* Cashless device is initializing */
    STATE_START_SESSION,            /* Cashless device Card is inserted */
    STATE_CANCEL_SESSION,           /* Cashless device Card is removed */
    STATE_APPROVE_VEND_REQ,                 /* Cashless device Vend request */
    STATE_DENY_VEND_REQ,           /* Cashless device Deny Vend request */
    STATE_VEND_PROCESS,             /* Cashless device Vend process */
    STATE_ERROR,                    /* Cashless device encountered an error */
    STATE_WAIT_ACK                  /* Cashless device is waiting for ACK */
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
    CMD_RX_READY = 0,       /* Command is ready to be received */
    CMD_RX_INPROGRESS,      /* Command is being received */
    CMD_RX_DONE,            /* Command has been fully received */
    CMD_RX_BUSY             /* Command is busy being processed */
} CMD_RX_State;
typedef enum {
    VEND_ITEM_SUCCESS = 0,  /* Vend item successfully */
    VEND_ITEM_FAILURE,      /* Vend item failed */
} vend_item_state_t;
typedef struct {
    uint16_t buf[MDB_RING_LEN];      /* storage                       */
    volatile uint16_t wr;            /* write index  (ISR)            */
    volatile uint16_t rd;            /* read index   (task)           */
} mdb_ring_t;

typedef struct{
    uint8_t Req_Item_Price_Hbyte;
    uint8_t Req_Item_Price_Lbyte;
    uint8_t Req_Item_ID_Hbyte;
    uint8_t Req_Item_ID_Lbyte;
    uint8_t Res_Item_Price_Hbyte;
    uint8_t Res_Item_Price_Lbyte;
    vend_item_state_t Vend_Item_State;
}Vending_Item_Data_t;

typedef struct{
    Peripheral_State_t Cashless_StateHandler;                   /* State of the Cashless device */
    CMD_RX_State CMD_RX_StateHandler;                           /* State of the command reception */
    CMD_TX_State CMD_TX_StateHandler;                           /* State of the command transmission */
    CMD_Process_State CMD_Process_StateHandler;                 /* State of the command processing */
    bool Cashless_State_Change_Request;                         /* Flag indicating if state change is requested */
    Peripheral_State_t Cashless_Req_State;
}MDB_StateManager_t;


typedef struct {
	uint16_t MDB_RXbuffer[36];              /* Buffer for received MDB data */
	uint8_t RXBuffer_index;                 /* Current index in the RX buffer */
    uint8_t MDB_RX_CMD_Index;               /* Current index for RX commands */
    uint8_t MDB_Process_CMD_Index;          /* Current index for processing commands */
    bool ACK_Waiting;                       /* Flag indicating if ACK is awaited */
    uint8_t ACK_Skip_Count;                 /* Counter for skipping receives before ACK */
    uint8_t ACK_Expected_Skip;              /* How many receives to skip before ACK */
} MDB_BusManager_t;

typedef void (*CmdHandlerFn)(uint16_t *RxBuffer, uint8_t cmd_length);

typedef struct {
    CmdHandlerFn handler;
} CommandEntry_t;

typedef struct {
    uint8_t Max_balance;    /* Maximum balance allowed */
    uint8_t Cur_balance;    /* Current balance */
    uint8_t Revalue_limit;  /* Revalue limit */
} Peripheral_balance_t;
/**
 * @brief MDB module configuration structure
 */
typedef struct {
    UART_HandleTypeDef *mdb_uart;       /* MDB bus UART handle pointer */
    uint32_t bus_timeout;                  /* MDB bus timeout in milliseconds */
    bool debug_enabled;                    /* Enable/disable debug output */
} MDB_Config_t;

typedef enum {
    VEND_REQ_STATE_IDLE = 0,
    VEND_REQ_STATE_ENABLE,
    VEND_REQ_STATE_DISABLE
} vend_req_state_t;
/******************************************************************************
 *                     Global Variables (extern)                              *
 ******************************************************************************/
/* Note: UART handles are now configured via mdb_config variable */
extern MDB_Config_t mdb_config;                         /* MDB configuration structure */
extern MDB_StateManager_t MDB_StateManager;             /* State manager for MDB protocol */
extern MDB_BusManager_t MDB_BusManager;                 /* Bus manager for MDB protocol */
extern vend_req_state_t vend_request_current_state;     /* Current vend request state */
extern uint8_t balance_setValue;                         /* Balance set value */
/******************************************************************************
 *                              Public API                                    *
 ******************************************************************************/
/**
 * @brief Get the number of items currently in the MDB ring buffer
 * 
 * @details This function calculates the number of items in the ring buffer
 *          by subtracting the read index from the write index. It uses
 *          bit masking to efficiently wrap around the buffer size.
 * 
 * @param r Pointer to the MDB ring buffer structure
 * @return Number of items currently in the buffer
 * 
 * @note This function is not thread-safe and should only be called from
 *       a context where the ring buffer is not being modified.
 * 
 * @example
 * // Get the current item count in the ring buffer
 * uint16_t item_count = mdbRing_count(&rxRing);
 */
 static inline uint16_t mdbRing_count(const mdb_ring_t *r)
{ return (r->wr - r->rd) & (MDB_RING_LEN - 1U); }

/**
 * @brief Get the number of free slots in the MDB ring buffer
 * 
 * @details This function calculates the number of free slots in the ring buffer
 *          by subtracting the number of items from the total buffer size.
 * 
 * @param r Pointer to the MDB ring buffer structure
 * @return Number of free slots currently available in the buffer
 * 
 * @note This function is not thread-safe and should only be called from
 *       a context where the ring buffer is not being modified.
 * 
 * @example
 * // Get the current free slot count in the ring buffer
 * uint16_t free_slots = mdbRing_free(&rxRing);
 */
static inline uint16_t mdbRing_free(const mdb_ring_t *r)
{ return (MDB_RING_LEN - 1U) - mdbRing_count(r); }


/**
 * @brief Initialize the MDB ring buffer
 * 
 * @details Clears the read and write indices, effectively emptying the ring buffer
 *          and preparing it for use. This function should be called once during
 *          system initialization before any MDB communication begins.
 * 
 * @param r Pointer to the MDB ring buffer structure to initialize
 * 
 * @note This function is not thread-safe and should only be called during
 *       initialization when no other tasks are accessing the ring buffer.
 * 
 * @example
 * // Initialize the global MDB ring buffer
 * mdbRing_init(&rxRing);
 */
void     mdbRing_init(mdb_ring_t *r);

/**
 * @brief Push a 16-bit word into the MDB ring buffer
 * 
 * @details Stores one 16-bit MDB word into the ring buffer if space is available.
 *          Uses bit masking for efficient wraparound since MDB_RING_LEN is a power of two.
 *          Maintains one empty slot to differentiate between full and empty buffer states.
 * 
 * @param r Pointer to the MDB ring buffer structure
 * @param word 16-bit MDB word to store in the buffer
 * @return true if the word was successfully stored
 * @return false if the ring buffer is full (overflow condition)
 * 
 * @note Called ONLY from the USART RX ISR (single producer).
 *       This function is interrupt-safe but not thread-safe for multiple producers.
 * @warning Buffer overflow is indicated by return value false.
 *          Caller should handle overflow appropriately.
 * 
 * @example
 * // Store received MDB word in ISR
 * uint16_t mdb_word = mdb_rx_buf[0] & 0x1FF;
 * if (!mdbRing_push(&rxRing, mdb_word)) {
 *     // Handle buffer overflow
 * }
 */
bool     mdbRing_push(mdb_ring_t *r, uint16_t word);

/**
 * @brief Pop a 16-bit word from the MDB ring buffer
 * 
 * @details Retrieves the oldest 16-bit MDB word from the ring buffer if data is available.
 *          Uses bit masking for efficient wraparound and updates the read pointer.
 *          This function implements the consumer side of the single-producer, single-consumer pattern.
 * 
 * @param r Pointer to the MDB ring buffer structure
 * @param word Pointer to store the retrieved 16-bit word
 * @return true if a word was successfully retrieved and stored in *word
 * @return false if the ring buffer is empty (no data available)
 * 
 * @note Called ONLY from mdbTask() (single consumer).
 *       This function is not interrupt-safe if called from multiple contexts.
 * @warning The word parameter must be a valid pointer. No null pointer checking is performed.
 * 
 * @example
 * // Retrieve MDB word from buffer in task context
 * uint16_t received_word;
 * if (mdbRing_pop(&rxRing, &received_word)) {
 *     // Process received_word
 *     MDB_ReceiveCommand(received_word);
 * }
 */
bool mdbRing_pop (mdb_ring_t *r, uint16_t *word);


/**
 * @brief Handle and process a complete MDB command
 * 
 * @details Processes a complete MDB command by validating the command data,
 *          calling the appropriate command handler, and managing the command
 *          processing state machine. This function coordinates between command
 *          reception and response transmission.
 * 
 * @param RxBuffer Pointer to the received command data buffer
 * @param cmd_index Index of the command in the VMC command table
 * 
 * @note This function should be called when a complete command has been received
 *       and is ready for processing. It manages the command processing state machine.
 * @warning The RxBuffer must contain valid command data corresponding to cmd_index.
 *          Invalid parameters may cause undefined behavior.
 * 
 * @example
 * // Process received command (typically called from command processing task)
 * MDB_HandleCommand(MDB_BusManager.MDB_RXbuffer, cmd_index);
 */
void MDB_HandleCommand(uint16_t *RxBuffer, uint8_t cmd_index);

/**
 * @brief Send MDB response data with proper mode bit handling
 * 
 * @details Transmits response data over the MDB bus using interrupt-driven UART transmission.
 *          This function handles the physical transmission of response data after
 *          command processing is complete.
 * 
 * @param data Pointer to the response data buffer to transmit
 * @param dataLength Number of bytes to transmit from the data buffer
 * 
 * @note This function uses interrupt-driven transmission (HAL_UART_Transmit_IT).
 *       The transmission completion will be handled by the UART TX complete callback.
 * @warning The data buffer must remain valid until transmission is complete.
 *          The dataLength must accurately reflect the buffer size.
 * 
 * @example
 * // Send MDB response data
 * uint16_t response_data[] = {0x00AC, 0x0000, 0x00AC};
 * MDB_SendResponseWithModeBit(response_data, sizeof(response_data));
 */
void MDB_SendResponseWithModeBit(uint16_t *data, uint8_t dataLength);

/**
 * @brief Process received MDB command words and manage command reception state
 * 
 * @details Implements a state machine to handle MDB command reception.
 *          Recognizes command headers, accumulates command data, and transitions
 *          through reception states until a complete command is received.
 *          Notifies the command processing task when a complete command is ready.
 * 
 * @param word 16-bit MDB word received from the bus
 * 
 * @note This function maintains the command reception state machine and should
 *       be called for each word received from the MDB bus.
 * @warning Command processing is asynchronous - this function queues commands
 *          for processing by the command processing task.
 * 
 * @example
 * // Process received MDB word
 * uint16_t received_word;
 * if (mdbRing_pop(&rxRing, &received_word)) {
 *     MDB_ReceiveCommand(received_word);
 * }
 */
void MDB_ReceiveCommand(uint16_t word);

/**
 * @brief Initialize the MDB bus communication system
 * 
 * @details Initializes the MDB ring buffer and starts UART interrupt-based reception.
 *          This function sets up the complete MDB communication infrastructure
 *          and should be called once during system initialization.
 * 
 * @note This function must be called before any MDB communication can occur.
 *       It assumes that the MDB UART (mdb_config.mdb_uart) has already been configured.
 * @warning Ensure that the MDB UART is properly configured before calling this function.
 * 
 * @example
 * // Initialize MDB bus during system startup
 * MDB_BusInit();
 */
void MDB_BusInit(void);

/**
 * @brief UART receive complete callback for MDB communication
 * @param huart UART handle that triggered the callback
 * @note This function should be called from HAL_UART_RxCpltCallback
 */
void MDB_UART_RxCallback(UART_HandleTypeDef *huart);

/**
 * @brief Start UART receive interrupt for MDB communication
 * @note Call this function after MDB configuration to start receiving data
 */
void MDB_StartUARTReceive(void);

/**
 * @brief Get the current vend request state
 * @return Current vend request state
 */
vend_req_state_t GetVendReq_State(void);


/**
 * @brief Set the current vend request state
 * @param state New vend request state
 */
void SetVendReq_State(vend_req_state_t state);

#endif /* MDB_HANDLER_H_ */

