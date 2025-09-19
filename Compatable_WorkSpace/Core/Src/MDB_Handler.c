/*******************************************************************************
 * @file    MDB_Handler.c
 * @author  Amr Mohamed
 * @brief   Source file for the MDB handler module
 *
 * @details
 * Implements functionality for sending and receiving data over the MDB bus,
 * managing frame parsing, state control, and communication protocol handling.
 *
 * @version 1.0
 * @date    2025-07-06
 ******************************************************************************/

/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/
#include "MDB_Handler.h"
#include "main.h"
#include "VMC_Config.h"
#include "system_tasks.h"
#include <string.h>
/* FreeRTOS core */
#include "FreeRTOS.h"
#include "task.h"
/******************************************************************************
 *                             Module Config                                  *
 ******************************************************************************/

/******************************************************************************
 *                            Private Macros                                  *
 ******************************************************************************/

/******************************************************************************
 *                         Private Data Types                                 *
 ******************************************************************************/

/******************************************************************************
 *                          Private Variables                                 *
 ******************************************************************************/

/******************************************************************************
 *                         Private Prototypes                                 *
 ******************************************************************************/
static void handle_cmd_0x01E7(uint16_t *RxBuffer, uint8_t cmd_length);
static void handle_cmd_0x013B(uint16_t *RxBuffer, uint8_t cmd_length);
static void handle_cmd_0x01D5(uint16_t *RxBuffer, uint8_t cmd_length);
static void handle_cmd_0x0074(uint16_t *RxBuffer, uint8_t cmd_length);
static void handle_cmd_0x0077(uint16_t *RxBuffer, uint8_t cmd_length);
static void handle_cmd_0x0075(uint16_t *RxBuffer, uint8_t cmd_length);
static void handle_cmd_0x0076(uint16_t *RxBuffer, uint8_t cmd_length);
/******************************************************************************
 *                           Public Variables                                 *
 ******************************************************************************/
mdb_ring_t rxRing; /* our ring instance */
uint16_t mdb_rx_buf[1];

MDB_StateManager_t MDB_StateManager = {
    .Cashless_StateHandler = STATE_RESTART,             /* Initialize the state to RESTART */
    .CMD_RX_StateHandler = CMD_RX_READY,                /* Initialize the command reception state to READY */
    .CMD_TX_StateHandler = CMD_TX_READY,                /* Initialize the command transmission state to READY */
    .CMD_Process_StateHandler = CMD_PROCESS_READY       /* Initialize the command processing state to READY */
};

MDB_BusManager_t MDB_BusManager = {
    .MDB_RXbuffer = {0},
    .RXBuffer_index = 0,
    .MDB_RX_CMD_Index = VMC_CMD_MAX_NUMBER,             /* Initialize to a default value */
    .MDB_TX_CMD_Index = VMC_CMD_MAX_NUMBER,             /* Initialize to a default value */
    .MDB_Process_CMD_Index = VMC_CMD_MAX_NUMBER         /* Initialize to a default value */
};

const CommandEntry_t command_table[] = {
    {handle_cmd_0x01E7},             /* Command 0x01E7 handler */
    {handle_cmd_0x013B},             /* Command 0x013B handler */
    {handle_cmd_0x01D5},             /* Command 0x01D5 handler */
    {handle_cmd_0x0074},             /* Command 0x0074 handler */
    {handle_cmd_0x0077},             /* Command 0x0077 handler */
    {handle_cmd_0x0075},             /* Command 0x0075 handler */
    {handle_cmd_0x0076}              /* Command 0x0076 handler */
};

bool Vending_EN = false;

/******************************************************************************
 *                          Public Functions                                  *
 ******************************************************************************/
/**
 * @brief Initialize the MDB ring buffer
 * @note This function is not thread-safe and should only be called during
 *       initialization when no other tasks are accessing the ring buffer.
 */
void mdbRing_init(mdb_ring_t *r)
{
    r->wr = r->rd = 0U;
}

/**
 * @brief Push a 16-bit word into the MDB ring buffer
 * @note This function is not thread-safe and should only be called from the
 *       USART RX ISR context.
 */
bool mdbRing_push(mdb_ring_t *r, uint16_t word)
{
    /* Calculate the index that WRITER would have AFTER this push.
     * Because MDB_RING_LEN is a power of two (e.g. 256 = 0x100), we can
     * wrap the counter with a bit‑mask instead of an expensive “% len”.
     *
     * Example (LEN = 256):
     *     r->wr = 255
     *     next  = (255 + 1) & 255  -> 0   (wrap around)
     */
    uint16_t next = (r->wr + 1U) & (MDB_RING_LEN - 1U);

    /* If next equals the reader index, buffer is ALMOST full
     * (one slot must stay empty to differentiate full vs empty).
     */
    if (next == r->rd)
        return false; /* overflow */

    r->buf[r->wr] = word; /* store new sample */
    r->wr = next;         /* advance write pointer */
    return true;
}

/**
 * @brief Pop a 16-bit word from the MDB ring buffer
 * @note Called ONLY from mdbTask() (single consumer).
 */
bool mdbRing_pop(mdb_ring_t *r, uint16_t *word)
{
    if (r->rd == r->wr)
        return false; /* empty */

    *word = r->buf[r->rd]; /* fetch oldest entry */
    r->rd = (r->rd + 1U) & (MDB_RING_LEN - 1U);
    return true;
}

/**
 * @brief Initialize the MDB bus communication system
 * @note This function must be called before any MDB communication can occur.
 */
void MDB_BusInit(void)
{
    // Validate configuration
    if (mdb_config.mdb_uart == NULL) {
        MDB_DebugPrint("[MDB] Error: MDB UART handle not configured.\r\n");
        return;
    }
    mdbRing_init(&rxRing);
    MDB_StartUARTReceive();    
    MDB_DebugPrint("[MDB] Bus initialized successfully.\r\n");
}

/**
 * @brief Process received MDB command words and manage command reception state
 * @note Called ONLY from mdbTask() (single consumer).
 */
void MDB_ReceiveCommand(uint16_t word)
{
    uint8_t CMD_expectedLength =0;
    switch (MDB_StateManager.CMD_RX_StateHandler)
    {
    case CMD_RX_READY:
        for (uint8_t i = 0; i < VMC_CMD_MAX_NUMBER; ++i)
        {
            if (word == VMC_CMDs[i].CMD[0])
            {
                MDB_StateManager.CMD_RX_StateHandler = CMD_RX_INPROGRESS;
                MDB_BusManager.MDB_RX_CMD_Index = i;
                MDB_BusManager.MDB_RXbuffer[0] = word;
                MDB_BusManager.RXBuffer_index = 1;
                break;
            }
        }
        break;

    case CMD_RX_INPROGRESS:
    {
        MDB_BusManager
            .MDB_RXbuffer[MDB_BusManager.RXBuffer_index++] = word;
            CMD_expectedLength =
            VMC_CMDs[MDB_BusManager.MDB_RX_CMD_Index].CMD_Length;
        /* Check the command based on the first byte in the buffer */
        switch (MDB_BusManager.MDB_RX_CMD_Index) {
            case VMC_CMD_0x01E7:
                /* Handle specific command receiving if needed */
                if (MDB_BusManager.RXBuffer_index >= CMD_expectedLength)
                {
                    MDB_StateManager.CMD_RX_StateHandler = CMD_RX_DONE;
                }
                break;
            case VMC_CMD_0x013B:
                /* Handle specific command receiving if needed */
                if (MDB_BusManager.RXBuffer_index >= CMD_expectedLength)
                {
                    MDB_StateManager.CMD_RX_StateHandler = CMD_RX_DONE;
                }
                break;
            case VMC_CMD_0x01D5:
                /* Handle specific command receiving if needed */
                if (MDB_BusManager.RXBuffer_index >= CMD_expectedLength)
                {
                    MDB_StateManager.CMD_RX_StateHandler = CMD_RX_DONE;
                }
                break;
            case VMC_CMD_0x0074:
                /* Handle specific command receiving if needed */
                if (MDB_BusManager.RXBuffer_index >= CMD_expectedLength)
                {
                    MDB_StateManager.CMD_RX_StateHandler = CMD_RX_DONE;
                }
                break;
            case VMC_CMD_0x0077:
                /* Handle specific command receiving if needed */
                if (0x0000 == word)
                {
                    MDB_StateManager.CMD_RX_StateHandler = CMD_RX_DONE;
                }
                break;
            case VMC_CMD_0x0075:
                /* Handle specific command receiving if needed */
                if (MDB_BusManager.RXBuffer_index >= CMD_expectedLength)
                {
                    MDB_StateManager.CMD_RX_StateHandler = CMD_RX_DONE;
                }
                break;
            case VMC_CMD_0x0076:
                /* Handle specific command receiving if needed */
                switch (MDB_BusManager.MDB_RXbuffer[1])
                {
                case 0x00BF:
                    if (0x000F == word)
                    {
                        MDB_StateManager.CMD_RX_StateHandler = CMD_RX_DONE;
                    }
                    break;
                default:
                    if (0x0000 == word)
                    {
                        MDB_StateManager.CMD_RX_StateHandler = CMD_RX_DONE;
                    }
                    break;
                }
                break;
            default:
                /* Handle receiving unrecognized command */
                break;
        }
        
        if (CMD_RX_DONE == MDB_StateManager.CMD_RX_StateHandler)
        {
            // MDB_BusManager.RXBuffer_index = 0;
            /* ---- Notify mdbTask that data is ready --------------------- */
            xTaskNotify(mdbCMDProcessTaskHandle,
                        (uint32_t)MDB_BusManager.MDB_RX_CMD_Index,
                        eSetValueWithOverwrite);
            /* ---- Context‑switch immediately if mdbTask has higher prio  */
            taskYIELD();
        }
        break;
    }
    default:

        break;
    }
}

/**
 * @brief Handle and process a complete MDB command
 * @note This function should be called when a complete command has been received
 *       and is ready for processing. It manages the command processing state machine.
 */
void MDB_HandleCommand(uint16_t *RxBuffer, uint8_t cmd_index)
{
    switch (MDB_StateManager.CMD_Process_StateHandler)
    {
    case CMD_PROCESS_READY:
        int temp_Rx_CMD_length = MDB_BusManager.RXBuffer_index;
        int temp_VMC_CMD_Length = VMC_CMDs[cmd_index].CMD_Length;

        if (MDB_StateManager.CMD_RX_StateHandler == CMD_RX_DONE)
        {
            MDB_StateManager.CMD_RX_StateHandler = CMD_RX_BUSY; // Set the state to BUSY
            MDB_BusManager.MDB_Process_CMD_Index = cmd_index; // Set the command index to the command being processed
            command_table[MDB_BusManager.MDB_Process_CMD_Index].handler(RxBuffer, MDB_BusManager.RXBuffer_index);
            // Command reception is done, process the command
            
            // Reset the RX for a new command
            MDB_BusManager.RXBuffer_index = 0;
            MDB_StateManager.CMD_RX_StateHandler = CMD_RX_READY;           // Set the state to READY for the next command
            MDB_StateManager.CMD_Process_StateHandler = CMD_PROCESS_READY; // Set the command processing state to DONE
        }
        else
        {
            // No CMD to be processed
            // This means we are still waiting for the command to be fully received
            return;
        }
        break;
    case CMD_PROCESS_INPROGRESS:
        // TODO Handle error appropriately
        return;
    case CMD_PROCESS_DONE:
        // TODO Handle error appropriately
        break;
    default:
        // Error: Command processing state is not ready or in progress
        // TODO Handle error appropriately
        return;
    }
}

/**
 * @brief Send MDB response data with proper mode bit handling
 * @note This function uses interrupt-driven transmission (HAL_UART_Transmit_IT).
 */
void MDB_SendResponseWithModeBit(uint16_t *data, uint8_t dataLength)
{
    if (mdb_config.mdb_uart != NULL) {
        HAL_UART_Transmit_IT(mdb_config.mdb_uart, (uint8_t *)data, dataLength);
    } else {
        MDB_DebugPrint("[MDB] Error: MDB UART not configured for transmission.\r\n");
    }
}

/**
 * @brief Print debug message to debug UART
 * @param message Null-terminated string to print
 * @note Debug output can be controlled via mdb_config.debug_enabled
 */
void MDB_DebugPrint(const char *message)
{
    if (mdb_config.debug_enabled && mdb_config.debug_uart != NULL) {
        HAL_UART_Transmit(mdb_config.debug_uart, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
    }
}

/******************************************************************************
 *                          Private Functions                                 *
 ******************************************************************************/

/**
 * @brief Handle MDB SETUP command (0x01E7)
 * 
 * @details Processes the MDB SETUP command (0x01E7) which is typically 
 *          sent by the VMC during initialization. This function validates 
 *          the command structure and sends an appropriate response based 
 *          on the current cashless state. In the RESTART state, it will 
 *          transition the system to the INIT state after responding.
 *
 * @param RxBuffer Pointer to buffer containing the received command data
 * @param cmd_length Length of the received command in the buffer
 *
 * @note This function is called by the command dispatcher when command 0x01E7 is received.
 *       It performs structure validation before processing by checking first and last bytes.
 *
 * @warning An invalid command structure will result in no response being sent.
 *          The function uses the global MDB_StateManager to manage state transitions.
 */
static void handle_cmd_0x01E7(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    // Verify the command structure is valid
    if (RxBuffer[0] == VMC_CMDs[cmd_index].CMD[0] &&
        RxBuffer[cmd_length-1] == VMC_CMDs[cmd_index].CMD[VMC_CMDs[cmd_index].CMD_Length-1]) {
        
        // Process command based on current Cashless state
        switch (MDB_StateManager.Cashless_StateHandler) {
            case STATE_RESTART:
                // Handle command during RESTART state
                // Default response for restart state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                MDB_StateManager.Cashless_StateHandler = STATE_INIT; // Transition to INIT state
                break;
            default:
                // Unknown state, use default response
                //TODO Handle the error
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
        }
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) { 
        // Send the response
#if ENABLE_BV_TX == 1
        MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                    VMC_CMDs[cmd_index].CMD_Response_Length);
        }
#endif
    }
}

/**
 * @brief Handle MDB POLL command (0x013B)
 * 
 * @details Processes the MDB POLL command (0x013B) which is sent periodically 
 *          by the VMC to check the status of the cashless device. This function 
 *          monitors the vending GPIO pin to detect card insertion/removal events,
 *          manages session states, and provides appropriate responses based on 
 *          the current system state. It handles multiple states including INIT, 
 *          IDLE, START_SESSION, ACTIVE, VEND_REQ, VEND_PROCESS, and CANCEL_SESSION.
 *
 * @param RxBuffer Pointer to buffer containing the received command data
 * @param cmd_length Length of the received command in the buffer
 *
 * @note This function is one of the most complex handlers as it manages the 
 *       cashless state machine transitions based on both command data and 
 *       hardware signals (GPIO pins).
 *
 * @warning Proper GPIO configuration is required for the VENDING_Pin to correctly 
 *          detect card insertion/removal events.
 */
static void handle_cmd_0x013B(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    // Verify the command structure is valid
    if (HAL_GPIO_ReadPin(VENDING_GPIO_Port, VENDING_Pin) == GPIO_PIN_RESET && Vending_EN == false)
    {
        MDB_StateManager.Cashless_StateHandler = STATE_START_SESSION;
        Vending_EN = true;
    }
    else
    {
        //Do nothing
    }
    if (HAL_GPIO_ReadPin(VENDING_GPIO_Port, VENDING_Pin) == GPIO_PIN_SET && Vending_EN == true)
    {
        MDB_StateManager.Cashless_StateHandler = STATE_CANCEL_SESSION;
        Vending_EN = false;
    }
    if (RxBuffer[0] == VMC_CMDs[cmd_index].CMD[0] &&
        RxBuffer[cmd_length-1] == VMC_CMDs[cmd_index].CMD[VMC_CMDs[cmd_index].CMD_Length-1]) {
        
        // Process command based on current Cashless state
        switch (MDB_StateManager.Cashless_StateHandler) {
            case STATE_INIT:
                // Handle command during INIT state
                // No response needed for this command in init state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                break;

            case STATE_IDLE:
                // Handle command during IDLE state
                // No response in idle state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                break;

            case STATE_START_SESSION:
                // Handle command during INSERT_CARD state
                // Special handling for card inserted state with new data
                // Each element is a two-byte value from the provided data (11 elements)
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0003;
                VMC_CMDs[cmd_index].CMD_Response[1] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[2] = 0x00B9;
                VMC_CMDs[cmd_index].CMD_Response[3] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[4] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[5] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[6] = 0x0001;
                VMC_CMDs[cmd_index].CMD_Response[7] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[8] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[9] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[10] = 0x01BD;
                // Set the response length to exactly 11
                VMC_CMDs[cmd_index].CMD_Response_Length = 11;
                MDB_StateManager.Cashless_StateHandler = STATE_ACTIVE; // Transition to ACTIVE state
                break;
                
            case STATE_ACTIVE:
                // Handle command during ACTIVE state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                break;
                
            case STATE_VEND_REQ:
                // Handle command during VEND_REQ state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0005;
                VMC_CMDs[cmd_index].CMD_Response[1] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[2] = 0x000F;
                VMC_CMDs[cmd_index].CMD_Response[3] = 0x0114;
                VMC_CMDs[cmd_index].CMD_Response_Length = 4;
                MDB_StateManager.Cashless_StateHandler = STATE_VEND_PROCESS; // Transition to vend process state
                break;
                
            case STATE_VEND_PROCESS:
                // Handle command during VEND_PROCESS state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                break;
            
            case STATE_CANCEL_SESSION:
                // Handle command during VEND_PROCESS state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0004;
                VMC_CMDs[cmd_index].CMD_Response[1] = 0x0104;
                VMC_CMDs[cmd_index].CMD_Response_Length = 2;
            break;

            default:
                // Unknown state, no response
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
        }
        
        // Send the response if there is one
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
#if ENABLE_BV_TX == 1
            MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                        VMC_CMDs[cmd_index].CMD_Response_Length);
#endif
        }
    }
}

/**
 * @brief Handle MDB READER CANCEL command (0x01D5)
 * 
 * @details Processes the MDB READER CANCEL command (0x01D5) which is used to 
 *          terminate an ongoing cashless payment session. This function validates
 *          the command structure and sends appropriate responses based on the current
 *          state of the cashless device. The primary purpose of this command is to
 *          handle user-initiated cancellations during the payment process.
 *
 * @param RxBuffer Pointer to buffer containing the received command data
 * @param cmd_length Length of the received command in the buffer
 *
 * @note This function primarily handles the IDLE state with a standard ACK response,
 *       but can also respond to other states with a default ACK. Its behavior is
 *       simpler than other handlers as it mainly acknowledges the cancellation request.
 *
 * @warning Command validation checks both the first and last bytes of the command
 *          to ensure proper command structure before processing. Invalid command
 *          structure will result in no response being sent.
 */
static void handle_cmd_0x01D5(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    // Verify the command structure is valid
    if (RxBuffer[0] == VMC_CMDs[cmd_index].CMD[0] &&
        RxBuffer[cmd_length-1] == VMC_CMDs[cmd_index].CMD[VMC_CMDs[cmd_index].CMD_Length-1]) {
        
        // Process command based on current Cashless state
        switch (MDB_StateManager.Cashless_StateHandler) {
            case STATE_IDLE:
                // Handle command during IDLE state
                // Standard response in idle state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                break;
            default:
                // Unknown state, use default response
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                // VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
        }
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
            // Send the response
#if ENABLE_BV_TX == 1
        MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                    VMC_CMDs[cmd_index].CMD_Response_Length);
#endif
        }
    }
}

/**
 * @brief Handle MDB initialization command (0x0074)
 * 
 * @details Processes the MDB command (0x0074) which is part of the setup/initialization
 *          sequence sent by the VMC. This function validates the command structure
 *          and responds appropriately based on the current system state. During the
 *          initialization phase, it facilitates the transition to the IDLE state.
 *
 * @param RxBuffer Pointer to buffer containing the received command data
 * @param cmd_length Length of the received command in the buffer
 *
 * @note This command is larger than most (33 bytes), but for efficiency only the
 *       first and last bytes are checked for validation. During initialization,
 *       this command causes a transition to the IDLE state.
 *
 * @warning The command validation is simplified to only check first and last bytes
 *          rather than the entire command structure due to its length. This is a
 *          performance optimization but could potentially miss malformed commands.
 */
static void handle_cmd_0x0074(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    
    // We could compare the entire buffer, but for simplicity just check the first and last bytes
    if (RxBuffer[0] == VMC_CMDs[cmd_index].CMD[0] &&
        RxBuffer[cmd_length-1] == VMC_CMDs[cmd_index].CMD[VMC_CMDs[cmd_index].CMD_Length-1]) {
        
        // Process command based on current Cashless state
        switch (MDB_StateManager.Cashless_StateHandler) {
            case STATE_INIT:
                // During initialization state, provide initialization information
                // The response is already defined in VMC_CMDs
                MDB_StateManager.Cashless_StateHandler = STATE_IDLE;
                break;
            default:
                // Default device info response
                //TODO Handle the error
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
        }
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
            // Send the response which contains information about the device
#if ENABLE_BV_TX == 1
        MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                    VMC_CMDs[cmd_index].CMD_Response_Length);
#endif
        }
    }
}

/**
 * @brief Handle MDB initialization command (0x0077)
 * 
 * @details Processes the MDB command (0x0077) which is part of the setup/initialization
 *          sequence sent by the VMC. This function processes different subcommands 
 *          (identified by RxBuffer[1]) and provides appropriate responses based on
 *          the current state and subcommand received. It is primarily used during
 *          the initialization phase to establish communication parameters.
 *
 * @param RxBuffer Pointer to buffer containing the received command data
 * @param cmd_length Length of the received command in the buffer
 *
 * @note This function supports multiple subcommands:
 *       - 0x01F9: Configuration information response
 *       - 0x00FF: Standard ACK response
 *       Each subcommand requires a different response structure, which is handled
 *       within the nested switch statements in this function.
 *
 * @warning The function validates command structure by checking first and last bytes.
 *          Invalid command structure or unrecognized subcommands will result in 
 *          no response or error handling.
 */
static void handle_cmd_0x0077(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    // Verify the command structure is valid
    if (RxBuffer[0] == VMC_CMDs[cmd_index].CMD[0] &&
        RxBuffer[cmd_length-1] == VMC_CMDs[cmd_index].CMD[VMC_CMDs[cmd_index].CMD_Length-1]) {
        
        // Process command based on current Cashless state
        switch (MDB_StateManager.Cashless_StateHandler) {
            case STATE_INIT:
                // During initialization state
                // Has subcommand
                switch (RxBuffer[1]) {
                    case 0x01F9:
                        // Standard response for disabled state
                        VMC_CMDs[cmd_index].CMD_Response[0] = 0x0001;
                        VMC_CMDs[cmd_index].CMD_Response[1] = 0x0002;
                        VMC_CMDs[cmd_index].CMD_Response[2] = 0x0000;
                        VMC_CMDs[cmd_index].CMD_Response[3] = 0x0000;
                        VMC_CMDs[cmd_index].CMD_Response[4] = 0x0001;
                        VMC_CMDs[cmd_index].CMD_Response[5] = 0x0000;
                        VMC_CMDs[cmd_index].CMD_Response[6] = 0x0005;
                        VMC_CMDs[cmd_index].CMD_Response[7] = 0x0003;
                        VMC_CMDs[cmd_index].CMD_Response[8] = 0x010C;
                        
                        // Set the response length to 9
                        VMC_CMDs[cmd_index].CMD_Response_Length = 9;
                        break;
                    case 0x00FF:
                        // Standard ACK
                        VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                        VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                        break;
                    default:
                        // TODO Handle error
                        VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                        return;
                }
                break;
            default:
                // TODO Handle error
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
        }
        
        // Send the response if we have one
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
#if ENABLE_BV_TX == 1
            MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                        VMC_CMDs[cmd_index].CMD_Response_Length);
#endif
        }
    }
}

/**
 * @brief Handle MDB Session Start command (0x0075)
 * 
 * @details Processes the MDB command (0x0075) which is part of the session start
 *          process. This function validates the command structure and sends an
 *          appropriate response based on the current cashless state, particularly
 *          focusing on the ACTIVE state response where the session is established.
 *
 * @param RxBuffer Pointer to buffer containing the received command data
 * @param cmd_length Length of the received command in the buffer
 *
 * @note This function is primarily designed to handle session establishment
 *       in the ACTIVE state. When in this state, it sends a four-element response
 *       with specific data values (0x000F, 0x0001, 0x003B, 0x014B) to acknowledge
 *       the session start request.
 *
 * @warning Command validation checks both the first and last bytes of the command
 *          to ensure proper command structure before processing. The function will
 *          not respond to commands received in states other than ACTIVE unless 
 *          error handling is implemented.
 */
static void handle_cmd_0x0075(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    // Verify the command structure is valid
    if (RxBuffer[0] == VMC_CMDs[cmd_index].CMD[0] && 
        RxBuffer[cmd_length-1] == VMC_CMDs[cmd_index].CMD[VMC_CMDs[cmd_index].CMD_Length-1]) {
        
        // Process command based on current Cashless state
        switch (MDB_StateManager.Cashless_StateHandler) {
            case STATE_ACTIVE:
                // During active state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x000F;
                VMC_CMDs[cmd_index].CMD_Response[1] = 0x0001;
                VMC_CMDs[cmd_index].CMD_Response[2] = 0x003B;
                VMC_CMDs[cmd_index].CMD_Response[3] = 0x014B;
                VMC_CMDs[cmd_index].CMD_Response_Length = 4;
                break;
                
            default:
                //TODO Handle the error
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
        }
        
        // Send the response if we have one
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
#if ENABLE_BV_TX == 1
            MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                        VMC_CMDs[cmd_index].CMD_Response_Length);
#endif
        }
    }
}

/**
 * @brief Handle MDB Vending Request and Status command (0x0076)
 * 
 * @details Processes the MDB command (0x0076) which handles different aspects of
 *          the vending process based on subcommands. This function processes vending
 *          requests, successful vends, and failed vends according to the specific
 *          subcommand received (identified by RxBuffer[1]). It manages appropriate 
 *          state transitions based on both the current system state and the specific
 *          subcommand. It supports three main states: ACTIVE, VEND_PROCESS,
 *          and CANCEL_SESSION.
 *
 * @param RxBuffer Pointer to buffer containing the received command data
 * @param cmd_length Length of the received command in the buffer
 *
 * @note This function supports multiple subcommands and state combinations:
 *       - ACTIVE state + 0x01FF subcommand: Vending request - Transitions to VEND_REQ state
 *       - VEND_PROCESS state + 0x017F subcommand: Successful vend - Transitions to ACTIVE state
 *       - CANCEL_SESSION state + 0x00BF subcommand: Session termination - Transitions to IDLE state
 *       Each combination results in different responses and state transitions based on
 *       the vending operation status.
 *
 * @warning This is one of the most complex handler functions as it manages 
 *          multiple state transitions and vending operation statuses via subcommands. 
 *          Invalid subcommands or states will result in no response being sent.
 */
static void handle_cmd_0x0076(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    // This is a simple command with just one byte 
    // Process command based on current Cashless state
    switch (MDB_StateManager.Cashless_StateHandler) {
        case STATE_ACTIVE:
            // During active state
            switch (RxBuffer[1])  // Check the sub-command
            {
            case 0x01FF:
                // Standard ACK
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                // Transition to vend request state
                MDB_StateManager.Cashless_StateHandler = STATE_VEND_REQ;
                break;
            default:
                //TODO Handle error
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
            }
            break;
        
        case STATE_VEND_PROCESS:
            // During vend process state
            switch (RxBuffer[1])  // Check the sub-command
            {
            case 0x017F:
                // Standard ACK
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                // Transition to active state
                MDB_StateManager.Cashless_StateHandler = STATE_ACTIVE;
                break;
            default:
                //TODO Handle error
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
            }
            break;

        case STATE_CANCEL_SESSION:
            // During vend process state
            switch (RxBuffer[1])  // Check the sub-command
            {
            case 0x00BF:
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0007;
                VMC_CMDs[cmd_index].CMD_Response[1] = 0x0107;
                VMC_CMDs[cmd_index].CMD_Response_Length = 2;
                // Transition to active state
                MDB_StateManager.Cashless_StateHandler = STATE_IDLE;
                break;
            default:
                //TODO Handle error
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
            }
            break;
        default:
            // Unknown state
            //TODO Handle error
            VMC_CMDs[cmd_index].CMD_Response_Length = 0;
            break;
    }
    // If there is a response, send it
    if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
#if ENABLE_BV_TX == 1
        MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                    VMC_CMDs[cmd_index].CMD_Response_Length);
#endif
        }
}

/*************************** Private Functions *******************************/

/**
 * @brief UART receive complete callback for MDB communication
 * @param huart UART handle that triggered the callback
 * @note This function should be called from HAL_UART_RxCpltCallback
 */
void MDB_UART_RxCallback(UART_HandleTypeDef *huart) {
    uint16_t word = mdb_rx_buf[0] & 0x1FF;
    /* ---- 2. Store in ring buffer (overflow returns false) --------- */
    (void)mdbRing_push(&rxRing, word); /* ignore overflow for now   */

    /* ---- 3. Notify mdbTask that data is ready --------------------- */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(mdbRxTaskHandle, &xHigherPriorityTaskWoken);

    /* ---- 4. Context‑switch immediately if mdbTask has higher prio  */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    /* ---- 5. Re‑arm reception of the NEXT byte --------------------- */
    HAL_UART_Receive_IT(mdb_config.mdb_uart, (uint8_t *)mdb_rx_buf, 1);
}

/**
 * @brief Start UART receive interrupt for MDB communication
 * @note Call this function after MDB configuration to start receiving data
 */
void MDB_StartUARTReceive(void) {
    if (mdb_config.mdb_uart != NULL) {
        // Start UART receive with interrupt for single byte
        HAL_UART_Receive_IT(mdb_config.mdb_uart, (uint8_t *)mdb_rx_buf, 1);
        MDB_DebugPrint("MDB: UART Receive Started\r\n");
    }
}

