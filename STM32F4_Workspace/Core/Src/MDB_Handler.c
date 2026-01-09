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
/* System logging */
#include "SYS_Logger.h"
/*ESP Header*/
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

/* MDB_ReceiveCommand helper functions */
static bool MDB_ProcessAck(uint16_t word);
static uint8_t MDB_DetectCommand(uint16_t word);
static bool MDB_IsCommandComplete(uint8_t cmd_index, uint16_t buffer_index, uint16_t last_word);
static void MDB_InitCommandReception(uint8_t cmd_index, uint16_t first_word);
static void MDB_CompleteCommandReception(uint8_t cmd_index);
static void MDB_RequestAck(uint8_t skip_count, Peripheral_State_t req_state);
static void MDB_UpdateVendingItemData(uint16_t *RxBuffer);
static void Set_CurBalance(uint16_t new_balance);
static void Internal_VendReq_Change(vend_req_state_t state);
static void MDB_StateChangeHandler(Peripheral_State_t old_state, Peripheral_State_t new_state);
static void MDB_ChangeState(Peripheral_State_t new_state);
/******************************************************************************
 *                           Public Variables                                 *
 ******************************************************************************/
mdb_ring_t rxRing; /* our ring instance */
uint16_t mdb_rx_buf[1];

MDB_StateManager_t MDB_StateManager = {
    .Cashless_StateHandler = STATE_INACTIVE,             /* Initialize the state to INACTIVE */
    .CMD_RX_StateHandler = CMD_RX_READY,                /* Initialize the command reception state to READY */
    .CMD_TX_StateHandler = CMD_TX_READY,                /* Initialize the command transmission state to READY */
    .CMD_Process_StateHandler = CMD_PROCESS_READY,       /* Initialize the command processing state to READY */
    .Cashless_State_Change_Request = false              /* Initialize state change request flag to false */
};

MDB_BusManager_t MDB_BusManager = {
    .MDB_RXbuffer = {0},
    .RXBuffer_index = 0,
    .MDB_RX_CMD_Index = VMC_CMD_MAX_NUMBER,             /* Initialize to a default value */
    .MDB_Process_CMD_Index = VMC_CMD_MAX_NUMBER,        /* Initialize to a default value */
    .ACK_Waiting = false,                               /* Initialize ACK waiting flag */
    .ACK_Skip_Count = 0,                                /* Initialize to 0 */
    .ACK_Expected_Skip = 0                              /* Initialize to 0 */
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

Vending_Item_Data_t Vending_Item_Data;
Peripheral_balance_t Peripheral_Balance = {244, 0, 244};
bool Internal_VendReq_Change_flag = false;
vend_req_state_t vend_request_current_state = VEND_REQ_STATE_IDLE; /* Current vend request state */
uint16_t balance_setValue = 0;

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
        MDB_LOG_CRITICAL_ERROR(MDB_ERROR_UART_NOT_CONFIGURED,NO_DATA_PRESENT,NO_CONTEXT_PRESENT);
        return;
    }
    mdbRing_init(&rxRing);  
    MDB_PRINT_INFO("[MDB] Bus initialized successfully.");
}

/**
 * @brief Process received MDB command words and manage command reception state
 * @param word Received 16-bit word from MDB bus
 * @note This function is now modular and easier to test and maintain
 */
void MDB_ReceiveCommand(uint16_t word)
{
    /* Handle ACK processing first */
    if (MDB_ProcessAck(word)) {
        return; /* ACK was processed, nothing more to do */
    }
    
    switch (MDB_StateManager.CMD_RX_StateHandler) {
        case CMD_RX_READY:
            {
                uint8_t cmd_index = MDB_DetectCommand(word);
                if (cmd_index != VMC_CMD_MAX_NUMBER) {
                    MDB_InitCommandReception(cmd_index, word);
                    MDB_PRINT_INFO("[MDB][INFO] Command 0x%04X detected, starting reception.\r\n", word);
                }
            }
            break;
            
        case CMD_RX_INPROGRESS:
            {
                uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
                
                /* Store the received word */
                MDB_BusManager.MDB_RXbuffer[MDB_BusManager.RXBuffer_index++] = word;
                
                /* Check if command is complete */
                if (MDB_IsCommandComplete(cmd_index, MDB_BusManager.RXBuffer_index, word)) {
                    MDB_CompleteCommandReception(cmd_index);
                    MDB_PRINT_INFO("[MDB][INFO] Command 0x%04X reception complete.\r\n", VMC_CMDs[cmd_index].CMD[0]);
                }
            }
            break;
            
        default:
            /* Handle error state */
            /* Log the error */
            MDB_LOG_ERROR(MDB_ERROR_INVALID_RX_STATE,(uint16_t)MDB_StateManager.CMD_RX_StateHandler, (uint8_t)MDB_BusManager.MDB_RX_CMD_Index);
            MDB_PRINT_INFO("[MDB][ERROR] Invalid CMD_RX_StateHandler: %d\r\n", (uint16_t)MDB_StateManager.CMD_RX_StateHandler);
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

        if (MDB_StateManager.CMD_RX_StateHandler == CMD_RX_DONE)
        {
            MDB_PRINT_INFO("[MDB][INFO] Command 0x%04X being processed.\r\n", VMC_CMDs[cmd_index].CMD[0]);
            MDB_StateManager.CMD_RX_StateHandler = CMD_RX_BUSY; // Set the state to BUSY
            MDB_BusManager.MDB_Process_CMD_Index = cmd_index; // Set the command index to the command being processed
            command_table[MDB_BusManager.MDB_Process_CMD_Index].handler(RxBuffer, MDB_BusManager.RXBuffer_index);// Command reception is done, process the command
            MDB_PRINT_INFO("[MDB][INFO] Command 0x%04X processing complete.\r\n", VMC_CMDs[cmd_index].CMD[0]);
            
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
    case CMD_PROCESS_DONE:
    default:
        // Error: Command processing state is not ready or in progress
        /* Log the error */
        MDB_LOG_ERROR(MDB_ERROR_INVALID_PROCESS_STATE, (uint16_t)MDB_StateManager.CMD_Process_StateHandler, (uint8_t)MDB_BusManager.MDB_Process_CMD_Index);
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
        ESP_LOG_ERROR(MDB_ERROR_UART_NOT_CONFIGURED,NO_DATA_PRESENT,NO_CONTEXT_PRESENT);
    }
}

void SetVendReq_State(vend_req_state_t state){
	vend_request_current_state = state;
}

vend_req_state_t GetVendReq_State(void){
	return vend_request_current_state;
}


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
    }
}


/******************************************************************************
 *                          Private Functions                                 *
 ******************************************************************************/

/**
 * @brief Process ACK responses
 * @param word Received word
 * @return true if ACK was processed, false otherwise
 */
static bool MDB_ProcessAck(uint16_t word) {
    if (!MDB_BusManager.ACK_Waiting) {
        return false;
    }
    
    // Check if we need to skip more receives before looking for ACK
    if (MDB_BusManager.ACK_Skip_Count < MDB_BusManager.ACK_Expected_Skip) {
        MDB_BusManager.ACK_Skip_Count++;
        return true; // Skip this receive
    }

    if (word == 0x0000) // ACK received
    {
        if (MDB_StateManager.Cashless_State_Change_Request) {
            MDB_ChangeState(MDB_StateManager.Cashless_Req_State);
            MDB_StateManager.Cashless_State_Change_Request = false;
        }
        MDB_BusManager.ACK_Waiting = false;
        return true;
    } else {
        // Unexpected word while waiting for ACK
        MDB_BusManager.ACK_Waiting = false;
        MDB_StateManager.Cashless_State_Change_Request = false;
        /* Log the unexpected word */
        MDB_LOG_ERROR(MDB_ERROR_UNEXPECTED_ACK_WORD, word, (uint8_t)MDB_BusManager.MDB_RX_CMD_Index);
        return true; // ACK sequence handled (even if error)
    }
}

/**
 * @brief Detect command start
 * @param word Received word
 * @return Command index if found, VMC_CMD_MAX_NUMBER if not found
 */
static uint8_t MDB_DetectCommand(uint16_t word) {
    for (uint8_t i = 0; i < VMC_CMD_MAX_NUMBER; ++i) {
        if (word == VMC_CMDs[i].CMD[0]) {
            return i;
        }
    }
    return VMC_CMD_MAX_NUMBER; // Not found
}

/**
 * @brief Check if command reception is complete
 * @param cmd_index Command index
 * @param buffer_index Current buffer index
 * @param last_word Last received word
 * @return true if command is complete
 */
static bool MDB_IsCommandComplete(uint8_t cmd_index, uint16_t buffer_index, uint16_t last_word) {    
    switch (cmd_index) {
        case VMC_CMD_0x01E7:
        case VMC_CMD_0x013B:
        case VMC_CMD_0x01D5:
        case VMC_CMD_0x0075:
            return buffer_index >= VMC_CMDs[cmd_index].CMD_Length;
            
        case VMC_CMD_0x0074:
        case VMC_CMD_0x0077:
            return last_word == 0x0000;
            
        case VMC_CMD_0x0076:
            if (MDB_BusManager.MDB_RXbuffer[1] == 0x00BF) {
                return last_word == 0x000F;
            } else {
                return last_word == 0x0000;
            }
            
        default:
            return false;
    }
}

/**
 * @brief Initialize command reception
 * @param cmd_index Command index
 * @param first_word First word of command
 */
static void MDB_InitCommandReception(uint8_t cmd_index, uint16_t first_word) {
    MDB_StateManager.CMD_RX_StateHandler = CMD_RX_INPROGRESS;
    MDB_BusManager.MDB_RX_CMD_Index = cmd_index;
    MDB_BusManager.MDB_RXbuffer[0] = first_word;
    MDB_BusManager.RXBuffer_index = 1;
}

/**
 * @brief Complete command reception and notify processing task
 * @param cmd_index Command index
 */
static void MDB_CompleteCommandReception(uint8_t cmd_index) {
    MDB_StateManager.CMD_RX_StateHandler = CMD_RX_DONE;
    
    /* Notify mdbTask that data is ready */
    xTaskNotify(mdbCMDProcessTaskHandle,
                (uint32_t)cmd_index,
                eSetValueWithOverwrite);
    
    /* Context-switch immediately if mdbTask has higher priority */
    taskYIELD();
}
/**
 * @brief Validate command structure by checking first and last bytes
 * @param RxBuffer Pointer to buffer containing the received command data
 * @param cmd_length Length of the received command in the buffer
 * @param cmd_index Index of the command in VMC_CMDs array
 * @return true if command structure is valid, false otherwise
 */
static bool MDB_ValidateCommandStructure(uint16_t *RxBuffer, uint8_t cmd_length, uint8_t cmd_index) {
    return (RxBuffer[0] == VMC_CMDs[cmd_index].CMD[0] &&
            RxBuffer[cmd_length-1] == VMC_CMDs[cmd_index].CMD[VMC_CMDs[cmd_index].CMD_Length-1]);
}
/**
 * @brief Request an ACK from the VMC and optionally change state
 * @param skip_count Number of subsequent commands to skip ACK for
 * @param req_state State to transition to after ACK is received
 * @return void
 */
static void MDB_RequestAck(uint8_t skip_count, Peripheral_State_t req_state) {
    MDB_BusManager.ACK_Waiting = true;
    MDB_BusManager.ACK_Skip_Count = 0;
    MDB_BusManager.ACK_Expected_Skip = skip_count;
    if(STATE_WAIT_ACK == req_state)
    {
        MDB_StateManager.Cashless_State_Change_Request = false;
    }
    else
    {
        MDB_StateManager.Cashless_Req_State = req_state;
        MDB_StateManager.Cashless_State_Change_Request = true;
    }
}
/**
 * @brief Update vending item data from received MDB command buffer
 * 
 * @details Extracts and updates vending item information from the MDB command buffer.
 *          This function is typically called when processing VEND_REQUEST commands
 *          to capture the requested item's price and identification data. The function
 *          validates the input buffer before updating the global vending item data structure.
 *
 * @param RxBuffer Pointer to buffer containing the received command data
 *                 Expected format:
 *                 RxBuffer[2]: Item price byte 1 (high byte)
 *                 RxBuffer[3]: Item price byte 2 (low byte) 
 *                 RxBuffer[4]: Item ID byte 1 (high byte)
 *                 RxBuffer[5]: Item ID byte 2 (low byte)
 *
 * @note This function assumes the RxBuffer contains at least 6 elements.
 *       It updates the global Vending_Item_Data structure with price and ID information
 *       extracted from the VEND_REQUEST command (0x0076 with 0x01FF subcommand).
 *
 * @warning No bounds checking is performed on the RxBuffer. Caller must ensure
 *          the buffer contains valid data at the expected indices before calling.
 */
static void MDB_UpdateVendingItemData(uint16_t *RxBuffer) {
    
    // Update vending item price (16-bit value split into two bytes)
    Vending_Item_Data.Req_Item_Price_Hbyte = (uint8_t)RxBuffer[2];
    Vending_Item_Data.Req_Item_Price_Lbyte = (uint8_t)RxBuffer[3];

    // Update vending item ID (16-bit value split into two bytes)
    Vending_Item_Data.Req_Item_ID_Hbyte = (uint8_t)RxBuffer[4];
    Vending_Item_Data.Req_Item_ID_Lbyte = (uint8_t)RxBuffer[5];
    
    //Update the price in the response field also
    Vending_Item_Data.Res_Item_Price_Hbyte = (0xFF - (Vending_Item_Data.Req_Item_Price_Lbyte))/2;
    Vending_Item_Data.Res_Item_Price_Lbyte = (0xFF - (Vending_Item_Data.Req_Item_Price_Hbyte))/2;

}

/**
 * @brief Calculate MDB checksum for command validation
 * 
 * @details Calculates a simple additive checksum for MDB command validation.
 *          This is the standard checksum method used in most MDB implementations.
 *          The checksum is calculated by summing all bytes in the command buffer
 *          and returning the result as a 16-bit value for validation purposes.
 *
 * @param buffer Pointer to buffer containing the MDB command data
 * @param length Length of the buffer in 16-bit words (not bytes)
 *
 * @return uint16_t Calculated checksum value
 *
 * @note This function assumes the buffer contains 16-bit words and calculates
 *       the checksum by summing both high and low bytes of each word.
 *       For MDB protocol, this checksum is typically used for command validation
 *       rather than error detection during transmission.
 *
 * @warning Input validation should be performed by the caller. This function
 *          does not check for null pointers or zero length.
 */
static uint16_t MDB_CalculateChecksum(uint16_t *buffer, uint8_t length) {
    uint16_t checksum = 0;
    // Sum all bytes in the buffer
    for (uint8_t i = 0; i < length; i++) {
        checksum += buffer[i];
    }
    checksum &= 0x00FF; // Keep only the lower 8 bits
    return checksum;
}
/**
 * @brief Set the current balance of the cashless device
 * 
 * @details Updates the current balance maintained by the cashless device.
 *          This function is typically called after a successful transaction
 *          or when the balance needs to be adjusted due to refunds or errors.
 *          The balance is stored in a global structure and can be queried by
 *          other parts of the system as needed.
 *
 * @param new_balance New balance value to set (in cents)
 *
 * @note The balance is represented as a 16-bit unsigned integer, allowing
 *       for a maximum balance of 655.35 units (e.g., dollars) if using cents.
 *       Ensure that the new balance does not exceed this limit before calling.
 *
 * @warning No bounds checking is performed on the new_balance parameter.
 *          Caller must ensure the value is valid and within acceptable range.
 */
static void Set_CurBalance(uint16_t new_balance)
{
    if(new_balance <= Peripheral_Balance.Max_balance)
    {
        Peripheral_Balance.Cur_balance = new_balance;
    }
    else
    {
        // Handle error: balance exceeds maximum limit
        MDB_LOG_ERROR(MDB_ERROR_BALANCE_EXCEEDS_MAXIMUM, (uint16_t)new_balance, MDB_BusManager.MDB_RX_CMD_Index);
        Peripheral_Balance.Cur_balance = Peripheral_Balance.Max_balance; // Cap to max balance
    }
    Peripheral_Balance.Revalue_limit = Peripheral_Balance.Max_balance - Peripheral_Balance.Cur_balance;
    
}
/**
 * @brief Handle MDB RESET command (0x01E7)
 * 
 * @details Processes the MDB RESET command (0x01E7) which is used to request
 *          the cashless device to reset and initialize. This command is sent by
 *          the VMC to force the cashless device into a known state, typically
 *          during system startup or error recovery. The function validates the
 *          command structure and responds with device status while transitioning
 *          from INACTIVE to RESET state to begin the initialization sequence.
 *
 * @param RxBuffer Pointer to buffer containing the received command data
 * @param cmd_length Length of the received command in the buffer
 *
 * @note This is typically the first command sent by the VMC to establish
 *       communication with the cashless device and request a system reset.
 *       It performs structure validation before processing by checking first and last bytes.
 *
 * @warning An invalid command structure will result in no response being sent.
 *          The function uses the global MDB_StateManager to manage state transitions.
 */
static void handle_cmd_0x01E7(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    // Verify the command structure is valid
    if (MDB_ValidateCommandStructure(RxBuffer, cmd_length, cmd_index)) {
        // Process command based on current Cashless state
        switch (MDB_StateManager.Cashless_StateHandler) {
            case STATE_INACTIVE:
                // Handle command during INACTIVE state
                // Default response for inactive state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                MDB_ChangeState(STATE_RESET); // Transition to RESET state
                break;
            default:
                // Unknown state, use default response
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                MDB_ChangeState(STATE_RESET); // Transition to RESET state
                MDB_LOG_ERROR(MDB_ERROR_INVALID_STATE_FOR_RESET, (uint16_t)MDB_StateManager.Cashless_StateHandler, (uint8_t)cmd_index);
                break;
        }
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) { 
        // Send the response
        MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                    VMC_CMDs[cmd_index].CMD_Response_Length);
        }
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
    
    // if (HAL_GPIO_ReadPin(VENDING_GPIO_Port, VENDING_Pin) == GPIO_PIN_RESET && Vending_EN == false && MDB_StateManager.Cashless_StateHandler == STATE_ENABLED)
    // {
    //     MDB_StateManager.Cashless_StateHandler = STATE_START_SESSION;
    //     Set_CurBalance(135);
    //     Vending_EN = true;
    // }
    // else
    // {
    //     //Do nothing
    // }
    // if (HAL_GPIO_ReadPin(VENDING_GPIO_Port, VENDING_Pin) == GPIO_PIN_SET && Vending_EN == true && MDB_StateManager.Cashless_StateHandler == STATE_SESSION_IDLE)
    // {
    //     MDB_StateManager.Cashless_StateHandler = STATE_CANCEL_SESSION;
    //     Vending_EN = false;
    // }
    // else if (HAL_GPIO_ReadPin(VENDING_GPIO_Port, VENDING_Pin) == GPIO_PIN_SET && Vending_EN == true)
    // {
    //     Vending_EN = false;
    // }
    
    if (GetVendReq_State() ==  VEND_REQ_STATE_ENABLE
        && MDB_StateManager.Cashless_StateHandler == STATE_ENABLED)
    {
        MDB_ChangeState(STATE_START_SESSION);
        Set_CurBalance(balance_setValue);
        SetVendReq_State(VEND_REQ_STATE_IDLE);
    }
    else if (GetVendReq_State() == VEND_REQ_STATE_DISABLE
             && MDB_StateManager.Cashless_StateHandler == STATE_SESSION_IDLE)
    {
        MDB_ChangeState(STATE_CANCEL_SESSION);
        SetVendReq_State(VEND_REQ_STATE_IDLE);
    }
    else
    {
        //Do nothing
    }
    // Verify the command structure is valid
    if (MDB_ValidateCommandStructure(RxBuffer, cmd_length, cmd_index)) {
        
        // Process command based on current Cashless state
        switch (MDB_StateManager.Cashless_StateHandler) {
            case STATE_RESET:
                // Handle command during RESET state
                // Default response for reset state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[1] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 2;
                MDB_ChangeState(STATE_INIT); // Transition to INIT state
                break;
            case STATE_INIT:
                // Handle command during INIT state
                // No response needed for this command in init state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                break;

            case STATE_DISABLED:
                // Handle command during DISABLED state
                // No response needed for this command in disabled state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                break;
                case STATE_ENABLED:
                // Handle command during ENABLED state
                // No response needed for this command in enabled state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                break;
            case STATE_START_SESSION:
                // Handle command during INSERT_CARD state
                // Special handling for card inserted state with new data
                // Each element is a two-byte value from the provided data (11 elements)
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0003;
                VMC_CMDs[cmd_index].CMD_Response[1] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[2] = Peripheral_Balance.Cur_balance;
                VMC_CMDs[cmd_index].CMD_Response[3] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[4] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[5] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[6] = 0x0001;
                VMC_CMDs[cmd_index].CMD_Response[7] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[8] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[9] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[10] = MDB_CalculateChecksum(VMC_CMDs[cmd_index].CMD_Response, 10);
                VMC_CMDs[cmd_index].CMD_Response[10] |= 0x0100; // Set mode bit
                // Set the response length to exactly 11
                VMC_CMDs[cmd_index].CMD_Response_Length = 11;
                MDB_RequestAck(1, STATE_SESSION_IDLE);
                break;

                case STATE_SESSION_IDLE:
                // Handle command during SESSION_IDLE state
                // No response in session idle state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                break;
                
            case STATE_APPROVE_VEND_REQ:
                // Handle command during APPROVE_VEND_REQ state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0005;
                VMC_CMDs[cmd_index].CMD_Response[1] = Vending_Item_Data.Res_Item_Price_Hbyte;
                VMC_CMDs[cmd_index].CMD_Response[2] = Vending_Item_Data.Res_Item_Price_Lbyte;
                VMC_CMDs[cmd_index].CMD_Response[3] = MDB_CalculateChecksum(VMC_CMDs[cmd_index].CMD_Response, 3);
                VMC_CMDs[cmd_index].CMD_Response[3] |= 0x0100; // Set mode bit
                VMC_CMDs[cmd_index].CMD_Response_Length = 4;
                MDB_RequestAck(1, STATE_VEND_PROCESS);
                break;
            case STATE_DENY_VEND_REQ:
                // Handle command during DENY_VEND_REQ state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0006;
                VMC_CMDs[cmd_index].CMD_Response[1] = 0x0106;
                VMC_CMDs[cmd_index].CMD_Response_Length = 2;
                Vending_Item_Data.Vend_Item_State = VEND_ITEM_FAILURE;
                MDB_RequestAck(1, STATE_CANCEL_SESSION);
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
                MDB_RequestAck(1, STATE_WAIT_ACK);
            break;

            default:
                // Unknown state, no response
                /* Log the error */
                MDB_LOG_ERROR(MDB_ERROR_COMMAND_NOT_VALID_IN_STATE, (uint16_t)MDB_StateManager.Cashless_StateHandler, (uint8_t)cmd_index);
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
        }
        
        // Send the response if there is one
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
            MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                        VMC_CMDs[cmd_index].CMD_Response_Length);
        }
    }
}

/**
 * @brief Handle MDB READER ENABLE command (0x01D5)
 * 
 * @details Processes the MDB READER ENABLE command (0x01D5) which is used to 
 *          change the cashless device to be in enabled state. This function validates
 *          the command structure and transitions the cashless device from DISABLED
 *          to ENABLED state, allowing it to participate in payment transactions.
 *          The command sends an ACK response to confirm the state transition.
 *
 * @param RxBuffer Pointer to buffer containing the received command data
 * @param cmd_length Length of the received command in the buffer
 *
 * @note This function handles the state transition from DISABLED to ENABLED state
 *       with a standard ACK response. It enables the cashless device to accept
 *       payment transactions and participate in the vending process.
 *
 * @warning Command validation checks both the first and last bytes of the command
 *          to ensure proper command structure before processing. Invalid command
 *          structure will result in no response being sent.
 */
static void handle_cmd_0x01D5(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    // Verify the command structure is valid
    if (MDB_ValidateCommandStructure(RxBuffer, cmd_length, cmd_index)) {
        
        // Process command based on current Cashless state
        switch (MDB_StateManager.Cashless_StateHandler) {
            case STATE_DISABLED:
                // Handle command during DISABLED state
                // Standard response in disabled state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                MDB_ChangeState(STATE_ENABLED); // Transition to ENABLED state
                break;
            default:
                // Unknown state, use default response
                MDB_LOG_ERROR(MDB_ERROR_COMMAND_NOT_VALID_IN_STATE, (uint16_t)MDB_StateManager.Cashless_StateHandler, (uint8_t)cmd_index);
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                MDB_ChangeState(STATE_ENABLED); // Transition to ENABLED state
                break;
        }
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
            // Send the response
        MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                    VMC_CMDs[cmd_index].CMD_Response_Length);
        }
    }
}

/**
 * @brief Handle MDB Product Identification Exchange command (0x0074)
 * 
 * @details Processes the MDB command (0x0074) which is used to exchange product
 *          identification and serial number information between the VMC and cashless
 *          device. This command allows the VMC to inform the cashless device about
 *          its product identification and serial number while simultaneously requesting
 *          the same identification data from the cashless device. This bidirectional
 *          exchange is essential for system identification and audit trail purposes.
 *
 * @param RxBuffer Pointer to buffer containing the received command data
 * @param cmd_length Length of the received command in the buffer
 *
 * @note This command is larger than most (33 bytes) as it contains VMC product
 *       identification and serial number data. For efficiency, only the first and
 *       last bytes are checked for validation. The cashless device responds with
 *       its own identification data and transitions to DISABLED state.
 *
 * @warning The command validation is simplified to only check first and last bytes
 *          rather than the entire command structure due to its length. This is a
 *          performance optimization but could potentially miss malformed commands.
 */
static void handle_cmd_0x0074(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    
    // We could compare the entire buffer, but for simplicity just check the first and last bytes
    if (MDB_ValidateCommandStructure(RxBuffer, cmd_length, cmd_index)) {
        
        // Process command based on current Cashless state
        switch (MDB_StateManager.Cashless_StateHandler) {
            case STATE_INIT:
                // During initialization state, provide initialization information
                // The response is already defined in VMC_CMDs
                MDB_RequestAck(1, STATE_DISABLED);
                break;
            default:
                // Default device info response
                //TODO Handle the error
                MDB_LOG_ERROR(MDB_ERROR_COMMAND_NOT_VALID_IN_STATE, (uint16_t)MDB_StateManager.Cashless_StateHandler, (uint8_t)cmd_index);
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
        }
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
            // Send the response which contains information about the device
        MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                    VMC_CMDs[cmd_index].CMD_Response_Length);
        }
    }
}

/**
 * @brief Handle MDB VMC Data Request and Device Configuration command (0x0077)
 * 
 * @details Processes the MDB command (0x0077) which is used to request information
 *          about VMC data and ask for cashless device configuration data. This command
 *          allows the VMC to inform the cashless device about some VMC parameters while
 *          simultaneously requesting configuration information from the cashless device.
 *          The function processes different subcommands (identified by RxBuffer[1]) and
 *          provides appropriate responses based on the current state and subcommand received.
 *
 * @param RxBuffer Pointer to buffer containing the received command data
 * @param cmd_length Length of the received command in the buffer
 *
 * @note This function supports multiple subcommands:
 *       - 0x01F9: Cashless device configuration information response
 *       - 0x00FF: Standard ACK response for VMC data for Min/Max pricing
 *       Each subcommand facilitates bidirectional information exchange between
 *       VMC and cashless device during the setup phase.
 *
 * @warning The function validates command structure by checking first and last bytes.
 *          Invalid command structure or unrecognized subcommands will result in 
 *          no response or error handling.
 */
static void handle_cmd_0x0077(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    // Verify the command structure is valid
    if (MDB_ValidateCommandStructure(RxBuffer, cmd_length, cmd_index)){   
        // Process command based on current Cashless state
        switch (MDB_StateManager.Cashless_StateHandler) {
            case STATE_RESET:
            case STATE_INIT:
                // During initialization state
                // Has subcommand
                switch (RxBuffer[1]) {
                    case 0x01F9:
                        // Standard response for disabled state
                        VMC_CMDs[cmd_index].CMD_Response[0] = 0x0001;
                        /* Feature lvl */
                        VMC_CMDs[cmd_index].CMD_Response[1] = 0x0002;
                        /* Country code byte_1 */
                        VMC_CMDs[cmd_index].CMD_Response[2] = 0x0000;
                        /* Country code byte_2 */
                        VMC_CMDs[cmd_index].CMD_Response[3] = 0x0000;
                        /* Scale factor */
                        VMC_CMDs[cmd_index].CMD_Response[4] = 0x0001;
                        /* Decimal places */
                        VMC_CMDs[cmd_index].CMD_Response[5] = 0x0000;
                        /* Application maximum response time */
                        VMC_CMDs[cmd_index].CMD_Response[6] = 0x0055;
                        /* Miscellaneous options */
                        VMC_CMDs[cmd_index].CMD_Response[7] = 0x0003;
                        VMC_CMDs[cmd_index].CMD_Response[8] = 0x015C;
                        // Set the response length to 9
                        VMC_CMDs[cmd_index].CMD_Response_Length = 9;
                        MDB_ChangeState(STATE_INIT); // Transition to INIT state in case it was RESET
                        MDB_RequestAck(1, STATE_WAIT_ACK);
                        break;
                    case 0x00FF:
                        // Standard ACK
                        VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                        VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                        break;
                    default:
                        // TODO Handle error
                        MDB_LOG_ERROR(MDB_ERROR_SUB_COMMAND_NOT_RECOGNIZED, (uint16_t)RxBuffer[1], (uint8_t)cmd_index);
                        VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                        return;
                }
                break;
            default:
                MDB_LOG_ERROR(MDB_ERROR_COMMAND_NOT_VALID_IN_STATE, NO_DATA_PRESENT, (uint8_t)cmd_index);
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
        }
        
        // Send the response if we have one
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
            MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                        VMC_CMDs[cmd_index].CMD_Response_Length);
        }
    }
}

/**
 * @brief Handle MDB REVALUE LIMIT REQUEST command (0x0075)
 * 
 * @details Processes the MDB REVALUE LIMIT REQUEST command (0x0075) which is used
 *          to request revalue limit information from the cashless device. This command
 *          allows the VMC to query the cashless device about its revalue capabilities
 *          and limits. The function validates the command structure and sends an
 *          appropriate response containing revalue limit data based on the current
 *          cashless state, particularly during SESSION_IDLE state.
 *
 * @param RxBuffer Pointer to buffer containing the received command data
 * @param cmd_length Length of the received command in the buffer
 *
 * @note This function is primarily designed to handle revalue limit requests
 *       in the SESSION_IDLE state. When in this state, it sends a four-element response
 *       with specific revalue limit data (0x000F, 0x0001, 0x003B, 0x014B) to inform
 *       the VMC about the cashless device's revalue capabilities and limitations.
 *
 * @warning Command validation checks both the first and last bytes of the command
 *          to ensure proper command structure before processing. The function will
 *          not respond to commands received in states other than SESSION_IDLE unless 
 *          error handling is implemented.
 */
static void handle_cmd_0x0075(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    // Verify the command structure is valid
    if (MDB_ValidateCommandStructure(RxBuffer, cmd_length, cmd_index)) {
        
        // Process command based on current Cashless state
        switch (MDB_StateManager.Cashless_StateHandler) {
            case STATE_SESSION_IDLE:
                // During active state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x000F;
                VMC_CMDs[cmd_index].CMD_Response[1] = 0x0001;
                VMC_CMDs[cmd_index].CMD_Response[2] = Peripheral_Balance.Revalue_limit;
                VMC_CMDs[cmd_index].CMD_Response[3] = MDB_CalculateChecksum(VMC_CMDs[cmd_index].CMD_Response, 3);
                VMC_CMDs[cmd_index].CMD_Response[3] |= 0x0100; // Set mode bit
                // Set the response length to exactly 4
                VMC_CMDs[cmd_index].CMD_Response_Length = 4;
                break;
                
            default:
                // Unknown state, no response
                /* Log the error */
                MDB_LOG_ERROR(MDB_ERROR_COMMAND_NOT_VALID_IN_STATE, NO_DATA_PRESENT, (uint8_t)cmd_index);
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
        }
        
        // Send the response if we have one
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
            MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                        VMC_CMDs[cmd_index].CMD_Response_Length);
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
        case STATE_SESSION_IDLE:
            // During session idle state
            switch (RxBuffer[1])  // Check the sub-command
            {
            case 0x01FF:
                // Standard ACK
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                // Transition to vend request state
                MDB_UpdateVendingItemData(RxBuffer);
                Vending_Item_Data.Vend_Item_State = VEND_ITEM_FAILURE;
                if(Vending_Item_Data.Res_Item_Price_Lbyte > Peripheral_Balance.Cur_balance)
                {
                    // Not enough balance to vend
                    MDB_ChangeState(STATE_DENY_VEND_REQ);
                }
                else
                {
                    // Enough balance to vend
                    MDB_ChangeState(STATE_APPROVE_VEND_REQ);
                }
                break;
            case 0x00BF:
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0007;
                VMC_CMDs[cmd_index].CMD_Response[1] = 0x0107;
                VMC_CMDs[cmd_index].CMD_Response_Length = 2;
                MDB_RequestAck(1, STATE_ENABLED);
                break;
            default:
                // No other sub-commands valid in this state
                // Log the error
                MDB_LOG_ERROR(MDB_ERROR_SUB_COMMAND_NOT_RECOGNIZED, (uint16_t)RxBuffer[1], (uint8_t)cmd_index);
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
                // Transition to session idle state
                MDB_ChangeState(STATE_SESSION_IDLE);
                Vending_Item_Data.Vend_Item_State = VEND_ITEM_SUCCESS;
                Internal_VendReq_Change(VEND_REQ_STATE_DISABLE);
                //Set the balance to be zero after successful vend
                Set_CurBalance(0);
                break;
            default:
                // No other sub-commands valid in this state
                // Log the error
                MDB_LOG_ERROR(MDB_ERROR_SUB_COMMAND_NOT_RECOGNIZED, (uint16_t)RxBuffer[1], (uint8_t)cmd_index);
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
            }
            break;
        case STATE_CANCEL_SESSION:
            // During cancel session state
            switch (RxBuffer[1])  // Check the sub-command
            {
            case 0x00BF:
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0007;
                VMC_CMDs[cmd_index].CMD_Response[1] = 0x0107;
                VMC_CMDs[cmd_index].CMD_Response_Length = 2;
                //Set the balance to be zero after successful vend
                Set_CurBalance(0);
                // Prepare item data string for publishing
                char itemDataStr[64];  // Local variable to hold formatted data string
                char *status = (Vending_Item_Data.Vend_Item_State == VEND_ITEM_SUCCESS) ? "SUCCESS" : "FAILURE";
                snprintf(itemDataStr, sizeof(itemDataStr), "U:%d,ID:%d%d,%s", 
                            Vending_Item_Data.Res_Item_Price_Lbyte, 
                            Vending_Item_Data.Req_Item_ID_Hbyte, 
                            Vending_Item_Data.Req_Item_ID_Lbyte,
                            status);
                ESP_RequestPublish(topic_handlers[ESP_TOPIC_MAIN_PUBLISH].topic_pattern, itemDataStr, 1);
                MDB_RequestAck(1, STATE_ENABLED);
                break;
            default:
                // No other sub-commands valid in this state
                // Log the error
                MDB_LOG_ERROR(MDB_ERROR_SUB_COMMAND_NOT_RECOGNIZED, (uint16_t)RxBuffer[1], (uint8_t)cmd_index);
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
            }
            break;
        default:
            // Unknown state, no response
            /* Log the error */
            MDB_LOG_ERROR(MDB_ERROR_COMMAND_NOT_VALID_IN_STATE, NO_DATA_PRESENT, (uint8_t)cmd_index);
            VMC_CMDs[cmd_index].CMD_Response_Length = 0;
            break;
    }
    // If there is a response, send it
    if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
        MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                    VMC_CMDs[cmd_index].CMD_Response_Length);
        }
}
/**
 * @brief Internal function to change the vending request state
 * 
 * @details This function sets a flag indicating that the vending request state
 *          has changed and updates the current vending request state to the
 *          specified value. It is used internally to manage transitions between
 *          different vending request states.
 *
 * @param state The new vending request state to set
 *
 * @note This function sets the Internal_VendReq_Change_flag to true, indicating
 *       that a change has occurred. It then calls SetVendReq_State() to update
 *       the current vending request state.
 *
 * @warning This function is intended for internal use only and should not be
 *          called directly from outside the module.
 */
static void Internal_VendReq_Change(vend_req_state_t state){
    Internal_VendReq_Change_flag = true;
    SetVendReq_State(state);
}

/**
 * @brief Helper function to safely change state with callback
 */
static void MDB_ChangeState(Peripheral_State_t new_state)
{
    Peripheral_State_t old_state = MDB_StateManager.Cashless_StateHandler;
    
    if (old_state != new_state) {
        MDB_StateManager.Cashless_StateHandler = new_state;
        MDB_StateChangeHandler(old_state, new_state);
    }
}

/**
 * @brief MDB state change handler
 */
static void MDB_StateChangeHandler(Peripheral_State_t old_state, Peripheral_State_t new_state)
{
    if (new_state == STATE_ENABLED) {
        /* Send notification to server (optional) */
        ESP_RequestPublish(topic_handlers[ESP_TOPIC_MAIN_PUBLISH].topic_pattern, "SYSTEM:ENABLED", 1);
    }
}
/*************************** Private Functions *******************************/


