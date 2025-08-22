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
    .CMD_RX_StateHandler = CMD_RX_READY,          /* Initialize the command reception state to READY */
    .CMD_TX_StateHandler = CMD_TX_READY,          /* Initialize the command transmission state to BUSY */
    .CMD_Process_StateHandler = CMD_PROCESS_READY /* Initialize the command processing state to BUSY */
};

MDB_BusManager_t MDB_BusManager = {
    .MDB_RXbuffer = {0},
    .RXBuffer_index = 0,
    .MDB_RX_CMD_Index = VMC_CMD_MAX_NUMBER,     /* Initialize to a default value */
    .MDB_TX_CMD_Index = VMC_CMD_MAX_NUMBER,     /* Initialize to a default value */
    .MDB_Process_CMD_Index = VMC_CMD_MAX_NUMBER /* Initialize to a default value */
};

const CommandEntry_t command_table[] = {
    {handle_cmd_0x01E7},
    {handle_cmd_0x013B},
    {handle_cmd_0x01D5},
    {handle_cmd_0x0074},
    {handle_cmd_0x0077},
    {handle_cmd_0x0075},
    {handle_cmd_0x0076}
};

bool Vending_EN = false;

/******************************************************************************
 *                          Public Functions                                  *
 ******************************************************************************/
/* ------------------------------------------------------------
 * HAL_UART_RxCpltCallback
 *
 * Called on every RXNE (UART receive complete).
 * Copies the 9-bit MDB word to the ring buffer,
 * notifies rxTask, and re-arms the interrupt.
 * ---------------------------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        uint16_t word = mdb_rx_buf[0] & 0x1FF;

        /* ---- 2. Store in ring buffer (overflow returns false) --------- */
        (void)mdbRing_push(&rxRing, word); /* ignore overflow for now   */

        /* ---- 3. Notify mdbTask that data is ready --------------------- */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(mdbRxTaskHandle, &xHigherPriorityTaskWoken);

        /* ---- 4. Context‑switch immediately if mdbTask has higher prio  */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        /* ---- 5. Re‑arm reception of the NEXT byte --------------------- */
        HAL_UART_Receive_IT(huart, (uint8_t *)mdb_rx_buf, 1);
    }
}

/* ----------------------------------------------------------------------
 * mdbRing_init
 * ----------------------------------------------------------------------
 * Clear the read and write indices, effectively emptying the ring.
 * Caller: mdbTask() once at startup.
 * -------------------------------------------------------------------- */
void mdbRing_init(mdb_ring_t *r)
{
    r->wr = r->rd = 0U;
}

/* ----------------------------------------------------------------------
 * mdbRing_push
 * ----------------------------------------------------------------------
 * Put one 16‑bit word into the ring.
 * • Returns true  if the word was stored.
 * • Returns false if the ring is full (caller may drop byte or count error).
 *
 * Called ONLY from the USART RX ISR (single producer).
 * -------------------------------------------------------------------- */
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

/* ----------------------------------------------------------------------
 * mdbRing_pop
 * ----------------------------------------------------------------------
 * Retrieve one 16‑bit word from the ring.
 * • Returns true  and sets *word when data available.
 * • Returns false when ring is empty.
 *
 * Called ONLY from mdbTask() (single consumer).
 * -------------------------------------------------------------------- */
bool mdbRing_pop(mdb_ring_t *r, uint16_t *word)
{
    if (r->rd == r->wr)
        return false; /* empty */

    *word = r->buf[r->rd]; /* fetch oldest entry */
    r->rd = (r->rd + 1U) & (MDB_RING_LEN - 1U);
    return true;
}
void MDB_BusInit(void)
{
    mdbRing_init(&rxRing);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)mdb_rx_buf, 1);
}

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

void MDB_SendResponseWithModeBit(uint16_t *data, uint8_t dataLength)
{
    HAL_UART_Transmit_IT(&huart1, (uint8_t *)data, dataLength);
}

/******************************************************************************
 *                          Private Functions                                 *
 ******************************************************************************/

/**
 * @brief Handler for command 0x01E7
 * @param RxBuffer Buffer containing the received command
 * @param cmd_length Length of the command in RxBuffer
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
 * @brief Handler for command 0x013B
 * @param RxBuffer Buffer containing the received command
 * @param cmd_length Length of the command in RxBuffer
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
 * @brief Handler for command 0x01D5
 * @param RxBuffer Buffer containing the received command
 * @param cmd_length Length of the command in RxBuffer
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
 * @brief Handler for command 0x0074
 * @param RxBuffer Buffer containing the received command
 * @param cmd_length Length of the command in RxBuffer
 */
static void handle_cmd_0x0074(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    // For this command, we need to check all the data in the command
    // Since this is a larger command with 33 bytes
    
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
 * @brief Handler for command 0x0077
 * @param RxBuffer Buffer containing the received command
 * @param cmd_length Length of the command in RxBuffer
 */
static void handle_cmd_0x0077(uint16_t *RxBuffer, uint8_t cmd_length) {
    // Get the command index from the MDB_BusManager
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    // This command might have a subcommand
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
 * @brief Handler for command 0x0075
 * @param RxBuffer Buffer containing the received command
 * @param cmd_length Length of the command in RxBuffer
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
                // Default response for unknown state
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
 * @brief Handler for command 0x0076
 * @param RxBuffer Buffer containing the received command
 * @param cmd_length Length of the command in RxBuffer
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
            // May terminate vend request
            // MDB_StateManager.Cashless_StateHandler = STATE_INSERT_CARD;
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



