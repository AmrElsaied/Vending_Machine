# MDB Protocol Implementation

## Overview

This document describes the Multi-Drop Bus (MDB) protocol implementation used for communication with cashless payment devices and bill validators in the vending machine controller.

---

## Physical Layer

| Parameter | Value |
|-----------|-------|
| Baud Rate | 9600 bps |
| Data Format | 9 bits (8 data + 1 mode bit) |
| Mode Bit | 1 = Command/Address, 0 = Data |
| Voltage Levels | 5V/12V compatible |
| Bus Timeout | 10ms between characters |

---

## Command Set

Seven core commands manage device communication and control:

### 0x01E7 - Reset/Restart

**Purpose**: Initialize peripheral device and request system reset.

**Subcommands**: None

**State-Based Responses**:

| Current State | Response | Next State |
|---------------|----------|-----------|
| INACTIVE | ACK (0x0100) | RESET |
| Any Other | ACK (0x0100) + Error Log | RESET |

---

### 0x013B - Poll/Status

**Purpose**: Query device status and manage session state transitions.

**Subcommands**: None

**State-Based Responses**:

| Current State | Response | Next State | Details |
|---------------|----------|-----------|---------|
| RESET | 0x0000, 0x0100 | INIT | Transition on poll |
| INIT | 0x0100 (ACK) | INIT | Idle response |
| DISABLED | 0x0100 (ACK) | DISABLED | No change |
| ENABLED | 0x0100 (ACK) | ENABLED | Ready for transaction |
| START_SESSION | [11-element response] + Checksum | SESSION_IDLE + Wait ACK | Current balance in Response[2] |
| SESSION_IDLE | 0x0100 (ACK) | SESSION_IDLE | Status response |
| APPROVE_VEND_REQ | 0x0005, Price_H, Price_L, Checksum | VEND_PROCESS + Wait ACK | Response length: 4 |
| DENY_VEND_REQ | 0x0106 | CANCEL_SESSION + Wait ACK | Vend denied |
| VEND_PROCESS | 0x0100 (ACK) | VEND_PROCESS | Continue processing |
| CANCEL_SESSION | 0x0104 | WAIT_ACK | Cleanup response |

---

### 0x01D5 - Reader Enable

**Purpose**: Enable card reader and transition device to active state.

**Subcommands**: None

**State-Based Responses**:

| Current State | Response | Next State |
|---------------|----------|-----------|
| DISABLED | ACK (0x0100) | ENABLED |
| Any Other | ACK (0x0100) + Error Log | ENABLED |

---

### 0x0074 - Product Identification Exchange

**Purpose**: Exchange VMC and device product identification and serial numbers.

**Subcommands**: None

**Payload**: 33 bytes

**State-Based Responses**:

| Current State | Response | Next State | Details |
|---------------|----------|-----------|---------|
| INIT | Device Info (predefined) | DISABLED + Wait ACK | Bidirectional exchange |
| Any Other | No response + Error Log | No change | Command invalid in state |

---

### 0x0077 - VMC Data Request & Device Configuration

**Purpose**: Exchange VMC data and request cashless device configuration.

**Subcommands**:

#### Subcommand 0x01F9 - Capabilities Response

**Usage**: Query device features during initialization

**State-Based Responses**:

| Current State | Response | Next State | Details |
|---------------|----------|-----------|---------|
| RESET | Feature Level (0x0002), Country Code (0x0000), Scale Factor (0x0001), Decimal Places (0x0000), Max Response Time (0x0055), Options (0x0003, 0x015C) | INIT + Wait ACK | 9-element response |
| INIT | Feature Level (0x0002), Country Code (0x0000), Scale Factor (0x0001), Decimal Places (0x0000), Max Response Time (0x0055), Options (0x0003, 0x015C) | WAIT_ACK | Configuration data |
| Any Other | No response + Error Log | No change | Command invalid in state |

#### Subcommand 0x00FF - Min/Max Pricing Data

**Usage**: VMC sends pricing range information

**State-Based Responses**:

| Current State | Response | Next State |
|---------------|----------|-----------|
| RESET | ACK (0x0100) | INIT |
| INIT | ACK (0x0100) | INIT |
| Any Other | No response | No change |

---

### 0x0075 - Revalue Limit Request

**Purpose**: Query device revalue capability and limits.

**Subcommands**: None

**State-Based Responses**:

| Current State | Response | Next State | Details |
|---------------|----------|-----------|---------|
| SESSION_IDLE | 0x000F, 0x0001, Revalue_Limit, Checksum | SESSION_IDLE | Response length: 4 |
| Any Other | No response + Error Log | No change | Command invalid in state |

---

### 0x0076 - Vending Request & Session Management

**Purpose**: Handle vending requests, vend completion, and session termination.

**Subcommands**:

#### Subcommand 0x01FF - Vending Request

**Usage**: Device requests vend with item price and ID

**Request Format**: Price (2 bytes) + Item ID (2 bytes)

**State-Based Responses**:

| Current State | Balance Check | Response | Next State | Details |
|---------------|---------------|----------|-----------|---------|
| SESSION_IDLE | Balance ≥ Price | ACK (0x0100) | APPROVE_VEND_REQ | Vend approved |
| SESSION_IDLE | Balance < Price | ACK (0x0100) | DENY_VEND_REQ | Insufficient funds |

#### Subcommand 0x017F - Vend Success Notification

**Usage**: Device confirms vend completed successfully

**State-Based Responses**:

| Current State | Response | Next State | Details |
|---------------|----------|-----------|---------|
| VEND_PROCESS | ACK (0x0100) | SESSION_IDLE | Updates vend status to SUCCESS |

#### Subcommand 0x00BF - Session Complete

**Usage**: Terminate session and return to enabled state

**State-Based Responses**:

| Current State | Response | Next State | Details |
|---------------|----------|-----------|---------|
| SESSION_IDLE | 0x0107 | ENABLED + Wait ACK | End session |
| CANCEL_SESSION | 0x0107 | ENABLED + Wait ACK | Cleanup + Publish item data |

---

## State Machine Architecture

The peripheral operates through a defined state machine with 13 states:

```
INACTIVE
   ↓
RESET ────→ INIT
   ↓          ↓
(0x013B)   0x0074 → DISABLED
   ↓          ↓
   └─────→ 0x01D5 → ENABLED
              ↓
         0x013B (with request) → START_SESSION
              ↓
         SESSION_IDLE ←─────────────────────┐
              ↓  ↑                            │
         0x0076 │                            │
         ↙      ↓                            │
    APPROVE_    DENY_          VEND_PROCESS  │
    VEND_REQ    VEND_REQ ─→ 0x0076 ─→ (0x017F)
    (0x013B)        ↓       SESSION_IDLE──→ SESSION_IDLE
        ↓           └─────→ CANCEL_SESSION
        │                      ↓
        │                   0x0076 (0x00BF)
        │                      ↓
        └──────────────────→ WAIT_ACK
                               ↓
                            ENABLED
```

### State Definitions with Command Handling

| State | Purpose | Valid Commands | Next States |
|-------|---------|-----------------|------------|
| **INACTIVE** | Device offline | 0x01E7 | RESET |
| **RESET** | System resetting | 0x013B, 0x0077 | INIT |
| **INIT** | Initialization phase | 0x013B, 0x0074, 0x0077 | DISABLED, ENABLED |
| **DISABLED** | Device disabled | 0x01D5 | ENABLED |
| **ENABLED** | Ready for card insertion | 0x013B (with ext. request) | START_SESSION, CANCEL_SESSION |
| **START_SESSION** | Card detected, session open | 0x013B | SESSION_IDLE + Wait ACK |
| **SESSION_IDLE** | Ready for transaction | 0x013B, 0x0075, 0x0076 (0x01FF, 0x00BF) | APPROVE_VEND_REQ, DENY_VEND_REQ, CANCEL_SESSION |
| **APPROVE_VEND_REQ** | Vend approved by VMC | 0x013B | VEND_PROCESS + Wait ACK |
| **DENY_VEND_REQ** | Vend rejected (low balance) | 0x013B | CANCEL_SESSION + Wait ACK |
| **VEND_PROCESS** | Vending in progress | 0x013B, 0x0076 (0x017F) | SESSION_IDLE |
| **CANCEL_SESSION** | Session terminating | 0x0076 (0x00BF) | WAIT_ACK, ENABLED |
| **WAIT_ACK** | Waiting for ACK confirmation | (internal use) | DISABLED, ENABLED, SESSION_IDLE |
| **ERROR** | Error condition | 0x01E7 (Reset) | RESET |

---

## Data Structures

### Ring Buffer (UART Reception)

**Purpose**: Circular buffer for storing received MDB data from UART ISR.

**Size**: 40 entries (power of 2 for efficient modulo operation)

**Thread Safety**: ISR writes (wr), task reads (rd)

```c
typedef struct {
    uint16_t buf[MDB_RING_LEN];   // 40-entry power-of-2 buffer for UART RX data
    volatile uint16_t wr;         // Write index (updated by ISR)
    volatile uint16_t rd;         // Read index (updated by task)
} mdb_ring_t;
```

**Usage**: Decouples UART interrupt reception from command processing task.

---

### Command Definition

**Purpose**: Stores complete MDB command data and its response.

**Command Storage**: Up to 5 words (variable length commands)

**Response Storage**: Up to 36 words (for complex multi-element responses)

```c
typedef struct {
    uint16_t CMD[5];               // Incoming command data (max 5 words)
    uint16_t CMD_Length;           // Actual received command length
    uint16_t CMD_Response[36];     // Outgoing response data
    uint16_t CMD_Response_Length;  // Actual response length
} CMD_Type;
```

**Usage**: Each command (0x01E7, 0x013B, etc.) has one CMD_Type instance in global array.

---

### State Manager

**Purpose**: Tracks all state machines during command lifecycle.

**Manages**: Device state, RX state, TX state, processing state, and pending state changes.

```c
typedef struct {
    Peripheral_State_t Cashless_StateHandler;      // Current device state (INACTIVE, ENABLED, etc.)
    CMD_RX_State CMD_RX_StateHandler;              // Reception state (READY, INPROGRESS, DONE)
    CMD_TX_State CMD_TX_StateHandler;              // Transmission state
    CMD_Process_State CMD_Process_StateHandler;    // Processing state
    bool Cashless_State_Change_Request;            // Flag for pending state transition
    Peripheral_State_t Cashless_Req_State;         // Target state for transition
} MDB_StateManager_t;
```

**Usage**: Global instance coordinating all state transitions across RX, processing, and TX phases.

---

## Command Processing Pipeline

The communication flow involves multiple tasks and ISR, coordinated through FreeRTOS task notifications and ring buffers.

### Complete Communication Sequence

#### Step 1: UART Reception (ISR Context)

**Function**: `HAL_UART_RxCpltCallback()` → `MDB_UART_RxCallback()`

```c
void MDB_UART_RxCallback(UART_HandleTypeDef *huart) {
    // 1. Extract 9-bit word from UART (8 data + 1 mode bit)
    uint16_t word = mdb_rx_buf[0] & 0x1FF;
    
    // 2. Store word in ring buffer (ISR writes)
    (void)mdbRing_push(&rxRing, word);
    
    // 3. Notify mdbRxTask that data is available
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(mdbRxTaskHandle, &xHigherPriorityTaskWoken);
    
    // 4. Trigger context switch if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    
    // 5. Re-arm UART for next byte
    HAL_UART_Receive_IT(mdb_config.mdb_uart, (uint8_t *)mdb_rx_buf, 1);
}
```

**Ring Buffer Operation**: Power-of-2 circular buffer (40 entries) allows efficient wrap-around with bit masking.

---

#### Step 2: Reception Task Processing (mdbRxTask)

**Priority**: configMAX_PRIORITIES-3 (High)  
**Stack**: 384 words  
**Trigger**: ISR notification

```c
static void mdbRxTask(void *argument) {
    uint16_t word;
    TickType_t lastCallTick = xTaskGetTickCount();
    MDB_StartUARTReceive();  // Enable UART RX interrupt
    
    for (;;) {
        // 1. Block until ISR notification (ulTaskNotifyTake)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // 2. Check timeout (10ms bus timeout)
        TickType_t nowTick = xTaskGetTickCount();
        if ((nowTick - lastCallTick) > mdb_config.bus_timeout) {
            MDB_BusManager.RXBuffer_index = 0;
            MDB_StateManager.CMD_RX_StateHandler = CMD_RX_READY;
            MDB_StateManager.CMD_Process_StateHandler = CMD_PROCESS_READY;
        }
        lastCallTick = nowTick;
        
        // 3. Pop all available words from ring buffer
        while (mdbRing_pop(&rxRing, &word)) {
            // 4. Process each word
            MDB_ReceiveCommand(word);
        }
    }
}
```

**Timeout Recovery**: If no data received within 10ms, resets reception state to prevent hanging.

---

#### Step 3: Command Reception State Machine (mdbRxTask → mdbRxTask)

**Function**: `MDB_ReceiveCommand(uint16_t word)`

**State Transitions**:

| Phase | State | Action | Result |
|-------|-------|--------|--------|
| **ACK Check** | Any | Call `MDB_ProcessAck(word)` | If ACK (0x0000), handle pending state change |
| **Ready** | CMD_RX_READY | Call `MDB_DetectCommand(word)` | If match found, call `MDB_InitCommandReception()` |
| **Collect** | CMD_RX_INPROGRESS | Store word in buffer | Increment `RXBuffer_index` |
| **Complete** | CMD_RX_INPROGRESS | Call `MDB_IsCommandComplete()` | If complete, call `MDB_CompleteCommandReception()` |

**Key Structures Updated**:
- `MDB_BusManager.MDB_RXbuffer[]` - Stores command data
- `MDB_BusManager.RXBuffer_index` - Tracks buffer position
- `MDB_StateManager.CMD_RX_StateHandler` - State machine

---

#### Step 4: Command Completion & Notification

**Function**: `MDB_CompleteCommandReception(uint8_t cmd_index)`

```c
static void MDB_CompleteCommandReception(uint8_t cmd_index) {
    // 1. Mark command reception as complete
    MDB_StateManager.CMD_RX_StateHandler = CMD_RX_DONE;
    
    // 2. Notify mdbCMDProcessTask with command index
    xTaskNotify(mdbCMDProcessTaskHandle,
                (uint32_t)cmd_index,      // Command index as notification value
                eSetValueWithOverwrite);
    
    // 3. Yield for context switch
    taskYIELD();
}
```

---

#### Step 5: Command Processing Task (mdbCMDProcessTask)

**Priority**: configMAX_PRIORITIES-2 (Very High)  
**Stack**: 384 words  
**Trigger**: Task notification with command index

```c
static void mdbCMDProcessTask(void *argument) {
    uint32_t CMD_Index;
    
    for (;;) {
        // 1. Block until mdbRxTask notifies with command index
        xTaskNotifyWait(0,              // Don't clear bits
                    UINT32_MAX,         // Clear all bits on exit
                    &CMD_Index,         // Receives command index
                    portMAX_DELAY);     // Block forever
        
        // 2. Call command handler
        MDB_HandleCommand(MDB_BusManager.MDB_RXbuffer, CMD_Index);
    }
}
```

---

#### Step 6: Command Handler Execution

**Function**: `MDB_HandleCommand(uint16_t *RxBuffer, uint8_t cmd_index)`

```c
void MDB_HandleCommand(uint16_t *RxBuffer, uint8_t cmd_index) {
    switch (MDB_StateManager.CMD_Process_StateHandler) {
        case CMD_PROCESS_READY:
            if (MDB_StateManager.CMD_RX_StateHandler == CMD_RX_DONE) {
                // 1. Update processing state
                MDB_StateManager.CMD_Process_StateHandler = CMD_PROCESS_INPROGRESS;
                
                // 2. Call command-specific handler
                // command_table[cmd_index].handler(RxBuffer, buffer_length)
                if (cmd_index < VMC_CMD_MAX_NUMBER) {
                    command_table[cmd_index].handler(RxBuffer, cmd_length);
                }
                
                // 3. Reset states for next command
                MDB_StateManager.CMD_RX_StateHandler = CMD_RX_READY;
                MDB_StateManager.CMD_Process_StateHandler = CMD_PROCESS_READY;
                MDB_BusManager.RXBuffer_index = 0;
            }
            break;
    }
}
```

---

#### Step 7: Command-Specific Handler

**Example**: `handle_cmd_0x013B()` (Poll/Status)

```c
static void handle_cmd_0x013B(uint16_t *RxBuffer, uint8_t cmd_length) {
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    
    // 1. Validate command structure
    if (MDB_ValidateCommandStructure(RxBuffer, cmd_length, cmd_index)) {
        
        // 2. Process based on current state
        switch (MDB_StateManager.Cashless_StateHandler) {
            case STATE_ENABLED:
                // Generate response
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;  // ACK
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                break;
            
            case STATE_START_SESSION:
                // Multi-element response
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0003;
                VMC_CMDs[cmd_index].CMD_Response[1] = 0x0000;
                VMC_CMDs[cmd_index].CMD_Response[2] = Peripheral_Balance.Cur_balance;
                // ... more response elements ...
                VMC_CMDs[cmd_index].CMD_Response_Length = 11;
                
                // 3. Request ACK handling
                MDB_RequestAck(1, STATE_SESSION_IDLE);
                break;
        }
        
        // 4. Send response if available
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
            MDB_SendResponseWithModeBit(VMC_CMDs[cmd_index].CMD_Response,
                                       VMC_CMDs[cmd_index].CMD_Response_Length);
        }
    }
}
```

---

#### Step 8: Response Transmission

**Function**: `MDB_SendResponseWithModeBit(uint16_t *data, uint8_t dataLength)`

```c
void MDB_SendResponseWithModeBit(uint16_t *data, uint8_t dataLength) {
    if (mdb_config.mdb_uart != NULL) {
        // Use interrupt-driven transmission
        HAL_UART_Transmit_IT(mdb_config.mdb_uart,
                           (uint8_t *)data,
                           dataLength);
    }
}
```

**Mode Bit**: Handled by hardware UART configuration (9-bit mode).

---

### Task Scheduling Summary

| Task | Priority | Function | Trigger | Stack |
|------|----------|----------|---------|-------|
| **mdbRxTask** | MAX-3 | Reception & buffering | ISR notification | 384 |
| **mdbCMDProcessTask** | MAX-2 | Command processing | Task notification | 384 |
| **espCommunicationTask** | MAX-4 | MQTT publish | Queue or timeout | 600 |
| **espMqttProcessTask** | MAX-5 | Message handling | ISR notification | 512 |

---

## Response Formats

### ACK Response
```
ACK = 0x0100
```
Used for simple acknowledgment of command receipt and processing.

### Status Response
Varies by command type and peripheral state. Returns multi-byte status information during active sessions.

### Example: Poll Response in Active Session
```c
Response[0] = 0x0003  // Status bits
Response[1] = 0x0000  // Reserved
Response[2] = 0x00B9  // Credit (MSB)
Response[3] = ...     // Additional status bytes
Response_Length = 11
```

---

## Vending Request Handling

### Vending Item Data Structure

```c
typedef struct {
    uint8_t Req_Item_Price_Hbyte;
    uint8_t Req_Item_Price_Lbyte;
    uint8_t Req_Item_ID_Hbyte;
    uint8_t Req_Item_ID_Lbyte;
    uint8_t Res_Item_Price_Hbyte;
    uint8_t Res_Item_Price_Lbyte;
    vend_item_state_t Vend_Item_State;  // SUCCESS or FAILURE
} Vending_Item_Data_t;
```

### Vending Flow (Simulated Cashless Device)

In this implementation, the STM32F4 simulates a cashless device that receives vend requests from the VMC and performs validation:

1. **VMC sends vend request** (Command 0x0076, Subcommand 0x01FF)
   - Contains item price and item ID

2. **Simulated Cashless Device validates**
   - Extracts price and ID from request via `MDB_UpdateVendingItemData()`
   - Validates balance against requested price
   - If balance ≥ price: Transition to APPROVE_VEND_REQ state
   - If balance < price: Transition to DENY_VEND_REQ state

3. **Cashless Device responds**
   - STATE_APPROVE_VEND_REQ: Sends 0x0005 (vend approved) + approved price + checksum
   - STATE_DENY_VEND_REQ: Sends 0x0006 (vend denied)
   - State machine transitions to VEND_PROCESS or CANCEL_SESSION

4. **VMC confirms vend completion**
   - Sends Command 0x0076 with Subcommand 0x017F (vend success)

5. **Cashless Device completes**
   - Updates vend item status to SUCCESS via `Vending_Item_Data.Vend_Item_State`
   - Transitions back to SESSION_IDLE
   - Ready for next transaction

---

## Error Handling and Recovery

### Timeout Management
- **Inter-Character Timeout**: 10ms
- **Recovery**: Automatic state reset to INACTIVE
- **Persistent Errors**: Logged and reported

### Invalid Commands
- Unrecognized commands are silently ignored
- Malformed data triggers state reset
- Error state logs occurrence for diagnostics

---

## Debugging and Monitoring

### Available Logging
- SYS_Logger integration for protocol traces
- ESP8266 Handler for remote diagnostics (To be Done)

---
