# State Machine Documentation

## Overview
The VMC system uses multiple state machines to manage different aspects of operation. This document describes each state machine and their interactions.

## Cashless State Machine

### States Overview
The cashless device operates through the following states:

```
STATE_RESTART → STATE_INIT → STATE_IDLE → STATE_INSERT_CARD → STATE_VEND_REQ → STATE_REMOVE_CARD
     ↑              ↓            ↑               ↓                    ↓              ↓
     ←──────────────←─────────────←───────────────←────────────────────←──────────────←
```

### State Definitions

#### STATE_RESTART
**Purpose**: Initial system state after power-on or reset
**Entry Conditions**:
- System power-on
- Software reset command
- Error recovery

**Behavior**:
- Accept only reset/initialization commands
- Limited response set
- Prepare for initialization sequence

**Exit Conditions**:
- Successful reset command (0x01E7) → STATE_INIT

**Commands Accepted**:
- 0x01E7 (Reset) → Response: ACK, Transition to STATE_INIT

#### STATE_INIT
**Purpose**: Device initialization and capability negotiation
**Entry Conditions**:
- From STATE_RESTART after reset command
- Initialization sequence start

**Behavior**:
- Accept setup and configuration commands
- Negotiate capabilities with VMC
- Prepare device for normal operation

**Exit Conditions**:
- Successful setup completion → STATE_IDLE

**Commands Accepted**:
- 0x013B (Poll) → Response: ACK
- 0x0074 (Setup) → Response: Config data, Transition to STATE_IDLE
- 0x0077 (Reader Enable) → Response: Status data

#### STATE_IDLE
**Purpose**: Ready for card insertion, normal polling state
**Entry Conditions**:
- From STATE_INIT after successful setup
- From STATE_REMOVE_CARD after card removal
- From STATE_INSERT_CARD after session timeout

**Behavior**:
- Monitor for card insertion
- Respond to polling commands
- Ready for transaction start

**Exit Conditions**:
- Card insertion detected → STATE_INSERT_CARD

**Commands Accepted**:
- 0x013B (Poll) → Response: ACK
- 0x01D5 (Capabilities) → Response: Capability data
- 0x0075 (Card Type Enable) → Response: Status

#### STATE_INSERT_CARD
**Purpose**: Card detected, ready for transaction
**Entry Conditions**:
- Card insertion detected (GPIO signal)
- From STATE_IDLE

**Behavior**:
- Indicate card presence to VMC
- Provide transaction status
- Ready for vending requests

**Exit Conditions**:
- Vending request → STATE_VEND_REQ
- Card removal → STATE_REMOVE_CARD
- Session timeout → STATE_IDLE

**Commands Accepted**:
- 0x013B (Poll) → Response: Extended status (17 bytes)
- 0x0075 (Card Type Enable) → Response: Card present status

**Response Data (0x013B)**:
```c
VMC_CMDs[cmd_index].CMD_Response[0] = 0x0003;  // Card present status
VMC_CMDs[cmd_index].CMD_Response[1] = 0x0000;  // Reserved
VMC_CMDs[cmd_index].CMD_Response[2] = 0x00B9;  // Credit amount MSB
// ... additional status bytes (total 11 elements)
```

#### STATE_VEND_REQ
**Purpose**: Vending operation in progress
**Entry Conditions**:
- Vending request from VMC
- Sufficient credit available

**Behavior**:
- Process vending request
- Monitor vending operation
- Handle vending completion

**Exit Conditions**:
- Vending complete → STATE_INSERT_CARD
- Vending failure → STATE_ERROR
- Session complete → STATE_REMOVE_CARD

**Commands Accepted**:
- 0x013B (Poll) → Response: Vending status
- 0x0076 (Session Complete) → Response: ACK

#### STATE_REMOVE_CARD
**Purpose**: Card removal process
**Entry Conditions**:
- Card removal detected (GPIO signal)
- Session completion
- Timeout or error conditions

**Behavior**:
- Signal card removal to VMC
- Clean up session data
- Prepare for next transaction

**Exit Conditions**:
- Card physically removed → STATE_IDLE
- Timeout → STATE_IDLE

**Commands Accepted**:
- 0x013B (Poll) → Response: No response (length = 0)
- 0x0076 (Session Complete) → Response: ACK

#### STATE_ERROR
**Purpose**: Error handling and recovery
**Entry Conditions**:
- Communication errors
- Hardware failures
- Invalid command sequences

**Behavior**:
- Report error status
- Attempt error recovery
- Prevent unsafe operations

**Exit Conditions**:
- Error cleared → Previous valid state
- Reset command → STATE_RESTART

**Commands Accepted**:
- 0x01E7 (Reset) → Response: ACK, Transition to STATE_RESTART
- 0x013B (Poll) → Response: Error status

## Command Processing State Machine

### States Overview

#### CMD_RX_READY
**Purpose**: Ready to receive new command
**Behavior**:
- Monitor for command start (mode bit = 1)
- Initialize reception buffers
- Identify command type

**Transition**: Command detected → CMD_RX_INPROGRESS

#### CMD_RX_INPROGRESS
**Purpose**: Receiving command data
**Behavior**:
- Collect command bytes
- Validate command structure
- Check for completion conditions

**Transition**: Command complete → CMD_RX_DONE

#### CMD_RX_DONE
**Purpose**: Command reception complete
**Behavior**:
- Notify processing task
- Prepare for command processing
- Set processing state

**Transition**: Processing started → CMD_RX_BUSY

#### CMD_RX_BUSY
**Purpose**: Command being processed
**Behavior**:
- Wait for processing completion
- Prevent new command reception
- Maintain state consistency

**Transition**: Processing complete → CMD_RX_READY

## Command Transmission State Machine

### States Overview

#### CMD_TX_READY
**Purpose**: Ready to transmit response
**Behavior**:
- Response data prepared
- Transmission buffer available
- UART ready for transmission

#### CMD_TX_INPROGRESS
**Purpose**: Transmitting response data
**Behavior**:
- Send response bytes
- Handle transmission interrupts
- Monitor transmission completion

#### CMD_TX_DONE
**Purpose**: Transmission complete
**Behavior**:
- Verify transmission success
- Clean up transmission buffers
- Return to ready state

## State Transition Triggers

### GPIO-Based Triggers
```c
// Card insertion detection
if (HAL_GPIO_ReadPin(VENDING_GPIO_Port, VENDING_Pin) == GPIO_PIN_RESET && Vending_EN == false) {
    MDB_StateManager.Cashless_StateHnadler = STATE_INSERT_CARD;
    Vending_EN = true;
}

// Card removal detection
if (HAL_GPIO_ReadPin(VENDING_GPIO_Port, VENDING_Pin) == GPIO_PIN_SET && Vending_EN == true) {
    MDB_StateManager.Cashless_StateHnadler = STATE_REMOVE_CARD;
    Vending_EN = false;
}
```

### Command-Based Triggers
- Reset command (0x01E7) always causes transition to STATE_INIT
- Setup command (0x0074) transitions to STATE_IDLE
- Session complete (0x0076) may transition to STATE_REMOVE_CARD

### Timeout-Based Triggers
```c
// Bus timeout handling
TickType_t nowTick = xTaskGetTickCount();
if ((nowTick - lastCallTick) > MDB_BUS_TIMEOUT) {
    MDB_BusManager.RXBuffer_index = 0;
    MDB_StateManager.CMD_RX_StateHandler = CMD_RX_READY;
    MDB_StateManager.CMD_Process_StateHandler = CMD_PROCESS_READY;
}
```

## State Machine Interactions

### Inter-State Dependencies
1. **Cashless State → Command Response**: Response content depends on cashless state
2. **Command Processing → Cashless State**: Some commands trigger state changes
3. **GPIO State → Cashless State**: Hardware signals drive state transitions

### Synchronization
- State changes are atomic operations
- Critical sections protect state variables
- FreeRTOS notifications coordinate between tasks

## Error Handling in State Machines

### Invalid Transitions
- Log error and maintain current state
- Respond with appropriate error codes
- Attempt graceful recovery

### Timeout Handling
- Automatic state reset on communication timeout
- Buffer cleanup on timeout
- Return to safe state (typically IDLE)

### Hardware Failures
- Transition to ERROR state
- Disable unsafe operations
- Require explicit reset for recovery

## State Machine Validation

### State Consistency Checks
```c
// Validate state before transitions
bool isValidTransition(Peripheral_State_t from, Peripheral_State_t to) {
    switch (from) {
        case STATE_RESTART:
            return (to == STATE_INIT);
        case STATE_INIT:
            return (to == STATE_IDLE || to == STATE_ERROR);
        case STATE_IDLE:
            return (to == STATE_INSERT_CARD || to == STATE_ERROR);
        // ... other cases
        default:
            return false;
    }
}
```

### Debug and Monitoring
```c
// State transition logging
void logStateTransition(Peripheral_State_t from, Peripheral_State_t to) {
    DEBUG_PRINT("State transition: %d -> %d\n", from, to);
}
```

## Performance Considerations

### State Transition Timing
- State changes: < 100μs
- Command processing: < 5ms
- Response generation: < 2ms

### Memory Usage
- State variables: < 100 bytes
- Transition history: Optional debugging feature
- State machine tables: < 1KB

## Future Enhancements

### Planned Features
1. **State History**: Track recent state transitions
2. **Conditional Transitions**: More complex transition logic
3. **Sub-states**: Hierarchical state management
4. **State Persistence**: Save critical states to non-volatile memory
