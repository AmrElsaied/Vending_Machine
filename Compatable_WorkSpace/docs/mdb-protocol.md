# MDB Protocol Implementation

## Overview
The Multi-Drop Bus (MDB) protocol is implemented to communicate with cashless payment devices and other vending machine peripherals. This document describes the protocol implementation details.

## MDB Protocol Basics

### Physical Layer
- **Baud Rate**: 9600 bps
- **Data Format**: 9 bits (8 data + 1 mode bit)
- **Mode Bit**: 
  - 1 = Address/Command
  - 0 = Data
- **Voltage Levels**: 5V/12V compatible

### Frame Structure
```
┌─────────────────────────────────────────────┐
│ Mode Bit │ 8-bit Data │ Optional Checksum   │
└─────────────────────────────────────────────┘
```

## Implemented Commands

### 1. Command 0x01E7 - Reset/Restart
**Purpose**: Initializes cashless device
**State Handling**: Transitions from RESTART to INIT
**Response**: ACK (0x0100)

### 2. Command 0x013B - Poll/Status
**Purpose**: Regular polling for device status
**State Handling**: Different responses based on current state
**Responses**:
- INIT/IDLE: ACK (0x0100)
- INSERT_CARD: Extended status data (17 bytes)

### 3. Command 0x01D5 - Capabilities Request
**Purpose**: Requests device capabilities
**Response**: Device capability information

### 4. Command 0x0074 - Setup/Configuration
**Purpose**: Device setup and configuration
**Length**: 33 bytes
**Response**: Configuration acknowledgment

### 5. Command 0x0077 - Reader Enable/Disable
**Purpose**: Enable/disable card reader functionality
**Sub-commands**:
- 0x01F9: Enable with parameters
- 0x00FF: Disable

### 6. Command 0x0075 - Card Type Enable
**Purpose**: Enable specific card types
**Response**: Status acknowledgment

### 7. Command 0x0076 - Session Complete
**Purpose**: End payment session
**Response**: Session termination acknowledgment

## Command Processing Flow

### 1. Command Reception
```c
void MDB_ReceiveCommand(uint16_t word)
{
    switch (MDB_StateManager.CMD_RX_StateHandler)
    {
        case CMD_RX_READY:
            // Look for command start
            break;
        case CMD_RX_INPROGRESS:
            // Collect command data
            break;
    }
}
```

### 2. Command Validation
- Verify command structure
- Check data length
- Validate checksum (if present)

### 3. Command Dispatch
```c
const CommandEntry_t command_table[] = {
    {VMC_CMD_0x01E7, handle_cmd_0x01E7},
    {VMC_CMD_0x013B, handle_cmd_0x013B},
    // ... other commands
};
```

## State-Based Response Generation

### Cashless State Machine
The response to commands depends on the current cashless state:

#### STATE_RESTART
- Limited command set accepted
- Focus on initialization commands

#### STATE_INIT
- Setup and configuration commands
- Capability negotiation

#### STATE_IDLE
- Ready for card insertion
- Standard polling responses

#### STATE_INSERT_CARD
- Card detected responses
- Extended status information
- Vending preparation

#### STATE_REMOVE_CARD
- Card removal handling
- Session cleanup

## Data Structures

### Command Definition
```c
typedef struct {
    uint8_t CMD_Index;
    CmdHandlerFn handler;
} CommandEntry_t;
```

### Ring Buffer for UART Data
```c
typedef struct {
    uint16_t buf[MDB_RING_LEN];
    volatile uint16_t wr;
    volatile uint16_t rd;
} mdb_ring_t;
```

### State Managers
```c
typedef struct {
    Peripheral_State_t Cashless_StateHnadler;
    CMD_RX_State_t CMD_RX_StateHandler;
    CMD_TX_State_t CMD_TX_StateHandler;
    CMD_Process_State_t CMD_Process_StateHandler;
} MDB_StateManager_t;
```

## Error Handling

### Timeout Management
- **Bus Timeout**: 10ms maximum between characters
- **Command Timeout**: Reset state on timeout
- **Recovery**: Automatic state reset

### Invalid Command Handling
- Unrecognized commands ignored
- Malformed data rejected
- State consistency maintained

## Response Generation

### Response Types
1. **ACK (0x0100)**: Simple acknowledgment
2. **NAK**: Not implemented (typically ignore)
3. **Data Response**: Multi-byte response with status

### Example Response (INSERT_CARD state)
```c
VMC_CMDs[cmd_index].CMD_Response[0] = 0x0003;  // Status
VMC_CMDs[cmd_index].CMD_Response[1] = 0x0000;  // Reserved
VMC_CMDs[cmd_index].CMD_Response[2] = 0x00B9;  // Credit MSB
// ... additional status bytes
VMC_CMDs[cmd_index].CMD_Response_Length = 11;
```

## Timing Requirements

### Critical Timing
- **Inter-character**: < 5ms
- **Response Time**: < 5ms after command complete
- **Bus Idle**: > 100ms for bus reset

### Implementation Details
```c
#define MDB_BUS_TIMEOUT 10  // 10ms timeout in ticks
```

## Debug and Monitoring

### Debug Features
- Command logging capability
- State transition tracking
- Error counting and reporting

### Monitoring Points
- Command reception rate
- Response generation time
- State machine transitions
- Buffer utilization

## Testing and Validation

### Test Cases
1. **Command Reception**: All supported commands
2. **State Transitions**: Valid state sequences
3. **Error Recovery**: Timeout and invalid data
4. **Performance**: Response timing validation

### Compliance
- MDB specification compliance
- Electrical characteristics
- Protocol timing requirements

## Future Enhancements

### Planned Features
1. **Additional Commands**: Extended MDB command set
2. **Enhanced Error Handling**: More sophisticated recovery
3. **Performance Optimization**: Reduced response times
4. **Security Features**: Command authentication

### Extensibility
- Modular command handler design
- Easy addition of new commands
- Configurable response behaviors
