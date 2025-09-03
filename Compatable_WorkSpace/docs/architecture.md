# Architecture Overview

## System Architecture

The Vending Machine Controller (VMC) is built with a layered architecture that separates concerns and provides clear interfaces between components.

## High-Level Architecture

```
┌─────────────────────────────────────────────────┐
│                Application Layer                 │
├─────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌───────────┐ │
│  │ MDB_Handler │  │System_Tasks │  │VMC_Config │ │
│  └─────────────┘  └─────────────┘  └───────────┘ │
├─────────────────────────────────────────────────┤
│                  FreeRTOS Layer                  │
├─────────────────────────────────────────────────┤
│               STM32 HAL Layer                    │
├─────────────────────────────────────────────────┤
│                Hardware Layer                    │
└─────────────────────────────────────────────────┘
```

## Core Components

### 1. MDB_Handler Module
**Purpose**: Manages Multi-Drop Bus protocol communication
**Responsibilities**:
- UART communication with MDB devices
- Command parsing and response generation
- Ring buffer management for efficient data handling
- State-based command processing

**Key Files**:
- `MDB_Handler.h/c` - Core MDB functionality
- Ring buffer implementation for UART data

### 2. System_Tasks Module
**Purpose**: FreeRTOS task management and coordination
**Responsibilities**:
- Task creation and scheduling
- Inter-task communication
- System timing and synchronization

**Key Tasks**:
- `mdbRxTask` - Handles incoming MDB data
- `mdbCMDProcessTask` - Processes MDB commands

### 3. VMC_Config Module
**Purpose**: Configuration and command definitions
**Responsibilities**:
- MDB command definitions
- System configuration parameters
- Command-to-handler mapping

## Data Flow

### 1. MDB Communication Flow
```
MDB Device → UART ISR → Ring Buffer → mdbRxTask → Command Processing → Response
```

### 2. Command Processing Flow
```
Received Data → Command Parser → State Machine → Handler Function → Response Generation
```

## State Management

The system uses multiple state machines:

### 1. Cashless State Machine
- `STATE_RESTART` - Initial state
- `STATE_INIT` - Initialization phase
- `STATE_IDLE` - Ready for operation
- `STATE_INSERT_CARD` - Card detected
- `STATE_REMOVE_CARD` - Card removal
- `STATE_VEND_REQ` - Vending request
- `STATE_ERROR` - Error handling

### 2. Command Processing States
- `CMD_RX_READY` - Ready to receive
- `CMD_RX_INPROGRESS` - Receiving data
- `CMD_RX_DONE` - Reception complete
- `CMD_PROCESS_READY` - Ready to process

## Memory Management

### Ring Buffer Design
- **Size**: 256 entries (power of 2 for efficient wrapping)
- **Type**: 16-bit words to handle MDB 9-bit data + mode bit
- **Access**: Single producer (ISR), single consumer (task)

### Task Stack Allocation
- `mdbRxTask`: 384 words
- `mdbCMDProcessTask`: 256 words

## Error Handling

### Timeout Management
- **MDB Bus Timeout**: 10ms maximum between communications
- **Automatic Recovery**: Reset buffers and states on timeout

### Error Recovery
- Command validation before processing
- State machine reset on invalid sequences
- Buffer overflow protection

## Performance Considerations

### Real-time Requirements
- **ISR Response**: < 1ms for UART handling
- **Command Processing**: < 5ms typical
- **State Transitions**: Immediate (< 100μs)

### Resource Usage
- **RAM**: ~2KB for buffers and state management
- **Flash**: ~16KB for application code
- **CPU**: ~10% utilization under normal load

## Interfaces

### External Interfaces
1. **MDB Bus** (UART1)
   - 9600 baud, 9-bit mode
   - Address/data mode bit handling

2. **USB CDC** (Virtual COM Port)
   - Configuration interface
   - Debug output

3. **GPIO**
   - Vending control signals
   - Status indicators

### Internal Interfaces
1. **Task Notification** - FreeRTOS task communication
2. **Shared Memory** - State and buffer access
3. **Function Callbacks** - Command handler dispatch

## Extensibility

The architecture supports easy extension through:

1. **New Command Handlers**: Add to command table
2. **Additional States**: Extend state enumerations
3. **New Peripherals**: Add handler modules
4. **Enhanced Communication**: Additional UART/SPI interfaces

## Security Considerations

1. **Input Validation**: All MDB commands validated
2. **Buffer Bounds**: Ring buffer overflow protection
3. **State Consistency**: Atomic state transitions
4. **Timeout Protection**: Prevents hanging states
