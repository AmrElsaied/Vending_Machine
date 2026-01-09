# API Reference

## MDB_Handler Module

### Core Functions

#### `void MDB_BusInit(void)`
**Description**: Initializes the MDB bus communication system
**Parameters**: None
**Returns**: None
**Usage**: Call once during system initialization
```c
// Initialize MDB bus during startup
MDB_BusInit();
```

#### `void MDB_ReceiveCommand(uint16_t word)`
**Description**: Processes incoming MDB data word
**Parameters**: 
- `word` - 16-bit word containing 9-bit MDB data + mode bit
**Returns**: None
**Usage**: Called by mdbRx Task for each received word

#### `void MDB_HandleCommand(uint16_t *RxBuffer, uint8_t cmd_index)`
**Description**: Processes complete MDB command
**Parameters**:
- `RxBuffer` - Pointer to received command buffer
- `cmd_index` - Index of the command in command table
**Returns**: None
**Usage**: Called by command processing task

#### `void MDB_SendResponseWithModeBit(uint16_t *data, uint8_t dataLength)`
**Description**: Transmits response data with proper mode bit handling
**Parameters**:
- `data` - Pointer to response data array
- `dataLength` - Number of bytes to transmit
**Returns**: None

### Ring Buffer Functions

#### `void mdbRing_init(mdb_ring_t *r)`
**Description**: Initializes ring buffer structure
**Parameters**:
- `r` - Pointer to ring buffer structure
**Returns**: None

#### `bool mdbRing_push(mdb_ring_t *r, uint16_t word)`
**Description**: Adds word to ring buffer
**Parameters**:
- `r` - Pointer to ring buffer structure
- `word` - 16-bit word to add
**Returns**: `true` if successful, `false` if buffer full

#### `bool mdbRing_pop(mdb_ring_t *r, uint16_t *word)`
**Description**: Retrieves word from ring buffer
**Parameters**:
- `r` - Pointer to ring buffer structure
- `word` - Pointer to store retrieved word
**Returns**: `true` if word retrieved, `false` if buffer empty

### Command Handler Functions

#### `static void handle_cmd_0x01E7(uint16_t *RxBuffer, uint8_t cmd_length)`
**Description**: Handles reset/restart command
**Parameters**:
- `RxBuffer` - Received command buffer
- `cmd_length` - Length of received command
**State Transitions**: RESTART → INIT

#### `static void handle_cmd_0x013B(uint16_t *RxBuffer, uint8_t cmd_length)`
**Description**: Handles poll/status command
**Parameters**:
- `RxBuffer` - Received command buffer
- `cmd_length` - Length of received command
**Responses**: State-dependent responses

#### `static void handle_cmd_0x01D5(uint16_t *RxBuffer, uint8_t cmd_length)`
**Description**: Handles capabilities request
**Parameters**:
- `RxBuffer` - Received command buffer
- `cmd_length` - Length of received command

#### `static void handle_cmd_0x0074(uint16_t *RxBuffer, uint8_t cmd_length)`
**Description**: Handles setup/configuration command
**Parameters**:
- `RxBuffer` - Received command buffer (33 bytes)
- `cmd_length` - Length of received command

#### `static void handle_cmd_0x0077(uint16_t *RxBuffer, uint8_t cmd_length)`
**Description**: Handles reader enable/disable command
**Parameters**:
- `RxBuffer` - Received command buffer
- `cmd_length` - Length of received command
**Sub-commands**: 0x01F9 (enable), 0x00FF (disable)

#### `static void handle_cmd_0x0075(uint16_t *RxBuffer, uint8_t cmd_length)`
**Description**: Handles card type enable command
**Parameters**:
- `RxBuffer` - Received command buffer
- `cmd_length` - Length of received command

#### `static void handle_cmd_0x0076(uint16_t *RxBuffer, uint8_t cmd_length)`
**Description**: Handles session complete command
**Parameters**:
- `RxBuffer` - Received command buffer
- `cmd_length` - Length of received command

## System_Tasks Module

### Task Management Functions

#### `void System_TaskCreate(void)`
**Description**: Creates all system tasks
**Parameters**: None
**Returns**: None
**Tasks Created**:
- `mdbRxTask` - Priority: configMAX_PRIORITIES-3
- `mdbCMDProcessTask` - Priority: configMAX_PRIORITIES-2

### Task Functions

#### `static void mdbRxTask(void *argument)`
**Description**: Handles MDB data reception and timeout management
**Parameters**:
- `argument` - Task parameter (unused)
**Features**:
- Ring buffer processing
- 10ms timeout detection
- Automatic state reset on timeout

#### `static void mdbCMDProcessTask(void *argument)`
**Description**: Processes received MDB commands
**Parameters**:
- `argument` - Task parameter (unused)
**Features**:
- Command validation
- Handler dispatch
- Response generation

## Data Structures

### `mdb_ring_t`
```c
typedef struct {
    uint16_t buf[MDB_RING_LEN];  // Buffer array
    volatile uint16_t wr;        // Write index
    volatile uint16_t rd;        // Read index
} mdb_ring_t;
```

### `MDB_StateManager_t`
```c
typedef struct {
    Peripheral_State_t Cashless_StateHnadler;
    CMD_RX_State_t CMD_RX_StateHandler;
    CMD_TX_State_t CMD_TX_StateHandler;
    CMD_Process_State_t CMD_Process_StateHandler;
} MDB_StateManager_t;
```

### `MDB_BusManager_t`
```c
typedef struct {
    uint16_t MDB_RXbuffer[MAX_BUFFER_SIZE];
    uint8_t RXBuffer_index;
    uint8_t MDB_RX_CMD_Index;
    uint8_t MDB_TX_CMD_Index;
    uint8_t MDB_Process_CMD_Index;
} MDB_BusManager_t;
```

### `CommandEntry_t`
```c
typedef struct {
    uint8_t CMD_Index;
    CmdHandlerFn handler;
} CommandEntry_t;
```

## Constants and Macros

### MDB Configuration
```c
#define MDB_RING_LEN 256U        // Ring buffer size
#define MDB_BUS_TIMEOUT 10       // Bus timeout (ticks)
```

### State Enumerations

#### `Peripheral_State_t`
- `STATE_DISABLED` - Device disabled
- `STATE_IDLE` - Ready for operation
- `STATE_ACTIVE` - Device active
- `STATE_INIT` - Initializing
- `STATE_RESTART` - Restarting
- `STATE_START_SESSION` - Card inserted
- `STATE_CANCEL_SESSION` - Card removed
- `STATE_VEND_REQ` - Vend request
- `STATE_VEND_PROCESS` - Vending in progress
- `STATE_ERROR` - Error state

#### `CMD_RX_State_t`
- `CMD_RX_READY` - Ready to receive
- `CMD_RX_INPROGRESS` - Receiving data
- `CMD_RX_DONE` - Reception complete
- `CMD_RX_BUSY` - Processing

#### `CMD_Process_State_t`
- `CMD_PROCESS_READY` - Ready to process
- `CMD_PROCESS_INPROGRESS` - Processing command
- `CMD_PROCESS_DONE` - Processing complete

## Global Variables

### External Variables
```c
extern TaskHandle_t mdbRxTaskHandle;
extern TaskHandle_t mdbCMDProcessTaskHandle;
extern mdb_ring_t rxRing;
extern MDB_StateManager_t MDB_StateManager;
extern MDB_BusManager_t MDB_BusManager;
```

## Usage Examples

### Basic Initialization
```c
// System initialization
MDB_BusInit();
System_TaskCreate();
vTaskStartScheduler();
```

### Custom Command Handler
```c
static void handle_custom_command(uint16_t *RxBuffer, uint8_t cmd_length) {
    uint8_t cmd_index = MDB_BusManager.MDB_RX_CMD_Index;
    
    // Validate command
    if (RxBuffer[0] == VMC_CMDs[cmd_index].CMD[0]) {
        // Process command based on state
        switch (MDB_StateManager.Cashless_StateHnadler) {
            case STATE_IDLE:
                // Handle in idle state
                VMC_CMDs[cmd_index].CMD_Response[0] = 0x0100;
                VMC_CMDs[cmd_index].CMD_Response_Length = 1;
                break;
            default:
                VMC_CMDs[cmd_index].CMD_Response_Length = 0;
                break;
        }
        
        // Send response if available
        if (VMC_CMDs[cmd_index].CMD_Response_Length > 0) {
            MDB_SendResponseWithModeBit(
                VMC_CMDs[cmd_index].CMD_Response,
                VMC_CMDs[cmd_index].CMD_Response_Length
            );
        }
    }
}
```

### Ring Buffer Usage
```c
uint16_t word;

// In ISR - add data
if (mdbRing_push(&rxRing, received_word)) {
    // Successfully added
    vTaskNotifyGiveFromISR(mdbRxTaskHandle, &higherPriorityTaskWoken);
}

// In task - retrieve data
while (mdbRing_pop(&rxRing, &word)) {
    MDB_ReceiveCommand(word);
}
```
