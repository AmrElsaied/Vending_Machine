# Coding Standards

## Overview
This document defines the coding standards and conventions used in the VMC project to ensure consistency, readability, and maintainability.

## File Organization

### Header File Structure
```c
/*******************************************************************************
 * @file    filename.h
 * @author  Author Name
 * @brief   Brief description of the file
 *
 * @details
 * Detailed description of the file's purpose and functionality.
 *
 * @version X.Y
 * @date    YYYY-MM-DD
 ******************************************************************************/

#ifndef FILENAME_H_
#define FILENAME_H_

/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/

/******************************************************************************
 *                             Module Config                                  *
 ******************************************************************************/

/******************************************************************************
 *                          Macros & Constants                                *
 ******************************************************************************/

/******************************************************************************
 *                          Type Definitions                                  *
 ******************************************************************************/

/******************************************************************************
 *                     Global Variables (extern)                              *
 ******************************************************************************/

/******************************************************************************
 *                        Function Declarations                               *
 ******************************************************************************/

#endif /* FILENAME_H_ */
```

### Source File Structure
```c
/*******************************************************************************
 * @file    filename.c
 * @author  Author Name
 * @brief   Brief description of the file
 *
 * @details
 * Detailed description of the file's implementation.
 *
 * @version X.Y
 * @date    YYYY-MM-DD
 ******************************************************************************/

/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/

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
 *                           Public Variables                                 *
 ******************************************************************************/

/******************************************************************************
 *                         Private Prototypes                                 *
 ******************************************************************************/

/******************************************************************************
 *                          Public Functions                                  *
 ******************************************************************************/

/******************************************************************************
 *                          Private Functions                                 *
 ******************************************************************************/
```

## Naming Conventions

### Variables
```c
// Global variables: PascalCase
extern TaskHandle_t mdbRxTaskHandle;
extern MDB_StateManager_t MDB_StateManager;

// Local variables: camelCase
uint16_t bufferIndex;
bool isValidCommand;

// Constants: UPPER_CASE_WITH_UNDERSCORES
#define MDB_RING_LEN 256U
#define MAX_COMMAND_LENGTH 64

// Private/static variables: camelCase with prefix
static uint16_t rxBufferIndex;
static bool commandReceived;
```

### Functions
```c
// Public functions: Module_FunctionName
void MDB_BusInit(void);
bool MDB_SendCommand(uint16_t *data, uint8_t length);

// Private functions: camelCase
static void processReceivedData(uint16_t word);
static bool validateCommand(uint16_t *buffer);

// Callback functions: HAL convention
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
```

### Types and Structures
```c
// Typedefs: PascalCase with _t suffix
typedef struct {
    uint16_t cmd;
    uint8_t length;
} CommandData_t;

// Enums: UPPER_CASE with descriptive prefix
typedef enum {
    STATE_RESTART,
    STATE_INIT,
    STATE_IDLE,
    STATE_ERROR
} Peripheral_State_t;

// Structure members: camelCase
typedef struct {
    uint16_t rxBuffer[MAX_BUFFER_SIZE];
    uint8_t bufferIndex;
    bool commandReady;
} MDB_BusManager_t;
```

## Code Formatting

### Indentation and Spacing
```c
// Use 4 spaces for indentation (no tabs)
if (condition) {
    // 4-space indent
    function_call();
    if (nested_condition) {
        // 8-space indent
        nested_function();
    }
}

// Space after keywords and around operators
if (a == b) {
    result = x + y * z;
}

// No space after function names
function_name(parameter1, parameter2);
```

### Braces Placement
```c
// Opening brace on same line for functions and control structures
void function_name(void) {
    if (condition) {
        // code
    } else {
        // code
    }
    
    for (int i = 0; i < count; i++) {
        // code
    }
}

// Exception: struct/enum definitions
typedef struct {
    uint16_t data;
    uint8_t length;
} Structure_t;
```

### Line Length and Wrapping
```c
// Maximum line length: 100 characters
// Wrap long lines for readability

// Long function parameters
void long_function_name(uint16_t parameter1, uint8_t parameter2, 
                       bool parameter3, uint32_t parameter4);

// Long expressions
if ((condition1 && condition2) || 
    (condition3 && condition4)) {
    // code
}

// Long assignments
very_long_variable_name = calculate_complex_value(parameter1, parameter2,
                                                 parameter3, parameter4);
```

## Documentation Standards

### Function Documentation
```c
/**
 * @brief Brief description of the function
 * 
 * @details Detailed description of what the function does,
 *          including any important implementation details.
 * 
 * @param parameter1 Description of parameter1
 * @param parameter2 Description of parameter2
 * @return Description of return value
 * 
 * @note Any important notes about usage
 * @warning Any warnings about potential issues
 * 
 * @example
 * // Example usage
 * result = function_name(value1, value2);
 */
ReturnType_t function_name(ParameterType_t parameter1, uint8_t parameter2);
```

### Inline Comments
```c
// Single-line comments for brief explanations
uint16_t word = mdb_rx_buf[0] & 0x1FF;  // Extract 9-bit data

/* Multi-line comments for detailed explanations
 * This section handles the complex state transition
 * logic that depends on multiple conditions.
 */
if (complex_condition) {
    // Handle complex case
}
```

### Variable Documentation
```c
// Document complex or non-obvious variables
uint16_t ringBuffer[MDB_RING_LEN];    /**< Circular buffer for MDB data */
volatile uint16_t writeIndex;         /**< Write index (modified by ISR) */
TaskHandle_t mdbRxTaskHandle;         /**< Handle for MDB reception task */
```

## Error Handling

### Return Value Conventions
```c
// Use bool for success/failure
bool MDB_InitializeDevice(void) {
    if (initialization_failed) {
        return false;  // Failure
    }
    return true;      // Success
}

// Use specific error codes for detailed errors
typedef enum {
    MDB_OK = 0,
    MDB_ERROR_TIMEOUT,
    MDB_ERROR_INVALID_PARAM,
    MDB_ERROR_BUFFER_FULL
} MDB_Status_t;
```

### Assertion Usage
```c
// Use assertions for debugging
#ifdef DEBUG
    #define ASSERT(condition) assert(condition)
#else
    #define ASSERT(condition)
#endif

// Example usage
ASSERT(buffer != NULL);
ASSERT(length > 0);
```

### Error Logging
```c
// Consistent error reporting
#define LOG_ERROR(msg, ...) printf("ERROR: " msg "\n", ##__VA_ARGS__)
#define LOG_WARNING(msg, ...) printf("WARNING: " msg "\n", ##__VA_ARGS__)
#define LOG_INFO(msg, ...) printf("INFO: " msg "\n", ##__VA_ARGS__)

// Usage
if (error_condition) {
    LOG_ERROR("Failed to initialize MDB bus: error code %d", error_code);
    return false;
}
```

## Memory Management

### Dynamic Allocation
```c
// Prefer static allocation in embedded systems
static uint16_t commandBuffer[MAX_COMMAND_SIZE];

// If dynamic allocation is necessary, always check for NULL
uint8_t *buffer = malloc(size);
if (buffer == NULL) {
    LOG_ERROR("Memory allocation failed");
    return MDB_ERROR_MEMORY;
}

// Always free allocated memory
free(buffer);
buffer = NULL;  // Prevent use after free
```

### Buffer Management
```c
// Initialize buffers explicitly
memset(buffer, 0, sizeof(buffer));

// Check buffer bounds
if (index >= BUFFER_SIZE) {
    LOG_ERROR("Buffer index out of bounds: %d", index);
    return MDB_ERROR_BUFFER_OVERFLOW;
}
```

## Performance Guidelines

### Efficient Coding Practices
```c
// Use appropriate data types
uint8_t smallValue;     // For values 0-255
uint16_t mediumValue;   // For values 0-65535
bool flagValue;         // For true/false

// Minimize function call overhead in ISRs
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        // Keep ISR code minimal and fast
        uint16_t word = mdb_rx_buf[0] & 0x1FF;
        mdbRing_push(&rxRing, word);
        
        // Defer complex processing to tasks
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(mdbRxTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
```

### Memory Optimization
```c
// Use bit fields for flags
typedef struct {
    uint8_t commandReceived : 1;
    uint8_t responseReady : 1;
    uint8_t errorFlag : 1;
    uint8_t reserved : 5;
} SystemFlags_t;

// Pack structures when appropriate
typedef struct __attribute__((packed)) {
    uint8_t header;
    uint16_t data;
    uint8_t checksum;
} PackedMessage_t;
```

## Safety and Reliability

### Critical Sections
```c
// Protect shared data in multi-threaded environment
taskENTER_CRITICAL();
sharedVariable++;
taskEXIT_CRITICAL();

// Use FreeRTOS synchronization primitives
SemaphoreHandle_t dataMutex;

xSemaphoreTake(dataMutex, portMAX_DELAY);
// Access shared data
xSemaphoreGive(dataMutex);
```

### Input Validation
```c
// Always validate function parameters
bool MDB_ProcessCommand(uint16_t *buffer, uint8_t length) {
    // Validate inputs
    if (buffer == NULL) {
        LOG_ERROR("Null buffer pointer");
        return false;
    }
    
    if (length == 0 || length > MAX_COMMAND_LENGTH) {
        LOG_ERROR("Invalid command length: %d", length);
        return false;
    }
    
    // Process command
    return true;
}
```

### Timeout Handling
```c
// Always implement timeouts for blocking operations
uint32_t startTime = HAL_GetTick();
while (condition_not_met) {
    if ((HAL_GetTick() - startTime) > TIMEOUT_MS) {
        LOG_WARNING("Operation timed out");
        return MDB_ERROR_TIMEOUT;
    }
    // Brief delay to prevent tight loop
    HAL_Delay(1);
}
```

## Version Control Guidelines

### Commit Messages
```
feat: Add MDB command handler for cashless payments
fix: Resolve ring buffer overflow in UART ISR
docs: Update API documentation for MDB module
refactor: Improve state machine error handling
test: Add unit tests for command validation
```

### Code Review Checklist
- [ ] Code follows naming conventions
- [ ] Functions are properly documented
- [ ] Error handling is implemented
- [ ] Memory management is correct
- [ ] Performance considerations addressed
- [ ] Thread safety considered
- [ ] Input validation implemented
- [ ] Timeout mechanisms in place

## Testing Standards

### Unit Testing
```c
// Test function naming: test_ModuleName_FunctionName_Condition
void test_MDB_ValidateCommand_ValidInput_ReturnsTrue(void) {
    // Arrange
    uint16_t testCommand[] = {0x01E7, 0x0000};
    
    // Act
    bool result = MDB_ValidateCommand(testCommand, 2);
    
    // Assert
    assert(result == true);
}
```

### Integration Testing
```c
// Test complete workflows
void test_MDB_CompleteCommandCycle(void) {
    // Test command reception, processing, and response
    // Verify state transitions
    // Check timing requirements
}
```
