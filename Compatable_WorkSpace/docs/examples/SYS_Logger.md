# SYS_Logger Module Documentation

## Overview

The SYS_Logger module provides comprehensive error logging and debugging capabilities for the STM32-based vending machine system. It features UART-based output, structured error tracking, and real-time system monitoring.

## Features

- ✅ **UART-Based Logging**: Direct transmission via configurable UART interface
- ✅ **Structured Error Tracking**: Circular buffer with timestamps and context
- ✅ **Modular Design**: Separate logging for MDB and ESP modules
- ✅ **Real-Time Output**: Immediate error reporting for debugging
- ✅ **Comprehensive Error Codes**: 17 categorized error types
- ✅ **Thread-Safe**: Compatible with FreeRTOS environment

## File Structure

```
Core/
├── Inc/
│   └── SYS_Logger.h          # Header with error codes and function prototypes
└── Src/
    └── SYS_Logger.c          # Implementation with UART transmission
docs/
└── examples/
    └── SYS_Logger.md         # This documentation file
```

## Configuration

### Module Configuration (SYS_Logger.h)

```c
/* Buffer and Message Configuration */
#define ERROR_LOGS_BUFFER_SIZE          32      /* Number of error entries to store */
#define ERROR_MESSAGE_MAX_LENGTH        64      /* Maximum error message length */
#define DEBUG_MESSAGE_MAX_LENGTH        128     /* Maximum debug message length */

/* Module Enable/Disable Flags */
#define MDB_LOGGER_ERROR_ENABLED        1       /* Enable MDB error logging */
#define ESP_LOGGER_ERROR_ENABLED        1       /* Enable ESP error logging */
#define MDB_LOGGER_DEBUG_ENABLED        0       /* Enable MDB debug output */
#define ESP_LOGGER_DEBUG_ENABLED        0       /* Enable ESP debug output */
#define ERROR_LOGGER_TIMESTAMP_ENABLED  1       /* Enable timestamp logging */
```

### UART Configuration

The logger uses `huart3` by default. To change the UART interface:

```c
// In SYS_Logger.c, modify the Logger_Config
Error_Logger_Config_t Logger_Config = {
    .Logger_uart = &huart1  // Change to desired UART
};
```

## Error Codes

### Error Categories

| Category | Range | Description |
|----------|-------|-------------|
| General | 0x00 | No error states |
| Communication | 0x10-0x1F | UART and communication errors |
| Command Processing | 0x30-0x3F | Command parsing and validation |
| State Management | 0x40-0x4F | State machine errors |
| ACK Handling | 0x50-0x5F | Acknowledgment errors |
| Balance/Transaction | 0x60-0x6F | Financial transaction errors |
| Vending Operations | 0x70-0x7F | Vending mechanism errors |
| Configuration | 0x80-0x8F | System configuration errors |
| System Resources | 0x90-0x9F | Resource management errors |

### Complete Error Code List

```c
typedef enum {
    MDB_ERROR_NONE = 0x00,                          /* No error */
    
    /* Communication Errors (0x10-0x1F) */
    MDB_ERROR_UART_NOT_CONFIGURED = 0x10,           /* UART not configured */
    
    /* Command Processing Errors (0x30-0x3F) */
    MDB_ERROR_SUB_COMMAND_NOT_RECOGNIZED = 0x30,    /* Unknown subcommand */
    MDB_ERROR_INVALID_COMMAND_STRUCTURE = 0x31,     /* Invalid command format */
    MDB_ERROR_COMMAND_LENGTH_MISMATCH = 0x32,       /* Length mismatch */
    MDB_ERROR_INCOMPLETE_COMMAND = 0x33,            /* Incomplete command */
    
    /* State Management Errors (0x40-0x4F) */
    MDB_ERROR_INVALID_RX_STATE = 0x40,              /* Invalid RX state */
    MDB_ERROR_INVALID_PROCESS_STATE = 0x41,         /* Invalid process state */
    MDB_ERROR_COMMAND_NOT_VALID_IN_STATE = 0x42,    /* Command not valid */
    MDB_ERROR_INVALID_STATE_FOR_RESET = 0x43,       /* Invalid reset state */
    
    /* ACKnowledgment Errors (0x50-0x5F) */
    MDB_ERROR_UNEXPECTED_ACK_WORD = 0x50,           /* Unexpected ACK word */
    MDB_ERROR_ACK_TIMEOUT = 0x51,                   /* ACK timeout */
    
    /* Balance and Transaction Errors (0x60-0x6F) */
    MDB_ERROR_BALANCE_EXCEEDS_MAXIMUM = 0x60,       /* Balance too high */
    
    /* Vending Operation Errors (0x70-0x7F) */
    MDB_ERROR_VENDING_OPERATION_FAILED = 0x70,      /* Vending failed */
    
    /* Configuration Errors (0x80-0x8F) */
    MDB_ERROR_INITIALIZATION_FAILED = 0x80,         /* Init failed */
    MDB_ERROR_CONFIG_INVALID = 0x81,                /* Invalid config */
    
    /* System Resource Errors (0x90-0x9F) */
    MDB_ERROR_RESOURCE_BUSY = 0x90                   /* Resource busy */
} Error_code_t;
```

## API Reference

### Initialization Functions

#### `SYS_InitLogger()`
Initializes the logger system and clears any previous logs.

```c
void SYS_InitLogger(void);
```

**Usage:**
```c
// Call during system initialization
SYS_InitLogger();
```

### Error Logging Functions

#### `SYS_LogError()`
Logs a standard error with context information.

```c
void SYS_LogError(Error_code_t error_code, uint16_t error_data, uint8_t command_context);
```

**Parameters:**
- `error_code`: Error code from `Error_code_t` enum
- `error_data`: Additional error-specific data (optional)
- `command_context`: Command being processed when error occurred (optional)

**Usage:**
```c
// Log UART configuration error
SYS_LogError(MDB_ERROR_UART_NOT_CONFIGURED, 0, 1);

// Log command error with data
SYS_LogError(MDB_ERROR_COMMAND_LENGTH_MISMATCH, received_length, cmd_index);
```

#### `SYS_LogCriticalError()`
Logs a critical error that requires immediate attention.

```c
void SYS_LogCriticalError(Error_code_t error_code, uint16_t error_data, uint8_t command_context);
```

**Usage:**
```c
// Log critical initialization failure
SYS_LogCriticalError(MDB_ERROR_INITIALIZATION_FAILED, 0, 0);
```

### Information Logging Functions

#### `SYS_LogInfo()`
Outputs formatted information messages via UART.

```c
void SYS_LogInfo(const char* format, ...);
```

**Usage:**
```c
// Simple message
SYS_LogInfo("System initialized successfully");

// Formatted message
uint32_t balance = 1500;
SYS_LogInfo("Current balance: %lu cents", balance);

// Complex formatting
SYS_LogInfo("Command 0x%02X processed, length: %d, result: %s", 
           cmd_code, length, success ? "OK" : "FAILED");
```

### Error Retrieval Functions

#### `SYS_GetErrorByIndex()`
Retrieves error information by index from the log buffer.

```c
bool SYS_GetErrorByIndex(uint8_t index, Error_Info_t *error_info);
```

**Usage:**
```c
Error_Info_t error;
if (SYS_GetErrorByIndex(0, &error)) {
    // Process the oldest error in buffer
    SYS_LogInfo("Error Code: 0x%02X, Data: 0x%04X", 
               error.error_code, error.error_data);
}
```

#### `SYS_GetErrorString()`
Returns human-readable description for an error code.

```c
const char* SYS_GetErrorString(Error_code_t error_code);
```

**Usage:**
```c
Error_code_t error = MDB_ERROR_ACK_TIMEOUT;
const char* description = SYS_GetErrorString(error);
SYS_LogInfo("Error: %s", description);  // Output: "Error: ACK Timeout"
```

### System Reporting Functions

#### `SYS_PrintErrorReport()`
Outputs a comprehensive error report via UART.

```c
void SYS_PrintErrorReport(void);
```

**Usage:**
```c
// Generate comprehensive error report
SYS_PrintErrorReport();
```

**Example Output:**
```
[INFO] === ERROR REPORT ===
[INFO] Total Errors: 5
[INFO] Critical Errors: 1
[INFO] Buffer Status: 3/32
[INFO] Recent Errors:
[INFO] [0] Code: 0x10 (UART Not Configured), Data: 0x0000, Time: 1234, State: 1, Cmd: 0
[INFO] [1] Code: 0x42 (Command Not Valid In State), Data: 0x0076, Time: 2456, State: 2, Cmd: 1
[INFO] [2] Code: 0x51 (ACK Timeout), Data: 0x00FF, Time: 3678, State: 3, Cmd: 2
[INFO] === END REPORT ===
```

#### `SYS_ClearErrorLog()`
Clears all error entries from the log buffer.

```c
void SYS_ClearErrorLog(void);
```

## Macro Usage

### Module-Specific Logging Macros

#### MDB Module Macros
```c
// Error logging (enabled by MDB_LOGGER_ERROR_ENABLED)
MDB_LOG_ERROR(MDB_ERROR_UART_NOT_CONFIGURED, 0, cmd_index);
MDB_LOG_CRITICAL_ERROR(MDB_ERROR_INITIALIZATION_FAILED, 0, 0);

// Debug output (enabled by MDB_LOGGER_DEBUG_ENABLED)
MDB_PRINT_INFO("MDB Handler initialized successfully");
MDB_PRINT_INFO("Processing command 0x%02X", command_byte);
```

#### ESP Module Macros
```c
// Error logging (enabled by ESP_LOGGER_ERROR_ENABLED)
ESP_LOG_ERROR(MDB_ERROR_RESOURCE_BUSY, resource_id, 0);
ESP_LOG_CRITICAL_ERROR(MDB_ERROR_CONFIG_INVALID, config_value, 0);

// Debug output (enabled by ESP_LOGGER_DEBUG_ENABLED)
ESP_PRINT_INFO("ESP communication established");
ESP_PRINT_INFO("Received data: %s", received_string);
```

### Convenience Macros
```c
// For functions with no additional data or context
#define NO_DATA_PRESENT           (uint16_t)0
#define NO_CONTEXT_PRESENT        (uint8_t)0

// Usage example
MDB_LOG_ERROR(MDB_ERROR_UART_NOT_CONFIGURED, NO_DATA_PRESENT, NO_CONTEXT_PRESENT);
```

## Implementation Examples

### 1. Basic Integration in main.c

```c
#include "SYS_Logger.h"

int main(void) {
    // System initialization
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART3_UART_Init();  // Initialize debug UART
    
    // Initialize logger after UART setup
    SYS_InitLogger();
    
    // Log system startup
    SYS_LogInfo("System initialization complete");
    SYS_LogInfo("Firmware Version: 1.0.0");
    SYS_LogInfo("Build Date: %s %s", __DATE__, __TIME__);
    
    // Continue with application initialization...
    if (MDB_Init() != HAL_OK) {
        MDB_LOG_CRITICAL_ERROR(MDB_ERROR_INITIALIZATION_FAILED, 0, 0);
    } else {
        MDB_PRINT_INFO("MDB Handler initialized successfully");
    }
    
    while (1) {
        // Main application loop
    }
}
```

### 2. MDB Handler Integration

```c
// In MDB_Handler.c
#include "SYS_Logger.h"

HAL_StatusTypeDef MDB_SendResponseWithModeBit(uint8_t cmd_index) {
    // Validate UART configuration
    if (mdb_config.mdb_uart == NULL) {
        MDB_LOG_ERROR(MDB_ERROR_UART_NOT_CONFIGURED, 0, cmd_index);
        MDB_PRINT_INFO("UART not configured for command index %d", cmd_index);
        return HAL_ERROR;
    }
    
    // Validate command index
    if (cmd_index >= VMC_CMD_MAX_NUMBER) {
        MDB_LOG_ERROR(MDB_ERROR_INVALID_COMMAND_STRUCTURE, cmd_index, VMC_CMD_MAX_NUMBER);
        return HAL_ERROR;
    }
    
    // Process command...
    MDB_PRINT_INFO("Sending response for command 0x%02X", 
                  VMC_CMDs[cmd_index].CMD_Byte);
    
    return HAL_OK;
}

void MDB_ProcessCommand(uint8_t* command_buffer, uint8_t length) {
    if (length == 0) {
        MDB_LOG_ERROR(MDB_ERROR_INCOMPLETE_COMMAND, length, 0);
        return;
    }
    
    // Validate command structure
    if (command_buffer[0] != EXPECTED_HEADER) {
        MDB_LOG_ERROR(MDB_ERROR_INVALID_COMMAND_STRUCTURE, command_buffer[0], 0);
        MDB_PRINT_INFO("Invalid command header: 0x%02X", command_buffer[0]);
        return;
    }
    
    // Process valid command
    MDB_PRINT_INFO("Processing command: 0x%02X, length: %d", 
                  command_buffer[0], length);
}
```

### 3. State Machine Error Handling

```c
// In state machine handler
void MDB_StateMachine_Handler(void) {
    switch (MDB_StateManager.CMD_RX_StateHandler) {
        case CMD_RX_READY:
            // Normal state processing
            break;
            
        case CMD_RX_RECEIVING:
            // Command reception in progress
            break;
            
        case CMD_RX_COMPLETE:
            // Command reception complete
            MDB_PRINT_INFO("Command reception completed");
            break;
            
        default:
            // Invalid state detected
            MDB_LOG_ERROR(MDB_ERROR_INVALID_RX_STATE, 
                         MDB_StateManager.CMD_RX_StateHandler, 0);
            MDB_StateManager.CMD_RX_StateHandler = CMD_RX_READY;
            break;
    }
}
```

### 4. ACK Handling with Error Logging

```c
HAL_StatusTypeDef MDB_WaitForACK(uint32_t timeout_ms) {
    uint32_t start_time = HAL_GetTick();
    
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (ack_received) {
            MDB_PRINT_INFO("ACK received successfully");
            return HAL_OK;
        }
        
        // Check for unexpected data
        if (unexpected_word_received) {
            MDB_LOG_ERROR(MDB_ERROR_UNEXPECTED_ACK_WORD, 
                         unexpected_word, current_command);
            return HAL_ERROR;
        }
    }
    
    // Timeout occurred
    MDB_LOG_ERROR(MDB_ERROR_ACK_TIMEOUT, timeout_ms, current_command);
    MDB_PRINT_INFO("ACK timeout after %lu ms", timeout_ms);
    return HAL_TIMEOUT;
}
```

### 5. Periodic Error Reporting

```c
// In main application loop or timer callback
void Periodic_SystemMonitoring(void) {
    static uint32_t last_report_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Report every 60 seconds
    if ((current_time - last_report_time) >= 60000) {
        SYS_LogInfo("=== PERIODIC SYSTEM STATUS ===");
        
        // Generate error report
        SYS_PrintErrorReport();
        
        // Log system health metrics
        SYS_LogInfo("System uptime: %lu seconds", current_time / 1000);
        SYS_LogInfo("Free heap: %d bytes", xPortGetFreeHeapSize());
        
        last_report_time = current_time;
    }
}
```

### 6. Error Analysis and Recovery

```c
void Analyze_SystemErrors(void) {
    Error_Info_t error_info;
    uint8_t critical_error_count = 0;
    
    // Analyze recent errors
    for (uint8_t i = 0; i < ERROR_LOGS_BUFFER_SIZE; i++) {
        if (SYS_GetErrorByIndex(i, &error_info)) {
            // Count critical errors (0x80 and above)
            if (error_info.error_code >= MDB_ERROR_INITIALIZATION_FAILED) {
                critical_error_count++;
            }
            
            // Handle specific error types
            switch (error_info.error_code) {
                case MDB_ERROR_ACK_TIMEOUT:
                    SYS_LogInfo("Frequent ACK timeouts detected - checking communication");
                    // Implement recovery action
                    break;
                    
                case MDB_ERROR_UART_NOT_CONFIGURED:
                    SYS_LogInfo("UART configuration issue - reinitializing");
                    // Reinitialize UART
                    break;
                    
                default:
                    break;
            }
        }
    }
    
    // Take action if too many critical errors
    if (critical_error_count > 5) {
        SYS_LogInfo("High critical error count: %d - entering safe mode", 
                   critical_error_count);
        // Enter safe mode or trigger system reset
    }
}
```

## UART Setup Requirements

### Hardware Configuration
- **UART Interface**: Any available UART (default: USART3)
- **Baud Rate**: 115200 (recommended)
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None

### STM32CubeMX Configuration
1. Enable USART3 (or desired UART)
2. Set mode to "Asynchronous"
3. Configure baud rate to 115200
4. Generate code

### Terminal Setup
- **Software**: PuTTY, Tera Term, Arduino Serial Monitor, etc.
- **Connection**: USB-to-Serial adapter connected to UART TX pin
- **Settings**: 115200-8-N-1
- **Line Ending**: CR+LF

## Performance Considerations

### Memory Usage
- **Static RAM**: ~1.5KB for error buffer and variables
- **Stack Usage**: ~200 bytes for message formatting
- **Flash Usage**: ~3KB for code and error strings

### Timing Considerations
- **UART Transmission**: Blocking operation (uses HAL_MAX_DELAY)
- **Error Logging**: < 10µs for memory operations
- **Message Formatting**: ~50µs for complex formatting

### Recommendations
- Enable debug output only during development
- Use DMA for UART if high-frequency logging is required
- Monitor stack usage when using formatted messages
- Clear error log periodically to prevent buffer overflow

## Troubleshooting

### Common Issues

1. **No UART Output**
   - Check UART initialization before `SYS_InitLogger()`
   - Verify TX pin connection
   - Ensure debug output is enabled

2. **Compilation Errors**
   - Include `"SYS_Logger.h"` in source files
   - Verify HAL includes are present
   - Check error code definitions

3. **Missing Error Strings**
   - Ensure error code exists in `Error_code_t` enum
   - Verify error is added to string mapping table

4. **Buffer Overflow**
   - Increase `ERROR_LOGS_BUFFER_SIZE` if needed
   - Implement periodic log clearing
   - Monitor error frequency

### Debug Commands

```c
// Check logger status
SYS_LogInfo("Logger initialized: %s", logger_initialized ? "Yes" : "No");

// Test UART transmission
SYS_LogInfo("UART test message - if you see this, UART is working");

// Generate test errors
MDB_LOG_ERROR(MDB_ERROR_UART_NOT_CONFIGURED, 0x1234, 5);
SYS_PrintErrorReport();

// Clear errors and verify
SYS_ClearErrorLog();
SYS_LogInfo("Error log cleared");
```

## Integration Checklist

- [ ] Include `SYS_Logger.h` in relevant source files
- [ ] Initialize UART before calling `SYS_InitLogger()`
- [ ] Enable appropriate debug flags in header file
- [ ] Replace existing error handling with logger calls
- [ ] Set up serial terminal for UART output
- [ ] Test error logging and information output
- [ ] Implement periodic error reporting
- [ ] Add error analysis and recovery logic

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-10-17 | Initial implementation with UART output |
| 1.1 | 2025-10-18 | Updated error codes and module-specific macros |

---

**Author**: Amr Mohamed  
**Module**: SYS_Logger  
**Target**: STM32F103C8T6 Vending Machine Controller  
**Documentation**: Complete usage guide and examples