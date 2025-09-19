# MDB Handler Usage Examples

**Author:** Amr Mohamed  
**Date:** September 19, 2025  
**Version:** 1.0

## Overview

This document demonstrates how to use the integrated MDB configuration system directly within the MDB_Handler files without separate configuration files. The MDB (Multi-Drop Bus) protocol handler provides a flexible configuration system through the global `mdb_config` variable.

## Table of Contents

- [Default Configuration](#default-configuration)
- [Runtime Configuration](#runtime-configuration)
- [Debug Control](#debug-control)
- [Complete Examples](#complete-examples)
- [Usage Notes](#usage-notes)

## Default Configuration

The `mdb_config` variable is already initialized with default values in `MDB_PBCFG.c`:

```c
MDB_Config_t mdb_config = {
    .mdb_uart = &huart1,           /* MDB bus communication UART */
    .debug_uart = &huart2,         /* Debug output UART */
    .bus_timeout = 10,             /* MDB bus timeout in milliseconds */
    .debug_enabled = false         /* Debug output disabled by default */
};
```

### Example 1: Using Default Configuration

```c
void MDB_Example_DefaultUsage(void)
{
    // The mdb_config is already initialized with defaults, just use it
    MDB_BusInit();
    
    // MDB is now ready for communication
    MDB_DebugPrint("[MDB Example] Default configuration used.\r\n");
}
```

## Runtime Configuration

You can modify the `mdb_config` variable directly before calling `MDB_BusInit()`.

### Example 2: Modifying Configuration at Runtime

```c
void MDB_Example_CustomConfig(void)
{
    // Assume we have these UART handles available
    extern UART_HandleTypeDef huart1;  // For MDB bus
    extern UART_HandleTypeDef huart3;  // For debug (different from default)
    
    // Modify the global mdb_config directly
    mdb_config.mdb_uart = &huart1;        /* MDB bus on UART1 */
    mdb_config.debug_uart = &huart3;      /* Debug on UART3 instead of UART2 */
    mdb_config.bus_timeout = 15;          /* Custom timeout (15ms instead of 10ms) */
    mdb_config.debug_enabled = true;      /* Enable debug output */
    
    // Initialize with the modified configuration
    MDB_BusInit();
    
    MDB_DebugPrint("[MDB Example] Custom configuration applied.\r\n");
}
```

## Debug Control

Debug output can be controlled both at compile-time and runtime.

### Example 3: Disable Debug Output

```c
void MDB_Example_NoDebug(void)
{
    // Disable debug output
    mdb_config.debug_enabled = false;
    
    // Initialize MDB bus
    MDB_BusInit();
    
    // This debug message will not be printed because debug is disabled
    MDB_DebugPrint("[MDB Example] This message will not be printed.\r\n");
}
```

## Complete Examples

### Example 4: Runtime Configuration Check and Adjustment

```c
void MDB_Example_RuntimeCheck(void)
{
    // Check if MDB UART is configured
    if (mdb_config.mdb_uart == NULL) {
        // Not configured, set to default
        extern UART_HandleTypeDef huart1;
        mdb_config.mdb_uart = &huart1;
        MDB_DebugPrint("[MDB Example] MDB UART configured to huart1.\r\n");
    }
    
    // Check debug UART
    if (mdb_config.debug_uart == NULL) {
        extern UART_HandleTypeDef huart2;
        mdb_config.debug_uart = &huart2;
        MDB_DebugPrint("[MDB Example] Debug UART configured to huart2.\r\n");
    }
    
    // Adjust timeout if needed
    if (mdb_config.bus_timeout < 5) {
        mdb_config.bus_timeout = 10;  /* Set to minimum recommended */
        MDB_DebugPrint("[MDB Example] Bus timeout adjusted to 10ms.\r\n");
    }
    
    // Initialize MDB bus
    MDB_BusInit();
}
```

### Example 5: Complete Initialization with Configuration

```c
void MDB_Example_CompleteInit(void)
{
    // Step 1: Configure MDB module (already done with defaults)
    // mdb_config is initialized with default values
    
    // Step 2: Optionally modify configuration
    // mdb_config.bus_timeout = 12;     // Custom timeout if needed
    // mdb_config.debug_enabled = true; // Enable debug output
    
    // Step 3: Initialize MDB bus communication
    MDB_BusInit();
    
    // Step 4: Start system tasks (if using FreeRTOS)
    // System_TaskCreate();  // Uncomment if you have this function
    
    MDB_DebugPrint("[MDB Example] Complete initialization finished.\r\n");
    MDB_DebugPrint("[MDB Example] MDB system ready for operation.\r\n");
}
```

## Usage Notes

### Configuration Structure

The `MDB_Config_t` structure contains the following fields:

- `mdb_uart`: UART handle for MDB bus communication
- `debug_uart`: UART handle for debug output
- `bus_timeout`: MDB bus timeout in milliseconds
- `debug_enabled`: Enable/disable debug output at runtime

### Key Points

1. **Global Configuration**: The `mdb_config` variable is global and initialized with default values.

2. **Runtime Modification**: You can modify `mdb_config` directly before calling `MDB_BusInit()`:
   ```c
   mdb_config.mdb_uart = &huart3;
   mdb_config.debug_enabled = true;
   MDB_BusInit();
   ```

3. **Debug Control**: Debug output is controlled by:
   - `MDB_ENABLE_DEBUG_OUTPUT` (compile-time flag)
   - `mdb_config.debug_enabled` (runtime flag)
   - `mdb_config.debug_uart != NULL` (UART availability)

4. **Validation**: The MDB module will validate the configuration and print error messages if something is wrong.

5. **UART Initialization**: Make sure the UART handles are properly initialized before using them in `mdb_config`.

6. **Function Integration**: All MDB functions now use `mdb_config` directly:
   - `HAL_UART_RxCpltCallback()` checks `mdb_config.mdb_uart`
   - `MDB_BusInit()` uses `mdb_config.mdb_uart`
   - `MDB_SendResponseWithModeBit()` uses `mdb_config.mdb_uart`
   - `MDB_DebugPrint()` uses `mdb_config.debug_uart` and `debug_enabled`

### FreeRTOS Integration

For FreeRTOS-based applications with MDB communication:

1. Call the MDB callback from your UART interrupt handler:
   ```c
   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
   {
       // Handle MDB UART interrupts
       if (huart == mdb_config.mdb_uart) {
           // Process MDB data reception
           // Add your MDB data handling logic here
       }
   }
   ```

2. Enable debug output for development:
   ```c
   mdb_config.debug_enabled = true;
   MDB_BusInit();
   ```

3. Handle MDB operations in your tasks:
   ```c
   void MDB_Task(void *argument)
   {
       // Initialize MDB
       MDB_BusInit();
       
       while (1) {
           // Handle MDB protocol operations
           // Process commands, send responses, etc.
           vTaskDelay(pdMS_TO_TICKS(10));
       }
   }
   ```

## File Structure

```
Compatable_WorkSpace/
├── Core/
│   ├── Inc/
│   │   └── MDB_Handler.h        # MDB handler declarations
│   └── Src/
│       ├── MDB_Handler.c        # MDB handler implementation
│       └── MDB_PBCFG.c         # Post-build configuration
└── doc/
    └── MDB_Usage_Examples.md    # This documentation file
```

---

**Note**: This documentation replaces the previous `MDB_Usage_Example.c` file to prevent it from being included in the compilation process while maintaining all the usage examples and explanations.
