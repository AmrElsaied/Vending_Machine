# Configuration Guide

## Overview
This guide covers the configuration options and setup procedures for the Vending Machine Controller (VMC) system.

## Hardware Configuration

### STM32F103C8T6 Pin Configuration

#### UART1 (MDB Communication)
- **TX Pin**: PA9
- **RX Pin**: PA10
- **Configuration**: 9600 baud, 9-bit mode
- **Mode**: Asynchronous with 9th bit for address/data

#### USB Configuration
- **USB_DM**: PA11
- **USB_DP**: PA12
- **Function**: CDC Virtual COM Port

#### GPIO Configuration
- **VENDING_Pin**: Configurable GPIO for vending control
- **Status LEDs**: Optional status indication pins

### STM32CubeMX Configuration

#### Clock Configuration
```
System Clock: 72 MHz
AHB Clock: 72 MHz
APB1 Clock: 36 MHz
APB2 Clock: 72 MHz
USB Clock: 48 MHz (from PLL)
```

#### UART1 Settings
```
Baud Rate: 9600
Word Length: 9 bits
Stop Bits: 1
Parity: None
Hardware Flow Control: None
```

#### USB_OTG_FS Settings
```
Mode: Device_Only
Speed: Full Speed 12MBps
SOF output: Disabled
VBUS sensing: Enabled if hardware supports
```

## Software Configuration

### FreeRTOS Configuration
Edit `FreeRTOSConfig.h` for optimal performance:

```c
#define configUSE_PREEMPTION                     1
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCPU_CLOCK_HZ                       72000000
#define configTICK_RATE_HZ                       1000
#define configMAX_PRIORITIES                     7
#define configMINIMAL_STACK_SIZE                 128
#define configTOTAL_HEAP_SIZE                    15360
#define configMAX_TASK_NAME_LEN                  16
```

### MDB Handler Configuration
In `MDB_Handler.h`:

```c
#define MDB_RING_LEN  256U           // Ring buffer size (power of 2)
#define MDB_BUS_TIMEOUT 10           // Timeout in milliseconds
#define ENABLE_BV_TX 1               // Enable transmission (set to 0 for testing)
```

### Task Configuration
In `system_tasks.c`:

```c
// Task priorities (higher number = higher priority)
#define MDB_RX_TASK_PRIORITY     (configMAX_PRIORITIES-3)
#define MDB_PROCESS_TASK_PRIORITY (configMAX_PRIORITIES-2)

// Task stack sizes (in words)
#define MDB_RX_TASK_STACK        384
#define MDB_PROCESS_TASK_STACK   256
```

## Command Configuration

### Supported MDB Commands
Configure in `VMC_Config.c`:

```c
VMC_CMD_t VMC_CMDs[VMC_CMD_MAX_NUMBER] = {
    // Command 0x01E7 - Reset
    {
        .CMD = {0x01E7, 0x0000},
        .CMD_Length = 2,
        .CMD_Response = {0},
        .CMD_Response_Length = 0
    },
    
    // Command 0x013B - Poll
    {
        .CMD = {0x013B, 0x0000},
        .CMD_Length = 2,
        .CMD_Response = {0},
        .CMD_Response_Length = 0
    },
    
    // Add other commands...
};
```

### Command Handler Mapping
In `MDB_Handler.c`:

```c
const CommandEntry_t command_table[] = {
    {VMC_CMD_0x01E7, handle_cmd_0x01E7},
    {VMC_CMD_0x013B, handle_cmd_0x013B},
    {VMC_CMD_0x01D5, handle_cmd_0x01D5},
    {VMC_CMD_0x0074, handle_cmd_0x0074},
    {VMC_CMD_0x0077, handle_cmd_0x0077},
    {VMC_CMD_0x0075, handle_cmd_0x0075},
    {VMC_CMD_0x0076, handle_cmd_0x0076}
};
```

## Build Configuration

### Compiler Settings
In STM32CubeIDE project settings:

#### C/C++ Build → Settings → Tool Settings

**MCU GCC Compiler → Optimization**
- Optimization Level: `-O2` (for release) or `-Og` (for debug)
- Debug Level: `-g3` (for debugging)

**MCU GCC Compiler → Preprocessor**
Add defines:
```
USE_HAL_DRIVER
STM32F103xB
```

**MCU GCC Linker → General**
- Remove unused sections: Enabled
- Libraries: Add `m` (math library) if needed

### Debug Configuration

#### Debug Symbols
```c
// Enable debug features in debug builds
#ifdef DEBUG
    #define DEBUG_PRINT(x) printf(x)
    #define ASSERT(x) assert(x)
#else
    #define DEBUG_PRINT(x)
    #define ASSERT(x)
#endif
```

#### Memory Allocation Debug
```c
// Monitor heap usage
#define configCHECK_FOR_STACK_OVERFLOW   2
#define configUSE_MALLOC_FAILED_HOOK     1
```

## Runtime Configuration

### State Machine Configuration
Default state initialization:

```c
MDB_StateManager_t MDB_StateManager = {
    .Cashless_StateHnadler = STATE_RESTART,
    .CMD_RX_StateHandler = CMD_RX_READY,
    .CMD_TX_StateHandler = CMD_TX_READY,
    .CMD_Process_StateHandler = CMD_PROCESS_READY
};
```

### Buffer Configuration
```c
MDB_BusManager_t MDB_BusManager = {
    .MDB_RXbuffer = {0},
    .RXBuffer_index = 0,
    .MDB_RX_CMD_Index = VMC_CMD_MAX_NUMBER,
    .MDB_TX_CMD_Index = VMC_CMD_MAX_NUMBER,
    .MDB_Process_CMD_Index = VMC_CMD_MAX_NUMBER
};
```

## Performance Tuning

### Interrupt Priorities
Configure in `stm32f1xx_it.c`:

```c
// UART1 interrupt priority (higher priority for real-time response)
#define UART1_IRQ_PRIORITY       5

// USB interrupt priority (lower priority)
#define USB_LP_CAN1_RX0_IRQ_PRIORITY  6
```

### Memory Optimization

#### Stack Size Optimization
Monitor stack usage and adjust:
```c
// Use FreeRTOS stack monitoring
#define configCHECK_FOR_STACK_OVERFLOW   2

// In task functions, check high water mark
UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
```

#### Heap Optimization
```c
// Adjust heap size based on actual usage
#define configTOTAL_HEAP_SIZE    (15 * 1024)  // 15KB

// Use heap monitoring
size_t freeHeapSize = xPortGetFreeHeapSize();
```

## Testing Configuration

### Debug Output Configuration
```c
// Enable debug UART output (if available)
#define DEBUG_UART_ENABLED       1
#define DEBUG_UART               &huart2  // Use UART2 for debug

// Debug message levels
#define DEBUG_LEVEL_ERROR        1
#define DEBUG_LEVEL_WARNING      2
#define DEBUG_LEVEL_INFO         3
#define DEBUG_LEVEL_DEBUG        4

#define DEBUG_LEVEL              DEBUG_LEVEL_INFO
```

### Test Mode Configuration
```c
// Test mode without actual MDB hardware
#define TEST_MODE_ENABLED        0

// Simulation parameters
#define SIMULATE_CARD_INSERTION  0
#define SIMULATE_VENDING         0
```

## Production Configuration

### Release Build Settings
```c
// Disable debug features
#define DEBUG                    0
#define ENABLE_DEBUG_OUTPUT      0
#define ENABLE_ASSERT            0

// Optimize for size and performance
#define OPTIMIZE_FOR_SIZE        1
```

### Security Configuration
```c
// Enable watchdog for production
#define ENABLE_WATCHDOG          1

// Timeout values for production
#define MDB_BUS_TIMEOUT          10    // 10ms
#define COMMAND_TIMEOUT          100   // 100ms
#define SESSION_TIMEOUT          30000 // 30 seconds
```

## Troubleshooting Configuration Issues

### Common Configuration Problems

1. **UART Not Working**
   - Check pin configuration in STM32CubeMX
   - Verify clock configuration
   - Ensure interrupt priorities are correct

2. **FreeRTOS Not Starting**
   - Check heap size configuration
   - Verify stack sizes are adequate
   - Ensure configCPU_CLOCK_HZ matches actual clock

3. **USB Communication Issues**
   - Verify USB clock is 48MHz
   - Check USB descriptor configuration
   - Ensure proper enumeration sequence

4. **Memory Issues**
   - Monitor stack usage with `uxTaskGetStackHighWaterMark()`
   - Check heap usage with `xPortGetFreeHeapSize()`
   - Verify linker script memory definitions

### Debug Tools

#### STM32CubeMonitor
Use for real-time monitoring:
- CPU usage
- Memory usage
- Task states
- Interrupt statistics

#### Logic Analyzer
For MDB protocol debugging:
- UART TX/RX signals
- Timing analysis
- Protocol compliance verification

#### Debug Printf
Implement debug output:
```c
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
```
