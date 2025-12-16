# System Configuration Guide

This document describes the configuration of all major modules in the STM32F4 Vending Machine Controller system.

---

## Table of Contents

1. [Hardware Configuration](#hardware-configuration)
2. [MDB Handler Configuration](#mdb-handler-configuration)
3. [FreeRTOS Configuration](#freertos-configuration)
4. [ESP8266 WiFi Module Configuration](#esp8266-wifi-module-configuration)
5. [System Logger Configuration](#system-logger-configuration)
6. [LED Controller Configuration](#led-controller-configuration)
7. [Periodic Task Manager Configuration](#periodic-task-manager-configuration)

---

## Hardware Configuration

### Microcontroller

**Device**: STM32F401RCT6  
**Core**: ARM Cortex-M4 (84 MHz)  
**RAM**: 96 KB  
**Flash**: 256 KB

### GPIO Pins

Configured in `main.h`:

| Pin | Port | Function | Mode |
|-----|------|----------|------|
| **PB6** | GPIOB | Diagnostic LED | GPIO Output |
| **PB7** | GPIOB | Communication LED | GPIO Output |
| **PB8** | GPIOB | Reset Button | GPIO Input + EXTI9_5_IRQn |
| **PB9** | GPIOB | Vending Pin | GPIO Input |

### UART Peripherals

| UART | Baud Rate | Purpose | Flow Control |
|------|-----------|---------|--------------|
| **USART1** | 9600 bps | MDB Bus Communication | None (9-bit mode) |
| **USART2** | 115200 bps | ESP8266 Communication | None |
| **USART3** | 115200 bps | Debug Output | None |

---

## MDB Handler Configuration

**Module**: MDB_Handler (MDB_Handler.h/c)

**Purpose**: Manages Multi-Drop Bus protocol communication with cashless payment devices.

### Configuration Parameters

```c
#define MDB_RING_LEN        40U     // Ring buffer size (power of 2)
#define MDB_BUS_TIMEOUT     10      // Bus timeout in ticks (10ms at 1kHz)
```

### Ring Buffer Configuration

**Size**: 40 entries (power of 2)  
**Purpose**: Circular buffer for UART ISR data storage  
**Operation**:
- ISR writes data via `mdbRing_push()`
- Reception task reads via `mdbRing_pop()`
- Efficient wrap-around using bit masking (no modulo operator)

### Command Configuration

**Supported Commands**: 7 core MDB commands

| Command | Hex | Purpose |
|---------|-----|---------|
| Reset/Restart | 0x01E7 | Initialize device |
| Poll/Status | 0x013B | Query status |
| Reader Enable | 0x01D5 | Enable card reader |
| Product ID Exchange | 0x0074 | Exchange device info |
| VMC Data Request | 0x0077 | Configuration negotiation |
| Revalue Limit | 0x0075 | Query revalue capability |
| Vending Request | 0x0076 | Vend transaction |

### State Machine Configuration

**Device States**: 13 states

- INACTIVE, RESET, INIT, DISABLED, ENABLED
- START_SESSION, SESSION_IDLE
- APPROVE_VEND_REQ, DENY_VEND_REQ
- VEND_PROCESS, CANCEL_SESSION
- WAIT_ACK, ERROR

**Command Reception States**: 4 states

- CMD_RX_READY, CMD_RX_INPROGRESS
- CMD_RX_DONE, CMD_RX_BUSY

**Processing States**: 4 states

- CMD_PROCESS_READY, CMD_PROCESS_INPROGRESS
- CMD_PROCESS_DONE, CMD_PROCESS_BUSY

**Transmission States**: 4 states

- CMD_TX_READY, CMD_TX_INPROGRESS
- CMD_TX_DONE, CMD_TX_BUSY

### Balance Configuration

```c
Peripheral_Balance_t {
    uint8_t Max_balance = 244;      // Maximum balance value
    uint8_t Cur_balance = 0;        // Current balance
    uint8_t Revalue_limit = 244;    // Available revalue amount
}
```

### Data Structure Limits

| Structure | Field | Max Size |
|-----------|-------|----------|
| CMD_Type | CMD[] | 5 words (10 bytes) |
| CMD_Type | CMD_Response[] | 36 words (72 bytes) |
| mdb_ring_t | buf[] | 40 entries |

---

## FreeRTOS Configuration

**Version**: FreeRTOS Kernel V10.3.1  
**File**: FreeRTOSConfig.h

### Core Configuration

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `configUSE_PREEMPTION` | 1 | Enable task preemption |
| `configCPU_CLOCK_HZ` | SystemCoreClock | CPU frequency (84 MHz) |
| `configTICK_RATE_HZ` | 1000 | Tick rate (1ms) |
| `configMAX_PRIORITIES` | 56 | Maximum task priority levels |
| `configMINIMAL_STACK_SIZE` | 128 | Minimum stack (words) |
| `configTOTAL_HEAP_SIZE` | 15360 | Total heap (bytes) |

### Memory Configuration

**Total Heap**: 15,360 bytes (15.36 KB)  
**Minimal Stack**: 128 words (512 bytes minimum per task)  
**Support**: Dynamic + Static allocation enabled

### Task Configuration

**Supported Features**:
- Mutexes: `configUSE_MUTEXES = 1`
- Counting Semaphores: `configUSE_COUNTING_SEMAPHORES = 1`
- Recursive Mutexes: `configUSE_RECURSIVE_MUTEXES = 1`
- Software Timers: `configUSE_TIMERS = 1`

### Timer Task

| Parameter | Value |
|-----------|-------|
| Priority | 2 |
| Queue Length | 10 |
| Stack Depth | 256 words |

### Newlib Support

**Reentrancy**: `configUSE_NEWLIB_REENTRANT = 1`

---

## ESP8266 WiFi Module Configuration

**Module**: ESP8266_Handler (ESP8266_Handler.h)

**Purpose**: WiFi connectivity and MQTT communication.

### UART Configuration

| Parameter | Size | Purpose |
|-----------|------|---------|
| Baud Rate | 115200 bps | Communication speed with ESP8266 module |
| RX Buffer | 512 bytes | Stores incoming UART data from ESP8266 (AT responses, MQTT messages) |
| Response Buffer | 512 bytes | Holds parsed AT command responses for processing |
| Line Buffer | 128 bytes | Temporary storage for current line being parsed from RX data |
| Topic Buffer | 128 bytes | Stores MQTT topic string for received publish messages |
| Message Buffer | 128 bytes | Stores MQTT message payload for received publish messages |

### WiFi Configuration

**Default Credentials**:

```
SSID: MA_HOME
Password: 01289878405
```

**Unconfigured Defaults** (for first-time setup):

```
SSID: EMPTY_CONFIG
Password: EMPTY_CONFIG
```

### MQTT Broker Configuration

| Parameter | Value |
|-----------|-------|
| Broker IP | 192.168.1.106 |
| Broker Port | 1883 |
| Client ID | STM32 |
| Keep-Alive | 60 seconds |
| Username | magdy |
| Password | 987654321 |

### MQTT Topic Configuration

**Max Topic Length**: 64 bytes  
**Max Message Length**: 128 bytes

### Timeout Configuration

| Timeout | Duration |
|---------|----------|
| AT Command | 1000 ms |
| WiFi Connection | 20000 ms (20s) |
| TCP Connection | 10000 ms (10s) |
| MQTT Connection | 10000 ms (10s) |
| Restore Settings | 5000 ms (5s) |
| Ping Interval | 40000 ms (40s) |

### Module Status States

```c
ESP_STATUS_UNINITIALIZED    // Not initialized
ESP_STATUS_INITIALIZING     // Init in progress
ESP_STATUS_WIFI_DISCONNECTED // WiFi not connected
ESP_STATUS_WIFI_CONNECTED   // WiFi connected
ESP_STATUS_MQTT_DISCONNECTED // MQTT not connected
ESP_STATUS_MQTT_CONNECTED   // MQTT ready
ESP_STATUS_CONFIGURING      // Config mode
ESP_STATUS_ERROR            // Error state
```

---

## System Logger Configuration

**Module**: SYS_Logger (SYS_Logger.h)

**Purpose**: Error logging, debug output, and system event tracking.

### Logger Configuration

| Parameter | Value |
|-----------|-------|
| Error Buffer Size | 32 entries |
| Error Message Length | 64 bytes |
| Debug Message Length | 128 bytes |

### Logging Enable/Disable Flags

```c
#define MDB_LOGGER_ERROR_ENABLED        1   // MDB error logging
#define ESP_LOGGER_ERROR_ENABLED        1   // ESP8266 error logging
#define MDB_LOGGER_DEBUG_ENABLED        0   // MDB debug output
#define ESP_LOGGER_DEBUG_ENABLED        0   // ESP8266 debug output
#define ERROR_LOGGER_TIMESTAMP_ENABLED  1   // Timestamp in logs
```

### Error Codes

**MDB Errors**:

| Error Code | Hex | Description |
|------------|-----|-------------|
| UART_NOT_CONFIGURED | 0x01 | UART handle not configured |
| SUB_COMMAND_NOT_RECOGNIZED | 0x02 | Unknown subcommand |
| INVALID_RX_STATE | 0x03 | Invalid RX state |
| INVALID_PROCESS_STATE | 0x04 | Invalid process state |
| COMMAND_NOT_VALID_IN_STATE | 0x05 | Command invalid for current state |
| INVALID_STATE_FOR_RESET | 0x06 | RESET in wrong state |
| UNEXPECTED_ACK_WORD | 0x07 | Wrong ACK response |
| BALANCE_EXCEEDS_MAXIMUM | 0x08 | Balance overflow |

**System Task Errors**:

| Error Code | Hex | Description |
|------------|-----|-------------|
| TASK_CREATION_FAILED | 0x09 | FreeRTOS task creation failed |
| QUEUE_CREATION_FAILED | 0x0A | FreeRTOS queue creation failed |

**ESP8266 Errors**:

| Error Code | Hex | Description |
|------------|-----|-------------|
| WIFI_CONNECTION_FAILED | 0x0B | WiFi connection error |
| TCP_CONNECTION_FAILED | 0x0C | TCP broker connection failed |
| MQTT_CONNECTION_FAILED | 0x0D | MQTT connection failed |
| SERVER_LOSE_CONNECTION | 0x0E | MQTT server disconnected |
| INTERNAL_COMM_ERROR | 0x0F | ESP8266 communication error |

### Error Data Structure

```c
typedef struct {
    Error_code_t error_code;    // Error code
    uint8_t state_context;      // State when error occurred
    uint8_t command_context;    // Command being processed
    uint16_t error_data;        // Additional error info
    uint32_t timestamp;         // When error occurred
} Error_Info_t;
```

### Logging Macros

**Error Logging**:

| Macro | Purpose |
|-------|---------|
| `MDB_LOG_ERROR(code, data, context)` | Log MDB module error with error code, additional data, and context (state/command) |
| `MDB_LOG_CRITICAL_ERROR(code, data, context)` | Log critical MDB error that requires immediate attention and system intervention |
| `ESP_LOG_ERROR(code, data, context)` | Log ESP8266 module error with error code, additional data, and context |
| `ESP_LOG_CRITICAL_ERROR(code, data, context)` | Log critical ESP8266 error that requires immediate attention and system intervention |

**Debug Output**:

| Macro | Purpose |
|-------|---------|
| `MDB_PRINT_INFO(format, ...)` | Print formatted debug information for MDB module (enabled/disabled by flag) |
| `ESP_PRINT_INFO(format, ...)` | Print formatted debug information for ESP8266 module (enabled/disabled by flag) |
---

## LED Controller Configuration

**Module**: LED_Controller (LED_Controller.h)

**Purpose**: LED control for diagnostics and communication status indication.

### LED Channels

```c
typedef enum {
    LED_CHANNEL_DIAG,   // Diagnostic LED (PB6)
    LED_CHANNEL_COMM,   // Communication LED (PB7)
    LED_CHANNEL_MAX
} led_channel_t;
```

### LED States

```c
typedef enum {
    LED_STATE_OFF,      // LED off
    LED_STATE_ON,       // LED on
} led_state_t;
```

### LED Control API

| Function | Parameters | Purpose |
|----------|-----------|---------|
| `LED_CONTROLLER_Init()` | None | Initialize GPIO ports and setup LED channels for control |
| `LED_On(channel)` | `led_channel_t channel` | Turn on the specified LED (DIAG or COMM) |
| `LED_Off(channel)` | `led_channel_t channel` | Turn off the specified LED |
| `LED_Toggle(channel)` | `led_channel_t channel` | Toggle LED state (on→off, off→on) for the specified channel |
| `LED_GetState(channel)` | `led_channel_t channel` | Get current LED state (ON or OFF) for status checking |
| `LED_Blink(channel, period_ms)` | `led_channel_t channel`, `uint32_t period_ms` | Set LED to blink with specified period (on+off time in milliseconds) |

### LED Meanings

**Diagnostic LED (DIAG_LED)**:
- Off: System normal
- On: System error
- Blinking: Error recovery

**Communication LED (COMM_LED)**:
- Off: No connection
- On: Connected (WiFi + MQTT)
- Blinking: Connecting / Transferring data

---

## Periodic Task Manager Configuration

**Module**: Periodic_Task_Manager (Periodic_Task_Manager.h)

**Purpose**: Unified periodic task scheduling for 10ms+ based operations.

### Base Configuration

| Parameter | Value |
|-----------|-------|
| Base Period | 100 ms |
| Button Debounce | 20 ms |
| Long Press Time | 2000 ms (2s) |

### Periodic Modules

```c
typedef enum {
    PERIODIC_MODULE_BUTTON,          // Button debouncing & detection
    PERIODIC_MODULE_SYSTEM_MONITOR,  // System state monitoring
    PERIODIC_MODULE_COMMUNICATION,   // Communication status monitoring
    PERIODIC_MODULE_MAX
} periodic_module_t;
```

### Periodic Task Configuration

```c
typedef struct {
    periodic_module_t module_id;
    periodic_callback_t callback;
    uint32_t period_ms;              // 10ms, 20ms, 50ms, 100ms, etc.
    bool enabled;
} periodic_task_config_t;
```

### Module Configuration

**Button Module**:
- Period: 100 ms
- Debounce: 20 ms
- Long Press: 2000 ms

**System Monitor Module**:
- Period: 50 ms
- Monitors: Task health, memory, errors

**Communication Module**:
- Period: 1000 ms (1s)
- Monitors: WiFi status, MQTT connectivity
- LED updates based on status

### API Functions

| Function | Parameters | Purpose |
|----------|-----------|---------|
| `PERIODIC_TASK_Init()` | None | Initialize periodic task manager and setup internal structures for module management |
| `PERIODIC_TASK_Register(config)` | `const periodic_task_config_t *config` | Register a periodic module with callback, period, and enable flag |
| `PERIODIC_TASK_Unregister(module_id)` | `periodic_module_t module_id` | Remove a registered periodic module from the scheduler |
| `PERIODIC_TASK_Enable(module_id)` | `periodic_module_t module_id` | Enable a registered module to execute its callback at specified period |
| `PERIODIC_TASK_Disable(module_id)` | `periodic_module_t module_id` | Disable a module to prevent callback execution without unregistering |
| `PERIODIC_TASK_Execute()` | None | Execute all registered and enabled modules based on elapsed time and period |

---

## System Task Configuration

**Module**: system_tasks (system_tasks.h)

**Purpose**: FreeRTOS task creation and management.

### Task Definitions

| Task | Priority | Stack | Function | Purpose |
|------|----------|-------|----------|---------|
| mdbRxTask | MAX-3 | 384 | mdbRxTask() | MDB receive & buffering |
| mdbCMDProcessTask | MAX-2 | 384 | mdbCMDProcessTask() | Command processing |
| espCommunicationTask | MAX-4 | 600 | espCommunicationTask() | MQTT publish handler |
| espMqttProcessTask | MAX-5 | 512 | espMqttProcessTask() | Message processing |
| PeriodicMainTask | MAX-10 | 512 | PeriodicMainTask() | Periodic operations (disabled) |

### Task Priorities

**Priority Order** (highest to lowest):

1. mdbCMDProcessTask (MAX-2)
2. mdbRxTask (MAX-3)
3. espCommunicationTask (MAX-4)
4. espMqttProcessTask (MAX-5)
5. Timer Task (priority 2)
6. PeriodicMainTask (MAX-10, if enabled)

### Inter-Task Communication

**Ring Buffer**:
- ISR → mdbRxTask: `mdbRing_push/pop()`

**Task Notifications**:
- ISR → mdbRxTask: `vTaskNotifyGiveFromISR()`
- mdbRxTask → mdbCMDProcessTask: `xTaskNotify()`

**Queues**:
- Other Tasks → espCommunicationTask: `xQueueSend()` for publish requests
- Queue Size: 10 entries

---

## Configuration Summary

### Module Dependencies

```
main.h
  ├── MDB_Handler (MDB communication)
  ├── ESP8266_Handler (WiFi/MQTT)
  ├── system_tasks (Task management)
  ├── LED_Controller (LED control)
  ├── SYS_Logger (Error logging)
  ├── Periodic_Task_Manager (Periodic tasks)
  └── FreeRTOS (RTOS kernel)
```

### Initialization Sequence

1. **Hardware Init** (HAL, GPIO, UART, timers)
2. **FreeRTOS Init** (Kernel setup)
3. **Module Init** (Logger, LED, Periodic Task Manager)
4. **Task Creation** (`System_TaskCreate()`)
5. **Scheduler Start** (`vTaskStartScheduler()`)

### Configuration Validation Checklist

- [ ] UART baudrates configured correctly
- [ ] GPIO pins assigned to correct ports
- [ ] MDB ring buffer size is power of 2
- [ ] FreeRTOS heap size is sufficient
- [ ] Task priorities avoid conflicts
- [ ] Stack sizes are adequate for each task
- [ ] ESP8266 WiFi credentials set
- [ ] MQTT broker IP and port configured
