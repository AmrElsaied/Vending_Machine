# SYS_Logger — System Error Logging & Debug Output

## Purpose

Centralised logging facility for the entire VMC firmware. Provides a 32-entry circular error log with timestamps and context fields, a threshold-based critical error mechanism, and conditional debug output over USART6.

Source: [Core/Src/SYS_Logger.c](../Core/Src/SYS_Logger.c), [Core/Inc/SYS_Logger.h](../Core/Inc/SYS_Logger.h)

---

## Error Code Enumeration

```c
// Core/Inc/SYS_Logger.h
typedef enum {
    MDB_ERROR_UART_NOT_CONFIGURED        = 0x01,
    MDB_ERROR_SUB_COMMAND_NOT_RECOGNIZED = 0x02,
    MDB_ERROR_INVALID_RX_STATE           = 0x03,
    MDB_ERROR_INVALID_PROCESS_STATE      = 0x04,
    MDB_ERROR_COMMAND_NOT_VALID_IN_STATE = 0x05,
    MDB_ERROR_INVALID_STATE_FOR_RESET    = 0x06,
    MDB_ERROR_UNEXPECTED_ACK_WORD        = 0x07,
    MDB_ERROR_BALANCE_EXCEEDS_MAXIMUM    = 0x08,
    SYSTASK_ERROR_TASK_CREATION_FAILED   = 0x09,
    SYSTASK_ERROR_QUEUE_CREATION_FAILED  = 0x0A,
    ESP_WIFI_ERROR_CONNECTION_FAILED     = 0x0B,
    ESP_TCP_ERROR_CONNECTION_FAILED      = 0x0C,
    ESP_MQTT_ERROR_CONNECTION_FAILED     = 0x0D,
    ESP_SERVER_ERROR_LOSE_CONNECTION     = 0x0E,
    ESP_INTERNAL_COMM_ERROR              = 0x0F,
    ESP_ERROR_UART_NOT_CONFIGURED        = 0x10,
} Error_code_t;
```

---

## Key Types

### `Error_Info_t` — Single Log Entry

```c
// Core/Inc/SYS_Logger.h
typedef struct {
    Error_code_t error_code;       // Which error
    uint8_t      state_context;    // Device/module state when error occurred
    uint8_t      command_context;  // Command being processed (0 = none)
    uint16_t     error_data;       // Raw supplementary data
    uint32_t     timestamp;        // HAL_GetTick() value at log time
} Error_Info_t;
```

### `Error_Logger_t` — Circular Buffer State

```c
typedef struct {
    Error_Info_t error_logs[ERROR_LOGS_BUFFER_SIZE];  // 32 entries
    uint8_t      head;          // Next write position (wraps at 32)
    uint8_t      count;         // Entries currently stored (capped at 32)
    uint32_t     total_errors;  // Total errors since boot (never resets)
} Error_Logger_t;
```

`ERROR_LOGS_BUFFER_SIZE` = 32 (`SYS_Logger.h`). When the buffer is full, **new entries are silently discarded** — `total_errors` still increments to track total occurrences, but no slot is overwritten.

### `Critical_Error_logger_t` — Threshold Counter

```c
typedef struct {
    Error_code_t error_code;
    uint8_t      error_count;      // Incremented on each occurrence
    uint8_t      error_threshold;  // When count reaches threshold, action is taken
} Critical_Error_logger_t;
```

Used for errors that warrant system action (e.g., repeated MQTT disconnections). The action taken at threshold is defined in `SYS_LogCriticalError()`.

### `Error_Logger_Config_t`

Post-build config struct holding the `UART_HandleTypeDef*` for USART6 debug output.

---

## Configuration Macros

| Macro | Value | Purpose |
|-------|-------|---------|
| `ERROR_LOGS_BUFFER_SIZE` | 32 | Circular buffer depth |
| `ERROR_MESSAGE_MAX_LENGTH` | 64 | Max chars for formatted error string sent over UART6 |
| `DEBUG_MESSAGE_MAX_LENGTH` | 128 | Max chars for formatted debug string |
| `MDB_LOGGER_ERROR_ENABLED` | 1 | Compile-in MDB error logging |
| `ESP_LOGGER_ERROR_ENABLED` | 1 | Compile-in ESP error logging |
| `MDB_LOGGER_DEBUG_ENABLED` | 0 | MDB debug prints disabled by default |
| `ESP_LOGGER_DEBUG_ENABLED` | 0 | ESP debug prints disabled by default |
| `ERROR_LOGGER_TIMESTAMP_ENABLED` | 1 | Include `HAL_GetTick()` timestamp in each entry |
| `NO_DATA_PRESENT` | 0 | Pass as `error_data` when no extra data available |
| `NO_CONTEXT_PRESENT` | 0 | Pass as context when no context available |

---

## Public API

| Function | Signature | Purpose |
|----------|-----------|---------|
| `SYS_InitLogger` | `void SYS_InitLogger(void)` | Zero-initialise error buffer; mark logger as ready. `Logger_Config` (UART6 handle) is a static global in `SYS_Logger.c`. |
| `SYS_LogError` | `void SYS_LogError(Error_code_t, uint16_t data, uint8_t cmd)` | Record error in circular buffer; timestamp and state context captured automatically |
| `SYS_LogCriticalError` | `void SYS_LogCriticalError(Error_code_t, uint16_t data, uint8_t cmd)` | Record error + increment per-code threshold counter; trigger recovery action when threshold reached |
| `SYS_GetErrorByIndex` | `bool SYS_GetErrorByIndex(uint8_t index, Error_Info_t*)` | Retrieve entry by index (0 = oldest); returns false if index out of range |
| `SYS_ClearErrorLog` | `void SYS_ClearErrorLog(void)` | Zero-fill circular buffer; reset `head`, `count` (not `total_errors`) |
| `SYS_LogInfo` | `void SYS_LogInfo(const char *format, ...)` | Printf-style info message to UART6 with `[INFO]` prefix; always active |

---

## Critical Error Thresholds & Recovery Actions

`Critical_Error_Logger[]` is a static array in `SYS_Logger.c`, one entry per `Error_code_t`. Each entry holds an `error_count` and an `error_threshold`. When `error_count >= error_threshold`, `SYS_HandleCriticalErrorAction()` is called.

| Error Code | Threshold | Recovery Action |
|------------|----------|-----------------|
| `MDB_ERROR_UART_NOT_CONFIGURED` | 1 | DIAG LED on + system halt (`while(1)`) |
| `MDB_ERROR_SUB_COMMAND_NOT_RECOGNIZED` | 3 | No action (default case) |
| `MDB_ERROR_INVALID_RX_STATE` | 3 | No action |
| `MDB_ERROR_INVALID_PROCESS_STATE` | 3 | No action |
| `MDB_ERROR_COMMAND_NOT_VALID_IN_STATE` | 3 | No action |
| `MDB_ERROR_INVALID_STATE_FOR_RESET` | 3 | No action |
| `MDB_ERROR_UNEXPECTED_ACK_WORD` | 3 | No action |
| `MDB_ERROR_BALANCE_EXCEEDS_MAXIMUM` | 3 | No action |
| `SYSTASK_ERROR_TASK_CREATION_FAILED` | 1 | DIAG LED on + system halt |
| `SYSTASK_ERROR_QUEUE_CREATION_FAILED` | 1 | DIAG LED on + system halt |
| `ESP_WIFI_ERROR_CONNECTION_FAILED` | 1 | Retry `esp_setup_wifi_connection()` every 30 s; DIAG LED toggles 500 ms; `BUTTON_PollPreScheduler()` called in loop to allow factory reset |
| `ESP_TCP_ERROR_CONNECTION_FAILED` | 1 | Retry `esp_setup_tcp_connection()` every 5 s; DIAG LED toggles 500 ms |
| `ESP_MQTT_ERROR_CONNECTION_FAILED` | 1 | Retry `esp_send_mqtt_connect_packet()` every 5 s; DIAG LED toggles 500 ms |
| `ESP_SERVER_ERROR_LOSE_CONNECTION` | **4** | Retry full `ESP_Init()` every 5 s; DIAG LED toggles 500 ms |
| `ESP_INTERNAL_COMM_ERROR` | 1 | Retry `ESP_SendAT("AT", "OK", ...)` every 5 s; DIAG LED toggles 500 ms |
| `ESP_ERROR_UART_NOT_CONFIGURED` | 1 | DIAG LED on + system halt |

> All recovery loops are blocking (bare-metal `while(1)` with `HAL_Delay`). If the condition is resolved, the loop breaks and the LED is turned off. Only `ESP_WIFI_ERROR_CONNECTION_FAILED` also runs `BUTTON_PollPreScheduler()` to allow a factory reset during the retry loop.

---

## Logging Macros

These macros gate calls to `SYS_LogError` / `SYS_LogCriticalError` / UART print behind their compile-time flags.

```c
// Error macros — active when *_LOGGER_ERROR_ENABLED = 1
MDB_LOG_ERROR(code, data, context)
ESP_LOG_ERROR(code, data, context)

// Critical error macros
MDB_LOG_CRITICAL_ERROR(code, data, context)
ESP_LOG_CRITICAL_ERROR(code, data, context)

// Debug print macros — active when *_LOGGER_DEBUG_ENABLED = 1
MDB_PRINT_INFO(format, ...)
ESP_PRINT_INFO(format, ...)
```

To enable MDB or ESP debug output during development, set `MDB_LOGGER_DEBUG_ENABLED` or `ESP_LOGGER_DEBUG_ENABLED` to `1` in [Core/Inc/SYS_Logger.h](../Core/Inc/SYS_Logger.h).

---

## UART6 Debug Output Format

Each `SYS_LogError()` call produces a line over USART6 (115200 baud, 8N1):

```
[<timestamp_ms>] ERR 0x<code> state=<state> cmd=<cmd> data=0x<data>
```

`SYS_LogInfo()` sends the caller-formatted string directly.

---

## Inter-Module Dependencies

| Uses | For |
|------|-----|
| `main.h` | HAL UART types; `MDB_StateManager` for state context capture |
| `LED_Controller.h` | DIAG LED activation on critical errors |
| `ESP8266_Handler.h` | `esp_setup_wifi_connection()`, `esp_setup_tcp_connection()`, `esp_send_mqtt_connect_packet()`, `ESP_Init()`, `ESP_SendAT()` in recovery actions |
| `Periodic_Task_Manager.h` | `BUTTON_PollPreScheduler()` during WiFi error retry loop |
