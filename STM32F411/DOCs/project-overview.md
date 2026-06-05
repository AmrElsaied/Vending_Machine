# STM32F411 Vending Machine Controller — Project Overview

## Hardware Platform

| Item | Detail |
|------|--------|
| MCU | STM32F411CEUX (ARM Cortex-M4, 84 MHz) |
| RTOS | FreeRTOS 10.3.1 via CMSIS-RTOS V2 wrapper |
| Clock Source | External 25 MHz crystal → PLL → 84 MHz |
| Flash (user area) | 0x08010000 – 0x0801FFFF (sector 4, 64 KB) |

---

## Software Component Map

| # | Component | Source Files | Role | Peripheral |
|---|-----------|-------------|------|------------|
| 1 | **main** | `Core/Src/main.c` | System entry point; HAL init, peripheral init, FreeRTOS start | — |
| 2 | **MDB_Handler** | `Core/Src/MDB_Handler.c` | Multi-Drop Bus (MDB) protocol driver for cashless payment devices | USART1 |
| 3 | **ESP8266_Handler** | `Core/Src/ESP8266_Handler.c` | WiFi connectivity and MQTT communication with remote server | USART2 |
| 4 | **system_tasks** | `Core/Src/system_tasks.c` | Creates and owns all FreeRTOS tasks; defines the espPublishQueue IPC channel | — |
| 5 | **Periodic_Task_Manager** | `Core/Src/Periodic_Task_Manager.c` | Dispatches registered 100 ms periodic callbacks (button, LED, comms monitor) | GPIO PB10 |
| 6 | **SYS_Logger** | `Core/Src/SYS_Logger.c` | Centralised error logging (32-entry circular buffer) and debug UART output | USART6 |
| 7 | **VMC_Config** | `Core/Src/VMC_Config.c` | Static MDB command/response database used by MDB_Handler | — |
| 8 | **LED_Controller** | `Core/Src/LED_Controller.c` | ON/OFF/TOGGLE/BLINK control for three status LEDs | GPIO PB12/13/14 |
| 9 | **Flash_Driver** | `Core/Src/Flash_Driver.c` | Read/write/erase abstraction over internal Flash (persistent config storage) | Flash |
| 10 | **MDB_PBCFG** | `Core/Src/MDB_PBCFG.c` | Post-build configuration linking USART1 and timeout to MDB_Handler | — |
| 11 | **ESP8266_PBCFG** | `Core/Src/ESP8266_PBCFG.c` | Post-build configuration: default WiFi SSID, MQTT broker, client ID for ESP | — |

---

## Component Responsibilities

### main
- Runs the hardware init sequence before the scheduler starts.
- Calls `ESP_Init()` (blocking: connects WiFi + MQTT) then `MDB_BusInit()`.
- Routes UART interrupts to `MDB_UART_RxCallback()` and `ESP_UART_RxCallback()` inside `HAL_UART_RxCpltCallback()`.
- Routes GPIO EXTI interrupt (reset button) to `PERIODIC_TASK_ButtonInterrupt()`.
- → See [Core/Src/main.c](../Core/Src/main.c)

### MDB_Handler
- Implements the full MDB slave state machine (INACTIVE → RESET → DISABLED → ENABLED → SESSION → VEND).
- Uses an ISR-safe 40-word ring buffer (`mdb_ring_t`) to decouple the UART ISR from the processing task.
- Decodes 9-bit UART words (9th bit = MDB address/mode flag).
- Matches received command patterns against the `VMC_CMDs[]` database in VMC_Config.
- Notifies `mdbCMDProcessTask` via `xTaskNotify()` when a complete command is buffered.
- → See [Core/Src/MDB_Handler.c](../Core/Src/MDB_Handler.c), [Core/Inc/MDB_Handler.h](../Core/Inc/MDB_Handler.h)

### ESP8266_Handler
- Manages the ESP8266 module through AT commands (for init/WiFi/TCP) and raw binary packets (for MQTT).
- Maintains its own status state machine (`ESP_Status_t`: UNINITIALIZED → WIFI_CONNECTED → MQTT_CONNECTED).
- Provides `ESP_Publish()` called by `espCommunicationTask` and `ESP_ProcessMQTTMessage()` called by `espMqttProcessTask`.
- Stores WiFi credentials in Flash at `0x08010000`; enters AP mode if credentials are absent.
- Sends a keep-alive ping every 40 seconds if no other publish activity occurs.
- → See [Core/Src/ESP8266_Handler.c](../Core/Src/ESP8266_Handler.c), [Core/Inc/ESP8266_Handler.h](../Core/Inc/ESP8266_Handler.h)

### system_tasks
- Single call `System_TaskCreate()` from `main.c` creates all five application tasks and the `espPublishQueue`.
- Any module that needs to publish an MQTT message calls `ESP_RequestPublish()`, which posts to `espPublishQueue` without blocking the caller.
- → See [Core/Src/system_tasks.c](../Core/Src/system_tasks.c), [Core/Inc/system_tasks.h](../Core/Inc/system_tasks.h)

### Periodic_Task_Manager
- Provides a lightweight scheduler inside `PeriodicMainTask`; `PERIODIC_TASK_Execute()` is called every 100 ms.
- Modules register callbacks with `PERIODIC_TASK_Register()` specifying their desired period.
- Currently active callbacks: button debounce handler, communication monitoring / LED blink control.
- A 2-second press on PB10 triggers factory reset (clears Flash WiFi config + `HAL_NVIC_SystemReset()`).
- → See [Core/Src/Periodic_Task_Manager.c](../Core/Src/Periodic_Task_Manager.c), [Core/Inc/Periodic_Task_Manager.h](../Core/Inc/Periodic_Task_Manager.h)

### SYS_Logger
- All modules log errors via macros (`MDB_LOG_ERROR`, `ESP_LOG_ERROR`, etc.) that call `SYS_LogError()`.
- Errors are stored in a 32-entry circular buffer with timestamp, state context, command context, and raw data.
- Critical errors additionally go through a threshold counter (`Critical_Error_logger_t`) that can take system action.
- Debug prints are conditionally compiled via `MDB_LOGGER_DEBUG_ENABLED` / `ESP_LOGGER_DEBUG_ENABLED` flags.
- → See [Core/Src/SYS_Logger.c](../Core/Src/SYS_Logger.c), [Core/Inc/SYS_Logger.h](../Core/Inc/SYS_Logger.h)

### VMC_Config
- Defines the static `VMC_CMDs[]` array: 8 entries, each with the command word sequence and the expected response bytes.
- Indexed by `CMD_Data` enum; MDB_Handler uses the enum to look up responses without hard-coding byte arrays.
- → See [Core/Src/VMC_Config.c](../Core/Src/VMC_Config.c), [Core/Inc/VMC_Config.h](../Core/Inc/VMC_Config.h)

### LED_Controller
- Manages DIAG (PB14), COMM (PB13), and PWR (PB12) LEDs.
- `LED_PeriodicUpdate()` is called from `PeriodicMainTask` every 100 ms to advance blink timers.
- → See [Core/Src/LED_Controller.c](../Core/Src/LED_Controller.c), [Core/Inc/LED_Controller.h](../Core/Inc/LED_Controller.h)

### Flash_Driver
- Wraps HAL Flash erase/write/read for the user data sector (sector 4).
- Write operations require word-alignment and sizes that are multiples of 4 bytes.
- Used exclusively by ESP8266_Handler (WiFi credential persistence) and Periodic_Task_Manager (factory reset erase).
- → See [Core/Src/Flash_Driver.c](../Core/Src/Flash_Driver.c), [Core/Inc/Flash_Driver.h](../Core/Inc/Flash_Driver.h)

### MDB_PBCFG / ESP8266_PBCFG
- Contain the single configuration structure instances (`mdb_config`, `ESP8266_DefaultConfig`) that bind HAL UART handles to each handler.
- Separating config from logic allows changing peripheral assignments without touching handler code.
- → See [Core/Src/MDB_PBCFG.c](../Core/Src/MDB_PBCFG.c), [Core/Src/ESP8266_PBCFG.c](../Core/Src/ESP8266_PBCFG.c)

---

## UART Allocation Summary

| UART | Instance | Baud | Bits | Owner |
|------|----------|------|------|-------|
| UART1 | USART1 | 9600 | 9 | MDB_Handler |
| UART2 | USART2 | 115200 | 8 | ESP8266_Handler |
| UART6 | USART6 | 115200 | 8 | SYS_Logger (debug) |

## GPIO Allocation Summary

| Pin | Function | Owner |
|-----|----------|-------|
| PB10 | Reset button (EXTI, active low) | Periodic_Task_Manager |
| PB12 | PWR LED | LED_Controller |
| PB13 | COMM LED | LED_Controller |
| PB14 | DIAG LED | LED_Controller |

---

## FreeRTOS Task Overview

| Task | Priority | Stack (words) | Purpose |
|------|----------|---------------|---------|
| `mdbCMDProcessTask` | MAX−2 (54) | 384 | Process complete MDB commands |
| `mdbRxTask` | MAX−3 (53) | 384 | Receive and frame MDB protocol words |
| `espCommunicationTask` | MAX−4 (52) | 600 | Send queued MQTT publishes; 40 s ping |
| `espMqttProcessTask` | MAX−5 (51) | 512 | Parse and dispatch incoming MQTT messages |
| `PeriodicMainTask` | MAX−10 (46) | 512 | 100 ms periodic operations |

IPC: one FreeRTOS queue (`espPublishQueue`, depth 10) for MQTT publish requests; task notifications for all other synchronisation.

---

## Module Dependency Quick Reference

```
main ──────────────────────────────────────────────────────────────────┐
  ├─ MDB_Handler ──── VMC_Config                                        │
  │                ── SYS_Logger                                        │
  │                ── system_tasks (notifies mdbCMDProcessTask)         │
  │                ── ESP8266_Handler (calls ESP_RequestPublish)        │
  │                                                                     │
  ├─ ESP8266_Handler ── Flash_Driver                                    │
  │                  ── LED_Controller                                  │
  │                  ── SYS_Logger                                      │
  │                  ── system_tasks (notifies espMqttProcessTask)      │
  │                                                                     │
  ├─ system_tasks ─── MDB_Handler                                       │
  │               ── ESP8266_Handler                                    │
  │               ── Periodic_Task_Manager                              │
  │               ── SYS_Logger                                        │
  │                                                                     │
  ├─ Periodic_Task_Manager ── LED_Controller                            │
  │                        ── Flash_Driver                              │
  │                        ── ESP8266_Handler                           │
  │                                                                     │
  ├─ SYS_Logger ── LED_Controller                                       │
  └─ LED_Controller (no external module dependencies)                   │
                                                                        │
  Flash_Driver (no module dependencies, only HAL) ──────────────────────┘
```
