# Periodic_Task_Manager — 100 ms Cyclic Operation Dispatcher

## Purpose

Provides a lightweight sub-scheduler running inside `PeriodicMainTask`. Modules register callbacks with a desired period; `PERIODIC_TASK_Execute()` is called every 100 ms and dispatches each callback when its period has elapsed.

Currently manages: button debounce, factory-reset long-press detection, and communication LED monitoring.

Source: [Core/Src/Periodic_Task_Manager.c](../Core/Src/Periodic_Task_Manager.c), [Core/Inc/Periodic_Task_Manager.h](../Core/Inc/Periodic_Task_Manager.h)

---

## Key Types

### `periodic_module_t` — Module Identifiers

```c
// Core/Inc/Periodic_Task_Manager.h
typedef enum {
    PERIODIC_MODULE_BUTTON,
    PERIODIC_MODULE_SYSTEM_MONITOR,
    PERIODIC_MODULE_COMMUNICATION,
    PERIODIC_MODULE_MAX,
} periodic_module_t;
```

### `periodic_task_config_t` — Callback Registration Record

```c
typedef struct {
    periodic_module_t   module_id;
    periodic_callback_t callback;    // void (*)(uint32_t delta_ms)
    uint32_t            period_ms;   // Desired execution period
    bool                enabled;
} periodic_task_config_t;
```

### `periodic_task_instance_t` (internal)

Extends `periodic_task_config_t` with runtime state:
- `last_execution_time` — FreeRTOS tick count at last call
- `accumulated_time` — running accumulator for sub-100 ms periods

---

## Configuration Macros

| Macro | Value | Purpose |
|-------|-------|---------|
| `PERIODIC_TASK_BASE_PERIOD_MS` | 100 | Base period of `PeriodicMainTask` (ms) |
| `BUTTON_DEBOUNCE_TIME_MS` | 20 | Debounce window for reset button |
| `BUTTON_LONG_PRESS_TIME_MS` | 2000 | Long-press threshold to trigger factory reset |
| `MAX_PERIODIC_MODULES` | 5 | Maximum number of registerable modules |

---

## Public API

| Function | Signature | Purpose |
|----------|-----------|---------|
| `PERIODIC_TASK_Init` | `void PERIODIC_TASK_Init(void)` | Clear registry; initialize button GPIO state |
| `PERIODIC_TASK_Register` | `bool PERIODIC_TASK_Register(periodic_task_config_t*)` | Add a module to the registry; returns false if registry is full |
| `PERIODIC_TASK_Unregister` | `void PERIODIC_TASK_Unregister(periodic_module_t)` | Remove module from registry |
| `PERIODIC_TASK_Enable` | `void PERIODIC_TASK_Enable(periodic_module_t)` | Enable a registered module |
| `PERIODIC_TASK_Disable` | `void PERIODIC_TASK_Disable(periodic_module_t)` | Disable a registered module (callback not called) |
| `PERIODIC_TASK_IsEnabled` | `bool PERIODIC_TASK_IsEnabled(periodic_module_t)` | Query enabled state |
| `PERIODIC_TASK_Execute` | `void PERIODIC_TASK_Execute(void)` | Called every 100 ms by `PeriodicMainTask`; dispatches due callbacks |
| `PERIODIC_TASK_GetModuleCount` | `uint8_t PERIODIC_TASK_GetModuleCount(void)` | Return number of currently registered modules |
| `PERIODIC_TASK_ButtonInterrupt` | `void PERIODIC_TASK_ButtonInterrupt(void)` | Called from `HAL_GPIO_EXTI_Callback`; records button press timestamp |
| `BUTTON_PollPreScheduler` | `void BUTTON_PollPreScheduler(void)` | Bare-metal button poll used during `ESP_Init()` before FreeRTOS starts |

---

## Registered Callbacks

### `BUTTON_PeriodicCallback` (period: 100 ms)

- Reads GPIO PB10 (active low).
- 20 ms debounce: ignores transitions shorter than `BUTTON_DEBOUNCE_TIME_MS`.
- Long-press detection: if button held for ≥ `BUTTON_LONG_PRESS_TIME_MS` (2000 ms) and then released:
  1. Calls `SaveWiFiConfig(ESP_DEFAULT_UNCONFIGURED_SSID, ESP_DEFAULT_UNCONFIGURED_PASSWORD)` — writes the `"EMPTY_CONFIG"` sentinel to Flash, marking credentials as absent.
  2. Calls `HAL_NVIC_SystemReset()` — hard system reset.
- Edge detection is assisted by `PERIODIC_TASK_ButtonInterrupt()` which sets `button_interrupt_flag` and records the press timestamp from the EXTI ISR.

> **Note:** Factory reset does not erase the Flash sector. It overwrites the SSID/password fields with the `"EMPTY_CONFIG"` sentinel value so `ESP_Init()` recognises unconfigured credentials on the next boot.

GPIO: PB10, configured as EXTI with pull-up. See [Core/Inc/main.h](../Core/Inc/main.h) (`RESET_BUTTON_Pin = GPIO_PIN_10`).

### `PERIODIC_TASK_CommunicationMonitoringCallback` (period: 100 ms)

- Monitors `ESP_GetStatus()` and drives the COMM LED (PB13) accordingly:
  - `ESP_STATUS_MQTT_CONNECTED` → start `LED_Blink(LED_CHANNEL_COMM, 900 ms)` (once, guarded by `Comm_Led_Blinking` flag), then call `LED_PeriodicUpdate()` every tick to advance the blink timer.
  - Any other state → `LED_StopBlinking(LED_CHANNEL_COMM)` + clear `Comm_Led_Blinking` flag (LED goes off).

> COMM LED state summary: **blinking at 900 ms** = MQTT connected; **off** = not connected.

---

## Dispatch Mechanism

```
PeriodicMainTask
  └─ vTaskDelay(100 ms)
  └─ PERIODIC_TASK_Execute()
       ├─ For each registered module (up to MAX_PERIODIC_MODULES):
       │    ├─ Skip if !enabled
       │    ├─ Accumulate delta since last call
       │    ├─ If accumulated_time >= period_ms:
       │    │    ├─ Call callback(delta_ms)
       │    │    └─ Reset accumulated_time
       └─ (repeat next 100 ms)
```

Because the base period is 100 ms, callbacks with `period_ms` = 100 are called every cycle. Callbacks with `period_ms` = 200, 500, etc. are called every 2nd, 5th cycle respectively.

---

## Inter-Module Dependencies

| Uses | For |
|------|-----|
| `ESP8266_Handler.h` | `SaveWiFiConfig()` in factory reset; `ESP_GetStatus()` in communication monitoring callback |
| `LED_Controller.h` | COMM LED blink control in communication monitoring callback |
| `main.h` | HAL GPIO for button read |
