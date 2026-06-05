# System_Tasks — FreeRTOS Task Orchestration

## Purpose

Creates and owns all five application-level FreeRTOS tasks and the inter-task `espPublishQueue`. Provides `ESP_RequestPublish()` as the single API for any module to queue an outbound MQTT message without directly touching the ESP layer.

Source: [Core/Src/system_tasks.c](../Core/Src/system_tasks.c), [Core/Inc/system_tasks.h](../Core/Inc/system_tasks.h)

---

## Task Registry

| Task Name | Handle | Priority | Stack (words) | Wake Trigger |
|-----------|--------|----------|---------------|--------------|
| `mdbRxTask` | `mdbRxTaskHandle` | MAX−3 (53) | 384 | `xTaskNotifyFromISR` from `MDB_UART_RxCallback` |
| `mdbCMDProcessTask` | `mdbCMDProcessTaskHandle` | MAX−2 (54) | 384 | `xTaskNotify` from `mdbRxTask` (carries cmd index) |
| `espCommunicationTask` | `espCommunicationTaskHandle` | MAX−4 (52) | 600 | `xQueueReceive` on `espPublishQueue` (40 s timeout) |
| `espMqttProcessTask` | `espMqttProcessTaskHandle` | MAX−5 (51) | 512 | `xTaskNotifyFromISR` from `ESP_UART_RxCallback` |
| `PeriodicMainTask` | `periodicMainTaskHandle` | MAX−10 (46) | 512 | `vTaskDelay(100 ms)` fixed period |

`configMAX_PRIORITIES` = 56, so MAX−2 = 54 is the effective highest user priority.

---

## Task Descriptions

### `mdbRxTask` (Priority 53)

- Unblocks on UART ISR notification carrying a 9-bit MDB word.
- Checks for bus timeout: if ring buffer is empty for more than `MDB_BUS_TIMEOUT` (10 ms) ticks, resets the receive state machine.
- Drains the ring buffer word by word, calling `MDB_ReceiveCommand(word)` for each.
- When `MDB_ReceiveCommand` frames a complete command it calls `xTaskNotify(mdbCMDProcessTaskHandle, cmd_index)`.

### `mdbCMDProcessTask` (Priority 54)

- Unblocks with the command index passed as the notification value.
- Calls `MDB_HandleCommand(cmd_index)`, which selects the appropriate handler and sends the MDB response.
- Runs at higher priority than `mdbRxTask` so a completed command is processed before the next reception window.

### `espCommunicationTask` (Priority 52)

- Blocks on `xQueueReceive(&espPublishQueue, &req, pdMS_TO_TICKS(ESP_PING_TIMEOUT_MS))`.
- On message received: calls `ESP_Publish(req.topic, req.message, req.qos)`.
- On 40-second queue timeout: publishes a keep-alive ping to prevent broker disconnect.
- Stack is larger (600 words) because `ESP_Publish` builds MQTT binary frames on the stack.

### `espMqttProcessTask` (Priority 51)

- Unblocks on task notification from `ESP_UART_RxCallback` when `payload_ready` is set.
- Calls `ESP_ProcessMQTTMessage(g_mqtt_topic, g_mqtt_message)`.
- Clears `g_mqtt_topic` and `g_mqtt_message` after processing so stale data is never re-used.

### `PeriodicMainTask` (Priority 46)

- Calls `PERIODIC_TASK_Execute()` then sleeps for 100 ms via `vTaskDelay`.
- Lowest-priority user task; all real-time work is done by higher-priority tasks.
- See [Periodic_Task_Manager.md](Periodic_Task_Manager.md) for the callbacks it dispatches.

---

## IPC: `espPublishQueue`

| Attribute | Value |
|-----------|-------|
| Type | FreeRTOS queue (`QueueHandle_t`) |
| Item type | `ESP_PublishRequest_t` |
| Depth | `ESP_PUBLISH_QUEUE_SIZE` = 10 |
| Producer | Any task / module via `ESP_RequestPublish()` |
| Consumer | `espCommunicationTask` exclusively |

```c
// Core/Inc/system_tasks.h
#define ESP_PUBLISH_QUEUE_SIZE  10
```

### `ESP_RequestPublish()`

```c
// Core/Src/system_tasks.c
void ESP_RequestPublish(const char *topic, const char *message, uint8_t qos);
```

- Fills an `ESP_PublishRequest_t` on the caller's stack.
- Calls `xQueueSend(&espPublishQueue, &req, 0)` — non-blocking; drops the request if the queue is full (full-queue condition is logged).
- Safe to call from any task context. **Not** ISR-safe (do not call from interrupt context).

### `ESP_GetPendingPublishCount()`

```c
uint32_t ESP_GetPendingPublishCount(void);
```

Returns the number of items currently waiting in `espPublishQueue`. Useful for diagnostics.

---

## Task Creation

`System_TaskCreate()` is called once from `main.c` after `osKernelInitialize()` and before `osKernelStart()`.

**Creation order:**
1. Create `espPublishQueue` — if this fails, `SYSTASK_ERROR_QUEUE_CREATION_FAILED` (0x0A) is logged and the system halts.
2. `mdbRxTask`
3. `mdbCMDProcessTask`
4. `espCommunicationTask`
5. `espMqttProcessTask`
6. `PeriodicMainTask`

Any `xTaskCreate` failure logs `SYSTASK_ERROR_TASK_CREATION_FAILED` (0x09) via `SYS_LogError()`.

---

## Inter-Module Dependencies

| Uses | For |
|------|-----|
| `MDB_Handler.h` | `MDB_ReceiveCommand()`, `MDB_HandleCommand()`, task handle exposure |
| `ESP8266_Handler.h` | `ESP_Publish()`, `ESP_ProcessMQTTMessage()`, task handle exposure |
| `Periodic_Task_Manager.h` | `PERIODIC_TASK_Execute()` |
| `SYS_Logger.h` | Task/queue creation failure logging |
