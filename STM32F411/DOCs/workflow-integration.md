# Workflow & Integration Reference

## 1. Boot Sequence

`main()` runs entirely before the FreeRTOS scheduler starts.

```
main()
  │
  ├─ HAL_Init()                         HAL tick timer, NVIC priority grouping
  ├─ SystemClock_Config()               25 MHz HSE → PLL → 84 MHz SYSCLK
  ├─ MX_GPIO_Init()                     LEDs (PB12/13/14), Reset button (PB10 EXTI)
  ├─ MX_USART1_UART_Init()             9600 baud, 9-bit  (MDB)
  ├─ MX_USART2_UART_Init()             115200 baud, 8-bit (ESP8266)
  ├─ MX_USART6_UART_Init()             115200 baud, 8-bit (debug)
  ├─ FLASH_Init()                       Validate user Flash sector
  ├─ LED_CONTROLLER_Init()              Configure GPIO, all LEDs off
  ├─ SYS_InitLogger()                   Bind USART6, zero error buffer
  ├─ LED_On(LED_CHANNEL_PWR)            PWR LED on → hardware alive
  │
  ├─ ESP_Init()    ◄── BLOCKING ──────────────────────────────────────────┐
  │    ├─ LoadWiFiConfig() from Flash 0x08010000                          │
  │    ├─ If no credentials → ESP_AP_Mode() + ESP_WaitForCredentials()    │
  │    │    └─ BUTTON_PollPreScheduler() runs here (bare-metal button)    │
  │    ├─ AT reset + WiFi connect (timeout 10 s)                          │
  │    ├─ TCP connect to broker (timeout 10 s)                            │
  │    └─ MQTT CONNECT handshake (timeout 10 s)  ──────────────────────  │
  │                                                                        │
  ├─ MDB_BusInit()                      Arm USART1 RX interrupt           │
  │                                                                        │
  ├─ osKernelInitialize()               Init CMSIS-RTOS V2 kernel         │
  ├─ osThreadNew(StartDefaultTask)      CMSIS default task                │
  ├─ System_TaskCreate()                Create all 5 tasks + espPublishQueue
  └─ osKernelStart()                    ◄── scheduler takes over          │
```

After `osKernelStart()` returns (it never does under normal operation), all execution is driven by FreeRTOS task scheduling.

---

## 2. MDB Reception & Response Flow

Covers the path from a hardware byte arriving on USART1 to the VMC sending its response.

```
[MDB Peripheral]
      │  9-bit UART word (USART1, 9600 baud)
      ▼
HAL_UART_RxCpltCallback()                          [Core/Src/main.c]
  └─► MDB_UART_RxCallback(word)                    [MDB_Handler.c]
        ├─ mdbRing_push(&rxRing, word)
        └─ xTaskNotifyFromISR(mdbRxTaskHandle)
                │
                ▼  (unblocks mdbRxTask, priority 53)
mdbRxTask
  ├─ Bus-timeout check: if ring empty > MDB_BUS_TIMEOUT (10 ms) → reset RX state
  ├─ mdbRing_pop(&rxRing, &word)
  └─ MDB_ReceiveCommand(word)                      [MDB_Handler.c]
        ├─ Advance CMD_RX_State machine
        └─ On complete frame:
             xTaskNotify(mdbCMDProcessTaskHandle, cmd_index)
                │
                ▼  (unblocks mdbCMDProcessTask, priority 54)
mdbCMDProcessTask
  └─ MDB_HandleCommand(cmd_index)                  [MDB_Handler.c]
        └─ handle_cmd_0xXXXX()
              ├─ Look up VMC_CMDs[enum_index]       [VMC_Config.c]
              ├─ Build dynamic fields if needed
              └─ MDB_SendResponseWithModeBit()
                    └─ HAL_UART_Transmit_IT(USART1, ...)
                              │
                              ▼
                    [MDB Peripheral receives response]
```

---

## 3. MQTT Inbound Message Flow

Covers the path from a TCP byte arriving on USART2 to a keyword handler executing.

```
[MQTT Broker]
      │  Binary MQTT PUBLISH packet (USART2, 115200 baud)
      ▼
HAL_UART_RxCpltCallback()                          [Core/Src/main.c]
  └─► ESP_UART_RxCallback(byte)                    [ESP8266_Handler.c]
        ├─ Accumulate byte in receive buffer
        ├─ Parse MQTT packet header (type + remaining-length)
        ├─ Extract topic → g_mqtt_topic[]
        ├─ Extract payload → g_mqtt_message[]
        └─ On payload_ready:
             xTaskNotifyFromISR(espMqttProcessTaskHandle)
                │
                ▼  (unblocks espMqttProcessTask, priority 51)
espMqttProcessTask
  ├─ ESP_ProcessMQTTMessage(g_mqtt_topic, g_mqtt_message)
  │     ├─ Match topic against topic_handlers[] table
  │     └─ Match keyword prefix in message against SubscribeMainTopicKeywordHandlers[]
  │           ├─ "ACTIVATION:" → handle_vend_request_topic()
  │           │     └─ SetVendReq_State(VEND_REQ_APPROVED / VEND_REQ_DENIED)
  │           └─ "ping"       → handle_ping_topic()
  │                 └─ ESP_RequestPublish(publish_topic, pong_message, qos)
  └─ Clear g_mqtt_topic[] and g_mqtt_message[]
```

---

## 4. MQTT Outbound Publish Flow

Any module queues a publish without blocking. The dedicated task serialises all transmissions.

```
Any module (e.g., MDB_Handler vend result, ping handler)
  └─ ESP_RequestPublish(topic, message, qos)        [system_tasks.c]
        └─ xQueueSend(&espPublishQueue, &req, 0)    non-blocking
                │
                ▼  (espCommunicationTask unblocks, priority 52)
espCommunicationTask
  └─ ESP_Publish(req.topic, req.message, req.qos)   [ESP8266_Handler.c]
        ├─ Build binary MQTT PUBLISH packet
        └─ ESP_SendBinary(packet, len)
              └─ HAL_UART_Transmit(USART2, ...)
                        │
                        ▼
              [MQTT Broker receives message]

  If espPublishQueue empty for 40 s (ESP_PING_TIMEOUT_MS):
    └─ Publish keep-alive ping to ESP_MQTT_TOPIC_MAIN_PUBLISH
```

---

## 5. Vend Transaction End-to-End

Shows how a remote vend command translates into a physical cashless payment approval.

```
[Server]
  │  MQTT PUBLISH on "server-publish/84c4def7-87ec-4f7c-9ea0-dda68adeb977"
  │  Payload: "ACTIVATION:<Vend_Amount>:<session_id>"
  ▼
ESP_UART_RxCallback (accumulates packet)
  └─► espMqttProcessTask
        └─► handle_vend_request_topic()
              ├─ Parse Vend_Amount, session_id
              ├─ Store in Vending_Item_Data (MDB_Handler.c globals)
              └─ SetVendReq_State(VEND_REQ_APPROVED)

[MDB Peripheral sends vend request command 0x0075]
  └─► mdbRxTask → mdbCMDProcessTask
        └─► handle_cmd_0x0075()
              ├─ GetVendReq_State() == VEND_REQ_APPROVED
              ├─ Build approve-vend response with item price
              └─ MDB_SendResponseWithModeBit()   → [MDB Peripheral]

[MDB Peripheral completes vend / reports outcome]
  └─► handle_cmd_0x0076() (session end)
        ├─ Read Vending_Item_Data.Vend_Item_State
        └─ ESP_RequestPublish("vmc-publish/84c4def7-87ec-4f7c-9ea0-dda68adeb977",
                              "VEND_SUCCESS / VEND_FAIL + details", 0)
              └─► [Server receives result]
```

---

## 6. Factory Reset Flow

```
User holds PB10 (Reset Button) for ≥ 2 seconds
  │
  ├─ EXTI interrupt fires on button press
  │    └─ HAL_GPIO_EXTI_Callback(GPIO_PIN_10)      [main.c]
  │          └─ PERIODIC_TASK_ButtonInterrupt()    records press timestamp
  │
  └─ PeriodicMainTask tick (every 100 ms)
       └─ PERIODIC_TASK_Execute()
             └─ BUTTON_PeriodicCallback()
                   ├─ Debounce: button held > BUTTON_DEBOUNCE_TIME_MS (20 ms)
                   ├─ Press duration > BUTTON_LONG_PRESS_TIME_MS (2000 ms), released:
                   │    ├─ SaveWiFiConfig("EMPTY_CONFIG", "EMPTY_CONFIG")  [ESP8266_Handler.c]
                   │    │    └─ Writes sentinel to Flash 0x08010000 via Flash_Driver
                   │    └─ HAL_NVIC_SystemReset()        hard reset
                   └─ (short press: no action)
```

After reset, `ESP_Init()` finds the `"EMPTY_CONFIG"` sentinel in Flash and enters AP configuration mode.

---

## 7. FreeRTOS IPC Summary

| Mechanism | Instance | Producer | Consumer | Purpose |
|-----------|----------|----------|----------|---------|
| Task notification | `mdbRxTaskHandle` | `MDB_UART_RxCallback` (ISR) | `mdbRxTask` | Signal new MDB word in ring |
| Task notification | `mdbCMDProcessTaskHandle` | `mdbRxTask` | `mdbCMDProcessTask` | Pass completed command index |
| Task notification | `espMqttProcessTaskHandle` | `ESP_UART_RxCallback` (ISR) | `espMqttProcessTask` | Signal complete MQTT payload |
| Queue (`espPublishQueue`) | depth 10 | Any task via `ESP_RequestPublish` | `espCommunicationTask` | Serialise outbound MQTT publishes |

No mutexes or semaphores are used. Ring-buffer thread safety in MDB relies on single-writer (ISR) / single-reader (task) discipline. Global MQTT buffers (`g_mqtt_topic`, `g_mqtt_message`) are protected by the task-notification handshake: the ISR does not re-arm receive until `espMqttProcessTask` clears the buffers.
