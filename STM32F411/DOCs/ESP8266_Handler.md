# ESP8266_Handler — WiFi & MQTT Communication Driver

## Purpose

Manages the ESP8266 WiFi module for network connectivity and MQTT-based communication with the vending machine back-end server. Handles AT commands for initialization and WiFi association, then switches to raw binary MQTT packet exchange for all data traffic.

Source: [Core/Src/ESP8266_Handler.c](../Core/Src/ESP8266_Handler.c), [Core/Inc/ESP8266_Handler.h](../Core/Inc/ESP8266_Handler.h)  
Config: [Core/Src/ESP8266_PBCFG.c](../Core/Src/ESP8266_PBCFG.c)

---

## Key Types

### `ESP_Status_t` — Module State Machine

```c
// Core/Inc/ESP8266_Handler.h
typedef enum {
    ESP_STATUS_UNINITIALIZED,
    ESP_STATUS_INITIALIZING,
    ESP_STATUS_WIFI_DISCONNECTED,
    ESP_STATUS_WIFI_CONNECTED,
    ESP_STATUS_MQTT_DISCONNECTED,
    ESP_STATUS_MQTT_CONNECTED,
    ESP_STATUS_CONFIGURING,     // AP mode active, waiting for credentials
    ESP_STATUS_ERROR,
} ESP_Status_t;
```

### `ESP_Config_t` — Runtime Configuration

```c
// Core/Inc/ESP8266_Handler.h
typedef struct {
    char wifi_ssid[32];
    char wifi_password[64];
    char mqtt_broker_ip[32];
    uint16_t mqtt_broker_port;
    char mqtt_client_id[16];
    UART_HandleTypeDef *esp_uart;   // Bound to USART2 in ESP8266_PBCFG.c
} ESP_Config_t;
```

Default values are defined in `ESP8266_PBCFG.c` and listed in the [Configuration Macros](#configuration-macros) section below.

### `esp_rx_state_t` — UART RX State

```c
typedef enum {
    ESP_STATE_PARSING_HEADER,
    ESP_STATE_RECEIVING_PAYLOAD,
} esp_rx_state_t;
```

### `esp_buffer_state` — Buffer Validation State

```c
typedef enum {
    ESP_BUFFER_CHECK_START,
    ESP_BUFFER_CHECK_STOP,
    ESP_BUFFER_CHECK_NOK,
} esp_buffer_state;
```

### `esp_topic_index_t` — Topic Index Enum

```c
typedef enum {
    ESP_TOPIC_MAIN_SUBSCRIBE,
    ESP_TOPIC_MAIN_PUBLISH,
    ESP_TOPIC_MAXNUM,
} esp_topic_index_t;
```

### `esp_error_type_t` — ESP Error Types

```c
typedef enum {
    ESP_ERROR_NONE = 0,
    ESP_ERROR_TIMEOUT,
    ESP_ERROR_INVALID_CMD_RESPONSE,
} esp_error_type_t;
```

### `ESP_PublishRequest_t` — MQTT Publish Descriptor

```c
// Core/Inc/system_tasks.h  (used by ESP handler)
typedef struct {
    char topic[ESP_TASK_MAX_TOPIC_LEN];     // max 64 chars
    char message[ESP_TASK_MAX_MESSAGE_LEN]; // max 128 chars
    uint8_t qos;
} ESP_PublishRequest_t;
```

### `mqtt_topic_entry_t` — Topic Handler Registry Entry

```c
// Core/Src/ESP8266_Handler.c  (internal)
typedef struct {
    const char *topic_pattern;
    mqtt_topic_handler_t handler;
    esp_topic_type_t topic_type;    // PUBLISH_TOPIC, SUBSCRIBE_TOPIC, or MULTI_TOPIC
    const char *description;
} mqtt_topic_entry_t;
```

---

## Configuration Macros

### Default Connection Parameters (`ESP8266_PBCFG.c` / `ESP8266_Handler.h`)

| Macro | Value | Purpose |
|-------|-------|---------|
| `ESP_DEFAULT_SSID` | `"DEFAULT_SSID"` | Default WiFi SSID (override in PBCFG) |
| `ESP_DEFAULT_PASSWORD` | `"DEFAULT_PASS"` | Default WiFi password (override in PBCFG) |
| `ESP_DEFAULT_UNCONFIGURED_SSID` | `"EMPTY_CONFIG"` | Sentinel value — triggers AP mode on boot |
| `ESP_DEFAULT_UNCONFIGURED_PASSWORD` | `"EMPTY_CONFIG"` | Sentinel value |
| `ESP_DEFAULT_MQTT_BROKER` | `"vpn.silicon-mind.com"` | MQTT broker hostname |
| `ESP_DEFAULT_MQTT_PORT` | `1883` | MQTT broker port |
| `ESP_DEFAULT_CLIENT_ID` | `"STM32"` | MQTT client identifier |
| `MQTT_PACKET_USERNAME` | `"84c4def7-87ec-4f7c-9ea0-dda68adeb977"` | MQTT username (machine UUID) |
| `MQTT_PACKET_KEEPALIVE` | `60` | MQTT keep-alive interval (seconds) |

### MQTT Topic Strings

| Macro | Value | Direction |
|-------|-------|-----------|
| `ESP_MQTT_TOPIC_MAIN_SUBSCRIBE` | `"server-publish/84c4def7-87ec-4f7c-9ea0-dda68adeb977"` | Inbound (server → VMC) |
| `ESP_MQTT_TOPIC_MAIN_PUBLISH` | `"vmc-publish/84c4def7-87ec-4f7c-9ea0-dda68adeb977"` | Outbound (VMC → server) |

### Buffer Sizes

| Macro | Value | Purpose |
|-------|-------|---------|
| `ESP_UART_RX_BUFFER_SIZE` | 512 | UART receive ring buffer |
| `ESP_RESPONSE_BUFFER_SIZE` | 512 | AT command response scratch buffer |
| `ESP_TOPIC_MAX_LENGTH` | 64 | Max MQTT topic string length |
| `ESP_MESSAGE_MAX_LENGTH` | 128 | Max MQTT message payload length || `MAX_LINE_BUFFER_SIZE` | 128 | Line parse buffer |
| `MAX_TOPIC_BUFFER_SIZE` | 128 | Global topic buffer (`g_mqtt_topic`) |
| `MAX_MESSAGE_BUFFER_SIZE` | 128 | Global message buffer (`g_mqtt_message`) |
| `MAX_SESSION_ID_SIZE` | 45 | Session UUID string (8-4-4-4-12 + null) |
| `RX_BUF_LEN` | 64 | AT command receive scratch buffer |
| `MAX_SSID_LEN` | 32 | Maximum SSID string length |
| `MAX_PASS_LEN` | 64 | Maximum password string length |
### Timeouts

| Macro | Value | Purpose |
|-------|-------|---------|
| `ESP_AT_COMMAND_TIMEOUT` | 1000 ms | Generic AT command response wait |
| `ESP_WIFI_CONNECT_TIMEOUT` | 10000 ms | WiFi association wait |
| `ESP_TCP_CONNECT_TIMEOUT` | 10000 ms | TCP socket open to broker |
| `ESP_MQTT_CONNECT_TIMEOUT` | 10000 ms | MQTT CONNECT handshake |
| `ESP_RESTORE_TIMEOUT` | 5000 ms | Factory restore AT command wait |
| `ESP_PING_TIMEOUT_MS` | 40000 ms | Idle period before keep-alive ping |

---

## Public API

| Function | Purpose |
|----------|---------|
| `ESP_Init()` | Full initialization: AT reset → WiFi connect → TCP open → MQTT CONNECT. Blocking. Called from `main()` before scheduler start. |
| `ESP_Restore()` | Send AT+RESTORE to factory-reset the ESP8266 module |
| `ESP_SetConfig(ESP_Config_t*)` | Overwrite runtime configuration |
| `ESP_GetConfig()` | Return pointer to active `ESP_Config_t` |
| `ESP_SendAT(cmd, expected, timeout)` | Send AT command string; wait for expected response substring |
| `ESP_SendBinary(data, len, expect, timeout)` | Send raw bytes and optionally wait for expected response (MQTT packets) |
| `ESP_Publish(topic, message, qos)` | Build MQTT PUBLISH packet and transmit; called by `espCommunicationTask` |
| `ESP_Subscribe(topic, qos)` | Build MQTT SUBSCRIBE packet and transmit |
| `ESP_GetStatus()` | Return current `ESP_Status_t` |
| `ESP_AP_Mode()` | Switch ESP to AP mode for credential configuration |
| `ESP_WaitForCredentials()` | Block until new WiFi credentials are received via HTTP |
| `ESP_StartUARTReceive()` | Arm the first UART RX interrupt after config is set |
| `PingREQ()` | Build and send a raw MQTT PINGREQ packet |
| `ResetMessageBuffer()` | Clear vend request state and global MQTT message buffers |
| `ESP_UART_RxCallback(huart)` | Called from `HAL_UART_RxCpltCallback`; accumulates bytes and detects complete MQTT packets |
| `ESP_ProcessMQTTMessage(topic, message)` | Match topic and dispatch to registered keyword handler; called by `espMqttProcessTask` |
| `SaveWiFiConfig(ssid, password)` | Write SSID + password strings to Flash at `0x08010000` |
| `LoadWiFiConfig(ESP_Config_t*)` | Read config from Flash; returns false if slot contains sentinel |
| `esp_setup_wifi_connection()` | AT sequence for WiFi association (used by critical error recovery) |
| `esp_setup_tcp_connection()` | AT sequence for TCP connection to broker (used by critical error recovery) |
| `esp_send_mqtt_connect_packet()` | Build and send MQTT CONNECT packet (used by critical error recovery) |

---

## MQTT Message Handling

### Subscribe Topic Keyword Handlers

Incoming messages on `ESP_MQTT_TOPIC_MAIN_SUBSCRIBE` are scanned for these keyword prefixes:

| Keyword | Handler | Action |
|---------|---------|--------|
| `"ACTIVATION:"` | `handle_vend_request_topic()` | Parse item ID and price, set vend approval state via `SetVendReq_State()` |
| `"ping"` | `handle_ping_topic()` | Publish a status/pong response |

Unknown keywords are ignored (no error logged — by design, to tolerate future server additions).

### Binary MQTT Packet Reception

The ESP8266 is configured for transparent TCP passthrough. Received bytes are accumulated in a ring buffer inside `ESP_UART_RxCallback()`. Detection flow:

1. First byte identifies MQTT packet type (PUBLISH = 0x30).
2. Remaining length field is decoded (1–4 bytes, variable encoding).
3. Once `payload_ready` flag is set, `xTaskNotifyFromISR(espMqttProcessTaskHandle)` is called.
4. `espMqttProcessTask` calls `ESP_ProcessMQTTMessage()` with `g_mqtt_topic[]` and `g_mqtt_message[]`.

Global buffers: `g_mqtt_topic[MAX_TOPIC_BUFFER_SIZE]`, `g_mqtt_message[MAX_MESSAGE_BUFFER_SIZE]`, `g_session_id[MAX_SESSION_ID_SIZE]`.

---

## WiFi Credential Storage (Flash)

| Operation | Function | Flash Address |
|-----------|----------|---------------|
| Save | `SaveWiFiConfig(ssid, password)` | `0x08010000` (FLASH_CONFIG_ADDR) |
| Load | `LoadWiFiConfig(ESP_Config_t*)` | `0x08010000` |
| Clear (factory reset) | `SaveWiFiConfig(ESP_DEFAULT_UNCONFIGURED_SSID, ESP_DEFAULT_UNCONFIGURED_PASSWORD)` | Writes `"EMPTY_CONFIG"` sentinel to `0x08010000` |

`SaveWiFiConfig()` takes two string parameters (`const char *ssid`, `const char *password`) — not a struct pointer. It writes via `Flash_Driver`.

If Flash contains the sentinel value `"EMPTY_CONFIG"` as the SSID, `ESP_Init()` switches to AP mode and calls `ESP_WaitForCredentials()`.

---

## Keep-Alive / Ping Watchdog

`espCommunicationTask` uses a 40-second (`ESP_PING_TIMEOUT_MS`) queue receive timeout. If no publish request arrives within that window, it publishes a ping message to the main publish topic. This prevents the broker from closing the connection due to inactivity (MQTT keep-alive = 60 s).

---

## Error Codes Used

Logged via `ESP_LOG_ERROR()` / `ESP_LOG_CRITICAL_ERROR()`. See [SYS_Logger.md](SYS_Logger.md).

| Code | Meaning |
|------|---------|
| `ESP_WIFI_ERROR_CONNECTION_FAILED` (0x0B) | WiFi association failed |
| `ESP_TCP_ERROR_CONNECTION_FAILED` (0x0C) | TCP connection to broker failed |
| `ESP_MQTT_ERROR_CONNECTION_FAILED` (0x0D) | MQTT CONNECT handshake failed |
| `ESP_SERVER_ERROR_LOSE_CONNECTION` (0x0E) | MQTT server disconnected unexpectedly |
| `ESP_INTERNAL_COMM_ERROR` (0x0F) | ESP8266 internal AT command failure |
| `ESP_ERROR_UART_NOT_CONFIGURED` (0x10) | UART handle is NULL at init |

---

## Inter-Module Dependencies

| Uses | For |
|------|-----|
| `Flash_Driver.h` | Load/save/erase WiFi credentials |
| `LED_Controller.h` | COMM LED state changes during connect/disconnect |
| `SYS_Logger.h` | Error and debug logging |
| `main.h` | HAL types |
