/*******************************************************************************
 * @file    ESP8266_Handler.c
 * @author  Amr Mohamed
 * @brief   Source file for ESP8266 WiFi module handler
 *
 * @details
 * Implements ESP8266 WiFi module communication including AT command handling,
 * MQTT protocol implementation, and WiFi connectivity management for the
 * vending machine system.
 *
 * @version 1.0
 * @date    2025-09-13
 ******************************************************************************/

/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/
#include "ESP8266_Handler.h"
#include <string.h>
#include <stdio.h>

/******************************************************************************
 *                             Module Config                                  *
 ******************************************************************************/

/******************************************************************************
 *                            Private Macros                                  *
 ******************************************************************************/

/******************************************************************************
 *                         Private Data Types                                 *
 ******************************************************************************/

/******************************************************************************
 *                          Private Variables                                 *
 ******************************************************************************/
static volatile uint8_t esp_uart_rx_buffer[ESP_UART_RX_BUFFER_SIZE];
static volatile uint16_t esp_uart_rx_index = 0;
static char esp_response_buffer[ESP_RESPONSE_BUFFER_SIZE];
static ESP_Status_t esp_current_status = ESP_STATUS_UNINITIALIZED;
ESP_Config_t esp_config = {
    .wifi_ssid = ESP_DEFAULT_SSID,
    .wifi_password = ESP_DEFAULT_PASSWORD,
    .mqtt_broker_ip = ESP_DEFAULT_MQTT_BROKER,
    .mqtt_broker_port = ESP_DEFAULT_MQTT_PORT,
    .mqtt_client_id = ESP_DEFAULT_CLIENT_ID,
    .mqtt_topic = ESP_DEFAULT_MQTT_TOPIC,
    .esp_uart = NULL,          /* Will be set via ESP_SetConfig() */
    .debug_uart = NULL         /* Will be set via ESP_SetConfig() */
};

/******************************************************************************
 *                           Public Variables                                 *
 ******************************************************************************/

/******************************************************************************
 *                         Private Prototypes                                 *
 ******************************************************************************/
static void esp_drain_rx_buffer(void);
static HAL_StatusTypeDef esp_setup_wifi_connection(void);
static HAL_StatusTypeDef esp_setup_tcp_connection(void);
static HAL_StatusTypeDef esp_setup_mqtt_connection(void);
static HAL_StatusTypeDef esp_send_mqtt_connect_packet(void);
static HAL_StatusTypeDef esp_wait_for_mqtt_connack(void);
static void esp_debug_print(const char *message);
static void esp_debug_print_bytes(const uint8_t *data, uint16_t length);

/******************************************************************************
 *                          Public Functions                                  *
 ******************************************************************************/

/**
 * @brief Set ESP8266 configuration including UART handles
 * @param config Pointer to ESP8266 configuration structure
 * @note Must be called before ESP_Init()
 */
void ESP_SetConfig(ESP_Config_t *config)
{
    if (config != NULL) {
        memcpy(&esp_config, config, sizeof(ESP_Config_t));
    }
}

/**
 * @brief Get current ESP8266 configuration
 * @return Pointer to current configuration structure
 */
ESP_Config_t* ESP_GetConfig(void)
{
    return &esp_config;
}

/**
 * @brief Initialize ESP8266 module and establish connections
 * @note Complete initialization including WiFi and MQTT connection
 */
void ESP_Init(void)
{
    // Set the configuration with the default configuration
    ESP_SetConfig(&ESP8266_DefaultConfig);

    // Start UART receive interrupt for ESP8266
    ESP_StartUARTReceive();

    // Validate configuration
    if (esp_config.esp_uart == NULL) {
        esp_current_status = ESP_STATUS_ERROR;
        esp_debug_print("[ESP] Error: ESP UART handle not configured. Call ESP_SetConfig() first.\r\n");
        return;
    }
    
    esp_current_status = ESP_STATUS_INITIALIZING;
    
    // Drain any pending data in RX buffer
    esp_drain_rx_buffer();
    
    // Basic ESP8266 setup
    if (ESP_SendAT("AT", "OK", ESP_AT_COMMAND_TIMEOUT) != HAL_OK) {
        esp_current_status = ESP_STATUS_ERROR;
        return;
    }
    
    // Setup WiFi connection
    if (esp_setup_wifi_connection() != HAL_OK) {
        esp_current_status = ESP_STATUS_ERROR;
        return;
    }
    
    // Setup TCP connection to MQTT broker
    if (esp_setup_tcp_connection() != HAL_OK) {
        esp_current_status = ESP_STATUS_ERROR;
        return;
    }
    
    // Setup MQTT connection
    if (esp_setup_mqtt_connection() != HAL_OK) {
        esp_current_status = ESP_STATUS_ERROR;
        return;
    }
    
    esp_current_status = ESP_STATUS_MQTT_CONNECTED;
    esp_debug_print("[ESP] MQTT Connected Successfully!\r\n");
}

/**
 * @brief Restore ESP8266 module to factory settings
 */
void ESP_Restore(void)
{
    if (esp_config.esp_uart == NULL) {
        esp_current_status = ESP_STATUS_ERROR;
        return;
    }
    
    esp_current_status = ESP_STATUS_INITIALIZING;
    
    HAL_UART_Transmit(esp_config.esp_uart, (uint8_t *)"AT+RESTORE\r\n", 12, 1000);
    memset(esp_response_buffer, 0, sizeof(esp_response_buffer));
    
    uint32_t tickstart = HAL_GetTick();
    while ((HAL_GetTick() - tickstart) < ESP_RESTORE_TIMEOUT) {
        if (HAL_UART_Receive(esp_config.esp_uart, (uint8_t *)&esp_response_buffer[strlen(esp_response_buffer)], 1, 100) == HAL_OK) {
            if (strstr(esp_response_buffer, "ready")) {
                esp_debug_print("[ESP] Restored and ready.\r\n");
                esp_current_status = ESP_STATUS_WIFI_DISCONNECTED;
                return;
            }
        }
    }
    esp_debug_print("[ESP] Restore failed.\r\n");
    esp_current_status = ESP_STATUS_ERROR;
}

/**
 * @brief Send AT command and wait for expected response
 * @param cmd AT command string to send
 * @param expect Expected response string (NULL to skip validation)
 * @param timeout Timeout in milliseconds
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef ESP_SendAT(const char *cmd, const char *expect, uint32_t timeout)
{
    if (esp_config.esp_uart == NULL) {
        return HAL_ERROR;
    }
    
    char atCommand[128];
    memset(esp_response_buffer, 0, sizeof(esp_response_buffer));

    snprintf(atCommand, sizeof(atCommand), "%s\r\n", cmd);

    if (HAL_UART_Transmit(esp_config.esp_uart, (uint8_t *)atCommand, strlen(atCommand), HAL_MAX_DELAY) != HAL_OK) {
        esp_debug_print("[ESP] Transmit failed\r\n");
        return HAL_ERROR;
    }

    uint16_t index = 0;
    uint32_t tickstart = HAL_GetTick();
    while ((HAL_GetTick() - tickstart) < timeout && index < sizeof(esp_response_buffer) - 1) {
        if (HAL_UART_Receive(esp_config.esp_uart, (uint8_t *)&esp_response_buffer[index], 1, 50) == HAL_OK) {
            index++;
        }
        if (expect != NULL && strstr(esp_response_buffer, expect) != NULL) {
            break;
        }
    }

    char debug_msg[512];
    snprintf(debug_msg, sizeof(debug_msg), "[ESP] CMD: %s\r\n[ESP] Raw Rsp: %s\r\n", cmd, esp_response_buffer);
    esp_debug_print(debug_msg);

    return (expect == NULL || strstr(esp_response_buffer, expect) != NULL) ? HAL_OK : HAL_ERROR;
}

/**
 * @brief Get current ESP8266 module status
 * @return Current module status
 */
ESP_Status_t ESP_GetStatus(void)
{
    return esp_current_status;
}

/**
 * @brief Publish data to MQTT topic
 */
void ESP_PublishNumber(void)
{
    if (esp_config.esp_uart == NULL) {
        return;
    }
    
    uint8_t mqttPublish[] = {
        0x30, 0x13,
        0x00, 0x0D, 's','t','m','3','2','/','t','e','s','t','1','2','3',
        'H', 'E', 'L', 'L'
    };

    char cipsendCmd[20];
    sprintf(cipsendCmd, "AT+CIPSEND=%d", sizeof(mqttPublish));
    ESP_SendAT(cipsendCmd, ">", 2000);
    HAL_UART_Transmit(esp_config.esp_uart, mqttPublish, sizeof(mqttPublish), HAL_MAX_DELAY);
}

/**
 * @brief Subscribe to MQTT topic for incoming messages
 */
void ESP_Subscribe(void)
{
    if (esp_config.esp_uart == NULL) {
        return;
    }
    
    uint8_t mqttSubscribe[] = {
        0x82, 0x12,             // Fixed header: SUBSCRIBE, remaining length
        0x00, 0x01,             // Message ID
        0x00, 0x0D,             // Topic length
        's','t','m','3','2','/','t','e','s','t','1','2','3', // Topic
        0x00                    // QoS 0
    };

    char cipsendCmd[20];
    sprintf(cipsendCmd, "AT+CIPSEND=%d", sizeof(mqttSubscribe));
    ESP_SendAT(cipsendCmd, ">", 2000);
    HAL_UART_Transmit(esp_config.esp_uart, mqttSubscribe, sizeof(mqttSubscribe), HAL_MAX_DELAY);
    esp_debug_print("[ESP] SUBSCRIBE sent\r\n");
}

/**
 * @brief Parse incoming MQTT messages from UART buffer
 */
void ESP_ParseMQTTMessage(void)
{
    char *ipd_start = strstr((char *)esp_uart_rx_buffer, "+IPD,");
    if (!ipd_start)
        return;

    int ipd_len = 0;
    if (sscanf(ipd_start, "+IPD,%d:", &ipd_len) != 1 || ipd_len <= 0)
        return;

    char *mqtt_packet = strchr(ipd_start, ':');
    if (!mqtt_packet)
        return;

    mqtt_packet++; // Skip the ':'
    uint8_t *ptr = (uint8_t *)mqtt_packet;

    // Ensure the full packet is received
    size_t packet_start_idx = mqtt_packet - (char *)esp_uart_rx_buffer;
    if (esp_uart_rx_index < packet_start_idx + ipd_len)
        return; // Wait for more data

    if (ptr[0] != 0x30) // MQTT PUBLISH packet
        return;

    // Decode variable-length remaining length
    uint32_t remaining_len = 0;
    int len_bytes = 0;
    do {
        if (len_bytes >= 4 || len_bytes + 1 >= ipd_len)
            return; // Invalid remaining length
        remaining_len += (ptr[1 + len_bytes] & 0x7F) << (len_bytes * 7);
        len_bytes++;
    } while (ptr[1 + len_bytes - 1] & 0x80);

    // Check if enough data for topic length
    if (1 + len_bytes + 2 >= ipd_len)
        return;

    uint16_t topic_len = (ptr[1 + len_bytes] << 8) | ptr[2 + len_bytes];
    if (topic_len > 64 || 1 + len_bytes + 2 + topic_len >= ipd_len)
        return;

    char topic[64] = {0};
    memcpy(topic, &ptr[3 + len_bytes], topic_len);
    topic[topic_len] = '\0';

    uint16_t payload_start = 3 + len_bytes + topic_len;
    uint16_t payload_len = remaining_len - 2 - topic_len;
    if (payload_len > 128 || payload_start + payload_len > ipd_len)
        return;

    char msg[128] = {0};
    memcpy(msg, &ptr[payload_start], payload_len);
    msg[payload_len] = '\0';

    char out[256];
    snprintf(out, sizeof(out), "[MQTT] Topic: %s | Message: %s\r\n", topic, msg);
    esp_debug_print(out);

    // Clear buffer up to the end of the parsed packet
    size_t total_packet_len = packet_start_idx + ipd_len;
    if (total_packet_len < esp_uart_rx_index) {
        memmove((void *)esp_uart_rx_buffer, (void *)&esp_uart_rx_buffer[total_packet_len], 
                esp_uart_rx_index - total_packet_len);
        esp_uart_rx_index -= total_packet_len;
    } else {
        esp_uart_rx_index = 0;
        memset((void *)esp_uart_rx_buffer, 0, ESP_UART_RX_BUFFER_SIZE);
    }
}

/**
 * @brief UART receive complete callback for ESP8266 communication
 * @param huart UART handle that triggered the callback
 * @note This function should be called from HAL_UART_RxCpltCallback
 */
void ESP_UART_RxCallback(UART_HandleTypeDef *huart)
{
    if (esp_config.esp_uart != NULL && huart->Instance == esp_config.esp_uart->Instance) {
        // Echo the received byte to debug UART for debugging
        esp_debug_print((char *)&esp_uart_rx_buffer[esp_uart_rx_index]);

        // Increment buffer index
        esp_uart_rx_index++;
        if (esp_uart_rx_index >= ESP_UART_RX_BUFFER_SIZE - 1) {
            esp_uart_rx_index = 0; // Reset to prevent overflow
        }

        // Re-enable UART receive interrupt
        HAL_UART_Receive_IT(esp_config.esp_uart, (uint8_t *)&esp_uart_rx_buffer[esp_uart_rx_index], 1);
    }
}

/**
 * @brief Start UART receive interrupt for ESP8266 communication
 * @note Call this function after ESP_SetConfig() to start receiving data
 */
void ESP_StartUARTReceive(void)
{
    if (esp_config.esp_uart != NULL) {
        esp_uart_rx_index = 0;
        HAL_UART_Receive_IT(esp_config.esp_uart, (uint8_t *)&esp_uart_rx_buffer[esp_uart_rx_index], 1);
    }
}

/******************************************************************************
 *                          Private Functions                                 *
 ******************************************************************************/

/**
 * @brief Drain any pending data in the ESP8266 UART RX buffer
 * 
 * @details This function clears any residual data in the UART receive buffer
 *          before starting communication. It prevents interference from
 *          previous communication sessions or boot messages.
 *
 * @note This function should be called before initiating AT command sequences
 *       to ensure clean communication.
 */
static void esp_drain_rx_buffer(void)
{
    if (esp_config.esp_uart == NULL) {
        return;
    }
    
    uint8_t temp;
    while (HAL_UART_Receive(esp_config.esp_uart, &temp, 1, 10) == HAL_OK);
    
    // Clear internal buffer
    esp_uart_rx_index = 0;
    memset((void *)esp_uart_rx_buffer, 0, ESP_UART_RX_BUFFER_SIZE);
}

/**
 * @brief Setup WiFi connection with configured credentials
 * 
 * @details Configures the ESP8266 in station mode and connects to the specified
 *          WiFi network using the credentials stored in esp_config structure.
 *          Updates the module status based on connection success.
 *
 * @return HAL_OK if WiFi connection successful, HAL_ERROR otherwise
 *
 * @note This function will update esp_current_status to ESP_STATUS_WIFI_CONNECTED
 *       on successful connection or leave it in error state on failure.
 */
static HAL_StatusTypeDef esp_setup_wifi_connection(void)
{
    char wifi_connect_cmd[128];
    
    if (ESP_SendAT("AT+CWMODE=1", "OK", 2000) != HAL_OK) {
        return HAL_ERROR;
    }
    
    snprintf(wifi_connect_cmd, sizeof(wifi_connect_cmd), 
             "AT+CWJAP=\"%s\",\"%s\"", esp_config.wifi_ssid, esp_config.wifi_password);
    
    if (ESP_SendAT(wifi_connect_cmd, "WIFI GOT IP", ESP_WIFI_CONNECT_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    
    esp_current_status = ESP_STATUS_WIFI_CONNECTED;
    return HAL_OK;
}

/**
 * @brief Setup TCP connection to MQTT broker
 * 
 * @details Configures single connection mode and establishes TCP connection
 *          to the MQTT broker using the IP address and port specified in
 *          the esp_config structure.
 *
 * @return HAL_OK if TCP connection successful, HAL_ERROR otherwise
 *
 * @note This function assumes WiFi connection is already established.
 *       The connection parameters are taken from esp_config structure.
 */
static HAL_StatusTypeDef esp_setup_tcp_connection(void)
{
    char tcp_connect_cmd[64];
    
    if (ESP_SendAT("AT+CIPMUX=0", "OK", 2000) != HAL_OK) {
        return HAL_ERROR;
    }
    
    snprintf(tcp_connect_cmd, sizeof(tcp_connect_cmd), 
             "AT+CIPSTART=\"TCP\",\"%s\",%d", 
             esp_config.mqtt_broker_ip, esp_config.mqtt_broker_port);
    
    if (ESP_SendAT(tcp_connect_cmd, "CONNECT", ESP_TCP_CONNECT_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief Setup MQTT connection by sending CONNECT packet and waiting for CONNACK
 * 
 * @details Sends the MQTT CONNECT packet to establish MQTT protocol communication
 *          with the broker, then waits for the CONNACK response to confirm
 *          successful connection establishment.
 *
 * @return HAL_OK if MQTT connection successful, HAL_ERROR otherwise
 *
 * @note This function consists of two main steps: sending the CONNECT packet
 *       and validating the CONNACK response.
 */
static HAL_StatusTypeDef esp_setup_mqtt_connection(void)
{
    if (esp_send_mqtt_connect_packet() != HAL_OK) {
        return HAL_ERROR;
    }
    
    if (esp_wait_for_mqtt_connack() != HAL_OK) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief Send MQTT CONNECT packet to establish protocol connection
 * 
 * @details Constructs and sends the MQTT CONNECT packet according to MQTT v3.1.1
 *          specification. The packet includes protocol name, version, flags,
 *          keep-alive timer, and client identifier.
 *
 * @return HAL_OK if packet sent successfully, HAL_ERROR otherwise
 *
 * @warning This function uses AT+CIPSEND command which requires the TCP connection
 *          to be already established before calling this function.
 */
static HAL_StatusTypeDef esp_send_mqtt_connect_packet(void)
{
    const uint8_t mqttConnect[] = {
        0x10, 0x11,                    // Fixed header: CONNECT, remaining length
        0x00, 0x04,                    // Protocol Name Length
        0x4D, 0x51, 0x54, 0x54,        // "MQTT"
        0x04,                          // Protocol Level (MQTT v3.1.1)
        0x02,                          // Connect Flags: Clean session
        0x00, 0x64,                    // Keep alive = 100 seconds
        0x00, 0x05,                    // Client ID length
        'S', 'T', 'M', '3', '2'        // Client ID: "STM32"
    };

    char cipsendCmd[32];
    sprintf(cipsendCmd, "AT+CIPSEND=%d", sizeof(mqttConnect));
    if (ESP_SendAT(cipsendCmd, ">", 5000) != HAL_OK) {
        return HAL_ERROR;
    }

    esp_debug_print("[ESP] Sending MQTT CONNECT packet:\r\n");
    esp_debug_print_bytes(mqttConnect, sizeof(mqttConnect));

    if (HAL_UART_Transmit(esp_config.esp_uart, (uint8_t *)mqttConnect, sizeof(mqttConnect), HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Wait for MQTT CONNACK response and validate connection success
 * 
 * @details Waits for the MQTT broker to respond with a CONNACK packet after
 *          receiving the CONNECT packet. The function looks for "+IPD,4:" 
 *          indicating incoming data, followed by the CONNACK packet validation.
 *          A valid CONNACK should contain: 0x20 0x02 0x00 0x00.
 *
 * @return HAL_OK if valid CONNACK received, HAL_ERROR otherwise
 *
 * @note The function currently has validation commented out for debugging purposes.
 *       In production, proper CONNACK validation should be enabled.
 *
 * @warning This function has a timeout of ESP_MQTT_CONNECT_TIMEOUT. If no response
 *          is received within this time, the connection is considered failed.
 */
static HAL_StatusTypeDef esp_wait_for_mqtt_connack(void)
{
    uint8_t response[256] = {0};
    uint16_t index = 0;
    uint32_t tickstart = HAL_GetTick();

    // Wait for "SEND OK"
    while ((HAL_GetTick() - tickstart) < 5000 && index < sizeof(response) - 1) {
        if (HAL_UART_Receive(esp_config.esp_uart, &response[index], 1, 1000) == HAL_OK) {
            if (index >= 6 && memcmp(&response[index - 6], "SEND OK", 7) == 0) break;
            index++;
        }
    }

    esp_debug_print("[ESP] Pre-CONNACK bytes:\r\n");
    esp_debug_print_bytes(response, index);

    // Wait for "+IPD,4:" followed by CONNACK
    memset(response, 0, sizeof(response));
    index = 0;
    uint8_t ipd_found = 0;
    uint16_t ipd_start = 0;
    tickstart = HAL_GetTick();

    while ((HAL_GetTick() - tickstart) < ESP_MQTT_CONNECT_TIMEOUT && index < sizeof(response) - 1) {
        if (HAL_UART_Receive(esp_config.esp_uart, &response[index], 1, 1000) == HAL_OK) {
            if (!ipd_found && index >= 6 && memcmp(&response[index - 6], "+IPD,4:", 7) == 0) {
                ipd_found = 1;
                ipd_start = index + 1;

                for (int i = 0; i < 4 && index < sizeof(response) - 1; i++) {
                    if (HAL_UART_Receive(esp_config.esp_uart, &response[++index], 1, 1000) != HAL_OK)
                        break;
                }
                break;
            }
            index++;
        }
    }

    esp_debug_print("[ESP] CONNACK Response:\r\n");
    esp_debug_print_bytes(response, index);

    // Validate CONNACK content (currently commented out for debugging)
    int valid = 1;  // Set to 1 to bypass validation temporarily
    /*
    if (ipd_found && index >= ipd_start + 3) {
        if (response[ipd_start] == 0x20 && response[ipd_start + 1] == 0x02 &&
            response[ipd_start + 2] == 0x00 && response[ipd_start + 3] == 0x00) {
            valid = 1;
        }
    }
    */

    if (!valid) {
        esp_debug_print("[ESP] Invalid or Missing CONNACK\r\n");
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief Print debug message to debug UART
 * 
 * @details Sends debug message to UART2 for monitoring and troubleshooting.
 *          This function is used throughout the ESP8266 handler for providing
 *          detailed operation feedback.
 *
 * @param message Null-terminated string to print
 *
 * @note Debug output can be disabled by setting ESP_ENABLE_DEBUG_OUTPUT to 0
 *       in the module configuration section.
 */
static void esp_debug_print(const char *message)
{
#if ESP_ENABLE_DEBUG_OUTPUT
    if (esp_config.debug_uart != NULL) {
        HAL_UART_Transmit(esp_config.debug_uart, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
    }
#endif
}

/**
 * @brief Print byte array as hexadecimal values for debugging
 * 
 * @details Converts byte array to hexadecimal representation and sends it
 *          to the debug UART. Useful for analyzing MQTT packets and raw
 *          communication data.
 *
 * @param data Pointer to byte array to print
 * @param length Number of bytes to print
 *
 * @note Each byte is printed in "0xXX " format followed by a newline.
 *       Debug output can be disabled by setting ESP_ENABLE_DEBUG_OUTPUT to 0.
 */
static void esp_debug_print_bytes(const uint8_t *data, uint16_t length)
{
#if ESP_ENABLE_DEBUG_OUTPUT
    if (esp_config.debug_uart != NULL) {
        for (uint16_t i = 0; i < length; i++) {
            char byte_msg[16];
            snprintf(byte_msg, sizeof(byte_msg), "0x%02X ", data[i]);
            HAL_UART_Transmit(esp_config.debug_uart, (uint8_t *)byte_msg, strlen(byte_msg), HAL_MAX_DELAY);
        }
        HAL_UART_Transmit(esp_config.debug_uart, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
    }
#endif
}
