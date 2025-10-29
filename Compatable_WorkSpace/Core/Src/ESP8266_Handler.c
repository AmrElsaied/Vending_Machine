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
uint8_t esp_binary_rx_buffer[ESP_UART_RX_BUFFER_SIZE];  // dedicated binary response buffer
uint8_t esp_uart_callback_buffer[ESP_UART_RX_BUFFER_SIZE];
volatile char esp_line_buffer[MAX_LINE_BUFFER_SIZE];
volatile uint16_t esp_expected_len = 0;
static uint16_t current_mqtt_msg_id = 0;

char g_mqtt_topic[MAX_TOPIC_BUFFER_SIZE];
char g_mqtt_message[MAX_MESSAGE_BUFFER_SIZE];

uint8_t esp_rx_byte;
static volatile uint16_t esp_rx_head = 0;
static volatile uint16_t esp_rx_tail = 0;
static volatile uint16_t esp_uart_rx_index = 0;

static char esp_response_buffer[ESP_RESPONSE_BUFFER_SIZE];
static ESP_Status_t esp_current_status = ESP_STATUS_UNINITIALIZED;
esp_buffer_state esp_buffer_current_state = ESP_BUFFER_CHECK_NOK;

ESP_Config_t esp_config = {
		.wifi_ssid = ESP_DEFAULT_SSID,
		.wifi_password = ESP_DEFAULT_PASSWORD,
		.mqtt_broker_ip = ESP_DEFAULT_MQTT_BROKER,
		.mqtt_broker_port = ESP_DEFAULT_MQTT_PORT,
		.mqtt_client_id = ESP_DEFAULT_CLIENT_ID,
		.mqtt_topic = ESP_DEFAULT_MQTT_TOPIC,
		.esp_uart = NULL,          /* Will be set via ESP_SetConfig() */
		.debug_uart = NULL,         /* Will be set via ESP_SetConfig() */
		.debug_enabled = false              /* Enable debug output by default */
};



static esp_rx_state_t esp_current_state = ESP_STATE_PARSING_HEADER;

/******************************************************************************
 *                           Public Variables                                 *
 ******************************************************************************/

/******************************************************************************
 *                         Private Prototypes                                 *
 ******************************************************************************/
static void esp_drain_rx_buffer(void);
static HAL_StatusTypeDef esp_setup_wifi_connection(void);
static HAL_StatusTypeDef esp_setup_tcp_connection(void);
static HAL_StatusTypeDef esp_send_mqtt_connect_packet(void);
static void esp_debug_print(const char *message);

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


	HAL_Delay(2000);
	// Validate configuration
	if (esp_config.esp_uart == NULL) {
		esp_current_status = ESP_STATUS_ERROR;
		//esp_debug_print("[ESP] Error: ESP UART handle not configured. Call ESP_SetConfig() first.\r\n");
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

	//    if (ESP_SendAT("ATE0", "OK", ESP_AT_COMMAND_TIMEOUT) != HAL_OK) {
	//            esp_current_status = ESP_STATUS_ERROR;
	//            return;
	//        }


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
	if (esp_send_mqtt_connect_packet() != HAL_OK) {
		esp_current_status = ESP_STATUS_ERROR;
		return;
	}

	esp_current_status = ESP_STATUS_MQTT_CONNECTED;
	//esp_debug_print("[ESP] MQTT Connected Successfully!\r\n");


	ESP_Subscribe(&(esp_config.mqtt_topic), 1);
	HAL_Delay(3000);
	ESP_Subscribe("stm32/sub1", 1);
	HAL_Delay(3000);
	ESP_Subscribe("stm32/sub2", 1);
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
		//esp_debug_print("[ESP] UART not initialized\r\n");
		return HAL_ERROR;
	}

	// Validate inputs
	if (cmd == NULL || *cmd == '\0') {
		//esp_debug_print("[ESP] Invalid command\r\n");
		return HAL_ERROR;
	}

	uint16_t idx = 0;
	uint32_t tickstart = HAL_GetTick();
	bool found = false;
	const uint32_t busy_delay_ms = 1500;

	// Clear response buffer efficiently
	esp_response_buffer[0] = '\0';

	// Build AT command with bounds checking
	char atCommand[128];
	int cmd_len = snprintf(atCommand, sizeof(atCommand), "%s\r\n", cmd);
	if (cmd_len < 0 || (size_t)cmd_len >= sizeof(atCommand)) {
		//esp_debug_print("[ESP] Command too long\r\n");
		return HAL_ERROR;
	}

	//esp_debug_print("[ESP] Sending: ");
	//esp_debug_print(atCommand);

	// Transmit command
	if (HAL_UART_Transmit(esp_config.esp_uart, (uint8_t *)atCommand, (uint16_t)cmd_len, HAL_MAX_DELAY) != HAL_OK) {
		//esp_debug_print("[ESP] Transmit failed\r\n");
		return HAL_ERROR;
	}

	// Pre-calculate lengths for efficiency
	const char *busy_str = "busy";
	const size_t busy_len = strlen(busy_str);
	const size_t expect_len = (expect != NULL) ? strlen(expect) : 0;
	const size_t buffer_size = sizeof(esp_response_buffer);

	// Receive loop
	while ((HAL_GetTick() - tickstart) < timeout && idx < buffer_size - 1) {
		uint8_t ch;

		if (HAL_UART_Receive(esp_config.esp_uart, &ch, 1, 10) == HAL_OK) {
			// Store character
			esp_response_buffer[idx++] = ch;
			esp_response_buffer[idx] = '\0';

			// Only check for patterns if we have enough data
			if (idx >= busy_len && !found) {
				// Check for busy response (only check recent characters to avoid scanning entire buffer)
				if (idx >= busy_len &&
						memcmp(&esp_response_buffer[idx - busy_len], busy_str, busy_len) == 0) {
					//esp_debug_print("[ESP] Module busy, retrying...\r\n");
					HAL_Delay(busy_delay_ms);

					// Reset for retry
					idx = 0;
					esp_response_buffer[0] = '\0';
					tickstart = HAL_GetTick();

					// Retransmit command
					if (HAL_UART_Transmit(esp_config.esp_uart, (uint8_t *)atCommand, (uint16_t)cmd_len, HAL_MAX_DELAY) != HAL_OK) {
						//esp_debug_print("[ESP] Retransmit failed\r\n");
						return HAL_ERROR;
					}
					continue;
				}

				// Check for expected response
				if (expect != NULL && idx >= expect_len) {
					// Only search the recent portion where the response could be
					size_t search_start = (idx > expect_len + 32) ? idx - expect_len - 32 : 0;
					if (strstr(&esp_response_buffer[search_start], expect) != NULL) {
						char dbg[64];
						snprintf(dbg, sizeof(dbg), "[ESP] Matched ACK: %s\r\n", expect);
						//esp_debug_print(dbg);
						found = true;
						// Don't break - continue to collect remaining data
					}
				}
			}
		}

		// Early exit if we found what we're looking for and no more data is coming
		if (found) {
			// Check if we've had a period without new data (indicates end of response)
			uint32_t data_timeout = 100; // ms without new data
			uint32_t data_tick = HAL_GetTick();
			while ((HAL_GetTick() - data_tick) < data_timeout && idx < buffer_size - 1) {
				if (HAL_UART_Receive(esp_config.esp_uart, &ch, 1, 10) == HAL_OK) {
					esp_response_buffer[idx++] = ch;
					esp_response_buffer[idx] = '\0';
					data_tick = HAL_GetTick(); // Reset timeout
				}
			}
			break;
		}
	}

	// Print response if we have data
	if (idx > 0) {
		//esp_debug_print("[ESP] Raw Response:\r\n");
		//esp_debug_print(esp_response_buffer);
	} else {
		//esp_debug_print("[ESP] No response received\r\n");
	}

	// Evaluate result
	if (found) {
		return HAL_OK;
	} else if (idx == 0) {
		return HAL_TIMEOUT;
	} else {
		//esp_debug_print("[ESP] Response didn't match expected pattern\r\n");
		return HAL_ERROR;
	}
}

HAL_StatusTypeDef ESP_SendBinary(uint8_t *bin, size_t len, const char *expect, uint32_t timeout)
{

	if (esp_config.esp_uart == NULL) {
		//esp_debug_print("[ESP] UART not initialized\r\n");
		return HAL_ERROR;
	}

	if (bin == NULL && len > 0) {
		//esp_debug_print("[ESP] Invalid binary data\r\n");
		return HAL_ERROR;
	}

	uint16_t idx = 0;
	uint32_t tickstart = HAL_GetTick();
	bool found = false;
	const char *error_str = "ERROR";
	const size_t error_len = strlen(error_str);
	const size_t expect_len = (expect != NULL) ? strlen(expect) : 0;
	const size_t buffer_size = sizeof(esp_binary_rx_buffer);

	// Clear buffer efficiently (only first byte for string functions)
	esp_binary_rx_buffer[0] = '\0';

	// Transmit binary data
	if (len > 0) {
		//esp_debug_print("[ESP] Sending Binary Packet (");

		char len_msg[32];
		snprintf(len_msg, sizeof(len_msg), "%lu bytes)\r\n", (unsigned long)len);
		//esp_debug_print(len_msg);

		if (HAL_UART_Transmit(esp_config.esp_uart, bin, len, HAL_MAX_DELAY) != HAL_OK) {
			//esp_debug_print("[ESP] Binary transmit failed\r\n");
			return HAL_ERROR;
		}
	}

	// Receive response
	while ((HAL_GetTick() - tickstart) < timeout && idx < buffer_size - 1) {
		uint8_t ch;

		if (HAL_UART_Receive(esp_config.esp_uart, &ch, 1, 10) == HAL_OK) {
			esp_binary_rx_buffer[idx++] = ch;
			esp_binary_rx_buffer[idx] = '\0'; // Keep string safe for strstr

			// Debug print (optional - can be disabled for production)
#if ESP_DEBUG_VERBOSE
			char dbg[16];
			snprintf(dbg, sizeof(dbg), "[BIN] 0x%02X\r\n", ch);
			//esp_debug_print(dbg);
#endif

			// Only check for patterns when we have enough data
			if (idx >= error_len) {
				// Check for ERROR response first (highest priority)
				if (memcmp(&esp_binary_rx_buffer[idx - error_len], error_str, error_len) == 0) {
					//esp_debug_print("[ESP] Link Error: Disconnected\r\n");
					return HAL_ERROR;
				}

				// Check for expected ACK
				if (!found && expect != NULL && idx >= expect_len) {
					// Search only the recent portion of the buffer
					size_t search_start = (idx > expect_len + 16) ? idx - expect_len - 16 : 0;
					if (strstr((char *)&esp_binary_rx_buffer[search_start], expect) != NULL) {
						char dbg_ack[64];
						snprintf(dbg_ack, sizeof(dbg_ack), "[ESP] Matched ACK: %s\r\n", expect);
						//esp_debug_print(dbg_ack);
						found = true;

						// Continue to collect any remaining data but with shorter timeout
						timeout = HAL_GetTick() - tickstart + 100; // Give 100ms more
					}
				}
			}
		}
	}

	// Evaluate result
	if (found) {
		//esp_debug_print("[ESP] Binary Response OK\r\n");
#if ESP_DEBUG_VERBOSE
		//esp_debug_print("Response: ");
		//esp_debug_print((char *)esp_binary_rx_buffer);
		//esp_debug_print("\r\n");
#endif
		return HAL_OK;
	}

	if (idx == 0) {
		//esp_debug_print("[ESP] No response received (timeout)\r\n");
		return HAL_TIMEOUT;
	}

	//esp_debug_print("[ESP] Timeout or No ACK\r\n");
#if ESP_DEBUG_VERBOSE
	//esp_debug_print("Received: ");
	//esp_debug_print((char *)esp_binary_rx_buffer);
	//esp_debug_print("\r\n");
#endif

	return HAL_ERROR;
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
void ESP_Publish(const char *topic, const char *message, uint8_t qos)
{
	//esp_debug_print("[ESP_PUBLISH] >>> Entering ESP_Publish()\r\n");
	HAL_UART_AbortReceive(esp_config.esp_uart);
	// NOTE: Changed esp_config.esp_uart to esp_config_uart for global pointer compatibility.
	if (esp_config.esp_uart == NULL || topic == NULL || message == NULL)
	{
		//esp_debug_print("[ESP_PUBLISH] ERROR: Invalid argument(s)\r\n");
		return;
	}

	uint16_t topic_len = strlen(topic);
	uint16_t message_len = strlen(message);

	// --- Calculate Remaining Length ---
	// Topic length(2) + topic + message [+ MsgID(2) if QoS>0]
	uint16_t remaining_length = 2 + topic_len + message_len + (qos ? 2 : 0);

	// --- Build MQTT PUBLISH packet ---
	uint8_t packet[512];
	uint16_t index = 0;

	uint8_t header = 0x30 | (qos << 1); // 0x30 for PUBLISH, QoS bits in position 1–2
	packet[index++] = header;
	packet[index++] = remaining_length;

	// --- Topic ---
	packet[index++] = (topic_len >> 8) & 0xFF;
	packet[index++] = topic_len & 0xFF;
	memcpy(&packet[index], topic, topic_len);
	index += topic_len;

	// --- Packet ID (if QoS > 0) ---
	if (qos > 0)
	{
		packet[index++] = 0x00;
		packet[index++] = 0x01; // Using a fixed Packet ID of 1
	}

	// --- Message payload ---
	memcpy(&packet[index], message, message_len);
	index += message_len;

	// --- Send packet ---
	char cipsendCmd[32];
	sprintf(cipsendCmd, "AT+CIPSEND=%d", index);

	//esp_debug_print("[ESP_PUBLISH] Sending AT+CIPSEND command...\r\n");
	// Uses the synchronous ESP_SendAT
	if (ESP_SendAT(cipsendCmd, ">", 2000) != HAL_OK)
	{
		//esp_debug_print("[ESP_PUBLISH] ERROR: Failed to get '>' prompt\r\n");
		return;
	}

	//esp_debug_print("[ESP_PUBLISH] Sending MQTT PUBLISH packet...\r\n");
	// NOTE: Changed esp_config.esp_uart to esp_config_uart
	HAL_UART_Transmit(esp_config.esp_uart, packet, index, HAL_MAX_DELAY);
	//esp_debug_print("[ESP_PUBLISH] MQTT PUBLISH packet sent\r\n");

	// --- Wait for PUBACK if QoS = 1 ---
	if (qos == 1)
	{
		uint8_t rx_byte;
		uint32_t start_tick = HAL_GetTick();
		//esp_debug_print("[ESP_PUBLISH] Waiting for PUBACK (0x40)...\r\n");

		while (HAL_GetTick() - start_tick < 5000)  // 5 sec timeout
		{
			// NOTE: Changed esp_config.esp_uart to esp_config_uart
			if (HAL_UART_Receive(esp_config.esp_uart, &rx_byte, 1, 100) == HAL_OK)
			{
				if (rx_byte == 0x40)
				{
					//esp_debug_print("[ESP_PUBLISH] PUBACK received!\r\n");
					ESP_StartUARTReceive();
					//esp_debug_print("[ESP_PUBLISH] <<< Exiting ESP_Publish()\r\n");
					return;
				}
			}
		}
	}

	//esp_debug_print("[ESP_PUBLISH] <<< Exiting ESP_Publish() without cleanup or re-arming ASYNC\r\n");

}



/**
 * @brief Subscribe to MQTT topic for incoming messages
 */
HAL_StatusTypeDef ESP_Subscribe(const char *topic, uint8_t qos)
{
    // --- 1. Input Validation ---
	if (esp_config.esp_uart == NULL || topic == NULL) {
		//esp_debug_print("[ESP] Invalid parameters for SUBSCRIBE\r\n");
		return HAL_ERROR;
	}

	uint16_t topic_len = strlen(topic);
	if (topic_len == 0) {
		//esp_debug_print("[ESP] Empty topic for SUBSCRIBE\r\n");
		return HAL_ERROR;
	}

    // --- 2. Generate Unique Message ID (The Primary Fix) ---
    // QoS 1 and 2 require a unique message ID (non-zero).
    current_mqtt_msg_id++;
    if (current_mqtt_msg_id == 0) {
        current_mqtt_msg_id = 1; // Ensure ID is never 0 (reserved/invalid)
    }
    uint16_t message_id = current_mqtt_msg_id;

    // --- 3. Calculate Remaining Length ---
    // Remaining Length = Msg ID (2) + Topic Length Field (2) + Topic String (topic_len) + QoS Byte (1)
	uint16_t remaining_length = 2 + 2 + topic_len + 1;

	// Check if packet fits in buffer (Fixed Header size is 2 bytes for single-byte RL)
	if (remaining_length + 2 > 256) {
		//esp_debug_print("[ESP] Topic too long for SUBSCRIBE\r\n");
		return HAL_ERROR;
	}

    // NOTE: This assumes remaining_length < 128 (single byte encoding), which is safe for one topic.

	// --- 4. Build SUBSCRIBE Packet ---
	uint8_t packet[256];
	uint16_t index = 0;

	// Fixed header (SUBSCRIBE = 0x82 for QoS 1, or 0x80 for QoS 0)
    // The top 4 bits are 1000 (SUBSCRIBE). The bottom 4 are flags (e.g., QoS).
    // The control type is 0x80 (SUBSCRIBE) ORed with the flags.
	packet[index++] = 0x82; // Assuming QoS 1 is intended here (0x80 | 0x02)
	packet[index++] = (uint8_t)remaining_length; // Remaining Length

    // Variable header (UNIQUE MESSAGE ID)
	packet[index++] = (message_id >> 8) & 0xFF;  // Message ID MSB
	packet[index++] = message_id & 0xFF;         // Message ID LSB

    // Payload (List of Subscriptions)
	packet[index++] = (topic_len >> 8) & 0xFF;  // Topic length MSB
	packet[index++] = topic_len & 0xFF;         // Topic length LSB

	// Topic String
	memcpy(&packet[index], topic, topic_len);
	index += topic_len;

    // Requested QoS
	packet[index++] = qos;

    // --- 5. Send AT Command to Initiate Data Transfer ---
	char cipsendCmd[32];
	snprintf(cipsendCmd, sizeof(cipsendCmd), "AT+CIPSEND=%d", index);

	HAL_StatusTypeDef ret = ESP_SendAT(cipsendCmd, ">", 2000);
	if (ret != HAL_OK) {
		// ... [error handling] ...
		return ret;
	}

    // --- 6. Send Binary Data ---
	ret = ESP_SendBinary(packet, index, "SEND OK", 5000);

    // --- 7. Check Status and Handle Acknowledgment (CRITICAL) ---
	if (ret == HAL_OK) {

        // ***************************************************************
        // CRITICAL SECTION: SUBACK WAITING AND STATE CLEARING
        // This is the most likely reason for the second packet failing.
        // ***************************************************************

        // 1. Wait for SUBACK: The broker sends this packet (0x90) with the matching Message ID.
        // If you don't read this response, the ESP module's buffer may overflow
        // or its state machine will be confused, leading to a "malformed packet"
        // error when you send the next CIPSEND command.

        // Placeholder for your implementation:
        // ret = ESP_WaitForSUBACK(message_id, 5000);

        // 2. State Clearance Delay: Give the ESP module a small, fixed delay
        // to complete its internal processing of the received SUBACK packet.
        HAL_Delay(500); // 500ms is a safe buffer

		char success_msg[128];
		snprintf(success_msg, sizeof(success_msg), "[ESP] SUBSCRIBE sent for topic: %s (ID: %d). State cleared.\r\n",
				topic, message_id);
		//esp_debug_print(success_msg);
		//ESP_StartUARTReceive(); // Re-enable general receive if necessary

        // If you implement ESP_WaitForSUBACK, return its result here.
	} else {
		char error_msg[128];
		snprintf(error_msg, sizeof(error_msg), "[ESP] SUBSCRIBE send failed to topic: %s (error: %d)\r\n",
				topic, ret);
		//esp_debug_print(error_msg);
	}
	return ret;
}

/**
 * @brief UART receive complete callback for ESP8266 communication
 * @param huart UART handle that triggered the callback
 * @note This function should be called from HAL_UART_RxCpltCallback
 */
/**
 * @brief UART receive complete callback for ESP8266 communication
 * @param huart UART handle that triggered the callback
 * @note Collects bytes into esp_uart_rx_buffer, parses +IPD length, and processes full packet
 */
/**
 * @brief UART receive complete callback for ESP8266 communication
 * @param huart UART handle that triggered the callback
 * @note Collects bytes into esp_uart_rx_buffer, parses +IPD length, and processes full packet
 */
void ESP_UART_RxCallback(UART_HandleTypeDef *huart)
{

	// Guard clause: Ensure it's the correct UART instance
	if (esp_config.esp_uart == NULL || huart->Instance != esp_config.esp_uart->Instance) {
		return;
	}

	// --- State: Receiving a bulk payload (Triggered when the full transfer is COMPLETE) ---
	if (esp_current_state == ESP_STATE_RECEIVING_PAYLOAD) {

		uint8_t *payload_ptr = esp_uart_callback_buffer;
		uint16_t current_payload_len = esp_expected_len;

		bool parse_ok = true;

		//esp_debug_print("\r\n--- Starting Payload Parsing ---\r\n");

		// 1. Ignore the first two bytes and check length
		if (current_payload_len < 4) {
			//esp_debug_print("[ESP] Error: Payload too short for Topic/Message parsing.\r\n");
			parse_ok = false;
		}

		if (parse_ok) {
			payload_ptr += 2; // Skip first two bytes
			current_payload_len -= 2;

			// 2. Detect the Topic Length (Next 2 Bytes, Big-Endian)
			// Topic Length = (MSB << 8) | LSB
			uint16_t topic_len = (uint16_t)(payload_ptr[0] << 8) | payload_ptr[1];

			payload_ptr += 2; // Move past topic length bytes
			current_payload_len -= 2;

			// 3. Validate Topic Length against remaining payload
			if (current_payload_len < topic_len) {
				//esp_debug_print("[ESP] Error: Topic length exceeds remaining payload data.\r\n");
				parse_ok = false;
			}

			// 4. Extract and Store the Topic in Global Buffer
			if (parse_ok) {

				uint16_t copy_len = topic_len;

				// Safety check: Truncate topic if it exceeds global buffer size
				if (copy_len >= MAX_TOPIC_BUFFER_SIZE) {
					//esp_debug_print("[ESP] Warning: Topic truncated due to buffer limit.\r\n");
					copy_len = MAX_TOPIC_BUFFER_SIZE - 1;
				}

				// Original VLA (now removed): char topic_buffer[topic_len + 1];

				memcpy(g_mqtt_topic, payload_ptr, copy_len);
				g_mqtt_topic[copy_len] = '\0'; // Null-terminate the global buffer

//				char dbg[128];
//				snprintf(dbg, sizeof(dbg), "[ESP] Detected Topic: %s (Length: %u)\r\n", g_mqtt_topic, topic_len);
				//esp_debug_print(dbg);

				payload_ptr += topic_len; // Move past the original topic string length
				current_payload_len -= topic_len;

				// 5. Extract and Store the Message (as String) in Global Buffer
				uint16_t message_len = current_payload_len;

				if (message_len > 0 && message_len < MAX_MESSAGE_BUFFER_SIZE) {
					// Original VLA (now removed): char msg_buffer[message_len + 1];

					// Copy to global message buffer and null-terminate
					memcpy(g_mqtt_message, payload_ptr, message_len);
					g_mqtt_message[message_len] = '\0'; // Null-terminate the global buffer

//					char dbg_line[128];
//					snprintf(dbg_line, sizeof(dbg_line), "[ESP] Detected Message: %s\r\n", g_mqtt_message);
					//esp_debug_print(dbg_line);



				} else if (message_len == 0) {
					//esp_debug_print("[ESP] Detected Message: (Empty)\r\n");
					// If you want to re-publish empty messages, the call would go here as well.
				} else {
					//esp_debug_print("[ESP] Error: Message length too large or invalid.\r\n");
				}
			} // Now g_mqtt_topic and g_mqtt_message persist
		}

		// --- Cleanup and State Reset (Always executed after payload completion) ---
		//esp_debug_print("--- Payload Parse Complete ---\r\n");

		esp_expected_len = 0;
		esp_current_state = ESP_STATE_PARSING_HEADER;

		// Re-arm interrupt to look for the single byte that starts the next header ('+')
		HAL_UART_Receive_IT(esp_config.esp_uart, &esp_rx_byte, 1);

		return;
	}

	// --- State: Parsing the "+IPD,<len>:" header (Single byte reception) ---
	if (esp_current_state == ESP_STATE_PARSING_HEADER) {
		// ... (rest of the header parsing logic remains the same)
		static uint8_t state = 0;
		static char len_str[6];
		static uint8_t len_idx = 0;
		uint8_t ch = esp_rx_byte;

		// --- Detect "+IPD,<len>:" pattern ---
		switch (state) {
		case 0: // Looking for '+'
			if (ch == '+') state = 1;
			break;
		case 1: // Expecting 'I'
			state = (ch == 'I') ? 2 : 0;
			break;
		case 2: // Expecting 'P'
			state = (ch == 'P') ? 3 : 0;
			break;
		case 3: // Expecting 'D'
			state = (ch == 'D') ? 4 : 0;
			break;
		case 4: // Expecting ','
			state = (ch == ',') ? 5 : 0;
			break;
		case 5: // Collect digits until ':'
			if (ch >= '0' && ch <= '9') {
				if (len_idx < sizeof(len_str) - 1)
					len_str[len_idx++] = ch;
			} else if (ch == ':') {
				len_str[len_idx] = '\0';
				esp_expected_len = (uint16_t)atoi(len_str);

				// Cleanup header parser state
				len_idx = 0;
				state = 0;

				if (esp_expected_len > 0 && esp_expected_len <= ESP_UART_RX_BUFFER_SIZE) {

					// **FIX: Length detected, IMMEDIATELY start bulk reception**
					esp_current_state = ESP_STATE_RECEIVING_PAYLOAD;
					HAL_UART_Receive_IT(esp_config.esp_uart, esp_uart_callback_buffer, esp_expected_len);

					// We return here because the next interrupt will be for the full payload,
					// and it needs to hit the RECEIVING_PAYLOAD state block above.
					return;
				}
			} else {
				// Invalid char -> reset parser state
				state = 0;
				len_idx = 0;
			}
			break;
		}

		// If still in the PARSING_HEADER state, re-arm for the next single byte.
		HAL_UART_Receive_IT(esp_config.esp_uart, &esp_rx_byte, 1);
	}
}




/**
 * @brief Start UART receive interrupt for ESP8266 communication
 * @note Call this function after ESP_SetConfig() to start receiving data
 */
void ESP_StartUARTReceive(void)
{
	if (esp_config.esp_uart == NULL)
	{
		//esp_debug_print("[ESP] Error: UART handle NULL in Start\r\n");
		return;
	}



	esp_rx_head = 0;
	esp_rx_tail = 0;

	if (HAL_UART_Receive_IT(esp_config.esp_uart, &esp_rx_byte, 1) != HAL_OK)
	{
		//esp_debug_print("[ESP] Failed to start RX interrupt\r\n");
	}
	else{
		//esp_debug_print("[ESP] UART receive interrupt started\r\n");
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
	char cipdomain_cmd[64];

	//    if (ESP_SendAT("AT+CIPMODE=1", "OK", 2000) != HAL_OK) {
	//               //esp_debug_print("[ESP] Failed to enable transparent mode\r\n");
	//               return HAL_ERROR;
	//           }

	if (ESP_SendAT("AT+CIPMUX=0", "OK", 2000) != HAL_OK) {
		return HAL_ERROR;
	}

	snprintf(cipdomain_cmd, sizeof(cipdomain_cmd),
	             "AT+CIPDOMAIN=\"%s\"", esp_config.mqtt_broker_ip);

	if (ESP_SendAT(cipdomain_cmd, "OK",  5000) != HAL_OK) {
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
	sprintf(cipsendCmd, "AT+CIPSEND=%d", (int)sizeof(mqttConnect));

	//esp_debug_print("[ESP] Preparing MQTT CONNECT packet...\r\n");

	// Step 1: Send AT+CIPSEND and wait for '>'
	if (ESP_SendAT(cipsendCmd, ">", 5000) != HAL_OK) {
		//esp_debug_print("[ESP] AT+CIPSEND failed (no '>')\r\n");
		return HAL_ERROR;
	}

	// Step 2: Send MQTT CONNECT packet using the binary sender
	if (ESP_SendBinary((uint8_t *)mqttConnect, sizeof(mqttConnect), "\x20", 8000) != HAL_OK) {
		//esp_debug_print("[ESP] MQTT CONNECT send failed or CONNACK not received\r\n");
		return HAL_ERROR;
	}

	//esp_debug_print("[ESP] MQTT CONNECT packet sent successfully and CONNACK detected\r\n");
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
	if (esp_config.debug_enabled && esp_config.debug_uart != NULL) {
		HAL_UART_Transmit(esp_config.debug_uart, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
	}
}


void PingREQ(void){

	HAL_UART_AbortReceive(esp_config.esp_uart);

	if (esp_config.esp_uart == NULL)
	{
		//esp_debug_print("[ESP] Error: UART handle NULL in Start\r\n");
		return;
	}

	char cmd[32];
	uint8_t packet[2];

	//esp_debug_print("Sending PINGREQ\r\n");
	packet[0] = 0xC0; 	packet[1] = 0x00;  // PINGREQ
	snprintf(cmd, sizeof(cmd), "AT+CIPSEND=2");

	if(ESP_SendAT(cmd, ">", 2000) != HAL_OK){
		//esp_debug_print("Failed to send CIPSEND\r\n");
		return;
	}

	if(ESP_SendBinary(packet, 2, "\xD0", 2000) != HAL_OK){
		//esp_debug_print("Failed to send PINGREQ\r\n");
		return;
	}

	//esp_debug_print("Sent PINGREQ\r\n");
	ESP_StartUARTReceive();

}


esp_buffer_state GetBuffer_State(void){
	if(g_mqtt_message[0] == 'O' && g_mqtt_message[1] == 'K'){
		esp_buffer_current_state = ESP_BUFFER_CHECK_START;
	}
	else if(g_mqtt_message[0] == 'N' && g_mqtt_message[1] == 'K'){
		esp_buffer_current_state = ESP_BUFFER_CHECK_STOP;
	}
	else
	{
		esp_buffer_current_state = ESP_BUFFER_CHECK_NOK;
	}
	return esp_buffer_current_state;
}

void ResetBuffer_State(void) {

	if(esp_buffer_current_state ==  ESP_BUFFER_CHECK_START){

		memset(g_mqtt_message, 0, MAX_MESSAGE_BUFFER_SIZE);
		memset(g_mqtt_topic, 0, MAX_MESSAGE_BUFFER_SIZE);

		esp_buffer_current_state = ESP_BUFFER_CHECK_NOK;
	}
}
