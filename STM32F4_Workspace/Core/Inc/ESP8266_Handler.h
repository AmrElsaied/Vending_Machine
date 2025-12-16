/*******************************************************************************
 * @file    ESP8266_Handler.h
 * @author  Amr Mohamed
 * @brief   Header file for ESP8266 WiFi module handler
 *
 * @details
 * This file contains the declarations for ESP8266 WiFi module communication,
 * including AT command handling, MQTT protocol implementation, and WiFi
 * connectivity management for the vending machine system.
 *
 * @version 1.0
 * @date    2025-09-13
 ******************************************************************************/

#ifndef ESP8266_HANDLER_H_
#define ESP8266_HANDLER_H_

/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
/* FreeRTOS core */
#include "FreeRTOS.h"
#include "task.h"
/******************************************************************************
 *                             Module Config                                  *
 ******************************************************************************/
/* UART Configuration */
#define ESP_PING_TIMEOUT_MS             (40000)     /* 40 seconds ping timeout */
#define ESP_TASK_MAX_TOPIC_LEN          (64)        /* Maximum topic length */
#define ESP_TASK_MAX_MESSAGE_LEN        (128)       /* Maximum message length */

/******************************************************************************
 *                          Macros & Constants                                *
 ******************************************************************************/
#define ESP_UART_RX_BUFFER_SIZE     512  /* Size of ESP UART receive buffer */
#define ESP_RESPONSE_BUFFER_SIZE    512  /* Size of AT command response buffer */
#define ESP_TOPIC_MAX_LENGTH        64   /* Maximum MQTT topic length */
#define ESP_MESSAGE_MAX_LENGTH      128  /* Maximum MQTT message length */
#define MAX_LINE_BUFFER_SIZE 		128
#define MAX_TOPIC_BUFFER_SIZE 		128
#define MAX_MESSAGE_BUFFER_SIZE 	128

/* Default WiFi Credentials */
#define ESP_DEFAULT_SSID            "MA_HOME"
#define ESP_DEFAULT_PASSWORD        "01289878405"

#define ESP_DEFAULT_UNCONFIGURED_SSID            "EMPTY_CONFIG"
#define ESP_DEFAULT_UNCONFIGURED_PASSWORD        "EMPTY_CONFIG"

/* Default MQTT Broker Settings */
#define ESP_DEFAULT_MQTT_BROKER     "192.168.1.106"
#define ESP_DEFAULT_MQTT_PORT       1883

#define ESP_DEFAULT_CLIENT_ID       "STM32"

/* Timeout Values (milliseconds) */
#define ESP_AT_COMMAND_TIMEOUT      1000
#define ESP_WIFI_CONNECT_TIMEOUT    20000
#define ESP_TCP_CONNECT_TIMEOUT     10000
#define ESP_MQTT_CONNECT_TIMEOUT    10000
#define ESP_RESTORE_TIMEOUT         5000

#define RX_BUF_LEN   64
#define MAX_SSID_LEN 32
#define MAX_PASS_LEN 64


#define MQTT_PACKET_USERNAME 		"magdy"
#define MQTT_PACKET_PASSWORD 		"987654321"
#define MQTT_PACKET_KEEPALIVE		60
/******************************************************************************
 *                          Type Definitions                                  *
 ******************************************************************************/

/**
 * @brief ESP8266 module status enumeration
 */
typedef enum {
    ESP_STATUS_UNINITIALIZED = 0,    /* Module not initialized */
    ESP_STATUS_INITIALIZING,         /* Module initialization in progress */
    ESP_STATUS_WIFI_DISCONNECTED,    /* WiFi not connected */
    ESP_STATUS_WIFI_CONNECTED,       /* WiFi connected */
    ESP_STATUS_MQTT_DISCONNECTED,    /* MQTT not connected */
    ESP_STATUS_MQTT_CONNECTED,       /* MQTT connected and ready */
    ESP_STATUS_CONFIGURING,         /* Module in configuration mode */
    ESP_STATUS_ERROR                 /* Error state */
} ESP_Status_t;

typedef enum {
	ESP_STATE_PARSING_HEADER,
	ESP_STATE_RECEIVING_PAYLOAD
} esp_rx_state_t;

typedef enum {
	ESP_BUFFER_CHECK_START,
    ESP_BUFFER_CHECK_STOP,
	ESP_BUFFER_CHECK_NOK
} esp_buffer_state;

typedef enum{
    ESP_TOPIC_MAIN_SUBSCRIBE,
    ESP_TOPIC_MAIN_PUBLISH,
    ESP_TOPIC_MAXNUM
} esp_topic_index_t;

typedef enum{
    PUBLISH_TOPIC,
    SUBSCRIBE_TOPIC,
    MULTI_TOPIC
}esp_topic_type_t;

typedef enum{
    ESP_ERROR_NONE = 0,
    ESP_ERROR_TIMEOUT,
    ESP_ERROR_INVALID_CMD_RESPONSE
}esp_error_type_t;
/**
 * @brief ESP publish request structure
 */
typedef struct {
    char topic[ESP_TASK_MAX_TOPIC_LEN];             /* MQTT topic */
    char message[ESP_TASK_MAX_MESSAGE_LEN];         /* MQTT message */
    uint8_t qos;                                    /* Quality of Service level */
} ESP_PublishRequest_t;
/**
 * @brief ESP8266 configuration structure
 */
typedef struct {
    char wifi_ssid[32];              /* WiFi network SSID */
    char wifi_password[64];          /* WiFi network password */
    char mqtt_broker_ip[16];         /* MQTT broker IP address */
    uint16_t mqtt_broker_port;       /* MQTT broker port */
    char mqtt_client_id[16];         /* MQTT client identifier */
    UART_HandleTypeDef *esp_uart;    /* ESP8266 UART handle pointer */
} ESP_Config_t;

/**
 * @brief MQTT message callback function type
 * @param message The message payload
 */
typedef void (*mqtt_message_handler_t)(const char *message);

/**
 * @brief MQTT keyword handler entry
 */
typedef struct {
    const char *keyword;                /* Keyword to search for in message */
    mqtt_message_handler_t handler;     /* Handler function for this keyword */
    const char *description;            /* Description for documentation */
} mqtt_keyword_handler_t;


/**
 * @brief Function pointer type for MQTT topic handlers
 * @param topic The MQTT topic that was received
 * @param message The message payload
 */
typedef void (*mqtt_topic_handler_t)(const char* message);

/**
 * @brief Structure for mapping topics to their handlers
 */
typedef struct {
    const char* topic_pattern;          /* Topic string or pattern */
    mqtt_topic_handler_t handler;       /* Function to handle this topic */
    esp_topic_type_t topic_type;        /* Type of topic: publish, subscribe, multi */
    const char* description;            /* Human-readable description */
} mqtt_topic_entry_t;

/******************************************************************************
 *                     Global Variables (extern)                              *
 ******************************************************************************/
/* Note: UART handles are now configured via ESP_SetConfig() function */

/**
 * @brief Default ESP8266 configuration variable
 * @note Defined in ESP8266_PBCFG.c with default WiFi, MQTT, and UART settings
 */
extern ESP_Config_t ESP8266_DefaultConfig;
extern ESP_Config_t esp_config;
extern volatile bool esp_length_detected;
extern volatile bool payload_ready;

extern char g_mqtt_topic[MAX_TOPIC_BUFFER_SIZE];
extern char g_mqtt_message[MAX_MESSAGE_BUFFER_SIZE];
extern esp_buffer_state esp_buffer_current_state;
extern TaskHandle_t espMqttProcessTaskHandle;
extern const mqtt_topic_entry_t topic_handlers[];
extern ESP_Status_t esp_current_status;
/******************************************************************************
 *                        Function Declarations                               *
 ******************************************************************************/

/**
 * @brief Set ESP8266 configuration including UART handles
 * @param config Pointer to ESP8266 configuration structure
 * @note Must be called before ESP_Init()
 */
void ESP_SetConfig(ESP_Config_t *config);

/**
 * @brief Get current ESP8266 configuration
 * @return Pointer to current configuration structure
 */
ESP_Config_t* ESP_GetConfig(void);

/**
 * @brief Initialize ESP8266 module and establish connections
 * @note Complete initialization including WiFi and MQTT connection
 */
void ESP_Init(void);

/**
 * @brief Restore ESP8266 module to factory settings
 */
void ESP_Restore(void);

/**
 * @brief Send AT command and wait for expected response
 * @param cmd AT command string to send
 * @param expect Expected response string (NULL to skip validation)
 * @param timeout Timeout in milliseconds
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef ESP_SendAT(const char *cmd, const char *expect, uint32_t timeout);



HAL_StatusTypeDef ESP_SendBinary(uint8_t *bin, size_t len, const char *expect, uint32_t timeout);
/**
 * @brief Get current ESP8266 module status
 * @return Current module status
 */
ESP_Status_t ESP_GetStatus(void);

/**
 * @brief Publish data to MQTT topic
 */
void ESP_Publish(const char *topic, const char *message, uint8_t qos);

/**
 * @brief Subscribe to MQTT topic for incoming messages
 */
HAL_StatusTypeDef ESP_Subscribe(const char *topic, uint8_t qos);

/**
 * @brief Parse incoming MQTT messages from UART buffer
 */
void ESP_ParseMQTTMessage(void);

/**
 * @brief UART receive complete callback for ESP8266 communication
 * @param huart UART handle that triggered the callback
 * @note This function should be called from HAL_UART_RxCpltCallback
 */
void ESP_UART_RxCallback(UART_HandleTypeDef *huart);

/**
 * @brief Start UART receive interrupt for ESP8266 communication
 * @note Call this function after ESP_SetConfig() to start receiving data
 */
void ESP_StartUARTReceive(void);


/**
 * @brief Send periodic MQTT PINGREQ to keep connection alive
 */
void PingREQ(void);

/**
 * @brief Reset the vend request state and clear buffers
 */
void ResetMessageBuffer(void);
/**
 * @brief Process an incoming MQTT message
 * @param topic The MQTT topic that was received
 * @param message The message payload
 */
void ESP_ProcessMQTTMessage(const char* topic, const char* message);

/**
 * @brief Save WiFi configuration to non-volatile memory
 * @param ssid The WiFi SSID (max 31 chars)
 * @param password The WiFi password (max 63 chars)
 */
void SaveWiFiConfig(const char *ssid, const char *password);

HAL_StatusTypeDef ESP_AP_Mode(void);

HAL_StatusTypeDef ESP_WaitForCredentials(void);

HAL_StatusTypeDef esp_setup_wifi_connection(void);
HAL_StatusTypeDef esp_setup_tcp_connection(void);
HAL_StatusTypeDef esp_send_mqtt_connect_packet(void);
#endif /* ESP8266_HANDLER_H_ */
