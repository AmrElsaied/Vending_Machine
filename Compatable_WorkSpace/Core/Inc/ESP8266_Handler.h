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
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 *                             Module Config                                  *
 ******************************************************************************/
/* UART Configuration */
#define ESP_UART_INSTANCE           USART1    /* ESP8266 UART instance */
#define ESP_DEBUG_UART_INSTANCE     USART2    /* Debug UART instance */

/******************************************************************************
 *                          Macros & Constants                                *
 ******************************************************************************/
#define ESP_UART_RX_BUFFER_SIZE     256  /* Size of ESP UART receive buffer */
#define ESP_RESPONSE_BUFFER_SIZE    512  /* Size of AT command response buffer */
#define ESP_TOPIC_MAX_LENGTH        64   /* Maximum MQTT topic length */
#define ESP_MESSAGE_MAX_LENGTH      128  /* Maximum MQTT message length */

/* Default WiFi Credentials */
#define ESP_DEFAULT_SSID            "MA_HOME"
#define ESP_DEFAULT_PASSWORD        "01289878405"

/* Default MQTT Broker Settings */
#define ESP_DEFAULT_MQTT_BROKER     "192.168.1.104"
#define ESP_DEFAULT_MQTT_PORT       1883
#define ESP_DEFAULT_MQTT_TOPIC      "stm32/test123"
#define ESP_DEFAULT_CLIENT_ID       "STM32"

/* Timeout Values (milliseconds) */
#define ESP_AT_COMMAND_TIMEOUT      1000
#define ESP_WIFI_CONNECT_TIMEOUT    20000
#define ESP_TCP_CONNECT_TIMEOUT     10000
#define ESP_MQTT_CONNECT_TIMEOUT    10000
#define ESP_RESTORE_TIMEOUT         5000

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
    ESP_STATUS_ERROR                 /* Error state */
} ESP_Status_t;

/**
 * @brief MQTT message structure
 */
typedef struct {
    char topic[ESP_TOPIC_MAX_LENGTH];        /* MQTT topic */
    char message[ESP_MESSAGE_MAX_LENGTH];    /* MQTT message payload */
    uint16_t topic_length;                   /* Length of topic string */
    uint16_t message_length;                 /* Length of message string */
} ESP_MQTT_Message_t;

/**
 * @brief ESP8266 configuration structure
 */
typedef struct {
    char wifi_ssid[32];              /* WiFi network SSID */
    char wifi_password[64];          /* WiFi network password */
    char mqtt_broker_ip[16];         /* MQTT broker IP address */
    uint16_t mqtt_broker_port;       /* MQTT broker port */
    char mqtt_client_id[16];         /* MQTT client identifier */
    char mqtt_topic[ESP_TOPIC_MAX_LENGTH];   /* Default MQTT topic */
    UART_HandleTypeDef *esp_uart;    /* ESP8266 UART handle pointer */
    UART_HandleTypeDef *debug_uart;  /* Debug UART handle pointer */
    bool debug_enabled;              /* Enable/disable debug output */
} ESP_Config_t;

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

/**
 * @brief Get current ESP8266 module status
 * @return Current module status
 */
ESP_Status_t ESP_GetStatus(void);

/**
 * @brief Publish data to MQTT topic
 */
void ESP_PublishNumber(void);

/**
 * @brief Subscribe to MQTT topic for incoming messages
 */
void ESP_Subscribe(void);

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

#endif /* ESP8266_HANDLER_H_ */
