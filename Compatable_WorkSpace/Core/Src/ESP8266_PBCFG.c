/*******************************************************************************
 * @file    ESP8266_PBCFG.c
 * @author  Amr Mohamed
 * @brief   Post-Build Configuration for ESP8266 module
 *
 * @details
 * This file contains the post-build configuration variables for the ESP8266
 * WiFi module, including default settings for WiFi credentials, MQTT broker
 * configuration, and UART assignments.
 *
 * @version 1.0
 * @date    2025-09-13
 ******************************************************************************/

/******************************************************************************
 *                                Includes                                    *
 ******************************************************************************/
#include "ESP8266_Handler.h"
#include "main.h"

/******************************************************************************
 *                          External UART Handles                             *
 ******************************************************************************/
/* These UART handles should be defined in main.c or respective UART init files */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/******************************************************************************
 *                     ESP8266 Configuration Variable                         *
 ******************************************************************************/

/**
 * @brief Default ESP8266 configuration structure
 * 
 * @details This configuration variable contains the default settings for the
 *          ESP8266 module including WiFi credentials, MQTT broker settings,
 *          and UART handle assignments. This variable can be modified at
 *          runtime using ESP_SetConfig() function or used as-is for default
 *          operation.
 *
 * @note WiFi Credentials:
 *       - SSID: "MA_HOME"
 *       - Password: "01289878405"
 *
 * @note MQTT Settings:
 *       - Broker IP: "192.168.1.104"
 *       - Port: 1883
 *       - Client ID: "STM32"
 *       - Default Topic: "stm32/test123"
 *
 * @note UART Configuration:
 *       - ESP8266 Communication: UART1 (huart1)
 *       - Debug Output: UART2 (huart2)
 */
ESP_Config_t ESP8266_DefaultConfig = {
    .wifi_ssid = ESP_DEFAULT_SSID,
    .wifi_password = ESP_DEFAULT_PASSWORD,
    .mqtt_broker_ip = ESP_DEFAULT_MQTT_BROKER,
    .mqtt_broker_port = ESP_DEFAULT_MQTT_PORT,
    .mqtt_client_id = ESP_DEFAULT_CLIENT_ID,
    .mqtt_topic = ESP_DEFAULT_MQTT_TOPIC,
    .esp_uart = &huart2,        /* ESP8266 communication UART */
    .debug_uart = &huart3       /* Debug output UART */
};
