# ESP8266 Handler Usage Examples

**Author:** Amr Mohamed  
**Date:** September 19, 2025  
**Version:** 1.0

## Overview

This document demonstrates how to properly configure and use the ESP8266 handler with configurable UART instances. The ESP8266 handler provides WiFi connectivity and MQTT communication capabilities for the vending machine system.

## Table of Contents

- [Basic Configuration](#basic-configuration)
- [UART Configuration Examples](#uart-configuration-examples)
- [Production Configurations](#production-configurations)
- [Integration Examples](#integration-examples)
- [Usage Notes](#usage-notes)

## Basic Configuration

The ESP8266 handler uses a configuration structure to set up WiFi and MQTT parameters along with UART assignments.

### Configuration Structure

```c
typedef struct {
    char wifi_ssid[32];           /* WiFi network name */
    char wifi_password[64];       /* WiFi password */
    char mqtt_broker_ip[16];      /* MQTT broker IP address */
    uint16_t mqtt_broker_port;    /* MQTT broker port */
    char mqtt_client_id[32];      /* MQTT client identifier */
    char mqtt_topic[64];          /* MQTT topic for publishing */
    UART_HandleTypeDef *esp_uart; /* UART handle for ESP8266 communication */
    UART_HandleTypeDef *debug_uart; /* UART handle for debug output */
} ESP_Config_t;
```

### Example 1: Basic ESP8266 Initialization

```c
void ESP8266_Example_Init(void)
{
    // Step 1: Create configuration structure
    ESP_Config_t esp_config = {
        .wifi_ssid = "YourWiFiNetwork",
        .wifi_password = "YourPassword",
        .mqtt_broker_ip = "192.168.1.100",
        .mqtt_broker_port = 1883,
        .mqtt_client_id = "VMC_Device",
        .mqtt_topic = "vmc/data",
        .esp_uart = &huart1,      /* ESP8266 connected to UART1 */
        .debug_uart = &huart2     /* Debug output via UART2 */
    };
    
    // Step 2: Apply configuration to ESP8266 handler
    ESP_SetConfig(&esp_config);
    
    // Step 3: Start UART receive interrupt for ESP8266
    ESP_StartUARTReceive();
    
    // Step 4: Initialize ESP8266 (connects to WiFi and MQTT broker)
    ESP_Init();
    
    // Step 5: Check initialization status
    ESP_Status_t status = ESP_GetStatus();
    if (status == ESP_STATUS_MQTT_CONNECTED) {
        // ESP8266 is ready for use
        // You can now subscribe to topics and publish messages
        ESP_Subscribe();
    }
}
```

## UART Configuration Examples

### Example 2: Alternative UART Configuration

This example shows how to configure ESP8266 to use different UART instances.

```c
void ESP8266_Example_AlternativeUART(void)
{
    // Assuming you have UART3 configured for ESP8266
    extern UART_HandleTypeDef huart3;
    
    ESP_Config_t esp_config = {
        .wifi_ssid = "OfficeNetwork",
        .wifi_password = "SecurePassword123",
        .mqtt_broker_ip = "192.168.0.50",
        .mqtt_broker_port = 1883,
        .mqtt_client_id = "VMC_Office",
        .mqtt_topic = "office/vmc/status",
        .esp_uart = &huart3,      /* ESP8266 connected to UART3 */
        .debug_uart = &huart2     /* Debug output via UART2 */
    };
    
    ESP_SetConfig(&esp_config);
    ESP_StartUARTReceive();
    ESP_Init();
}
```

### Example 3: Using Default Configuration

The ESP8266 module includes a default configuration that can be retrieved and modified:

```c
void ESP8266_Example_UseDefault(void)
{
    // Get the default configuration
    ESP_Config_t *default_config = ESP_GetConfig();
    
    // Modify only what you need
    strcpy(default_config->wifi_ssid, "MyNetwork");
    strcpy(default_config->wifi_password, "MyPassword");
    strcpy(default_config->mqtt_broker_ip, "192.168.1.50");
    
    // The UART handles are already set to defaults
    // esp_uart = &huart1, debug_uart = &huart2
    
    ESP_StartUARTReceive();
    ESP_Init();
}
```

## Production Configurations

### Example 4: Production Build (No Debug Output)

This configuration disables debug output for production builds:

```c
void ESP8266_Example_NoDebug(void)
{
    ESP_Config_t esp_config = {
        .wifi_ssid = "ProductionNetwork",
        .wifi_password = "ProductionPass",
        .mqtt_broker_ip = "10.0.1.100",
        .mqtt_broker_port = 1883,
        .mqtt_client_id = "VMC_PROD_001",
        .mqtt_topic = "production/vmc/telemetry",
        .esp_uart = &huart1,      /* ESP8266 connected to UART1 */
        .debug_uart = NULL        /* No debug output */
    };
    
    ESP_SetConfig(&esp_config);
    ESP_StartUARTReceive();
    ESP_Init();
}
```

### Example 5: Secure MQTT Configuration

```c
void ESP8266_Example_SecureMQTT(void)
{
    ESP_Config_t esp_config = {
        .wifi_ssid = "SecureNetwork",
        .wifi_password = "ComplexPassword!@#",
        .mqtt_broker_ip = "secure.broker.com",
        .mqtt_broker_port = 8883,           /* Secure MQTT port */
        .mqtt_client_id = "VMC_SECURE_001",
        .mqtt_topic = "secure/vmc/data",
        .esp_uart = &huart1,
        .debug_uart = &huart2
    };
    
    ESP_SetConfig(&esp_config);
    ESP_StartUARTReceive();
    ESP_Init();
}
```

## Integration Examples

### UART Receive Callback Integration

This callback should be placed in your `main.c` file to handle UART receive interrupts:

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Forward ESP8266 UART interrupts to the handler
    ESP_UART_RxCallback(huart);
    
    // Handle other UART interrupts if needed
    // if (huart->Instance == USART3) {
    //     // Handle other UART device
    // }
}
```

### Main Loop Integration

```c
void ESP8266_Example_MainLoop(void)
{
    while (1) {
        // Check for incoming MQTT messages
        ESP_ParseMQTTMessage();
        
        // Check ESP8266 status periodically
        ESP_Status_t status = ESP_GetStatus();
        if (status == ESP_STATUS_ERROR) {
            // Handle error - might want to reinitialize
            ESP_Init();
        }
        
        // Publish data periodically (example)
        static uint32_t last_publish = 0;
        if (HAL_GetTick() - last_publish > 30000) { // Every 30 seconds
            ESP_PublishNumber();
            last_publish = HAL_GetTick();
        }
        
        // Other application tasks...
        HAL_Delay(100);
    }
}
```

### FreeRTOS Task Integration

For FreeRTOS-based applications:

```c
void ESP8266_Task(void *argument)
{
    // Initialize ESP8266
    ESP8266_Example_Init();
    
    while (1) {
        // Wait for notifications from UART callback
        uint32_t notification;
        if (xTaskNotifyWait(0, ULONG_MAX, &notification, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (notification & ESP_NOTIFICATION_DATA_RECEIVED) {
                // Process ESP8266 data
                ESP_ParseMQTTMessage();
            }
        }
        
        // Periodic status check
        ESP_Status_t status = ESP_GetStatus();
        if (status != ESP_STATUS_MQTT_CONNECTED) {
            // Handle disconnection
            ESP_Init();
        }
        
        // Periodic data publishing
        ESP_PublishNumber();
    }
}
```

## Advanced Configuration Examples

### Example 6: Multiple MQTT Topics

```c
void ESP8266_Example_MultiplTopics(void)
{
    ESP_Config_t esp_config = {
        .wifi_ssid = "VendingNetwork",
        .wifi_password = "VendingPass123",
        .mqtt_broker_ip = "192.168.10.1",
        .mqtt_broker_port = 1883,
        .mqtt_client_id = "VMC_Multi_001",
        .mqtt_topic = "vending/machine1/status",  /* Primary topic */
        .esp_uart = &huart1,
        .debug_uart = &huart2
    };
    
    ESP_SetConfig(&esp_config);
    ESP_StartUARTReceive();
    ESP_Init();
    
    // After connection, you can publish to different topics
    // ESP_PublishToTopic("vending/machine1/sales", sales_data);
    // ESP_PublishToTopic("vending/machine1/inventory", inventory_data);
}
```

### Example 7: Dynamic Configuration from Flash/EEPROM

```c
void ESP8266_Example_DynamicConfig(void)
{
    ESP_Config_t esp_config;
    
    // Load configuration from non-volatile memory
    // This is pseudocode - implement according to your storage method
    FLASH_ReadConfig(&esp_config);
    
    // Validate configuration
    if (strlen(esp_config.wifi_ssid) == 0) {
        // Use fallback configuration
        strcpy(esp_config.wifi_ssid, "FallbackNetwork");
        strcpy(esp_config.wifi_password, "FallbackPass");
        strcpy(esp_config.mqtt_broker_ip, "192.168.1.1");
        esp_config.mqtt_broker_port = 1883;
    }
    
    // Set UART handles
    esp_config.esp_uart = &huart1;
    esp_config.debug_uart = &huart2;
    
    ESP_SetConfig(&esp_config);
    ESP_StartUARTReceive();
    ESP_Init();
}
```

## Usage Notes

### Configuration Guidelines

1. **WiFi Settings**:
   - Ensure SSID and password are correct and within length limits
   - Use WPA2 security for better compatibility

2. **MQTT Settings**:
   - Verify broker IP address and port accessibility
   - Use unique client IDs to avoid conflicts
   - Keep topic names descriptive and organized

3. **UART Configuration**:
   - Ensure UART handles are properly initialized before use
   - ESP8266 typically uses 115200 baud rate
   - Debug UART can be set to NULL to disable debug output

### Error Handling

- Always check `ESP_GetStatus()` after initialization
- Implement reconnection logic for network failures
- Monitor UART communication for hardware issues

### Performance Considerations

- Use appropriate delays between operations
- Implement timeout handling for network operations
- Consider using interrupts instead of polling for better performance

### Status Codes

```c
typedef enum {
    ESP_STATUS_IDLE,
    ESP_STATUS_INITIALIZING,
    ESP_STATUS_WIFI_CONNECTING,
    ESP_STATUS_WIFI_CONNECTED,
    ESP_STATUS_MQTT_CONNECTING,
    ESP_STATUS_MQTT_CONNECTED,
    ESP_STATUS_ERROR
} ESP_Status_t;
```

### Common Functions

- `ESP_SetConfig()`: Apply configuration
- `ESP_GetConfig()`: Get current configuration
- `ESP_Init()`: Initialize ESP8266 module
- `ESP_GetStatus()`: Get current status
- `ESP_StartUARTReceive()`: Start UART receive interrupt
- `ESP_UART_RxCallback()`: UART receive callback
- `ESP_ParseMQTTMessage()`: Process incoming MQTT data
- `ESP_Subscribe()`: Subscribe to MQTT topics
- `ESP_PublishNumber()`: Publish data to MQTT broker

### File Structure

```
Compatable_WorkSpace/
├── Core/
│   ├── Inc/
│   │   └── ESP8266_Handler.h        # ESP8266 handler declarations
│   └── Src/
│       ├── ESP8266_Handler.c        # ESP8266 handler implementation
│       └── ESP8266_PBCFG.c         # Post-build configuration
└── doc/
    ├── ESP8266_Usage_Examples.md    # This documentation file
    └── MDB_Usage_Examples.md        # MDB documentation
```

---

**Note**: This documentation replaces the previous `ESP8266_Example.c` file to prevent it from being included in the compilation process while maintaining all the usage examples and explanations for the ESP8266 WiFi and MQTT functionality.
