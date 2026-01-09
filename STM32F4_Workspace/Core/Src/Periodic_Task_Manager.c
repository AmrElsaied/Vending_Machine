/*******************************************************************************
 * @file    Periodic_Task_Manager.c
 * @author  Vending Machine Team
 * @brief   Unified periodic task manager implementation
 * @version 1.0
 * @date    2025-11-04
 ******************************************************************************/

#include "Periodic_Task_Manager.h"
#include <string.h>
#include <stdio.h>
#include "Flash_Driver.h"
#include "main.h"
#include "ESP8266_Handler.h"
#include "LED_Controller.h"

/******************************************************************************
 *                          Private Definitions
 ******************************************************************************/

typedef struct {
    periodic_task_config_t config;
    uint32_t last_execution_time;  /* Track when it last ran */
    uint32_t accumulated_time;     /* Accumulate time for non-10ms tasks */
} periodic_task_instance_t;

#define MAX_PERIODIC_MODULES    5

/******************************************************************************
 *                          Private Variables
 ******************************************************************************/

static periodic_task_instance_t periodic_tasks[MAX_PERIODIC_MODULES];
static uint8_t registered_module_count = 0;
static uint32_t task_manager_tick = 0;

static uint32_t button_press_start_time = 0;
static bool button_long_press_notified = false;
static bool button_interrupt_flag = false;  /* NEW: Flag from GPIO interrupt */
static bool Comm_Led_Blinking = false;

/******************************************************************************
 *                          Public Functions
 ******************************************************************************/

/**
 * @brief Initialize periodic task manager
 */
void PERIODIC_TASK_Init(void)
{
    memset(periodic_tasks, 0, sizeof(periodic_tasks));
    registered_module_count = 0;
    task_manager_tick = 0;
}

/**
 * @brief Register a periodic module
 */
bool PERIODIC_TASK_Register(const periodic_task_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    
    if (registered_module_count >= MAX_PERIODIC_MODULES) {
        return false;
    }
    
    /* Check for duplicate module */
    for (uint8_t i = 0; i < registered_module_count; i++) {
        if (periodic_tasks[i].config.module_id == config->module_id) {
            return false;
        }
    }
    
    /* Register new module */
    periodic_tasks[registered_module_count].config = *config;
    periodic_tasks[registered_module_count].last_execution_time = 0;
    periodic_tasks[registered_module_count].accumulated_time = 0;

    registered_module_count++;
    return true;
}

/**
 * @brief Unregister a periodic module
 */
bool PERIODIC_TASK_Unregister(periodic_module_t module_id)
{
    for (uint8_t i = 0; i < registered_module_count; i++) {
        if (periodic_tasks[i].config.module_id == module_id) {
            /* Shift remaining tasks down */
            for (uint8_t j = i; j < registered_module_count - 1; j++) {
                periodic_tasks[j] = periodic_tasks[j + 1];
            }
            registered_module_count--;
            return true;
        }
    }
    return false;
}

/**
 * @brief Enable a periodic module
 */
void PERIODIC_TASK_Enable(periodic_module_t module_id)
{
    for (uint8_t i = 0; i < registered_module_count; i++) {
        if (periodic_tasks[i].config.module_id == module_id) {
            periodic_tasks[i].config.enabled = true;
            return;
        }
    }
}

/**
 * @brief Disable a periodic module
 */
void PERIODIC_TASK_Disable(periodic_module_t module_id)
{
    for (uint8_t i = 0; i < registered_module_count; i++) {
        if (periodic_tasks[i].config.module_id == module_id) {
            periodic_tasks[i].config.enabled = false;
            return;
        }
    }
}

/**
 * @brief Check if module is enabled
 */
bool PERIODIC_TASK_IsEnabled(periodic_module_t module_id)
{
    for (uint8_t i = 0; i < registered_module_count; i++) {
        if (periodic_tasks[i].config.module_id == module_id) {
            return periodic_tasks[i].config.enabled;
        }
    }
    return false;
}

/**
 * @brief Execute all registered periodic tasks
 * @note Call every 10ms from main periodic task
 */
void PERIODIC_TASK_Execute(void)
{
    task_manager_tick += PERIODIC_TASK_BASE_PERIOD_MS;
    
    for (uint8_t i = 0; i < registered_module_count; i++) {
        if (!periodic_tasks[i].config.enabled) {
            continue;
        }
        
        periodic_tasks[i].accumulated_time += PERIODIC_TASK_BASE_PERIOD_MS;
        
        /* Check if it's time to execute this task */
        if (periodic_tasks[i].accumulated_time >= periodic_tasks[i].config.period_ms) {
            uint32_t delta_time = periodic_tasks[i].accumulated_time;
            periodic_tasks[i].accumulated_time = 0;
            
            /* Execute callback */
            if (periodic_tasks[i].config.callback != NULL) {
                periodic_tasks[i].config.callback(delta_time);
            }
        }
    }
}

/**
 * @brief Get number of registered modules
 */
uint8_t PERIODIC_TASK_GetModuleCount(void)
{
    return registered_module_count;
}

/**
 * @brief Button periodic callback - processes button state
 * @param delta_ms Time elapsed since last call
 * @note Register this with period 100ms
 */
void BUTTON_PeriodicCallback(uint32_t delta_ms)
{
    /* Only process if interrupt occurred or button is being held */
    if (!button_interrupt_flag && button_press_start_time == 0) {
        return;  /* Nothing to do, skip processing */
    }
    button_interrupt_flag = false;  /* Clear interrupt flag */

    uint32_t current_time = HAL_GetTick();
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(RESET_BUTTON_GPIO_Port, RESET_BUTTON_Pin);
    bool button_pressed = (pin_state == GPIO_PIN_RESET);  /* Active low */

    if (button_pressed) {
        /* Button is pressed */
        if (button_press_start_time == 0) {
            button_press_start_time = current_time;
        }

        /* Check if 2 seconds elapsed */
        uint32_t elapsed = current_time - button_press_start_time;
        if (elapsed >= BUTTON_LONG_PRESS_TIME_MS && !button_long_press_notified) {
            button_long_press_notified = true;
        }
    } else {
        /* Button released */
        if (button_press_start_time > 0) {
            uint32_t press_duration = current_time - button_press_start_time;

            if (press_duration >= BUTTON_LONG_PRESS_TIME_MS) {

                /* Clear WiFi credentials */
                SaveWiFiConfig(ESP_DEFAULT_UNCONFIGURED_SSID, ESP_DEFAULT_UNCONFIGURED_PASSWORD);
                /* Reset the system */
                HAL_NVIC_SystemReset();
            }

            button_press_start_time = 0;
            button_long_press_notified = false;
        }
    }
}
/**
 * @brief Called from GPIO interrupt - just sets a flag
 * @note Much faster than processing in ISR
 */
void PERIODIC_TASK_ButtonInterrupt(void)
{
    button_interrupt_flag = true;  /* Signal that button activity occurred */
}

void PERIODIC_TASK_CommunicationMonotoringCallback(uint32_t delta_ms)
{
    ESP_Status_t esp_cur_status = ESP_GetStatus();
    if (esp_cur_status == ESP_STATUS_MQTT_CONNECTED) {
        if (!Comm_Led_Blinking) {
            LED_Blink(LED_CHANNEL_COMM, 900);  /* Blink every 900ms */
            Comm_Led_Blinking = true;
        }
        LED_PeriodicUpdate(delta_ms);
    }
    else {
        LED_StopBlinking(LED_CHANNEL_COMM);
        Comm_Led_Blinking = false;
    }
}