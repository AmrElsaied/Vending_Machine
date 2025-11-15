/*******************************************************************************
 * @file    Periodic_Task_Manager.h
 * @author  Vending Machine Team
 * @brief   Unified periodic task manager for all 10ms-based operations
 * @version 1.0
 * @date    2025-11-04
 ******************************************************************************/

#ifndef PERIODIC_TASK_MANAGER_H_
#define PERIODIC_TASK_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 *                          Macros & Constants
 ******************************************************************************/

#define PERIODIC_TASK_BASE_PERIOD_MS    100      /* 100ms base period */

/* Button configuration constants */
#define BUTTON_DEBOUNCE_TIME_MS     20      /* Debounce time */
#define BUTTON_LONG_PRESS_TIME_MS   2000    /* 2 seconds */

/******************************************************************************
 *                          Type Definitions
 ******************************************************************************/

/**
 * @brief Callback function type for periodic operations
 * @param delta_ms Time elapsed since last call
 */
typedef void (*periodic_callback_t)(uint32_t delta_ms);

/**
 * @brief Periodic task module enumeration
 */
typedef enum {
    PERIODIC_MODULE_BUTTON,          /* Button debouncing */
    PERIODIC_MODULE_SYSTEM_MONITOR,  /* System state monitoring */
    PERIODIC_MODULE_COMMUNICATION,   /* Communication state monitoring */
    PERIODIC_MODULE_MAX              /* Must be last */
} periodic_module_t;

/**
 * @brief Periodic task configuration
 */
typedef struct {
    periodic_module_t module_id;
    periodic_callback_t callback;
    uint32_t period_ms;              /* 10ms, 20ms, 50ms, 100ms, etc. */
    bool enabled;
} periodic_task_config_t;

/******************************************************************************
 *                        Function Declarations
 ******************************************************************************/

/**
 * @brief Initialize periodic task manager
 */
void PERIODIC_TASK_Init(void);

/**
 * @brief Register a periodic module callback
 * @param config Pointer to task configuration
 * @return true if registered successfully
 */
bool PERIODIC_TASK_Register(const periodic_task_config_t *config);

/**
 * @brief Unregister a periodic module
 * @param module_id Module to unregister
 * @return true if unregistered successfully
 */
bool PERIODIC_TASK_Unregister(periodic_module_t module_id);

/**
 * @brief Enable a periodic module
 * @param module_id Module to enable
 */
void PERIODIC_TASK_Enable(periodic_module_t module_id);

/**
 * @brief Disable a periodic module
 * @param module_id Module to disable
 */
void PERIODIC_TASK_Disable(periodic_module_t module_id);

/**
 * @brief Check if module is enabled
 * @param module_id Module to check
 * @return true if enabled
 */
bool PERIODIC_TASK_IsEnabled(periodic_module_t module_id);

/**
 * @brief Execute all registered periodic tasks
 * @note Call this from main periodic task every 10ms
 */
void PERIODIC_TASK_Execute(void);

/**
 * @brief Get number of registered modules
 * @return Number of active modules
 */
uint8_t PERIODIC_TASK_GetModuleCount(void);

/**
 * @brief Button periodic callback - processes button state
 * @param delta_ms Time elapsed since last call
 * @note Register this with period 100ms
 */
void BUTTON_PeriodicCallback(uint32_t delta_ms);

/**
 * @brief Signal button press from GPIO interrupt
 * @note Call this from HAL_GPIO_EXTI_Callback()
 */
void PERIODIC_TASK_ButtonInterrupt(void);
/**
 * @brief Communication monitoring callback
 * @param delta_ms Time elapsed since last call
 * @note Register this with period 50ms
 */
void PERIODIC_TASK_CommunicationMonotoringCallback(uint32_t delta_ms);
#endif /* PERIODIC_TASK_MANAGER_H_ */
