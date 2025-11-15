/*******************************************************************************
 * @file    LED_Controller.h
 * @author  Vending Machine Team
 * @brief   Simple LED control module (ON/OFF/TOGGLE)
 * @version 1.0
 * @date    2025-11-04
 ******************************************************************************/

#ifndef LED_CONTROLLER_H_
#define LED_CONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 *                          Type Definitions
 ******************************************************************************/

/**
 * @brief LED channel enumeration
 */
typedef enum {
    LED_CHANNEL_DIAG,           /* Diagnostic LED (errors/warnings) */
    LED_CHANNEL_COMM,           /* Communication LED (WiFi/MQTT status) */
    LED_CHANNEL_MAX             /* Must be last */
} led_channel_t;

/**
 * @brief LED state enumeration
 */
typedef enum {
    LED_STATE_OFF,          /* LED is off */
    LED_STATE_ON,           /* LED is on */
} led_state_t;

typedef enum{
    LED_BLINK_MODE_BLINK,
    LED_BLINK_MODE_TOGGLE
} led_blink_mode_t;
/******************************************************************************
 *                        Function Declarations
 ******************************************************************************/

/**
 * @brief Initialize LED controller
 */
void LED_CONTROLLER_Init(void);

/**
 * @brief Turn LED on
 * @param channel LED channel (LED_CHANNEL_DIAG or LED_CHANNEL_COMM)
 */
void LED_On(led_channel_t channel);

/**
 * @brief Turn LED off
 * @param channel LED channel
 */
void LED_Off(led_channel_t channel);

/**
 * @brief Toggle LED (if on, turn off; if off, turn on)
 * @param channel LED channel
 */
void LED_Toggle(led_channel_t channel);

/**
 * @brief Get current LED state
 * @param channel LED channel
 * @return Current state (ON or OFF)
 */
led_state_t LED_GetState(led_channel_t channel);

/**
 * @brief Set LED blinking (toggle periodically)
 * @param channel LED channel
 * @param period_ms Blink period in milliseconds (total on+off)
 */
void LED_Blink(led_channel_t channel, uint32_t period_ms);

/**
 * @brief Stop LED blinking
 * @param channel LED channel
 */
void LED_StopBlinking(led_channel_t channel);

/**
 * @brief Update LED blinking state (call periodically, e.g., every 50ms)
 * @param delta_ms Time elapsed since last call
 */
void LED_PeriodicUpdate(uint32_t delta_ms);

/**
 * @brief Set LED blink mode
 */
void LED_SetMode(led_channel_t channel, led_blink_mode_t mode);

#endif /* LED_CONTROLLER_H_ */