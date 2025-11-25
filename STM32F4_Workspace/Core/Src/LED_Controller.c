/*******************************************************************************
 * @file    LED_Controller.c
 * @author  Vending Machine Team
 * @brief   Simple LED control module implementation
 * @version 1.0
 * @date    2025-11-04
 ******************************************************************************/

#include "LED_Controller.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "stm32f401xc.h"
/******************************************************************************
 *                          Private Definitions
 ******************************************************************************/

typedef struct {
    led_state_t state;              /* Current state (ON/OFF/BLINKING) */
    led_blink_mode_t blink_mode;    /* Blink mode */
    uint32_t blink_period_ms;       /* Blink period (0 if not blinking) */
    uint32_t blink_timer;           /* Blink timer */
    GPIO_TypeDef *gpio_port;        /* GPIO port */
    uint16_t gpio_pin;              /* GPIO pin */
} led_instance_t;

/******************************************************************************
 *                          Private Variables
 ******************************************************************************/

static led_instance_t leds[2];

/******************************************************************************
 *                    Private Helper Functions
 ******************************************************************************/

/******************************************************************************
 *                          Public Functions
 ******************************************************************************/

/**
 * @brief Initialize LED controller
 */
void LED_CONTROLLER_Init(void)
{
    /* Initialize DIAG LED */
    leds[LED_CHANNEL_DIAG].blink_mode = LED_BLINK_MODE_BLINK;
    leds[LED_CHANNEL_DIAG].blink_period_ms = 0;
    leds[LED_CHANNEL_DIAG].blink_timer = 0;
    leds[LED_CHANNEL_DIAG].gpio_port = DIAG_LED_GPIO_Port;
    leds[LED_CHANNEL_DIAG].gpio_pin = DIAG_LED_Pin;
    
    /* Initialize COMM LED */
    leds[LED_CHANNEL_COMM].blink_mode = LED_BLINK_MODE_BLINK;
    leds[LED_CHANNEL_COMM].blink_period_ms = 0;
    leds[LED_CHANNEL_COMM].blink_timer = 0;
    leds[LED_CHANNEL_COMM].gpio_port = COMM_LED_GPIO_Port;
    leds[LED_CHANNEL_COMM].gpio_pin = COMM_LED_Pin;
    
    /* Apply initial state to GPIO */
    LED_Off(LED_CHANNEL_DIAG);
    LED_Off(LED_CHANNEL_COMM);
}

/**
 * @brief Turn LED on
 */
void LED_On(led_channel_t channel)
{
    if (channel >= 2) return;
    
    leds[channel].state = LED_STATE_ON;
    
    HAL_GPIO_WritePin(leds[channel].gpio_port, leds[channel].gpio_pin,(GPIO_PinState) leds[channel].state);
}

/**
 * @brief Turn LED off
 */
void LED_Off(led_channel_t channel)
{
    if (channel >= 2) return;
    
    leds[channel].state = LED_STATE_OFF;
    
    HAL_GPIO_WritePin(leds[channel].gpio_port, leds[channel].gpio_pin,(GPIO_PinState) leds[channel].state);
}

/**
 * @brief Toggle LED
 */
void LED_Toggle(led_channel_t channel)
{
    if (channel >= LED_CHANNEL_MAX) return;
    
    if (leds[channel].state == LED_STATE_OFF) {
        LED_On(channel);
    } else {
        LED_Off(channel);
    }
}
/**
 * @brief Set LED blink mode
 */
void LED_SetMode(led_channel_t channel, led_blink_mode_t mode)
{
    if (channel >= LED_CHANNEL_MAX) return;
    
    leds[channel].blink_mode = mode;
}
/**
 * @brief Get current LED state
 */
led_state_t LED_GetState(led_channel_t channel)
{
    if (channel >= LED_CHANNEL_MAX) return LED_STATE_OFF;
    return leds[channel].state;
}


/**
 * @brief Set LED to blink
 */
void LED_Blink(led_channel_t channel, uint32_t period_ms)
{
    if (channel >= LED_CHANNEL_MAX|| period_ms == 0) return;
    
    LED_SetMode(channel, LED_BLINK_MODE_BLINK);
    leds[channel].blink_period_ms = period_ms;
    leds[channel].blink_timer = 0;
    
    LED_Toggle(channel);
}

/**
 * @brief Stop LED blinking
 */
void LED_StopBlinking(led_channel_t channel)
{
    if (channel >= LED_CHANNEL_MAX) return;
    
    if (leds[channel].blink_mode == LED_BLINK_MODE_BLINK) {
        leds[channel].blink_period_ms = 0;
        leds[channel].blink_timer = 0;
        LED_Off(channel);
    }
}

/**
 * @brief Update LED blinking state (call periodically)
 */
void LED_PeriodicUpdate(uint32_t delta_ms)
{
    for (int i = 0; i < LED_CHANNEL_MAX; i++) {
        if (leds[i].blink_mode != LED_BLINK_MODE_BLINK || leds[i].blink_period_ms == 0) {
            continue;
        }
        
        leds[i].blink_timer += delta_ms;
        
        if (leds[i].blink_timer >= (leds[i].blink_period_ms)) {
            leds[i].blink_timer = 0;
            LED_Toggle((led_channel_t)i);
        }
    }
}