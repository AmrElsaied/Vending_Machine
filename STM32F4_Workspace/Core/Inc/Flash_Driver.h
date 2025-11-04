/*******************************************************************************
 * @file    Flash_Driver.h
 * @author  Vending Machine Team
 * @brief   Simple Flash memory abstraction layer for STM32F4
 * @version 1.0
 * @date    2025-11-04
 ******************************************************************************/

#ifndef FLASH_DRIVER_H_
#define FLASH_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/******************************************************************************
 *                          Macros & Constants
 ******************************************************************************/

/* STM32F401CC Flash memory layout */
#define FLASH_BASE_ADDR              0x08000000U      /* Flash base address */
#define FLASH_USER_START_ADDR        0x08070000U      /* User data area start (192 KB) */
#define FLASH_USER_END_ADDR          0x0807FFFFU      /* User data area end (256 KB) */

/* Configuration storage offsets */
#define FLASH_CONFIG_ADDR            0x08070000U      /* WiFi & MQTT config */


/* Operation status codes */
typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR,
    FLASH_TIMEOUT,
    FLASH_INVALID_ADDR,
    FLASH_INVALID_SIZE
} flash_status_t;

/******************************************************************************
 *                        Function Declarations
 ******************************************************************************/

/**
 * @brief Initialize Flash driver
 * @return FLASH_OK if successful
 */
flash_status_t FLASH_Init(void);

/**
 * @brief Read data from Flash
 * @param addr Start address to read from
 * @param data_ptr Pointer to destination buffer
 * @param size Number of bytes to read
 * @return FLASH_OK if successful
 */
flash_status_t FLASH_Read(uint32_t addr, uint8_t *data_ptr, uint32_t size);

/**
 * @brief Write data to Flash
 * @param addr Address to write to (must be word-aligned: address % 4 == 0)
 * @param data_ptr Pointer to source data
 * @param size Number of bytes to write (must be multiple of 4)
 * @return FLASH_OK if successful
 */
flash_status_t FLASH_Write(uint32_t addr, const uint8_t *data_ptr, uint32_t size);

/**
 * @brief Erase Flash sector containing given address
 * @param addr Address within the sector to erase
 * @return FLASH_OK if successful
 */
flash_status_t FLASH_Erase(uint32_t addr);

/**
 * @brief Get sector number from address
 * @param addr Address within the sector
 * @return Sector number (0-15 for STM32F401CC)
 */
uint8_t FLASH_GetSectorNumber(uint32_t addr);

/**
 * @brief Check if address is valid for user operations
 * @param addr Address to check
 * @param size Size to access
 * @return true if valid, false otherwise
 */
bool FLASH_IsAddressValid(uint32_t addr, uint32_t size);

#endif /* FLASH_DRIVER_H_ */