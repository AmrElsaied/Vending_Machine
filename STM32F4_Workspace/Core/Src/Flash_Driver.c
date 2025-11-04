/*******************************************************************************
 * @file    Flash_Driver.c
 * @author  Vending Machine Team
 * @brief   Simple Flash memory driver implementation
 * @version 1.0
 * @date    2025-11-04
 ******************************************************************************/

#include "Flash_Driver.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/******************************************************************************
 *                          Private Definitions
 ******************************************************************************/

/* Sector definitions for STM32F401CC */
typedef struct {
    uint8_t sector_num;
    uint32_t start_addr;
    uint32_t size;
} flash_sector_t;

/* STM32F401CC has 16 sectors total, user area is sectors 12-15 */
static const flash_sector_t flash_sectors[] = {
    {0, 0x08000000, 0x4000},      /* 16 KB */
    {1, 0x08004000, 0x4000},      /* 16 KB */
    {2, 0x08008000, 0x4000},      /* 16 KB */
    {3, 0x0800C000, 0x4000},      /* 16 KB */
    {4, 0x08010000, 0x10000},     /* 64 KB */
    {5, 0x08020000, 0x20000},     /* 128 KB */
    {6, 0x08040000, 0x20000},     /* 128 KB (extends beyond F401CC) */
    {7, 0x08060000, 0x20000}     /* 128 KB (extends beyond F401CC) */
};

static bool flash_initialized = false;

/******************************************************************************
 *                          Private Functions
 ******************************************************************************/

/**
 * @brief Wait for Flash to be ready
 * @param timeout_ms Timeout in milliseconds
 * @return FLASH_OK if successful
 */
static flash_status_t FLASH_WaitForReady(uint32_t timeout_ms)
{
    uint32_t tick_start = HAL_GetTick();
    
    /* Wait for BUSY flag to be cleared */
    while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {
        if ((HAL_GetTick() - tick_start) > timeout_ms) {
            return FLASH_TIMEOUT;
        }
    }
    
    return FLASH_OK;
}

/**
 * @brief Check for Flash errors
 * @return FLASH_OK if no errors
 */
static flash_status_t FLASH_CheckErrors(void)
{
    uint32_t error = HAL_FLASH_GetError();
    
    if (error != HAL_FLASH_ERROR_NONE) {
        return FLASH_ERROR;
    }
    
    return FLASH_OK;
}

/******************************************************************************
 *                          Public Functions
 ******************************************************************************/

/**
 * @brief Initialize Flash driver
 */
flash_status_t FLASH_Init(void)
{
    if (flash_initialized) {
        return FLASH_OK;
    }
    
    flash_initialized = true;
    return FLASH_OK;
}

/**
 * @brief Read data from Flash
 */
flash_status_t FLASH_Read(uint32_t addr, uint8_t *data_ptr, uint32_t size)
{
    /* Validate inputs */
    if (data_ptr == NULL || size == 0) {
        return FLASH_INVALID_SIZE;
    }
    
    if (!FLASH_IsAddressValid(addr, size)) {
        return FLASH_INVALID_ADDR;
    }
    
    /* Flash is directly addressable, just copy data */
    memcpy(data_ptr, (const void *)addr, size);
    
    return FLASH_OK;
}

/**
 * @brief Write data to Flash
 */
flash_status_t FLASH_Write(uint32_t addr, const uint8_t *data_ptr, uint32_t size)
{
    /* Validate inputs */
    if (!flash_initialized) {
        return FLASH_ERROR;
    }
    
    if (data_ptr == NULL || size == 0) {
        return FLASH_INVALID_SIZE;
    }
    
    /* Size must be multiple of 4 (word size) */
    if (size % 4 != 0) {
        return FLASH_INVALID_SIZE;
    }
    
    /* Address must be word-aligned */
    if (addr % 4 != 0) {
        return FLASH_INVALID_ADDR;
    }
    
    if (!FLASH_IsAddressValid(addr, size)) {
        return FLASH_INVALID_ADDR;
    }
    
    /* Wait for Flash to be ready */
    if (FLASH_WaitForReady(5000) != FLASH_OK) {
        return FLASH_TIMEOUT;
    }
    
    /* Unlock Flash */
    HAL_FLASH_Unlock();
    
    /* Write data word by word (4 bytes at a time) */
    for (uint32_t i = 0; i < size; i += 4) {
        uint32_t data = (data_ptr[i + 0] << 0) |
                       (data_ptr[i + 1] << 8) |
                       (data_ptr[i + 2] << 16) |
                       (data_ptr[i + 3] << 24);
        
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, data) != HAL_OK) {
            HAL_FLASH_Lock();
            return FLASH_ERROR;
        }
    }
    
    /* Lock Flash */
    HAL_FLASH_Lock();
    
    /* Check for errors */
    return FLASH_CheckErrors();
}

/**
 * @brief Erase Flash sector
 */
flash_status_t FLASH_Erase(uint32_t addr)
{
    if (!flash_initialized) {
        return FLASH_ERROR;
    }
    
    if (!FLASH_IsAddressValid(addr, 4)) {
        return FLASH_INVALID_ADDR;
    }
    
    /* Get sector number */
    uint8_t sector_num = FLASH_GetSectorNumber(addr);
    
    /* Wait for Flash to be ready */
    if (FLASH_WaitForReady(5000) != FLASH_OK) {
        return FLASH_TIMEOUT;
    }
    
    /* Unlock Flash */
    HAL_FLASH_Unlock();
    
    /* Configure erase parameters */
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;
    
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = sector_num;
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;  /* 2.7V to 3.6V */
    
    /* Perform erase */
    if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return FLASH_ERROR;
    }
    
    /* Lock Flash */
    HAL_FLASH_Lock();
    
    /* Check for errors */
    return FLASH_CheckErrors();
}

/**
 * @brief Get sector number from address
 */
uint8_t FLASH_GetSectorNumber(uint32_t addr)
{
    /* For STM32F401CC user area (0x08030000 onwards) */
    /* Sectors 0-11: Code area (0x08000000 - 0x0802FFFF) */
    /* Sectors 12-15: Available but not on F401CC */
    
    if (addr < 0x08004000) return 0;
    if (addr < 0x08008000) return 1;
    if (addr < 0x0800C000) return 2;
    if (addr < 0x08010000) return 3;
    if (addr < 0x08020000) return 4;
    if (addr < 0x08040000) return 5;
    if (addr < 0x08060000) return 6;
    if (addr < 0x08080000) return 7;
    
    /* Default to highest available */
    return 7;
}

/**
 * @brief Check if address is valid
 */
bool FLASH_IsAddressValid(uint32_t addr, uint32_t size)
{
    uint32_t end_addr = addr + size;
    
    /* Check if within user area */
    if (addr < FLASH_USER_START_ADDR) {
        return false;
    }
    
    if (end_addr > FLASH_USER_END_ADDR) {
        return false;
    }
    
    return true;
}
