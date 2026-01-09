# Flash Driver Usage Guide

## Overview

Simple Flash driver for STM32F4 vending machine project. No FreeRTOS dependencies - suitable for early initialization before scheduler starts.

## Memory Layout (STM32F401CC)

```
0x08000000 - 0x0806FFFF: Program Code (448 KB)
0x08070000 - 0x0807FFFF: User Data Area (64 KB)
    └── 0x08070000: Configuration Data (WiFi, MQTT settings)
```

## Quick Start

### 1. Initialize in main.c

```c
#include "Flash_Driver.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART_Init();
    
    /* Initialize Flash driver EARLY */
    flash_status_t status = FLASH_Init();
    if (status != FLASH_OK) {
        printf("Flash init failed: %s\n", FLASH_GetErrorString(status));
        Error_Handler();
    }
    
    /* Now safe to read/write/erase Flash */
    
    return 0;
}
```

### 2. Read Data

```c
uint8_t config_data[64];  /* Size must be multiple of 4 for write operations */

flash_status_t status = FLASH_Read(
    FLASH_CONFIG_ADDR,      /* Start address */
    config_data,             /* Destination buffer */
    64                       /* Bytes to read */
);

if (status != FLASH_OK) {
    printf("Read failed: %s\n", FLASH_GetErrorString(status));
}
```

### 3. Write Data

```c
uint8_t config_data[] = {0x01, 0x02, 0x03, 0x04}; /* Must be 4-byte aligned size */

flash_status_t status = FLASH_Write(
    FLASH_CONFIG_ADDR,      /* Start address (must be 4-byte aligned) */
    config_data,             /* Source data */
    4                        /* Size (must be multiple of 4) */
);

if (status != FLASH_OK) {
    printf("Write failed: %s\n", FLASH_GetErrorString(status));
}
```

### 4. Erase Sector

```c
flash_status_t status = FLASH_Erase(FLASH_CONFIG_ADDR);

if (status != FLASH_OK) {
    printf("Erase failed: %s\n", FLASH_GetErrorString(status));
}
```

## Real-World Examples

### Store WiFi Credentials

```c
void SaveWiFiConfig(const char *ssid, const char *password)
{
    uint8_t buffer[96] = {0};  /* 4-byte aligned size */
    
    /* Pack data into buffer */
    strncpy((char *)buffer, ssid, 32);
    strncpy((char *)buffer + 32, password, 64);
    
    /* Erase sector first */
    FLASH_Erase(FLASH_CONFIG_ADDR);
    
    /* Write new data (must be multiple of 4) */
    flash_status_t status = FLASH_Write(
        FLASH_CONFIG_ADDR,
        buffer,
        96  /* Size must be multiple of 4 */
    );
    
    if (status != FLASH_OK) {
        printf("Failed to save WiFi config\n");
    }
}

void LoadWiFiConfig(char *ssid, char *password)
{
    uint8_t buffer[96] = {0};
    
    flash_status_t status = FLASH_Read(FLASH_CONFIG_ADDR, buffer, 96);
    
    if (status == FLASH_OK) {
        strcpy(ssid, (char *)buffer);
        strcpy(password, (char *)buffer + 32);
    } else {
        printf("Failed to load WiFi config: %s\n", FLASH_GetErrorString(status));
    }
}
```

### Store Balance Information

```c
typedef struct {
    uint32_t current_balance;
    uint32_t total_spent;
    uint32_t last_update;
} balance_data_t;

void SaveBalance(uint32_t balance)
{
    uint8_t buffer[12] = {0};  /* sizeof(balance_data_t) = 12 bytes, pad to 16 for alignment */
    buffer[0] = (balance >> 24) & 0xFF;
    buffer[1] = (balance >> 16) & 0xFF;
    buffer[2] = (balance >> 8) & 0xFF;
    buffer[3] = balance & 0xFF;
    
    /* Note: Store at offset within same sector to preserve other data */
    flash_status_t status = FLASH_Write(
        FLASH_CONFIG_ADDR + 0x40,  /* Offset within sector */
        buffer,
        16  /* Pad to multiple of 4 */
    );
    
    if (status != FLASH_OK) {
        printf("Failed to save balance: %s\n", FLASH_GetErrorString(status));
    }
}

uint32_t LoadBalance(void)
{
    uint8_t buffer[12] = {0};
    
    flash_status_t status = FLASH_Read(FLASH_CONFIG_ADDR + 0x40, buffer, 12);
    
    if (status == FLASH_OK) {
        return (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
    }
    
    return 0;
}
```

### Store MQTT Configuration

```c
void SaveMQTTConfig(const char *broker, uint16_t port, const char *topic)
{
    uint8_t buffer[80] = {0};  /* Multiple of 4 for alignment */
    int offset = 0;
    
    /* Pack broker IP (16 bytes) */
    strncpy((char *)buffer + offset, broker, 16);
    offset += 16;
    
    /* Pack port (2 bytes) */
    buffer[offset] = (port >> 8) & 0xFF;
    buffer[offset + 1] = port & 0xFF;
    offset += 2;
    
    /* Pack topic (64 bytes) */
    strncpy((char *)buffer + offset, topic, 64);
    
    /* Store at offset within config sector */
    flash_status_t status = FLASH_Write(
        FLASH_CONFIG_ADDR + 0x80,  /* Different offset to preserve WiFi config */
        buffer,
        80
    );
    
    if (status != FLASH_OK) {
        printf("Failed to save MQTT config: %s\n", FLASH_GetErrorString(status));
    }
}

void LoadMQTTConfig(char *broker, uint16_t *port, char *topic)
{
    uint8_t buffer[80] = {0};
    
    flash_status_t status = FLASH_Read(FLASH_CONFIG_ADDR + 0x80, buffer, 80);
    
    if (status == FLASH_OK) {
        strcpy(broker, (char *)buffer);
        *port = ((uint16_t)buffer[16] << 8) | buffer[17];
        strcpy(topic, (char *)buffer + 18);
    } else {
        printf("Failed to load MQTT config: %s\n", FLASH_GetErrorString(status));
    }
}
```

## Important Notes

### Address Requirements
- ✅ `FLASH_Read()`: Any address, any size
- ⚠️ `FLASH_Write()`: 
  - Address must be **4-byte aligned** (address % 4 == 0)
  - Size must be **multiple of 4 bytes**
- ✅ `FLASH_Erase()`: Any address within sector

### Erasing Before Writing

Flash sectors must be erased before writing new data:

```c
/* CORRECT: Erase first, then write */
FLASH_Erase(FLASH_CONFIG_ADDR);
FLASH_Write(FLASH_CONFIG_ADDR, data, size);

/* WRONG: Writing to non-erased sector may fail */
// FLASH_Write(FLASH_CONFIG_ADDR, data, size);
```

### Data Alignment

```c
/* CORRECT: 4-byte aligned */
uint8_t data[4] = {0x01, 0x02, 0x03, 0x04};
FLASH_Write(0x08070000, data, 4);  // Address % 4 == 0, Size % 4 == 0

/* WRONG: Not aligned */
uint8_t data[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
FLASH_Write(0x08070000, data, 5);  // Size % 4 != 0 - ERROR
```

## API Reference

| Function | Purpose |
|----------|---------|
| `FLASH_Init()` | Initialize driver (call once at startup) |
| `FLASH_Read(addr, ptr, size)` | Read bytes from Flash |
| `FLASH_Write(addr, ptr, size)` | Write bytes to Flash (must be 4-byte aligned) |
| `FLASH_Erase(addr)` | Erase sector containing address |
| `FLASH_GetSectorNumber(addr)` | Get sector number from address |
| `FLASH_IsAddressValid(addr, size)` | Check if address is valid |

## Error Handling

```c
flash_status_t status = FLASH_Write(addr, data, size);

switch (status) {
    case FLASH_OK:
        printf("Success\n");
        break;
    case FLASH_INVALID_ADDR:
        printf("Address out of user area\n");
        break;
    case FLASH_INVALID_SIZE:
        printf("Size must be multiple of 4\n");
        break;
    case FLASH_TIMEOUT:
        printf("Operation timeout\n");
        break;
    case FLASH_ERROR:
        printf("Flash error\n");
        break;
}
```

## Performance

| Operation | Time |
|-----------|------|
| Read 256 bytes | ~100 μs |
| Write 256 bytes | ~30 ms |
| Erase sector | ~500 ms |

## Troubleshooting

### Problem: Write fails with FLASH_INVALID_SIZE
**Solution**: Ensure size is multiple of 4 and address is 4-byte aligned
```c
// Check alignment
if (addr % 4 != 0) printf("Address not aligned\n");
if (size % 4 != 0) printf("Size not multiple of 4\n");
```

### Problem: Write fails with FLASH_INVALID_ADDR
**Solution**: Check address is in user area (0x08070000 - 0x0807FFFF)
```c
if (!FLASH_IsAddressValid(addr, size)) {
    printf("Address outside user area (0x08070000 - 0x0807FFFF)\n");
}
```

### Problem: Data not persisting
**Solution**: Erase sector before writing
```c
FLASH_Erase(addr);  // Must erase first!
FLASH_Write(addr, data, size);
```

## Integration with Other Modules

### With ESP8266 Handler
```c
/* In ESP8266_Handler.c */
#include "Flash_Driver.h"

void SaveESPCredentials(const char *ssid, const char *password)
{
    uint8_t config[96] = {0};
    
    /* Pack SSID and password */
    strncpy((char *)config, ssid, 32);
    strncpy((char *)config + 32, password, 64);
    
    FLASH_Erase(FLASH_CONFIG_ADDR);
    FLASH_Write(FLASH_CONFIG_ADDR, config, 96);
}

void LoadESPCredentials(char *ssid, char *password)
{
    uint8_t config[96] = {0};
    FLASH_Read(FLASH_CONFIG_ADDR, config, 96);
    
    strcpy(ssid, (char *)config);
    strcpy(password, (char *)config + 32);
}
```

### With MDB Handler
```c
/* In MDB_Handler.c */
#include "Flash_Driver.h"

void SaveBalance(uint32_t balance)
{
    uint8_t data[4];
    data[0] = (balance >> 24) & 0xFF;
    data[1] = (balance >> 16) & 0xFF;
    data[2] = (balance >> 8) & 0xFF;
    data[3] = balance & 0xFF;
    
    FLASH_Write(FLASH_CONFIG_ADDR + 0x40, data, 4);
}

uint32_t LoadBalance(void)
{
    uint8_t data[4] = {0};
    FLASH_Read(FLASH_CONFIG_ADDR + 0x40, data, 4);
    
    return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
}
```