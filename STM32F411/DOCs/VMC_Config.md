# VMC_Config — MDB Command/Response Database

## Purpose

Defines the static `VMC_CMDs[]` array that maps every MDB command the VMC must respond to into its command word sequence and its pre-built response bytes. `MDB_Handler` uses the `CMD_Data` enum as an index into this array rather than embedding raw byte literals in command handler functions.

Source: [Core/Src/VMC_Config.c](../Core/Src/VMC_Config.c), [Core/Inc/VMC_Config.h](../Core/Inc/VMC_Config.h)

---

## Key Types

### `CMD_Type` — Command Entry

```c
// Core/Inc/VMC_Config.h
typedef struct {
    uint16_t CMD[5];              // Command word sequence (up to 5 words)
    uint16_t CMD_Length;          // Number of valid words in CMD[]
    uint16_t CMD_Response[36];    // Pre-built response word sequence
    uint16_t CMD_Response_Length; // Number of valid words in CMD_Response[]
} CMD_Type;
```

### `CMD_Data` — Array Index Enum

```c
typedef enum {
    VMC_CMD_0x01E7 = 0,   // RESET
    VMC_CMD_0x013B = 1,   // Setup / status poll
    VMC_CMD_0x01D5 = 2,   // Heartbeat / health check
    VMC_CMD_0x0074 = 3,   // Identity / device info
    VMC_CMD_0x0077 = 4,   // Multi-part command
    VMC_CMD_0x0075 = 5,   // Vend request
    VMC_CMD_0x0076 = 6,   // Session command
    VMC_CMD_0x01D7 = 7,   // Extended command
    VMC_CMD_MAX_NUMBER,
} CMD_Data;
```

---

## Command Database (`VMC_CMDs[]`)

The array is declared `extern` in `VMC_Config.h` and defined in `VMC_Config.c`.

| Index | Enum | CMD Words | CMD_Length | Response Words | Resp_Length | Description |
|-------|------|-----------|------------|----------------|-------------|-------------|
| 0 | `VMC_CMD_0x01E7` | `0x01E7, 0x000F` | 2 | `0x0100` | 1 | RESET — ACK response |
| 1 | `VMC_CMD_0x013B` | `0x013B, 0x007B` | 2 | — | 0 | Setup / status (no response) |
| 2 | `VMC_CMD_0x01D5` | `0x01D5, 0x0000` | 2 | `0x0100` | 1 | Heartbeat — ACK response |
| 3 | `VMC_CMD_0x0074` | `0x0074, 0x0000` | 2 | Identity string + model info | 31 | Device identification |
| 4 | `VMC_CMD_0x0077` | `0x0077, 0x0000` | 2 | — | 0 | Multi-part (no static response) |
| 5 | `VMC_CMD_0x0075` | `0x0075, 0x00FF, 0x007A` | 3 | `0x000F, 0x0001, 0x0000, 0x0000` | 4 | Vend request |
| 6 | `VMC_CMD_0x0076` | `0x0076, 0x0000` | 2 | — | 0 | Session command (dynamic response) |
| 7 | `VMC_CMD_0x01D7` | `0x01D7, 0x0000` | 2 | `0x0100` | 1 | Extended command — ACK response |

> Commands with `Resp_Length = 0` have dynamic responses built at runtime inside the corresponding `handle_cmd_*` function in `MDB_Handler.c`.

---

## Configuration Flags

```c

```

---

## Usage Pattern

```c
// Core/Src/MDB_Handler.c — typical lookup
uint16_t *resp     = VMC_CMDs[VMC_CMD_0x01E7].CMD_Response;
uint16_t  resp_len = VMC_CMDs[VMC_CMD_0x01E7].CMD_Response_Length;
MDB_SendResponseWithModeBit(resp, resp_len);
```

MDB_Handler matches the received opcode to the enum index and uses the pre-built response directly. For commands with `Resp_Length = 0`, the handler builds the response bytes dynamically (e.g., using current balance, session data, or vend result).
