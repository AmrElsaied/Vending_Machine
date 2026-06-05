# MDB_Handler — Multi-Drop Bus Protocol Driver

## Purpose

Implements the MDB (Multi-Drop Bus) slave protocol for communication with cashless payment peripherals (card readers, etc.). Operates over a 9-bit UART link at 9600 baud where the 9th bit distinguishes address/command words from data words.

Source: [Core/Src/MDB_Handler.c](../Core/Src/MDB_Handler.c), [Core/Inc/MDB_Handler.h](../Core/Inc/MDB_Handler.h)  
Config: [Core/Src/MDB_PBCFG.c](../Core/Src/MDB_PBCFG.c)

---

## Key Types

### `Peripheral_State_t` — Cashless Device State Machine

```c
// Core/Inc/MDB_Handler.h
typedef enum {
    STATE_INACTIVE,
    STATE_RESET,
    STATE_DISABLED,
    STATE_ENABLED,
    STATE_SESSION_IDLE,
    STATE_INIT,
    STATE_START_SESSION,
    STATE_CANCEL_SESSION,
    STATE_APPROVE_VEND_REQ,
    STATE_DENY_VEND_REQ,
    STATE_VEND_PROCESS,
    STATE_ERROR,
    STATE_WAIT_ACK,
} Peripheral_State_t;
```

### `CMD_RX_State` / `CMD_TX_State` / `CMD_Process_State`

Bus-level handshake states. Each tracks progress independently so RX, processing, and TX pipelines do not block each other.

### `mdb_ring_t` — ISR-Safe Ring Buffer

```c
// Core/Inc/MDB_Handler.h
typedef struct {
    uint16_t buf[MDB_RING_LEN];   // 40-word circular buffer (power-of-2 size)
    volatile uint16_t wr;          // Write index — updated only from UART ISR
    volatile uint16_t rd;          // Read index — updated only from mdbRxTask
} mdb_ring_t;
```

`MDB_RING_LEN` = 40 (`Core/Inc/MDB_Handler.h`). No mutex needed: single producer (ISR), single consumer (task).

### `MDB_StateManager_t`

Tracks the cashless peripheral's current state, any pending state-change request, and the current RX/TX/process sub-states.

### `MDB_BusManager_t`

Tracks raw bus traffic: the receive buffer (`MDB_RXbuffer[36]`), current buffer index, command index being received, command index being processed, ACK wait flag, and ACK skip counters.

### `Vending_Item_Data_t`

Holds the in-progress vend request data: requested price (H/L byte), requested item ID (H/L byte), response price (H/L byte), and `vend_item_state_t` outcome (`VEND_ITEM_SUCCESS` / `VEND_ITEM_FAILURE`).

### `Peripheral_balance_t`

Tracks `Max_balance`, `Cur_balance`, and `Revalue_limit` for the connected cashless peripheral.

### `MDB_Config_t`

Post-build configuration: `mdb_uart` (`UART_HandleTypeDef*`) and `bus_timeout` (ticks). Instantiated in [Core/Src/MDB_PBCFG.c](../Core/Src/MDB_PBCFG.c).

---

## Configuration Macros

| Macro | Value | Location | Purpose |
|-------|-------|----------|---------|
| `MDB_RING_LEN` | 40 | `MDB_Handler.h` | Ring buffer depth (must be power of 2) |
| `MDB_BUS_TIMEOUT` | 10 | `MDB_Handler.h` | Bus idle timeout in FreeRTOS ticks (10 ms) |

---

## Public API

| Function | Signature | Purpose |
|----------|-----------|---------|
| `MDB_BusInit` | `void MDB_BusInit(void)` | Apply config, enable UART RX interrupt, arm first receive |
| `MDB_UART_RxCallback` | `void MDB_UART_RxCallback(uint16_t word)` | Called from `HAL_UART_RxCpltCallback`; pushes word to ring and notifies `mdbRxTask` |
| `MDB_ReceiveCommand` | `void MDB_ReceiveCommand(uint16_t word)` | Called by `mdbRxTask`; drives the receive state machine; triggers `xTaskNotify(mdbCMDProcessTask)` when a complete command is framed |
| `MDB_HandleCommand` | `void MDB_HandleCommand(uint8_t cmd_index)` | Called by `mdbCMDProcessTask`; dispatches to the correct command handler |
| `MDB_SendResponseWithModeBit` | `void MDB_SendResponseWithModeBit(...)` | Builds and sends a response frame with the MDB mode bit set |
| `SetVendReq_State` | `void SetVendReq_State(vend_req_state_t)` | Set current vend request state (called from MQTT keyword handler) |
| `GetVendReq_State` | `vend_req_state_t GetVendReq_State(void)` | Read current vend request state |
| `mdbRing_init` | `void mdbRing_init(mdb_ring_t*)` | Initialise ring buffer |
| `mdbRing_push` | `bool mdbRing_push(mdb_ring_t*, uint16_t)` | ISR-safe push; returns false if full |
| `mdbRing_pop` | `bool mdbRing_pop(mdb_ring_t*, uint16_t*)` | Task-safe pop; returns false if empty |
| `mdbRing_count` | `uint16_t mdbRing_count(mdb_ring_t*)` | Items currently in buffer |
| `mdbRing_free` | `uint16_t mdbRing_free(mdb_ring_t*)` | Free slots in buffer |

---

## Command Handler Table

Each handler corresponds to a specific MDB command opcode received on the bus.

| Handler | Opcode | Description |
|---------|--------|-------------|
| `handle_cmd_0x01E7` | 0x01E7 | RESET — resets cashless device state |
| `handle_cmd_0x013B` | 0x013B | Setup / status poll |
| `handle_cmd_0x01D5` | 0x01D5 | Heartbeat / health check |
| `handle_cmd_0x0074` | 0x0074 | Identity / device info request |
| `handle_cmd_0x0077` | 0x0077 | Multi-part command |
| `handle_cmd_0x0075` | 0x0075 | Vend request processing |
| `handle_cmd_0x0076` | 0x0076 | Session command |
| `handle_cmd_0x01D7` | 0x01D7 | Extended command |

Response data for each command is looked up from `VMC_CMDs[]` via the `CMD_Data` enum index. See [VMC_Config.md](VMC_Config.md).

---

## RX Task Notification Flow

```
USART1 RX interrupt (1 word)
  └─► HAL_UART_RxCpltCallback()          [main.c]
        └─► MDB_UART_RxCallback(word)     [MDB_Handler.c]
              ├─ mdbRing_push(&rxRing, word)
              └─ xTaskNotifyFromISR(mdbRxTaskHandle)

mdbRxTask unblocks
  ├─ Check bus timeout (reset state if idle > MDB_BUS_TIMEOUT ticks)
  ├─ mdbRing_pop(&rxRing, &word)
  └─ MDB_ReceiveCommand(word)
        ├─ Drive CMD_RX_State machine
        └─ When complete: xTaskNotify(mdbCMDProcessTaskHandle, cmd_index)

mdbCMDProcessTask unblocks
  └─ MDB_HandleCommand(cmd_index)
        └─ handle_cmd_0xXXXX()
              └─ MDB_SendResponseWithModeBit()
                    └─ HAL_UART_Transmit_IT(USART1, ...)
```

---

## Vend Request Lifecycle

1. MQTT message arrives with keyword `ACTIVATION:` → `handle_vend_request_topic()` calls `SetVendReq_State(VEND_REQ_APPROVED)` or `VEND_REQ_DENIED`.
2. `handle_cmd_0x0075` checks `GetVendReq_State()` to build the correct MDB vend-approve / vend-deny response.
3. Outcome is stored in `Vending_Item_Data.Vend_Item_State` (`VEND_ITEM_SUCCESS` / `VEND_ITEM_FAILURE`).
4. Result is published back via `ESP_RequestPublish()`.

---

## Error Codes Used

All logged via `MDB_LOG_ERROR()` / `MDB_LOG_CRITICAL_ERROR()` macros. See [SYS_Logger.md](SYS_Logger.md) for the full error code table.

| Code | Meaning |
|------|---------|
| `MDB_ERROR_UART_NOT_CONFIGURED` (0x01) | UART handle is NULL at init |
| `MDB_ERROR_SUB_COMMAND_NOT_RECOGNIZED` (0x02) | Unknown sub-command byte |
| `MDB_ERROR_INVALID_RX_STATE` (0x03) | RX state machine in unexpected state |
| `MDB_ERROR_INVALID_PROCESS_STATE` (0x04) | Process state machine in unexpected state |
| `MDB_ERROR_COMMAND_NOT_VALID_IN_STATE` (0x05) | Command received in wrong device state |
| `MDB_ERROR_INVALID_STATE_FOR_RESET` (0x06) | RESET command in invalid state |
| `MDB_ERROR_UNEXPECTED_ACK_WORD` (0x07) | Unexpected ACK received |
| `MDB_ERROR_BALANCE_EXCEEDS_MAXIMUM` (0x08) | `Cur_balance > Max_balance` |

---

## Inter-Module Dependencies

| Uses | For |
|------|-----|
| `VMC_Config.h` | `VMC_CMDs[]` command/response database |
| `system_tasks.h` | `mdbCMDProcessTaskHandle` for `xTaskNotify()` |
| `SYS_Logger.h` | Error/debug logging |
| `ESP8266_Handler.h` | `ESP_RequestPublish()` to report vend results |
| `main.h` | HAL types, GPIO pin definitions |
