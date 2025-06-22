#ifndef BV_HEADER_H_
#define BV_HEADER_H_

/* ###################################################################################################################
                                                        Includes
################################################################################################################### */
#include <stdint.h>
#include <stdbool.h>
/* ###################################################################################################################
                                                    DATA DECLARATIONS
################################################################################################################### */
typedef struct {
    uint16_t MDB_RXbuffer[36];
    uint16_t MDB_TXbuffer[36];
    uint8_t RXBuffer_index;
    uint8_t TXBuffer_index;
    bool CurCycle_Processed; /* Flag to indicate if the current cycle has been processed */
    bool CurCycle_Missed; /* Flag to indicate if the current cycle has been missed */
    uint8_t MDB_RX_CMD_Index;
    uint8_t MDB_TX_CMD_Index;
    uint8_t MDB_Process_CMD_Index;
} MDB_BusManager;

typedef struct{
    uint16_t CMD[36];   /* Command buffer for MDB commands */
    uint16_t CMD_Length; /* Length of the command buffer */
    uint16_t CMD_Response[36]; /* Response buffer for MDB commands */
    uint16_t CMD_Response_Length; /* Length of the response buffer */
}CMD_Type;



typedef enum {
    STATE_DISABLED,   /* Bill validator is disabled */
    STATE_BUSY,       /* Bill validator is busy processing */
    STATE_READY,      /* Bill validator is ready to accept bills */
    STATE_RESTART,    /* Bill validator is restarting */
    STATE_ERROR       /* Bill validator encountered an error */
} BV_State;

typedef enum {
    CMD_RX_READY = 0, /* Command is ready to be received */
    CMD_RX_INPROGRESS, /* Command is being received */
    CMD_RX_DONE, /* Command has been fully received */
    CMD_RX_BUSY /* Command is busy being processed */
} CMD_RX_State;

typedef enum {
    CMD_TX_READY = 0,       /* Command is ready to be transmitted */
    CMD_TX_INPROGRESS,      /* Command is being transmitted */
    CMD_TX_DONE,            /* Command has been fully transmitted */
    CMD_TX_BUSY             /* Command is busy being processed */
} CMD_TX_State;

typedef enum {
    CMD_PROCESS_READY,      /* Command is ready to be processed */
    CMD_PROCESS_INPROGRESS, /* Command is being processed */
    CMD_PROCESS_DONE,       /* Command has been fully processed */
    CMD_PROCESS_BUSY        /* Command is busy being processed */
} CMD_Process_State;

typedef enum {
    VMC_CMD_0x01E6,
    VMC_CMD_0x01E7,
    VMC_CMD_0x0067,
    VMC_CMD_0x01F9,
    VMC_CMD_0x0133,
    VMC_CMD_0x0199,
    VMC_CMD_0x013B,
    VMC_CMD_0x0132,
    VMC_CMD_0x0077,
    VMC_CMD_0x0066,
    VMC_CMD_0x0074,
    VMC_CMD_0x0064,
    VMC_CMD_0x0099,
    VMC_CMD_0x009D,
    VMC_CMD_MAX_NUMBER
} CMD_Data;

typedef struct{
    BV_State BV_StateHnadler;                   /* State of the bill validator */
    CMD_RX_State BV_CMD_RX_StateHandler;           /* State of the command reception */
    CMD_TX_State BV_CMD_TX_StateHandler;           /* State of the command transmission */
    CMD_Process_State BV_CMD_Process_StateHandler; /* State of the command processing */
}State_Manager;
/* ###################################################################################################################
                                                    DATA DEFINITIONS
################################################################################################################### */
extern MDB_BusManager BV_MDB_BusManager;
extern State_Manager BV_StateManager;
extern CMD_Type VMC_CMDs[VMC_CMD_MAX_NUMBER];
/* ###################################################################################################################
                                                  FUNCTION DECLARATIONS
################################################################################################################### */
// Add your function declarations here

#endif /* BV_HEADER_H_ */
