/* bill_validator.c
 * Source file for bill validator functionality
 */

/* ###################################################################################################################
                                                        Includes
################################################################################################################### */
#include "bill_validator.h"
#include <stdint.h>
#include <stdbool.h>

/* ###################################################################################################################
                                                    Data Declarations
################################################################################################################### */
MDB_BusManager BV_MDB_BusManager = {
    .MDB_RXbuffer = {0},
    .MDB_TXbuffer = {0},
    .RXBuffer_index = 0,
    .TXBuffer_index = 0,
    .CurCycle_Processed = true, /* Initialize the flag to false */
    .CurCycle_Missed = false, /* Initialize the flag to false */
    .MDB_RX_CMD_Index = VMC_CMD_MAX_NUMBER, /* Initialize to a default value */
    .MDB_TX_CMD_Index = VMC_CMD_MAX_NUMBER, /* Initialize to a default value */
    .MDB_Process_CMD_Index = VMC_CMD_MAX_NUMBER /* Initialize to a default value */
};

State_Manager BV_StateManager = {
    .BV_StateHnadler = STATE_RESTART,               /* Initialize the state to RESTART */
    .BV_CMD_RX_StateHandler = CMD_RX_READY,            /* Initialize the command reception state to READY */
    .BV_CMD_TX_StateHandler = CMD_TX_READY,             /* Initialize the command transmission state to BUSY */
    .BV_CMD_Process_StateHandler = CMD_PROCESS_READY    /* Initialize the command processing state to BUSY */
};

CMD_Type VMC_CMDs [VMC_CMD_MAX_NUMBER] = {
    /*  VMC_CMD_0x01E6 */
    {
        .CMD = {0x01E6, 0x000E}, .CMD_Length = 2,
        .CMD_Response = {0x0100}, .CMD_Response_Length = 1
    },
    /*  VMC_CMD_0x01E7 */
    {
        .CMD = {0x01E7, 0x000F}, .CMD_Length = 2,
        .CMD_Response = {0x0100}, .CMD_Response_Length = 1
    },
    /*  VMC_CMD_0x0067 */
    {
        .CMD = {0x0067, 0x00E7}, .CMD_Length = 2,
        .CMD_Response = {
            0x0002, 0x0018, 0x0018, 0x0000, 0x0064, 0x0002, 0x0001, 0x002C,
            0x00FF, 0x00FF, 0x00FF, 0x0001, 0x0005, 0x000A, 0x0014, 0x0032,
            0x0064, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            0x0000, 0x0000, 0x0000, 0x017C
        }, .CMD_Response_Length = 28
    },
    /*  VMC_CMD_0x01F9 */
    {
        .CMD = {0x01F9, 0x01D7, 0x01FB, 0x01FD, 0x01A9, 0x0000}, .CMD_Length = 6,
        .CMD_Response = {
            0x0001, 0x0002, 0x0000, 0x0000, 0x0001, 0x0000, 0x0005, 0x0003, 0x010C
        }, .CMD_Response_Length = 9
    },
    /*  VMC_CMD_0x0133 */
    {
        .CMD = {0x0133, 0x01FF, 0x019B, 0x0000}, .CMD_Length = 4,
        .CMD_Response = {0x0100}, .CMD_Response_Length = 1
    },
    /*  VMC_CMD_0x0199 */
    {
        .CMD = {0x0199, 0x01FF, 0x01FF, 0x01FF, 0x0197, 0x0000}, .CMD_Length = 6,
        .CMD_Response = {0x0100}, .CMD_Response_Length = 1
    },
    /*  VMC_CMD_0x013B */
    {
        .CMD = {0x013B, 0x0000, 0x007B}, .CMD_Length = 3,
        .CMD_Response = {0x0100}, .CMD_Response_Length = 1
    },
    /*  VMC_CMD_0x0132 */
    {
        .CMD = {0x0132, 0x0072}, .CMD_Length = 2,
        .CMD_Response = {0x0000, 0x0000, 0x0100}, .CMD_Response_Length = 3
    },
    /*  VMC_CMD_0x0077 */
    {
        .CMD = {0x00FF, 0x006F, 0x01FF, 0x01FF, 0x004B, 0x0000}, .CMD_Length = 6,
        .CMD_Response = {0x0100}, .CMD_Response_Length = 1
    },
    /*  VMC_CMD_0x0066 */
    {
        .CMD = {0x0066, 0x00E6}, .CMD_Length = 2,
        .CMD_Response = {0}, .CMD_Response_Length = 0
    },
    /*  VMC_CMD_0x0074 */
    {
        .CMD = {0x0074, 0x0159, 0x016D, 0x0175, 0x019F, 0x019F, 0x019F, 0x019F, 0x019F, 0x019F,
                0x019F, 0x019D, 0x0193, 0x0191, 0x0193, 0x018D, 0x016F, 0x0153, 0x0175, 0x0119,
                0x0175, 0x0195, 0x019D, 0x018D, 0x019F, 0x019F, 0x019F, 0x0197, 0x017F, 0x01FD,
                0x00D7, 0x0000}, .CMD_Length = 32,
        .CMD_Response = {
            0x0009, 0x0046, 0x0041, 0x0047, 0x0030, 0x0030, 0x0030, 0x0030,
            0x0030, 0x0030, 0x0030, 0x0030, 0x0030, 0x0030, 0x0030, 0x0030,
            0x0046, 0x0041, 0x0047, 0x0020, 0x0043, 0x0041, 0x0049, 0x004D,
            0x0041, 0x004E, 0x0020, 0x004C, 0x0001, 0x0005, 0x0120
        }, .CMD_Response_Length = 31
    },
    /*  VMC_CMD_0x0064 */
    {
        .CMD = {0x0064, 0x0191, 0x0000}, .CMD_Length = 3,
        .CMD_Response = {
            0x0049, 0x0054, 0x004C, 0x0030, 0x0030, 0x0030, 0x0030, 0x0030,
            0x0035, 0x0031, 0x0030, 0x0037, 0x0035, 0x0035, 0x0034, 0x004E,
            0x0056, 0x0039, 0x0030, 0x0020, 0x0033, 0x0038, 0x0033, 0x0020,
            0x0030, 0x0030, 0x0030, 0x0003, 0x0025, 0x01E7
        }, .CMD_Response_Length = 30
    },
    /*  VMC_CMD_0x0099 */
    {
        .CMD = {0x019B, 0x0000}, .CMD_Length = 2,
        .CMD_Response = {0x0100}, .CMD_Response_Length = 1
    },
    /*  VMC_CMD_0x009D */
    {
        .CMD = {0x009D, 0x01D5, 0x0000}, .CMD_Length = 3,
        .CMD_Response = {0x0100}, .CMD_Response_Length = 1
    }
};
/* ###################################################################################################################
                                                    Function Prototypes
################################################################################################################### */
static void bill_validator_init(void);
static bool bill_validator_check(void);
static void bill_validator_process(void);

/* ###################################################################################################################
                                                    Function Definitions
################################################################################################################### */

/**
 * @brief Initialize the bill validator hardware/module.
 */
static void bill_validator_init(void) {
     // Initialization code here
}


/**
 * @brief Process the bill input and update system state.
 */
static void bill_validator_process(void) {
     // Processing code here
}

/**
 * @brief Check the status of the bill validator.
 * @return true if bill is valid, false otherwise
 */
static bool bill_validator_check(void) {
     // Status checking code here
     return false;
}
