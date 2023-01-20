#ifndef __SERIAL_DEFS_H__
#define __SERIAL_DEFS_H__

#include <stdint.h>

typedef enum __attribute__((__packed__))
{
    CMD_CONNECT = 0x00,			// NO Data
    CMD_ARM_DRIVES,             // NO Data
    CMD_DISARM_DRIVES,          // NO Data
    CMD_STOP,                   // NO Data
    CMD_POSITION,               // OUT Data(8): 2B->BASE 2B-> ARML 2B-> ARMR 2B-> EE
	CMD_GET_VELOCITY,           // OUT Data(8): 2B->BASE 2B-> ARML 2B-> ARMR 2B-> EE
    CMD_SET_VELOCITY,		    // IN Data(8): 2B->BASE 2B-> ARML 2B-> ARMR 2B-> EE
    CMD_PUMP_ON,                // NO Data
    CMD_PUMP_OFF,               // NO Data
	CMD_RESET = 0xFD,  			// IN Data(1): 1B->0xCD
    CMD_ERROR = 0xFE,  			// OUT Data(2): 1B->command, 1B->error code
    CMD_INVALID = 0xFF 			// NO Data
} CommandCode;

typedef enum __attribute__((__packed__))
{
	ERR_INVALID_PACKET = 0x00,
	ERR_UNSUPPORTED_CMD,
	ERR_INVALID_DATA_LEN,
	ERR_INVALID_PAYLOAD
} ErrorCode;

static inline CommandCode byte2Command(const uint8_t b)
{
    if(b >= (uint8_t)CMD_CONNECT && b <= (uint8_t)CMD_PUMP_OFF)
        return (CommandCode)b;
    else if(b == (uint8_t)CMD_RESET)
        return CMD_RESET;
    else if(b == (uint8_t)CMD_ERROR)
        return CMD_ERROR;
    else
        return CMD_INVALID;
}

#endif // __SERIAL_DEFS_H__