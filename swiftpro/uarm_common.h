#ifndef _UARM_COMMON_H_
#define _UARM_COMMON_H_

#include <Arduino.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "cpu_map/cpu_map_atmega2560.h"
#include "serial.h"
#include "print.h"
#include "serial_cobs.h"
#include "nuts_bolts.h"
#include "eeprom.h"

// Define system executor bit map. Used internally by realtime protocol as realtime command flags,
// which notifies the main program to execute the specified realtime command asynchronously.
// NOTE: The system executor uses an unsigned 8-bit volatile variable (8 flag limit.) The default
// flags are always false, so the realtime protocol only needs to check for a non-zero value to
// know when there is a realtime command to execute.
#define EXEC_STATUS_REPORT bit(0) // bitmask 00000001
#define EXEC_CYCLE_START bit(1)	  // bitmask 00000010
#define EXEC_CYCLE_STOP bit(2)	  // bitmask 00000100
#define EXEC_FEED_HOLD bit(3)	  // bitmask 00001000
#define EXEC_RESET bit(4)		  // bitmask 00010000
#define EXEC_SAFETY_DOOR bit(5)	  // bitmask 00100000
#define EXEC_MOTION_CANCEL bit(6) // bitmask 01000000

// Alarm executor bit map.
// NOTE: EXEC_CRITICAL_EVENT is an optional flag that must be set with an alarm flag. When enabled,
// this halts Grbl into an infinite loop until the user aknowledges the problem and issues a soft-
// reset command. For example, a hard limit event needs this type of halt and aknowledgement.
#define EXEC_CRITICAL_EVENT bit(0)	  // bitmask 00000001 (SPECIAL FLAG. See NOTE:)
#define EXEC_ALARM_HARD_LIMIT bit(1)  // bitmask 00000010
#define EXEC_ALARM_SOFT_LIMIT bit(2)  // bitmask 00000100
#define EXEC_ALARM_ABORT_CYCLE bit(3) // bitmask 00001000
#define EXEC_ALARM_PROBE_FAIL bit(4)  // bitmask 00010000
#define EXEC_ALARM_HOMING_FAIL bit(5) // bitmask 00100000

// Define system state bit map. The state variable primarily tracks the individual functions
// of Grbl to manage each without overlapping. It is also used as a messaging flag for
// critical events.
#define STATE_IDLE 0			   // Must be zero. No flags.
#define STATE_ALARM bit(0)		   // In alarm state. Locks out all g-code processes. Allows settings access.
#define STATE_CHECK_MODE bit(1)	   // G-code check mode. Locks out planner and motion only.
#define STATE_HOMING bit(2)		   // Performing homing cycle
#define STATE_CYCLE bit(3)		   // Cycle is running or motions are being executed.
#define STATE_HOLD bit(4)		   // Active feed hold
#define STATE_SAFETY_DOOR bit(5)   // Safety door is ajar. Feed holds and de-energizes system.
#define STATE_MOTION_CANCEL bit(6) // Motion cancel by feed hold and return to idle.

// Define system suspend states.
#define SUSPEND_DISABLE 0			 // Must be zero.
#define SUSPEND_ENABLE_HOLD bit(0)	 // Enabled. Indicates the cycle is active and currently undergoing a hold.
#define SUSPEND_ENABLE_READY bit(1)	 // Ready to resume with a cycle start command.
#define SUSPEND_ENERGIZE bit(2)		 // Re-energizes output before resume.
#define SUSPEND_MOTION_CANCEL bit(3) // Cancels resume motion. Used by probing routine.

volatile uint8_t sys_probe_state;	// Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state; // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm; // Global realtime executor bitflag variable for setting various alarms.

// uARM
#include "uarm_debug.h"
#include "uarm_angle.h"
#include "uarm_timer.h"
#include "uarm_peripheral.h"
#include "uarm_drive.h"
#include "uarm_coord_convert.h"

// The Atmega2560 has 4KB EEPROM.
// #define EEPROM_ADDR_ANGLE_REFER		 	3072U

// Default settings. Used when resetting EEPROM. Change to desired name in defaults.h
#define DEFAULTS_GENERIC

// Serial baud rate
#define BAUD_RATE 115200

// Default cpu mappings
#define CPU_MAP_ATMEGA2560

#define DATA_TYPE_BYTE 1
#define DATA_TYPE_INTEGER 2
#define DATA_TYPE_FLOAT 4

#define EXTERNAL_EEPROM_USER_ADDRESS 0xa0
#define EXTERNAL_EEPROM_SYS_ADDRESS 0xa2

#define EEPROM_ADDR_ANGLE_REFER 820U
#define EEPROM_OLD_ADDR_ANGLE_REFER 800U
#define EEPROM_ANGLE_REFER_FLAG 818U
#define EEPROM_BT_MAC_ADDR 1002U

#define EEPROM_ADDR_PARAM 3200U // refer to v3.2.0 firmware
#define EEPROM_MODE_ADDR 900U
#define EEPROM_HEIGHT_OFFSET_ADDR 910U
#define EEPROM_FRONT_OFFSET_ADDR 920U
#define EEPROM_USER_HEIGHT_OFFSET_ADDR 930U
#define EEPROM_USER_FRONT_OFFSET_ADDR 940U

#define EEPROM_EFFECT_ANGLE_OFFSET_ADDR 3202U

// <! 600 bytes
#define EEPROM_ADDR_ANGLE_CAL_MAP 1U

enum uarm_work_mode_e
{
	WORK_MODE_NORMAL = 0,
	WORK_MODE_LASER,
	WORK_MODE_PRINTER,
	WORK_MODE_PEN,
	WORK_MODE_STEPER_FLAT,	   // <! flat steper mode
	WORK_MODE_STEPER_STANDARD, // <! standard steper mode
	WORK_MODE_TOUCH_PEN,
	WORK_MODE_USER,
	WORK_MODE_ClAM_JAW,
	WORK_MODE_STEERING_GEAR,
	WORK_MODE_CLAMP,
	WORK_MODE_TEST,

};
enum uarm_device_e
{
	UARM_PRO = 0,
	UARM_2500,
	UARM_MINI2,
};

extern enum uarm_device_e uarm_device;

#define DEFAULT_NORMAL_HEIGHT 74.55
#define DEFAULT_NORMAL_FRONT 56.65

#define DEFAULT_LASER_HEIGHT 51.04
#define DEFAULT_LASER_FRONT 64.4

#define DEFAULT_3DPRINT_HEIGHT 74.43
#define DEFAULT_3DPRINT_FRONT 56.5

#define DEFAULT_PEN_HEIGHT 43
#define DEFAULT_PEN_FRONT 69.5

#define DEFAULT_STEP_FLAT_HEIGHT 39
#define DEFAULT_STEP_FLAT_FRONT 76.5

#define DEFAULT_STEP_STANDARD_HEIGHT 73.4
#define DEFAULT_STEP_STANDARD_FRONT 76

#define DEFAULT_ROUND_PEN_HEIGHT 77.2
#define DEFAULT_ROUND_PEN_FRONT 54.5

#define DEFAULT_CLAW_JAW_HEIGHT 0
#define DEFAULT_CLAW_JAW_FRONT 0

#define DEFAULT_STEERING_GEAR_HEIGHT 74.55
#define DEFAULT_STEEPING_GEAR_FRONT 56.65

#define DEFAULT_CLAMP_HEIGHT 74.55
#define DEFAULT_CLAMP_FRONT 56.65

#define DEFAULT_TEST_HEIGHT 25.49
#define DEFAULT_TEST_FRONT 44.5

struct key_param_t
{
	enum uarm_work_mode_e work_mode;
	float front_offset;
	float high_offset;
	float effect_angle_offset;
};

struct uarm_state_t
{
	struct key_param_t param;

	float arml_angle;
	float armr_angle;
	float base_angle;
	float coord_x;
	float coord_y;
	float coord_z;

	bool effect_origin_check;
	bool effect_ldie;
	bool beep_ldie;
	bool power_state;
	bool motor_position_check;
	bool beep_state;
};

extern struct uarm_state_t uarm;

#endif
