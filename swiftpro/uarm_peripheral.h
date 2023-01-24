#ifndef _UARM_PERIPHERAL_H_
#define _UARM_PERIPHERAL_H_

#include "uarm_common.h"

extern char hardware_version[8];
extern char bt_mac_addr[13];

typedef enum _pump_state_t
{
	PUMP_STATE_OFF,
	PUMP_STATE_ON,
	PUMP_STATE_SUCTION,
	PUMP_STATE_VALVE_ON,
	PUMP_STATE_VALVE_OFF,
	PUMP_STATE_COUNT
} pump_state_t;
#define VALVE_EN 4

void end_effector_init(void);
void end_effector_set_angle(float angle);
void end_effector_deinit(void);
float end_effector_get_angle(void);

void beep_tone(unsigned long duration, double frequency);

void read_sys_param(void);
void save_sys_param(void);
void save_user_endoffest(void);
void read_user_endoffest(void);
void write_sn_num(void);

void cycle_report_coord(void);
void read_hardware_version(void);

void pump_on(void);
void pump_off(void);
void pump_suction(void);
void pump_tick(void);
pump_state_t pump_get_state(void);

uint8_t get_pump_status(void);

void gripper_release(void);
void gripper_catch(void);
uint8_t get_gripper_status(void);

void laser_on(void);
void laser_off(void);
uint8_t get_laser_status(void);

void button_init();
uint8_t get_button_status(uint8_t button);

uint8_t get_limit_switch_status(void);
uint8_t get_power_status(void);

void check_motor_positon(void);
void update_motor_position(void);

unsigned long millis_2(void);
void setE2PROMData(unsigned char device, unsigned int addr, unsigned char type, float value);
double getE2PROMData(unsigned char device, unsigned int addr, unsigned char type);

#endif
