#include "uarm_common.h"
#include "uarm_iic.h"
#include "uarm_coord_convert.h"

/*	end-effector driver
 *
 */

/*************** servo mode ****************/
float duty_ms = 1.51;
static void servo_init(void)
{
	time4_pwm_init(20000);
	time4_set_duty(3, (duty_ms / 20.0) * 1023);
}

static void servo_set_angle(float angle)
{
	if (angle > 180)
	{
		angle = 0;
	}
	if (angle < 0)
	{
		angle = 180;
	}
	angle = 180 - angle;

	duty_ms = (angle / 180.0) * 2.0 + 0.5;
	time4_set_duty(3, (duty_ms / 20.0) * 1023);
	// delay_ms(500);
}

static void servo_deinit(void)
{
	time4_stop();
}

static float servo_get_angle(void)
{
	return 180 - (duty_ms - 0.5) / 2.0 * 180;
}

void end_effector_init(void)
{
	servo_init();
}

void end_effector_set_angle(float angle)
{
	if (angle > 360 || angle < 0)
	{
		return;
	}
	servo_set_angle(angle);
}

void end_effector_deinit(void)
{
	servo_deinit();
}

float end_effector_get_angle(void)
{
	return servo_get_angle();
}

/*  beep driver
 *
 */
unsigned long beep_duration = 520;
static void beep_creater_callback(void)
{
	static bool state = false;
	static unsigned long cnt = 0;
	if (!state)
	{
		state = true;
		PORTL |= (1 << 5);
	}
	else
	{
		state = false;
		PORTL &= ~(1 << 5);
	}
	if (cnt++ > beep_duration)
	{
		cnt = 0;
		time2_stop();
		uarm.beep_ldie = true;
	}
}

void beep_tone(unsigned long duration, double frequency)
{
	uarm.beep_ldie = false;
	DDRL |= 1 << 5;
	PORTL &= ~(1 << 5);
	beep_duration = (duration * 2 * frequency / 1000) - 1;
	//     DB_PRINT_STR("beep_duration");
	//  DB_PRINT_INT(beep_duration);
	time2_set(frequency, beep_creater_callback);
	time2_start();
}

/*	delay time  : ms
 *
 */
void cycle_report_coord(void)
{
	float x = 0, y = 0, z = 0, angle_e;
	float angle_l = 0, angle_r = 0, angle_b = 0;
	char l_str[20] = {0}, r_str[20] = {0}, b_str[20] = {0}, e_str[20] = {0};

	angle_l = calculate_current_angle(CHANNEL_ARML); // <! calculate init angle
	angle_r = calculate_current_angle(CHANNEL_ARMR);
	angle_b = calculate_current_angle(CHANNEL_BASE) - 90;
	angle_e = servo_get_angle();

	angle_to_coord(angle_l, angle_r, angle_b, &x, &y, &z);
	//
	coord_arm2effect(&x, &y, &z);

	dtostrf(x, 5, 4, l_str);
	dtostrf(y, 5, 4, r_str);
	dtostrf(z, 5, 4, b_str);
	dtostrf(angle_e, 5, 4, e_str);
	//	DB_PRINT_INT(sys.state);
	uart_printf("@3 X%s Y%s Z%s R%s\n", l_str, r_str, b_str, e_str);
}

/*	param to eeprom
 *
 */

void read_sys_param(void)
{
	int8_t read_size = 0;
	unsigned int read_addr = 0;
	char *p = NULL;

	p = (char *)(&(uarm.param.work_mode));
	read_size = sizeof(char);
	read_addr = EEPROM_MODE_ADDR;
	for (; read_size > 0; read_size--)
	{
		*(p++) = eeprom_get_char(read_addr++);
	}

	p = (char *)(&(uarm.param.front_offset));
	read_size = sizeof(uarm.param.front_offset);
	read_addr = EEPROM_FRONT_OFFSET_ADDR;
	for (; read_size > 0; read_size--)
	{
		*(p++) = eeprom_get_char(read_addr++);
	}

	p = (char *)(&(uarm.param.high_offset));
	read_size = sizeof(uarm.param.high_offset);
	read_addr = EEPROM_HEIGHT_OFFSET_ADDR;
	for (; read_size > 0; read_size--)
	{
		*(p++) = eeprom_get_char(read_addr++);
	}

	p = (char *)(&(uarm.param.effect_angle_offset));
	read_size = sizeof(uarm.param.effect_angle_offset);
	read_addr = EEPROM_EFFECT_ANGLE_OFFSET_ADDR;
	for (; read_size > 0; read_size--)
	{
		*(p++) = eeprom_get_char(read_addr++);
	}
	if (uarm.param.effect_angle_offset > 360 || isnan(uarm.param.effect_angle_offset))
	{
		uarm.param.effect_angle_offset = 0;
	}
	for (; read_size > 0; read_size--)
	{
		*(p++) = eeprom_get_char(read_addr++);
	}

	p = bt_mac_addr;
	read_size = 12;
	read_addr = EEPROM_BT_MAC_ADDR;
	for (; read_size > 0; read_size--)
	{
		*(p++) = eeprom_get_char(read_addr++);
	}

	//	char l_str[20], r_str[20], b_str[20];
	//	dtostrf( uarm.param.work_mode, 5, 4, l_str );
	//	dtostrf( uarm.param.front_offset, 5, 4, r_str );
	//	dtostrf( uarm.param.high_offset, 5, 4, b_str );
	//
	// DB_PRINT_STR( "mode:%s, front:%s, high:%s\r\n", l_str, r_str, b_str );

	char l_str[20], r_str[20], b_str[20];
	dtostrf(uarm.param.effect_angle_offset, 5, 4, b_str);
	// DB_PRINT_STR("angle offset:%s\r\n", b_str);
}

void save_sys_param(void)
{
	int8_t write_size = 0;
	unsigned int write_addr = 0;
	char *p = NULL;

	p = (char *)(&(uarm.param.work_mode));
	write_size = sizeof(char);
	write_addr = EEPROM_MODE_ADDR;
	for (; write_size > 0; write_size--)
	{
		eeprom_put_char(write_addr++, *(p++));
	}

	p = (char *)(&(uarm.param.front_offset));
	write_size = sizeof(uarm.param.front_offset);
	write_addr = EEPROM_FRONT_OFFSET_ADDR;
	for (; write_size > 0; write_size--)
	{
		eeprom_put_char(write_addr++, *(p++));
	}

	p = (char *)(&(uarm.param.high_offset));
	write_size = sizeof(uarm.param.high_offset);
	write_addr = EEPROM_HEIGHT_OFFSET_ADDR;
	for (; write_size > 0; write_size--)
	{
		eeprom_put_char(write_addr++, *(p++));
	}

	p = (char *)(&(uarm.param.effect_angle_offset));
	write_size = sizeof(uarm.param.effect_angle_offset);
	write_addr = EEPROM_EFFECT_ANGLE_OFFSET_ADDR;
	for (; write_size > 0; write_size--)
	{
		eeprom_put_char(write_addr++, *(p++));
	}
}

void save_user_endoffest(void)
{
	int8_t write_size = 0;
	unsigned int write_addr = 0;
	char *p = NULL;

	p = (char *)(&(uarm.param.front_offset));
	write_size = sizeof(uarm.param.front_offset);
	write_addr = EEPROM_USER_FRONT_OFFSET_ADDR;
	for (; write_size > 0; write_size--)
	{
		eeprom_put_char(write_addr++, *(p++));
	}

	p = (char *)(&uarm.param.high_offset);
	write_size = sizeof(uarm.param.high_offset);
	write_addr = EEPROM_USER_HEIGHT_OFFSET_ADDR;
	for (; write_size > 0; write_size--)
	{
		eeprom_put_char(write_addr++, *(p++));
	}
}

void read_user_endoffest(void)
{
	int8_t read_size = 0;
	unsigned int read_addr = 0;
	char *p = NULL;

	p = (char *)(&(uarm.param.front_offset));
	read_size = sizeof(uarm.param.front_offset);
	read_addr = EEPROM_USER_FRONT_OFFSET_ADDR;
	for (; read_size > 0; read_size--)
	{
		*(p++) = eeprom_get_char(read_addr++);
	}

	p = (char *)(&(uarm.param.high_offset));
	read_size = sizeof(uarm.param.high_offset);
	read_addr = EEPROM_USER_HEIGHT_OFFSET_ADDR;
	for (; read_size > 0; read_size--)
	{
		*(p++) = eeprom_get_char(read_addr++);
	}
}

void write_sn_num(void)
{
	int8_t write_size = 0;
	unsigned int write_addr = 0;
	char *p = NULL;

	p = bt_mac_addr;
	write_size = 12;
	write_addr = EEPROM_BT_MAC_ADDR;
	for (; write_size > 0; write_size--)
	{
		eeprom_put_char(write_addr++, *(p++));
	}
}

void setE2PROMData(unsigned char device, unsigned int addr, unsigned char type, float value)
{
	uint8_t deviceAddr;

	union
	{
		float fdata;
		uint8_t data[4];
	} FData;

	switch (device)
	{
	case 1:
		deviceAddr = EXTERNAL_EEPROM_USER_ADDRESS;
		break;

	case 2:
		deviceAddr = EXTERNAL_EEPROM_SYS_ADDRESS;
		break;
	}

	if (device == 1 || device == 2)
	{
		int num = 0;
		switch (type)
		{
		case DATA_TYPE_BYTE:
			FData.data[0] = (unsigned char)value;

			num = 1;
			break;

		case DATA_TYPE_INTEGER:
		{
			int i_val = 0;
			i_val = (int)value;
			FData.data[0] = (i_val & 0xff00) >> 8;
			FData.data[1] = i_val & 0xff;
			num = 2;
		}
		break;

		case DATA_TYPE_FLOAT:
			FData.fdata = (float)value;
			num = 4;
			break;
		}

		unsigned char i = 0;
		i = (addr % 128);
		// Since the eeprom's sector is 128 byte, if we want to write 5 bytes per cycle we need to care about when there's less than 5 bytes left
		if (i >= (129 - num))
		{
			i = 128 - i;
			eeprom_iic_writebuf(FData.data, deviceAddr, addr, i); // write data
			delay_ms(5);
			eeprom_iic_writebuf(FData.data + i, deviceAddr, addr + i, num - i); // write data
		}
		// if the left bytes are greater than 5, just do it
		else
		{
			eeprom_iic_writebuf(FData.data, deviceAddr, addr, num); // write data
		}
	}
}

double getE2PROMData(unsigned char device, unsigned int addr, unsigned char type)
{
	double result = 0;
	uint8_t deviceAddr;
	uint8_t x_str[20];
	union
	{
		float fdata;
		uint8_t data[4];
	} FData;

	switch (device)
	{
	case 1:
		deviceAddr = EXTERNAL_EEPROM_USER_ADDRESS;
		break;
	case 2:
		deviceAddr = EXTERNAL_EEPROM_SYS_ADDRESS;
		break;
	}

	if (device == 1 || device == 2)
	{
		unsigned char num = 0;
		switch (type)
		{
		case DATA_TYPE_BYTE:
			num = 1;
			break;

		case DATA_TYPE_INTEGER:
			num = 2;
			break;

		case DATA_TYPE_FLOAT:
			num = 4;
			break;
		}

		unsigned char i = 0;
		i = (addr % 128);

		if (i >= (129 - num))
		{
			i = 128 - i;
			eeprom_iic_readbuf(FData.data, deviceAddr, addr, i);
			delay_ms(5);
			eeprom_iic_readbuf(FData.data + i, deviceAddr, addr + i, num - i);
		}
		else
		{

			x_str[1] = eeprom_iic_readbuf(FData.data, deviceAddr, addr, num);
		}

		switch (type)
		{
		case DATA_TYPE_BYTE:
			result = FData.data[0];
			if (FData.data[0] == 32)
			{
				uart_printf("yes\n");
			}

			break;

		case DATA_TYPE_INTEGER:
			result = (FData.data[0] << 8) + FData.data[1];
			break;

		case DATA_TYPE_FLOAT:
			result = FData.fdata;
			break;
		}
	}
	return result;
}

void read_hardware_version(void)
{
#define HARD_MASK (1 << 6 | 1 << 5 | 1 << 4 | 1 << 3 | 1 << 2)
	DDRJ &= ~HARD_MASK;
	if ((PINJ & HARD_MASK) == (1 << 6))
	{
		strcpy(hardware_version, "V3.3.1");
	}
	else if ((PINJ & HARD_MASK) == 0x00)
	{ // <! old version hardware
		strcpy(hardware_version, "V3.3.0");
	}
	else if ((PINJ & HARD_MASK) == (1 << 5))
	{
		strcpy(hardware_version, "V3.3.2");
	}
	else if ((PINJ & HARD_MASK) == (1 << 4))
	{
		if (PINJ & HARD_MASK == (1 << 2))
		{

			strcpy(hardware_version, "V3.3.4"); // uArm Mini Version hardware
			uarm_device = UARM_MINI2;
		}
		else
		{

			strcpy(hardware_version, "V3.3.3"); // uArm Mini Version hardware
			uarm_device = UARM_MINI2;
		}
	}
	else
	{
		strcpy(hardware_version, "V0.0.0");
	}
}

static void _sort(unsigned int array[], unsigned int len)
{
	unsigned char i = 0, j = 0;
	unsigned int temp = 0;

	for (i = 0; i < len; i++)
	{
		for (j = 0; i + j < (len - 1); j++)
		{
			if (array[j] > array[j + 1])
			{
				temp = array[j];
				array[j] = array[j + 1];
				array[j + 1] = temp;
			}
		}
	}
}

static int adc_read_value(uint8_t pin)
{
	uint8_t low, high;
	if (pin >= 54)
		pin -= 54; // allow for channel or pin numbers

	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
	ADMUX = (1 << 6) | (pin & 0x07);
	ADCSRA |= (1 << ADSC);
	while (!(ADCSRA & (1 << ADSC)))
		;
	low = ADCL;
	high = ADCH;
	return (high << 8) | low;
}

static int getAnalogPinValue(unsigned int pin)
{
	unsigned int dat[8];

	for (int i = 0; i < 8; i++)
	{
		dat[i] = adc_read_value(pin);
	}

	_sort(dat, 8);

	unsigned int result = (dat[2] + dat[3] + dat[4] + dat[5]) / 4;

	return result;
}

/*	pump
 *
 */
#define VALVE_ON_TIMES_MAX 10

pump_state_t pump_state = PUMP_STATE_OFF;

void pump_set_state(pump_state_t state)
{
	if (state >= PUMP_STATE_COUNT)
		return;

	pump_state = state;
}
pump_state_t pump_get_state()
{
	return pump_state;
}

void pump_on(void)
{
	DDRG |= 0x10; // PG4 PUMP_D5_N
	DDRG |= 0x20; // PG5 PUMP_D5
	DDRF |= 0x08;

	if ((PINJ & HARD_MASK) == (1 << 4))
	{
		PORTG &= (~0x10);

		PORTG |= 0x20;
	}
	else
	{
		PORTG &= (~0x20);

		PORTG |= 0x10;
	}
	PORTF |= 0x08;

	pump_set_state(PUMP_STATE_ON);
}

void pump_suction(void)
{

	DDRG |= 0x10; // PG4 PUMP_D5_N
	DDRG |= 0x20; // PG5 PUMP_D5
	DDRF |= 0x08; //

	if ((PINJ & HARD_MASK) == (1 << 4))
	{

		PORTG &= (~0x20);

		PORTG |= 0x10;
	}
	else
	{
		PORTG &= (~0x10);

		PORTG |= 0x20;
	}
	PORTF &= (~0x08);

	pump_set_state(PUMP_STATE_SUCTION);
}

void pump_off(void)
{
	if ((pump_get_state() == PUMP_STATE_ON) || (pump_get_state() == PUMP_STATE_SUCTION))
	{

		DDRG |= 0x10; // PG4 PUMP_D5_N
		DDRG |= 0x20; // PG5 PUMP_D5
		DDRF |= 0x08; //

		PORTF &= (~0x08);
		PORTG &= (~0x10);
		if ((PINJ & HARD_MASK) == (1 << 4))
		{
			if (pump_get_state() == PUMP_STATE_ON)
			{
				PORTG &= (~0x20);
				PORTG |= 0x10;
				delay_ms(50);
				PORTG &= (~0x10);
			}
			else
			{

				PORTG |= 0x20;
			}
		}
		else
		{
			PORTG |= 0x20;
		}

		pump_set_state(PUMP_STATE_VALVE_ON);
	}
}
void pump_tick(void)
{
	static uint8_t valve_on_times = 0;
#define HARD_MASK (1 << 6 | 1 << 5 | 1 << 4 | 1 << 3)

	switch (pump_state)
	{
	case PUMP_STATE_OFF:
		valve_on_times = 0;
		break;
	case PUMP_STATE_SUCTION:
		valve_on_times = 0;
		break;
	case PUMP_STATE_ON:
		valve_on_times = 0;
		break;

	case PUMP_STATE_VALVE_ON:
		valve_on_times++;

		if ((PINJ & HARD_MASK) > 0x00)
		{

			valve_on_times = 0;
			DDRG |= 0x10; // PG4 PUMP_D5_N
			DDRG |= 0x20; // PG5 PUMP_D5
			DDRF |= 0x08; //

			PORTG &= (~0x10);

			//				PORTG |= 0x20;
			PORTF &= (~0x08);

			pump_state = PUMP_STATE_OFF;
		}
		else
		{

			PORTG |= 0x20;
			delay_ms(50);
			if (valve_on_times >= VALVE_ON_TIMES_MAX)
			{
				valve_on_times = 0;
				pump_state = PUMP_STATE_OFF;
			}
			else
			{
				pump_state = PUMP_STATE_VALVE_OFF;
			}
		}
		break;

	case PUMP_STATE_VALVE_OFF:
		PORTG &= (~0x20);
		delay_ms(50);
		pump_state = PUMP_STATE_VALVE_ON;

		break;
	}
}

uint16_t get_pump_adc()
{
	uint16_t pump_adc = 0;
	ADCSRA = 0;
	ADMUX = 0x41;
	ADCSRB |= 0x08;
	ADCSRA = 1 << ADEN			  // enable ADC
			 | 1 << ADSC		  // start conversion
			 | 1 << ADIF		  // clear flag
			 | 7;				  // divide by 128
	while (!(ADCSRA & 1 << ADIF)) // wait until flag is set
		pump_adc = ADC;
	ADCSRA |= (1 << ADIF);
	return pump_adc;
}
uint8_t get_pump_status(void)
{
	if ((PORTG & 0x10) == 0x00)
	{
		return 0;
	}
	uint16_t value = analogRead(A9);
	uint16_t value2 = get_pump_adc();
	DB_PRINT_STR("value:");
	DB_PRINT_INT(value);
	DB_PRINT_STR("      value2:");
	DB_PRINT_INT(value2);
	if (value < 10)
	{
		return 0;
	}
	else if (value <= 70)
	{
		return 2;
	}
	else
	{
		return 1;
	}
}

/* button
 *
 */
void button_init()
{
	PORTE |= 0xC0; // 12 8 4  11
	// set button pin as input
	DDRE &= 0X3F;
}

uint8_t get_button_status(uint8_t button)
{
	if (button)
	{
		if (PINE & (1 << 6))
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		if (PINE & (1 << 7))
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
}

/*	gripper
 *
 */

void gripper_release(void)
{
	DDRC |= 0x80; // PC7 gripper
	PORTC |= 0x80;
}

void gripper_catch(void)
{
	DDRC |= 0x80; // PC7 gripper
	PORTC &= (~0x80);
}

uint8_t get_gripper_status(void)
{
#define MASK_G (1 << 0)
	if (PORTC & 0x80)
	{
		return 0;
	}

	DDRA &= ~(MASK_G);
	if (PINA & MASK_G)
	{
		return 1;
	}
	else
	{
		return 2;
	}
}

/*	gripper
 *
 */
void laser_on(void)
{
	DDRH |= (1 << 5); // <! OUTPUT
	DDRH |= (1 << 6); // <! OUTPUT

	PORTH |= (1 << 5);
	PORTH |= (1 << 6);
}

void laser_off(void)
{
	DDRH |= (1 << 5);
	DDRH |= (1 << 6);

	PORTH &= ~(1 << 5);
	PORTH &= ~(1 << 6);
}

uint8_t get_laser_status(void)
{
	if (PORTH & (1 << 5))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t get_limit_switch_status(void)
{
	if ((PING & (1 << 3)) == 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t get_power_status(void)
{

	uint16_t power_adc_value = analogRead(A5);

	if (power_adc_value > 100)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void check_motor_positon(void)
{
	float current_angle_l = 0, current_angle_r = 0, current_angle_b = 0;

	coord_to_angle(uarm.coord_x, uarm.coord_y, uarm.coord_z, &current_angle_l, &current_angle_r, &current_angle_b);
	float reg_angle_l = calculate_current_angle(CHANNEL_ARML);
	float reg_angle_r = calculate_current_angle(CHANNEL_ARMR);
	float reg_angle_b = calculate_current_angle(CHANNEL_BASE);

	if (fabs(current_angle_l - reg_angle_l) > 0.5 || fabs(current_angle_r - reg_angle_r) > 0.5 || fabs(current_angle_b - reg_angle_b + 90) > 0.5)
	{
		char l_str[20], r_str[20], b_str[20];

		uarm.arml_angle = reg_angle_l;
		uarm.armr_angle = reg_angle_r;
		uarm.base_angle = reg_angle_b;
		if (uarm.beep_state)
			beep_tone(260, 1000);

		float target[3] = {0};
		target[X_AXIS] = uarm.coord_x;
		target[Y_AXIS] = uarm.coord_y;
		target[Z_AXIS] = uarm.coord_z;
		coord_arm2effect(&target[X_AXIS], &target[Y_AXIS], &target[Z_AXIS]);
	}
}

void update_motor_position(void)
{
	uarm.base_angle = calculate_current_angle(CHANNEL_BASE);
	uarm.arml_angle = calculate_current_angle(CHANNEL_ARML);
	uarm.armr_angle = calculate_current_angle(CHANNEL_ARMR);
	//angle_to_coord(uarm.arml_angle, uarm.armr_angle, uarm.base_angle - 90,
	//			   &(uarm.coord_x), &(uarm.coord_y), &(uarm.coord_z));
}