#include "uarm_swift.h"

struct uarm_state_t uarm = {0};
char hardware_version[8] = {0};
char bt_mac_addr[13] = {0};
enum uarm_device_e uarm_device = UARM_PRO;

void uarm_swift_init(void)
{
	// Configure control pins
	CONTROL_DDR &= ~(CONTROL_MASK); // Configure as input pins
#ifdef DISABLE_CONTROL_PIN_PULL_UP
	CONTROL_PORT &= ~(CONTROL_MASK); // Normal low operation. Requires external pull-down.
#else
	CONTROL_PORT |= CONTROL_MASK; // Enable internal pull-up resistors. Normal high operation.
#endif
	CONTROL_PCMSK |= CONTROL_MASK; // Enable specific pins of the Pin Change Interrupt
	PCICR |= (1 << CONTROL_INT);   // Enable Pin Change Interrupt
	delay_ms(10);

	// Initialize peripherals
	read_hardware_version();
	angle_sensor_init();
	button_init();

	//beep_tone(260, 1000);
	
	read_sys_param();
	drives_init();
	pump_off();
	gripper_release();
	laser_off();
	
	ADCSRA |= (1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0 | 1 << ADEN); // <!adc set
	DDRG &= ~(1 << 3);
	PORTG |= (1 << 3);

	// uart_printf("device name : %s\r\n", DEVICE_NAME);
	// uart_printf("soft ver : %s\r\n", SOFTWARE_VERSION);
	// uart_printf("api ver : %s\r\n", API_VERSION);
	// uart_printf("work mode : %d\r\n", uarm.param.work_mode);
	// printString("@1\n");
}

void uarm_swift_update(void)
{
	drives_update();
}