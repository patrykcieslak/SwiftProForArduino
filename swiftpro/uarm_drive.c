#include "uarm_common.h"

// Initialize and start the stepper motor subsystem
void drives_init(void)
{
  // Configure step and direction interface pins
  //  STEP_DDR |= STEP_MASK;
  //  STEPPERS_DISABLE_DDR |= 1<<STEPPERS_DISABLE_BIT;
  //  DIRECTION_DDR |= DIRECTION_MASK;
  // <!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  ARML_STEP_DDR |= ARML_STEP_MASK;
  ARML_DIRECTION_DDR |= ARML_DIRECTION_MASK;
  ARML_STEPPERS_DISABLE_DDR |= ARML_STEPPERS_DISABLE_MASK;
  ARMR_STEP_DDR |= ARMR_STEP_MASK;
  ARMR_DIRECTION_DDR |= ARMR_DIRECTION_MASK;
  ARMR_STEPPERS_DISABLE_DDR |= ARMR_STEPPERS_DISABLE_MASK;
  BASE_STEP_DDR |= BASE_STEP_MASK;
  BASE_DIRECTION_DDR |= BASE_DIRECTION_MASK;
  BASE_STEPPERS_DISABLE_DDR |= BASE_STEPPERS_DISABLE_MASK;

  MS3_DDRD |= MS3_MASK;
  MS2_DDRD |= MS2_MASK;
  MS1_DDRD |= MS1_MASK;

  // MS3=1 MS2=0 MS1=1	1/32 step
  MS3_PORT |= MS3_MASK;
  MS2_PORT &= ~(MS2_MASK);
  MS1_PORT |= MS1_MASK;

  ARML_STEPPERS_DISABLE_PORT |= ARML_STEPPERS_DISABLE_MASK;
  ARMR_STEPPERS_DISABLE_PORT |= ARMR_STEPPERS_DISABLE_MASK;
  BASE_STEPPERS_DISABLE_PORT |= BASE_STEPPERS_DISABLE_MASK;

  // Configure Timer 1: Stepper Driver Interrupt
  TCCR1B &= ~(1 << WGM13); // waveform generation = 0100 = CTC
  TCCR1B |= (1 << WGM12);
  TCCR1A &= ~((1 << WGM11) | (1 << WGM10));
  TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0)); // Disconnect OC1 output
  // TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // Set in st_go_idle().
  // TIMSK1 &= ~(1<<OCIE1A);  // Set in st_go_idle().

  // Configure Timer 0: Stepper Port Reset Interrupt
  TIMSK0 &= ~((1 << OCIE0B) | (1 << OCIE0A) | (1 << TOIE0)); // Disconnect OC0 outputs and OVF interrupt.
  TCCR0A = 0;                                                // Normal operation
  TCCR0B = 0;                                                // Disable Timer0 until needed
  TIMSK0 |= (1 << TOIE0);                                    // Enable Timer0 overflow interrupt
#ifdef STEP_PULSE_DELAY
  TIMSK0 |= (1 << OCIE0A); // Enable Timer0 Compare Match A interrupt
#endif

  end_effector_init();
}

void drives_arm(void)
{
  ARML_STEPPERS_DISABLE_PORT |= ARML_STEPPERS_DISABLE_MASK;
  ARMR_STEPPERS_DISABLE_PORT |= ARMR_STEPPERS_DISABLE_MASK;
  BASE_STEPPERS_DISABLE_PORT |= BASE_STEPPERS_DISABLE_MASK;
	end_effector_init();
	uarm.motor_state_bits = 0x00;
}

void drives_disarm(void)
{
  ARML_STEPPERS_DISABLE_PORT &= (~ARML_STEPPERS_DISABLE_MASK);
  ARMR_STEPPERS_DISABLE_PORT &= (~ARMR_STEPPERS_DISABLE_MASK);
  BASE_STEPPERS_DISABLE_PORT &= (~BASE_STEPPERS_DISABLE_MASK);
	end_effector_deinit();
	uarm.motor_state_bits = 0x00;
}