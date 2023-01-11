/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "uarm_swift.h"

void system_init(void);

int main(void)
{
  // Initialize system upon power-up.
  serial_init();  // Setup serial baud rate and interrupts
  system_init();  // Configure pinout pins and pin-change interrupt
  uarm_swift_init();
  sei(); // Enable interrupts
  serial_reset_read_buffer(); // Clear serial read buffer

  // Disable all drives 
  drives_disarm();

  //Check encoders
  
  while (1)
  {
    //uarm_swift_tick_run();

    uint16_t angle_reg_value[3];
    get_angle_reg_value(angle_reg_value);
    uart_printf("B%d L%d R%d\n", angle_reg_value[2], angle_reg_value[0], angle_reg_value[1]);

    delay_ms(1000);
  }

  return 0; /* Never reached */
}

void system_init(void)
{
  CONTROL_DDR &= ~(CONTROL_MASK); // Configure as input pins
#ifdef DISABLE_CONTROL_PIN_PULL_UP
  CONTROL_PORT &= ~(CONTROL_MASK); // Normal low operation. Requires external pull-down.
#else
  CONTROL_PORT |= CONTROL_MASK; // Enable internal pull-up resistors. Normal high operation.
#endif
  CONTROL_PCMSK |= CONTROL_MASK; // Enable specific pins of the Pin Change Interrupt
  PCICR |= (1 << CONTROL_INT);   // Enable Pin Change Interrupt
}
