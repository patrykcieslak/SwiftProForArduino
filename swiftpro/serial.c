/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
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

#include "uarm_common.h"

uint8_t serial_rx_buffer[RX_BUFFER_SIZE];
uint8_t serial_rx_buffer_head = 0;
volatile uint8_t serial_rx_buffer_tail = 0;

uint8_t serial_tx_buffer[TX_BUFFER_SIZE];
uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;
extern uint8_t syn_pack_remain;
enum serial_mode_e
{
  UART0 = 0,
  UART1,
  UART2,
  UART3,
} current_uart = UART0;
static bool check_flag = false;

#ifdef ENABLE_XONXOFF
volatile uint8_t flow_ctrl = XON_SENT; // Flow control state variable
#endif

// Returns the number of bytes used in the RX serial buffer.
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail)
  {
    return (serial_rx_buffer_head - rtail);
  }
  return (RX_BUFFER_SIZE - (rtail - serial_rx_buffer_head));
}

// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{
  uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_tx_buffer_head >= ttail)
  {
    return (serial_tx_buffer_head - ttail);
  }
  return (TX_BUFFER_SIZE - (ttail - serial_tx_buffer_head));
}

void serial_init()
{
/********************  uart0  ********************/
// Set baud rate
#if BAUD_RATE < 57600
  uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1) / 2;
  UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
#else
  uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1) / 2;
  UCSR0A |= (1 << U2X0); // baud doubler on for high baud rates, i.e. 115200
#endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;

  // enable rx and tx
  UCSR0B |= 1 << RXEN0;
  UCSR0B |= 1 << TXEN0;

  // enable interrupt on complete reception of a byte
  UCSR0B |= 1 << RXCIE0;
/********************  uart1  ********************/
// Set baud rate
#if BAUD_RATE < 57600
  uint16_t UBRR1_value = ((F_CPU / (8L * BAUD_RATE)) - 1) / 2;
  UCSR1A &= ~(1 << U2X1); // baud doubler off  - Only needed on Uno XXX
#else
  uint16_t UBRR1_value = ((F_CPU / (4L * BAUD_RATE)) - 1) / 2;
  UCSR1A |= (1 << U2X1); // baud doubler on for high baud rates, i.e. 115200
#endif
  UBRR1H = UBRR1_value >> 8;
  UBRR1L = UBRR1_value;

  // enable rx and tx
  UCSR1B |= 1 << RXEN1;
  UCSR1B |= 1 << TXEN1;

  // enable interrupt on complete reception of a byte
  UCSR1B |= 1 << RXCIE1;

/********************  uart2  ********************/
// Set baud rate
#if BAUD_RATE < 57600
  uint16_t UBRR2_value = ((F_CPU / (8L * BAUD_RATE)) - 1) / 2;
  UCSR2A &= ~(1 << U2X2); // baud doubler off  - Only needed on Uno XXX
#else
  uint16_t UBRR2_value = ((F_CPU / (4L * BAUD_RATE)) - 1) / 2;
  UCSR2A |= (1 << U2X2); // baud doubler on for high baud rates, i.e. 115200
#endif
  UBRR2H = UBRR2_value >> 8;
  UBRR2L = UBRR2_value;

  // enable rx and tx
  UCSR2B |= 1 << RXEN2;
  UCSR2B |= 1 << TXEN2;

  // enable interrupt on complete reception of a byte
  UCSR2B |= 1 << RXCIE2;

/********************  uart3  ********************/
// Set baud rate
#if BAUD_RATE < 57600
  uint16_t UBRR3_value = ((F_CPU / (8L * BAUD_RATE)) - 1) / 2;
  UCSR3A &= ~(1 << U2X3); // baud doubler off  - Only needed on Uno XXX
#else
  uint16_t UBRR3_value = ((F_CPU / (4L * BAUD_RATE)) - 1) / 2;
  UCSR3A |= (1 << U2X3); // baud doubler on for high baud rates, i.e. 115200
#endif
  UBRR3H = UBRR3_value >> 8;
  UBRR3L = UBRR3_value;

  // enable rx and tx
  UCSR3B |= 1 << RXEN3;
  UCSR3B |= 1 << TXEN3;

  // enable interrupt on complete reception of a byte
  UCSR3B |= 1 << RXCIE3;
  // defaults to 8-bit, no parity, 1 stop bit
}

// Writes one byte to the TX serial buffer. Called by main program.
// TODO: Check if we can speed this up for writing strings, rather than single bytes.
void serial_write(uint8_t data)
{
  // Calculate next head
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_BUFFER_SIZE)
  {
    next_head = 0;
  }

  // Wait until there is space in the buffer
  while (next_head == serial_tx_buffer_tail)
  {
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
    if (sys_rt_exec_state & EXEC_RESET)
    {
      return;
    } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  switch (current_uart)
  {
  case UART0:
    UCSR0B |= (1 << UDRIE0);
    break;
  case UART1:
    UCSR1B |= (1 << UDRIE1);
    break;
  case UART2:
    UCSR2B |= (1 << UDRIE2);
    break;
  case UART3:
    UCSR3B |= (1 << UDRIE3);
    break;
  }
}

// Data Register Empty Interrupt handler
ISR(USART0_UDRE_vect)
{
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)

#ifdef ENABLE_XONXOFF
  if (flow_ctrl == SEND_XOFF)
  {
    UDR0 = XOFF_CHAR;
    flow_ctrl = XOFF_SENT;
  }
  else if (flow_ctrl == SEND_XON)
  {
    UDR0 = XON_CHAR;
    flow_ctrl = XON_SENT;
  }
  else
#endif
  {
    // Send a byte from the buffer
    UDR0 = serial_tx_buffer[tail];

    // Update tail position
    tail++;
    if (tail == TX_BUFFER_SIZE)
    {
      tail = 0;
    }

    serial_tx_buffer_tail = tail;
  }

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial_tx_buffer_head)
  {
    UCSR0B &= ~(1 << UDRIE0);
  }
}

// Data Register Empty Interrupt handler
ISR(USART1_UDRE_vect)
{
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)

#ifdef ENABLE_XONXOFF
  if (flow_ctrl == SEND_XOFF)
  {
    UDR1 = XOFF_CHAR;
    flow_ctrl = XOFF_SENT;
  }
  else if (flow_ctrl == SEND_XON)
  {
    UDR1 = XON_CHAR;
    flow_ctrl = XON_SENT;
  }
  else
#endif
  {
    // Send a byte from the buffer
    UDR1 = serial_tx_buffer[tail];

    // Update tail position
    tail++;
    if (tail == TX_BUFFER_SIZE)
    {
      tail = 0;
    }

    serial_tx_buffer_tail = tail;
  }

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial_tx_buffer_head)
  {
    UCSR1B &= ~(1 << UDRIE1);
  }
}

ISR(USART2_UDRE_vect)
{
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)

#ifdef ENABLE_XONXOFF
  if (flow_ctrl == SEND_XOFF)
  {
    UDR2 = XOFF_CHAR;
    flow_ctrl = XOFF_SENT;
  }
  else if (flow_ctrl == SEND_XON)
  {
    UDR2 = XON_CHAR;
    flow_ctrl = XON_SENT;
  }
  else
#endif
  {
    // Send a byte from the buffer
    UDR2 = serial_tx_buffer[tail];

    // Update tail position
    tail++;
    if (tail == TX_BUFFER_SIZE)
    {
      tail = 0;
    }

    serial_tx_buffer_tail = tail;
  }

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial_tx_buffer_head)
  {
    UCSR2B &= ~(1 << UDRIE2);
  }
}

ISR(USART3_UDRE_vect)
{
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)

#ifdef ENABLE_XONXOFF
  if (flow_ctrl == SEND_XOFF)
  {
    UDR3 = XOFF_CHAR;
    flow_ctrl = XOFF_SENT;
  }
  else if (flow_ctrl == SEND_XON)
  {
    UDR3 = XON_CHAR;
    flow_ctrl = XON_SENT;
  }
  else
#endif
  {
    // Send a byte from the buffer
    UDR3 = serial_tx_buffer[tail];

    // Update tail position
    tail++;
    if (tail == TX_BUFFER_SIZE)
    {
      tail = 0;
    }

    serial_tx_buffer_tail = tail;
  }

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial_tx_buffer_head)
  {
    UCSR3B &= ~(1 << UDRIE3);
  }
}

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
  uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial_rx_buffer_head == tail)
  {
    return SERIAL_NO_DATA;
  }
  else
  {
    uint8_t data = serial_rx_buffer[tail];

    tail++;
    if (tail == RX_BUFFER_SIZE)
    {
      tail = 0;
    }
    serial_rx_buffer_tail = tail;

#ifdef ENABLE_XONXOFF
    if ((serial_get_rx_buffer_count() < RX_BUFFER_LOW) && flow_ctrl == XOFF_SENT)
    {
      flow_ctrl = SEND_XON;
      switch (current_uart)
      {
      case UART0:
        UCSR0B |= (1 << UDRIE0); // Force TX
        break;
      case UART1:
        UCSR1B |= (1 << UDRIE1); // Force TX
        break;
      case UART2:
        UCSR2B |= (1 << UDRIE2); // Force TX
        break;
      case UART3:
        UCSR3B |= (1 << UDRIE3); // Force TX
        break;
      }
    }
#endif

    return data;
  }
}

ISR(USART0_RX_vect)
{
  uint8_t data = UDR0;
  uint8_t next_head;
  current_uart = UART0;

  next_head = serial_rx_buffer_head + 1;
  if (next_head == RX_BUFFER_SIZE)
  {
    next_head = 0;
  }

  // Write data to buffer unless it is full.
  if (next_head != serial_rx_buffer_tail)
  {
    serial_rx_buffer[serial_rx_buffer_head] = data;
    serial_rx_buffer_head = next_head;

#ifdef ENABLE_XONXOFF
    if ((serial_get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT)
    {
      flow_ctrl = SEND_XOFF;
      UCSR0B |= (1 << UDRIE0); // Force TX
    }
#endif
  }
}

ISR(USART1_RX_vect)
{
  uint8_t data = UDR1;
  uint8_t next_head;
  current_uart = UART1;

  next_head = serial_rx_buffer_head + 1;
  if (next_head == RX_BUFFER_SIZE)
  {
    next_head = 0;
  }

  // Write data to buffer unless it is full.
  if (next_head != serial_rx_buffer_tail)
  {
    serial_rx_buffer[serial_rx_buffer_head] = data;
    serial_rx_buffer_head = next_head;

#ifdef ENABLE_XONXOFF
    if ((serial_get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT)
    {
      flow_ctrl = SEND_XOFF;
      UCSR1B |= (1 << UDRIE1); // Force TX
    }
#endif
  }
}

ISR(USART2_RX_vect)
{
  uint8_t data = UDR2;
  uint8_t next_head;
  current_uart = UART2;

  next_head = serial_rx_buffer_head + 1;
  if (next_head == RX_BUFFER_SIZE)
  {
    next_head = 0;
  }

  // Write data to buffer unless it is full.
  if (next_head != serial_rx_buffer_tail)
  {
    serial_rx_buffer[serial_rx_buffer_head] = data;
    serial_rx_buffer_head = next_head;

#ifdef ENABLE_XONXOFF
    if ((serial_get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT)
    {
      flow_ctrl = SEND_XOFF;
      UCSR2B |= (1 << UDRIE2); // Force TX
    }
#endif
  }
}

ISR(USART3_RX_vect)
{
  uint8_t data = UDR3;
  uint8_t next_head;
  current_uart = UART3;

  next_head = serial_rx_buffer_head + 1;
  if (next_head == RX_BUFFER_SIZE)
  {
    next_head = 0;
  }

  // Write data to buffer unless it is full.
  if (next_head != serial_rx_buffer_tail)
  {
    serial_rx_buffer[serial_rx_buffer_head] = data;
    serial_rx_buffer_head = next_head;

#ifdef ENABLE_XONXOFF
    if ((serial_get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT)
    {
      flow_ctrl = SEND_XOFF;
      UCSR3B |= (1 << UDRIE3); // Force TX
    }
#endif
  }
}

void serial_reset_read_buffer()
{
  serial_rx_buffer_tail = serial_rx_buffer_head;

#ifdef ENABLE_XONXOFF
  flow_ctrl = XON_SENT;
#endif
}
