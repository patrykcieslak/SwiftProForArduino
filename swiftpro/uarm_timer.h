#ifndef _UARM_TIMER_H_
#define _UARM_TIMER_H_

#include "uarm_common.h"

struct time_param_t
{
  void (*time_callback)();
  volatile uint16_t tcnt;
  unsigned short pwm_period;
  unsigned char clock_select_bits;
};

extern struct time_param_t time0;
extern struct time_param_t time1;
extern struct time_param_t time2;
extern struct time_param_t time3;
extern struct time_param_t time4;
extern struct time_param_t time5;

// 8-bit Timers
// Timer0 -> System update
// Timer2 -> Beep
void time0_set(float frequency, void (*callback)());
void time0_start(void);
void time0_stop(void);

void time2_set(float resolution, void (*callback)());
void time2_start(void);
void time2_stop(void);

// 16-bit Timers
// Timer1 -> BASE (CTC)
// Timer3 -> ARML (CTC)
// Timer5 -> ARMR (CTC)
// Timer4 -> Servo (PWM)

inline void time1_set(uint16_t frequency, void (*callback)())
{
  time1.time_callback = callback;
  TCCR1A = 0;
  TCCR1B = (1 << CS11) | (1 << WGM12); // CTC mode with 8x prescaler
  OCR1A = (uint16_t)roundf((float)F_CPU / ((float)frequency * 8.f)) - 1;
}

inline void time1_start(void)
{
  TCNT1 = 0;
  TIMSK1 |= (1 << OCIE1A);
}

inline void time1_stop(void)
{
  TIMSK1 &= ~(1 << OCIE1A);
}

inline void time3_set(uint16_t frequency, void (*callback)())
{
  time3.time_callback = callback;
  TCCR3A = 0;
  TCCR3B = (1 << CS31) | (1 << WGM32); // CTC mode with 8x prescaler
  OCR3A = (uint16_t)roundf((float)F_CPU / ((float)frequency * 8.f)) - 1;
}

inline void time3_start(void)
{
  TCNT3 = 0;
  TIMSK3 |= (1 << OCIE3A);
}

inline void time3_stop(void)
{
  TIMSK3 &= ~(1 << OCIE3A);
}

inline void time5_set(uint16_t frequency, void (*callback)())
{
  time5.time_callback = callback;
  TCCR5A = 0;
  TCCR5B = (1 << CS51) | (1 << WGM52); // CTC mode with 8x prescaler
  OCR5A = (uint16_t)roundf((float)F_CPU / ((float)frequency * 8.f)) - 1;
}

inline void time5_start(void)
{
  TCNT5 = 0;
  TIMSK5 |= (1 << OCIE5A);
}

inline void time5_stop(void)
{
  TIMSK5 &= ~(1 << OCIE5A);
}

void time4_set(float resolution, void (*callback)());
void time4_start(void);
void time4_stop(void);
void time4_pwm_init(unsigned long period_us);

extern volatile unsigned long time4DutyCycle;

inline void time4_set_duty(uint16_t duty)
{
  time4DutyCycle = time4.pwm_period;
  time4DutyCycle *= duty;
  time4DutyCycle >>= 10;
  OCR4A = time4DutyCycle;
  TCCR4B = _BV(WGM43) | time4.clock_select_bits;
}

void time4_set_period(unsigned long period_us);

#endif
