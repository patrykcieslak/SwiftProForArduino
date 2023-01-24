#include "uarm_timer.h"

struct time_param_t time0 = {0};
struct time_param_t time1 = {0};
struct time_param_t time2 = {0};
struct time_param_t time3 = {0};
struct time_param_t time4 = {0};
struct time_param_t time5 = {0};
volatile unsigned long time4DutyCycle;

void time0_set(float frequency, void (*callback)())
{
  time0.time_callback = callback;
  TCCR0A = (1 << WGM01); // CTC mode
  TCCR0B = (1 << CS02) | (1 << CS00); // 1024x prescaler
  OCR0A = (uint8_t)roundf((float)F_CPU / (frequency * 1024.f)) - 1;
}

void time0_start(void)
{
  TCNT0 = 0;
  TIMSK0 |= (1 << OCIE0A);
}

void time0_stop(void)
{
  TIMSK0 &= ~(1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
  if (time0.time_callback != NULL)
    time0.time_callback();
}

void time2_set(float resolution, void (*callback)())
{
  float prescaler = 0.0;
  time2.time_callback = callback;
  TIMSK2 &= ~(1 << TOIE2);
  TCCR2A &= ~((1 << WGM21) | (1 << WGM20));
  TCCR2B &= ~(1 << WGM22);
  ASSR &= ~(1 << AS2);
  TIMSK2 &= ~(1 << OCIE2A);

  //  if ((F_CPU >= 1000000UL) && (F_CPU < 16000000UL)) {  // prescaler set to 64
  //    TCCR2B |= ((1<<CS21)| (1<<CS20));
  //        TCCR2B &= ~(1<<CS22);
  //    prescaler = 64.0;
  //  } else if (F_CPU < 1000000UL) { // prescaler set to 8
  //    TCCR2B |= (1<<CS21);
  //    TCCR2B &= ~((1<<CS22) | (1<<CS20));
  //    prescaler = 8.0;
  //  } else { // F_CPU > 16Mhz, prescaler set to 128
  //    TCCR2B |= ((1<<CS22)| (1<<CS20)|(1<<CS21)) ;
  ////    TCCR2B &= ~(1<<CS21) ;
  //    prescaler = 512.0;
  //  }
  if (resolution < 500)
  {
    TCCR2B |= ((1 << CS22) | (1 << CS20) | (1 << CS21));
    ////    TCCR2B &= ~(1<<CS21) ;
    prescaler = 2048.0;
  }
  else
  {
    TCCR2B |= ((1 << CS22) | (1 << CS20));
    TCCR2B &= ~(1 << CS21);
    prescaler = 256.0;
  }
  time2.tcnt = 0xff + 1 - (int)((float)F_CPU / resolution / prescaler);
}

void time2_start(void)
{
  TCNT2 = time2.tcnt;
  TIMSK2 |= (1 << TOIE2);
}

void time2_stop(void)
{
  TIMSK2 &= ~(1 << TOIE2);
  TCCR2A = 0;
  TCCR2B = 0;
}

ISR(TIMER2_OVF_vect)
{
  TCNT2 = time2.tcnt;
  if (time2.time_callback != NULL)
  {
    time2.time_callback();
  }
}

void time4_set(float resolution, void (*callback)())
{
  time4.time_callback = callback;
  TCCR4A = 0;
  TCCR4B = (1 << CS42) | (1 << CS40);
  time4.tcnt = 0xffff + 1 - (int)((float)F_CPU * resolution / 1024);
}

void time4_start(void)
{
  TCNT4 = time4.tcnt;
  TIMSK4 |= (1 << TOIE4);
}

void time4_stop(void)
{
  TIMSK4 &= ~(1 << TOIE4);
  TCCR4A = 0;
  TCCR4B = 0;
}

ISR(TIMER4_OVF_vect)
{
  TCNT4 = time4.tcnt;
  if (time4.time_callback != NULL)
  {
    time4.time_callback();
  }
}
#define TIMER4_RESOLUTION 65536UL

void time4_set_period(unsigned long period_us)
{
  /********************** set period **************************/
  const unsigned long cycles = (F_CPU / 2000000) * period_us;
  if (cycles < TIMER4_RESOLUTION)
  {
    time4.clock_select_bits = _BV(CS40);
    time4.pwm_period = cycles;
  }
  else if (cycles < TIMER4_RESOLUTION * 8)
  {
    time4.clock_select_bits = _BV(CS41);
    time4.pwm_period = cycles / 8;
  }
  else if (cycles < TIMER4_RESOLUTION * 64)
  {
    time4.clock_select_bits = _BV(CS41) | _BV(CS40);
    time4.pwm_period = cycles / 64;
  }
  else if (cycles < TIMER4_RESOLUTION * 256)
  {
    time4.clock_select_bits = _BV(CS42);
    time4.pwm_period = cycles / 256;
  }
  else if (cycles < TIMER4_RESOLUTION * 1024)
  {
    time4.clock_select_bits = _BV(CS42) | _BV(CS40);
    time4.pwm_period = cycles / 1024;
  }
  else
  {
    time4.clock_select_bits = _BV(CS42) | _BV(CS40);
    time4.pwm_period = TIMER4_RESOLUTION - 1;
  }
  ICR4 = time4.pwm_period;
  TCCR4B = _BV(WGM43) | time4.clock_select_bits;
}

void time4_pwm_init(unsigned long period_us)
{
  TCCR4B = _BV(WGM43); // set mode as phase and frequency correct pwm, stop the timer
  TCCR4A = 0;          // clear control register A
  TCNT4 = 0;

  // Pin 3 output
  DDRH |= 1 << 3;
  TCCR4A |= _BV(COM4A1);

  time4_set_period(period_us);
}

ISR(TIMER1_COMPA_vect)
{
  if (time1.time_callback != NULL)
    time1.time_callback();
}

ISR(TIMER3_COMPA_vect)
{
  if (time3.time_callback != NULL)
    time3.time_callback();
}

ISR(TIMER5_COMPA_vect)
{
  if (time5.time_callback != NULL)
    time5.time_callback();
}