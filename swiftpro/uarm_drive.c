#include "uarm_common.h"

#define RAD2DEG(x)   ((x) * 57.29577f)
#define DEG2RAD(x)   ((x) * 0.0174533f)
#define PI2f             1.570796f

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

  drives_disarm();

  // Update state variables
  update_motor_position();
  base_drive_state.position = DEG2RAD(uarm.base_angle-90.f);
  base_drive_state.desired_velocity = 0.f;
  base_drive_state.limit = 0;
  
  arml_drive_state.position = DEG2RAD(90.f-uarm.arml_angle);
  arml_drive_state.desired_velocity = 0.f;
  arml_drive_state.limit = 0;
  
  armr_drive_state.position = DEG2RAD(uarm.armr_angle);
  armr_drive_state.desired_velocity = 0.f;
  armr_drive_state.limit = 0;
  
  ee_drive_state.position = 0.f;
  ee_drive_state.desired_velocity = 0.f;
  ee_drive_state.limit = 0;
}

void drives_arm(void)
{
  ARML_STEPPERS_DISABLE_PORT |= ARML_STEPPERS_DISABLE_MASK;
  ARMR_STEPPERS_DISABLE_PORT |= ARMR_STEPPERS_DISABLE_MASK;
  BASE_STEPPERS_DISABLE_PORT |= BASE_STEPPERS_DISABLE_MASK;
	end_effector_init();
}

void drives_disarm(void)
{
  ARML_STEPPERS_DISABLE_PORT &= (~ARML_STEPPERS_DISABLE_MASK);
  ARMR_STEPPERS_DISABLE_PORT &= (~ARMR_STEPPERS_DISABLE_MASK);
  BASE_STEPPERS_DISABLE_PORT &= (~BASE_STEPPERS_DISABLE_MASK);
	end_effector_deinit();
}

void drives_update(void)
{
  //Update positions
  update_motor_position();
  base_drive_state.position = DEG2RAD(uarm.base_angle-90.f);
  arml_drive_state.position = DEG2RAD(90.f-uarm.arml_angle);
  armr_drive_state.position = DEG2RAD(uarm.armr_angle);
  ee_drive_state.position = DEG2RAD(end_effector_get_angle()-90.f);

  // Apply safety action on joint limits (stop)
  if(base_drive_state.position <= -PI2f)
  {
    if(base_drive_state.desired_velocity < 0.f)
    {
      time1_stop();
      base_drive_state.desired_velocity = 0.f;
    }
    base_drive_state.limit = -1;
  }
  else if(base_drive_state.position >= PI2f)
  {
    if(base_drive_state.desired_velocity > 0.f)
    {
      time1_stop();
      base_drive_state.desired_velocity = 0.f;
    }
    base_drive_state.limit = 1;
  }
  else 
    base_drive_state.limit = 0;

  if(arml_drive_state.position <= 0.03f)
  {
    if(arml_drive_state.desired_velocity < 0.f)
    {
      time3_stop();
      arml_drive_state.desired_velocity = 0.f;
    }
    arml_drive_state.limit = -1;
  }
  else if(arml_drive_state.position >= PI2f)
  {
    if(arml_drive_state.desired_velocity > 0.f)
    {
      time3_stop();
      arml_drive_state.desired_velocity = 0.f;
    }
    arml_drive_state.limit = 1;
  }
  else 
    arml_drive_state.limit = 0;

  if(armr_drive_state.position <= -0.35f)
  {
    if(armr_drive_state.desired_velocity < 0.f)
    {
      time5_stop();
      armr_drive_state.desired_velocity = 0.f;
    }
    armr_drive_state.limit = -1;
  }
  else if(armr_drive_state.position >= PI2f)
  {
    if(armr_drive_state.desired_velocity > 0.f)
    {
      time5_stop();
      armr_drive_state.desired_velocity = 0.f;
    }
    armr_drive_state.limit = 1;
  }
  else 
    armr_drive_state.limit = 0;

  if(ee_drive_state.position <= -PI2f)
  {
    if(ee_drive_state.desired_velocity < 0.f)
    {
      ee_drive_state.desired_velocity = 0.f;
    }
    ee_drive_state.limit = -1;
  }
  else if(ee_drive_state.position >= PI2f)
  {
    if(ee_drive_state.desired_velocity > 0.f)
    {
      ee_drive_state.desired_velocity = 0.f;
    }
    ee_drive_state.limit = 1;
  }
  else
    ee_drive_state.limit = 0;

  // Update end-effector
  const float dt = 0.1f;
  ee_drive_state.position += ee_drive_state.desired_velocity * dt;
  end_effector_set_angle(RAD2DEG(ee_drive_state.position)+90.f);
}

static void drives_BASE_tick(void)
{
 	BASE_STEP_PORT ^= BASE_STEP_MASK;
}

static void drives_ARML_tick(void)
{
  ARML_STEP_PORT ^= ARML_STEP_MASK;
}

static void drives_ARMR_tick(void)
{
	ARMR_STEP_PORT ^= ARMR_STEP_MASK;
}

void drives_set_velocity(float* vel)
{
  // Base drive
  if( (base_drive_state.limit == -1 && vel[0] < 0.f)
     || (base_drive_state.limit == 1 && vel[0] > 0.f) )
    vel[0] = 0.f;

  if(fabsf(vel[0]) > 0.00348f) // More than 0.2 deg
  {
    // Limit top velocity (90 deg)
    base_drive_state.desired_velocity = vel[0] > PI2f ? PI2f : (vel[0] < -PI2f ? -PI2f : vel[0]); 
    
    // Set rotation direction
    if(base_drive_state.desired_velocity < 0.f)
      BASE_DIRECTION_PORT |= BASE_DIRECTION_MASK;
    else
      BASE_DIRECTION_PORT &= ~(BASE_DIRECTION_MASK);

    // Set up timer
    uint16_t steps_per_second = roundf(RAD2DEG(fabsf(base_drive_state.desired_velocity)) * steps_per_angle * 2.f);
    time1_set(steps_per_second, drives_BASE_tick);
    time1_start();
  }
  else
  {
    time1_stop();
    base_drive_state.desired_velocity = 0.f;
  }

  // Arm left drive
  if( (arml_drive_state.limit == -1 && vel[1] < 0.f)
     || (arml_drive_state.limit == 1 && vel[1] > 0.f) )
    vel[1] = 0.f;

  if(fabsf(vel[1]) > 0.00348f) // More than 0.2 deg
  {
    // Limit top velocity (90 deg)
    arml_drive_state.desired_velocity = vel[1] > PI2f ? PI2f : (vel[1] < -PI2f ? -PI2f : vel[1]); 

    // Set rotation direction
    if(arml_drive_state.desired_velocity > 0.f)
      ARML_DIRECTION_PORT |= ARML_DIRECTION_MASK;
    else
      ARML_DIRECTION_PORT &= ~(ARML_DIRECTION_MASK);

    // Set up timer
    uint16_t steps_per_second = roundf(RAD2DEG(fabsf(arml_drive_state.desired_velocity)) * steps_per_angle * 2.f);
    time3_set(steps_per_second, drives_ARML_tick);
    time3_start();
  }
  else
  {
    time3_stop();
    arml_drive_state.desired_velocity = 0.f;
  }

  // Arm right drive
  if( (armr_drive_state.limit == -1 && vel[2] < 0.f)
     || (armr_drive_state.limit == 1 && vel[2] > 0.f) )
    vel[2] = 0.f;

  if(fabsf(vel[2]) > 0.00348f) // More than 0.2 deg
  {
    // Limit top velocity (90 deg)
    armr_drive_state.desired_velocity = vel[2] > PI2f ? PI2f : (vel[2] < -PI2f ? -PI2f : vel[2]); 

    // Set rotation direction
    if(armr_drive_state.desired_velocity < 0.f)
      ARMR_DIRECTION_PORT |= ARMR_DIRECTION_MASK;
    else
      ARMR_DIRECTION_PORT &= ~(ARMR_DIRECTION_MASK);

    // Set up timer
    uint16_t steps_per_second = roundf(RAD2DEG(fabsf(armr_drive_state.desired_velocity)) * steps_per_angle * 2.f);
    time5_set(steps_per_second, drives_ARMR_tick);
    time5_start();
  }
  else
  {
    time5_stop();
    armr_drive_state.desired_velocity = 0.f;
  }

  // End-effector drive
  if( (ee_drive_state.limit == -1 && vel[3] < 0.f)
     || (ee_drive_state.limit == 1 && vel[3] > 0.f) )
    vel[3] = 0.f;

  if(fabsf(vel[3]) > 0.00348f) // More than 0.2 deg
  {
    // Limit top velocity (90 deg)
    ee_drive_state.desired_velocity = vel[3] > PI2f ? PI2f : (vel[3] < -PI2f ? -PI2f : vel[3]); 
  }
  else
  {
    ee_drive_state.desired_velocity = 0.f;
  }
}

void drives_stop(void)
{
  time1_stop();
  time3_stop();
  time5_stop();

  base_drive_state.desired_velocity = 0.f;
  arml_drive_state.desired_velocity = 0.f;
  armr_drive_state.desired_velocity = 0.f;
  ee_drive_state.desired_velocity = 0.f;
}
