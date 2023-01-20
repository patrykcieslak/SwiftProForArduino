#ifndef _UARM_DRIVE_H_
#define _UARM_DRIVE_H_

struct drive_state
{
    float position;
    float velocity;
    float desired_velocity;
};

volatile struct drive_state base_drive_state;
volatile struct drive_state arml_drive_state;
volatile struct drive_state armr_drive_state;
volatile struct drive_state ee_drive_state;

void drives_init(void);
void drives_arm(void);
void drives_disarm(void);
void drives_update(void);
void drives_set_velocity(float* vel);
void drives_stop(void);

#endif