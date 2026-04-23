#ifndef MOTORS_H
#define MOTORS_H

// --- PWM CONSTANTS ---
extern const int PWM_OFF; // Keeps it off.
extern const int PWM_MAX;

// --- THROTTLE ---
extern volatile int thro_des;

// --- FUNCTION DECLARATIONS ---
void initMotors();
void mixMotors();

#endif