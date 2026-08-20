#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// =====================================================
// Motor State
// =====================================================
enum MotorState {
  MOTOR_IDLE,
  MOTOR_FORWARD,
  MOTOR_REVERSE
};

extern MotorState currentState;
extern MotorState requestedState;

// Limit switch state
extern bool limitTopActive;
extern bool limitBottomActive;

// Motor timing
extern unsigned long motorStartTime;
extern bool motorTimingActive;
extern bool timeoutTriggered;

void init_Pins();
void setMotorState(MotorState state);
void updateMotor();
void checkLimitSwitches();
void emergencyStop(const char* reason);

#endif // MOTOR_H
