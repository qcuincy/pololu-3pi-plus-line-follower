#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "kinematics.h"
#include "linefollowing.h"
#include "pid.h"
#include "constants.h"
#include "globals.h"

#pragma once

// --- Define the functions ---
// - Init functions -
void initObjects(), initTimers();

// - Update functions -
void updateLineDetection(), updateTurnDetection(), updateKinematics(), updateWheels();

// - Reset PID controllers
void resetPID();

// - Debug functions -
void beepOn(), beepOff();
const char* getStateName(State state);
void debugPrint(bool currentState, bool lineDetection, bool kinematics, bool pid);

// - Calculate and constrain functions -
float constrainPWM(float pwm);
bool checkThreshold(float value, float threshold);
void determineTarget(char direction);

// - Main functions -
void driveStraight(float targetAngle=0), followLine(), turn(char direction);

// - State functions -
void idle(), leaveBox(), followSegment(), turning(), stop(), turnHome(), goHome();
