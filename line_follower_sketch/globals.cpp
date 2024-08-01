#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "kinematics.h"
#include "linesensor.h"
#include "wheel.h"
#include "linefollowing.h"
#include "pid.h"
#include "constants.h"
#include "globals.h"

// --- Global variables ---
// - Create class instances -
Wheel_c Globals::rightWheel(RIGHT_ENCODER_PIN);
Wheel_c Globals::leftWheel(LEFT_ENCODER_PIN);
Kinematics_c Globals::kinematics;
Motors_c Globals::motors;
LineFollowing_c Globals::lineFollowing;
PID_c Globals::leftPID(KP, KI, KD);
PID_c Globals::rightPID(KP, KI, KD);
PID_c Globals::headPID(KP, KI, KD);
PID_c Globals::leftTurnPID(turnKP, turnKI, turnKD);
PID_c Globals::rightTurnPID(turnKP, turnKI, turnKD);
PID_c Globals::headTurnPID(turnKP, turnKI, turnKD);
PID_c Globals::leftSensorPID(sensorKP, sensorKI, sensorKD);
PID_c Globals::rightSensorPID(sensorKP, sensorKI, sensorKD);
PID_c Globals::headSensorPID(sensorKP, sensorKI, sensorKD);
PID_c Globals::distancePID(sensorKP, sensorKI, sensorKD);
PID_c Globals::leftHomePID(homeKP, homeKI, homeKD);
PID_c Globals::rightHomePID(homeKP, homeKI, homeKD);
PID_c Globals::headHomePID(homeKP, homeKI, homeKD);

// - Define the line transition counter variables -
bool Globals::linesDetected = false;
bool Globals::prevLinesDetected = false;
int Globals::linesTransitionCount = 0;

// - Define the turn transition counter variables -
float Globals::initialHeading = 0;
float Globals::targetHeading = 0;
char Globals::activeTurn = 'S'; // 'N'
char Globals::prevActiveTurn = 'S'; // 'N'
char Globals::turnsDetected = 'S'; // 'N'
char Globals::turnUpdate = 'S'; // 'N'
char Globals::prevTurnsDetected = 'S'; // 'N'
int Globals::turnsTransitionCount = 0;
char Globals::previousTurns[TURN_HISTORY_SIZE] = {'S', 'S', 'S', 'S', 'S'}; // {'N', 'N', 'N', 'N', 'N'}

// - Define the update timers -
unsigned long Globals::lineDetectionTimer = 0;
unsigned long Globals::lineTransitionTimer = 0;
unsigned long Globals::turnTransitionTimer = 0;
unsigned long Globals::turnCooldownTimer = 0;
unsigned long Globals::kinematicsUpdateTimer = 0;
unsigned long Globals::motorUpdateTimer = 0;
unsigned long Globals::wheelsUpdateTimer = 0;
unsigned long Globals::pidUpdateTimer = 0;
unsigned long Globals::turnTimeoutTimer = 0;

// - Define the debug print timer -
unsigned long Globals::debugPrintTimer = 0;

// - Define the state timeout timer -
unsigned long Globals::stateTimoutTimer = 0;

// - Define the last air time variable -
unsigned long Globals::lastAirTime = 0;

// - Define the last moving time variable -
unsigned long Globals::lastMovingTime = 0;

// - Define errors to track -
double Globals::headingTurnError = 0;

// - Define the kinematics variables -
double Globals::currentX = 0;
double Globals::currentY = 0;
double Globals::currentHeading = 0;
double Globals::previousX = 0;
double Globals::previousY = 0;
double Globals::previousHeading = 0;
double Globals::distanceTravelled = 0;
double Globals::headingChange = 0;
double Globals::maxHeadingChange = 0;
float Globals::headingChanges[HEADING_HISTORY_SIZE];
float Globals::turnPathLengths[PATH_LENGTH_HISTORY_SIZE] = {0, 0, 0, 0, 0};

float Globals::angleToOrigin = 0;
float Globals::originMeasurement = 0;

float Globals::turnStartPosition[2] = {0, 0};
float Globals::turnEndPosition[2] = {0, 0};

int Globals::linesTransitionCountHome = 0;

double Globals::homeStartX = 0;
double Globals::homeStartY = 0;
double Globals::goHomeDistance = 0;
double Globals::homeTurnError = 1;
unsigned long Globals::boxLineTimer = 0;
