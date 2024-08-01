#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "kinematics.h"
#include "linesensor.h"
#include "wheel.h"
#include "linefollowing.h"
#include "pid.h"
#include "constants.h"

#pragma once

// In globals.h
class Globals {
public:
    // - Create class instances -
    static Wheel_c rightWheel;
    static Wheel_c leftWheel;
    static Kinematics_c kinematics;
    static Motors_c motors;
    static LineFollowing_c lineFollowing;
    static PID_c leftPID;
    static PID_c rightPID;
    static PID_c headPID;
    static PID_c leftTurnPID;
    static PID_c rightTurnPID;
    static PID_c headTurnPID;
    static PID_c leftSensorPID;
    static PID_c rightSensorPID;
    static PID_c headSensorPID;
    static PID_c leftHomePID;
    static PID_c rightHomePID;
    static PID_c headHomePID;
    static PID_c distancePID;


    // - Define the line transition counter variables -
    static bool linesDetected;
    static bool prevLinesDetected;
    static int linesTransitionCount;

    // - Define the turn transition counter variables -
    static float initialHeading;
    static float targetHeading;
    static char activeTurn;
    static char prevActiveTurn;
    static char turnsDetected;
    static char turnUpdate;
    static char prevTurnsDetected;
    static int turnsTransitionCount;
    static char previousTurns[TURN_HISTORY_SIZE];

    // - Define the update timers -
    static unsigned long lineDetectionTimer;
    static unsigned long lineTransitionTimer;
    static unsigned long turnTransitionTimer;
    static unsigned long turnCooldownTimer;
    static unsigned long kinematicsUpdateTimer;
    static unsigned long motorUpdateTimer;
    static unsigned long wheelsUpdateTimer;
    static unsigned long pidUpdateTimer;
    static unsigned long turnTimeoutTimer;

    // - Define the debug print timer -
    static unsigned long debugPrintTimer;

    // - Define the state timeout timer -
    static unsigned long stateTimoutTimer;

    // - Define the last air time variable -
    static unsigned long lastAirTime;

    // - Define the last moving time variable -
    static unsigned long lastMovingTime;

    // - Define the kinematics variables -
    static double currentX;
    static double currentY;
    static double currentHeading;
    static double previousX;
    static double previousY;
    static double previousHeading;
    static double distanceTravelled;
    static double headingChange;
    static double maxHeadingChange;
    static float headingChanges[HEADING_HISTORY_SIZE];
    static float turnPathLengths[PATH_LENGTH_HISTORY_SIZE];

    static float angleToOrigin;
    static float originMeasurement;

    static float turnStartPosition[2];
    static float turnEndPosition[2];

    // - Define errors to track -
    static double headingTurnError;

    // - Define the state variable -
    static State state;

    // - Define the last state variable -
    static State lastState;

    // - Define the line detection state variable -
    static DetectionState lineDetectionState;

    // - Define the turn detection state variable -
    static TurnState turnState;

    // - Define the line transition go home variable -
    static int linesTransitionCountHome;

    static double homeStartX;
    static double homeStartY;
    static double goHomeDistance;
    static double homeTurnError;
    static unsigned long boxLineTimer;
};
