#include <Arduino.h>
#include "stateactions.h"
#include "constants.h"
#include "globals.h"
#include "functions.h"

void executeState(State state) {
  switch (state) {
    case IDLE:
      executeIdle();
      break;
    case LEAVE_BOX:
      executeLeaveBox();
      break;
    case FOLLOW_SEGMENT:
      executeFollowSegment();
      break;
    case TURNING:
      executeTurning();
      break;
    case STOP:
      executeStop();
      break;
    case TURN_HOME:
      executeTurnHome();
      break;
    case GO_HOME:
      executeGoHome();
      break;
  }
}

void executeIdle() {
  // Actions to perform during the IDLE state
  Globals::motors.setMotorPower(0, 0);
}

void executeLeaveBox() {
  // Actions to perform during the LEAVE_BOX state
  updateKinematics();
  driveStraight();
  updateWheels();
}

void executeFollowSegment() {
  // Actions to perform during the FOLLOW_SEGMENT state
  updateKinematics();
  followLine();
  updateWheels();
}

void executeTurning() {
  // Actions to perform during the TURNING state
  updateKinematics();
  turn(Globals::activeTurn);
  updateWheels();
}

void executeStop() {
  Globals::motors.disableMotors();
}

void executeTurnHome() {
  // Actions to perform during the TURN_HOME state
  updateKinematics();
  turnHome();
  updateWheels();
}

void executeGoHome() {
  // Actions to perform during the GO_HOME state
  updateKinematics();
  goHome();
  updateWheels();
}
