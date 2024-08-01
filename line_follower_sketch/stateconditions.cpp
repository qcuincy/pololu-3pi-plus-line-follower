#include <Arduino.h>
#include "stateconditions.h"
#include "constants.h"
#include "globals.h"
#include "functions.h"


State determineNextState() {
  // Determine the next state based on the current state and the conditions
  updateKinematics();
  // updateLineDetection();
  switch (Globals::state) {
    case IDLE:
      return determineNextStateFromIdle();
    case LEAVE_BOX:
      return determineNextStateFromLeaveBox();
    case FOLLOW_SEGMENT:
      return determineNextStateFromFollowSegment();
    case TURNING:
      return determineNextStateFromTurning();
    case STOP:
      return determineNextStateFromStop();
    case TURN_HOME:
      return determineNextStateFromTurnHome();
    case GO_HOME:
      return determineNextStateFromGoHome();
  }
  updateWheels();
}

State determineNextStateFromIdle() {
  unsigned long currentMillis = millis();
  bool inAir = Globals::lineFollowing.inAir();

  // If the robot is in the air, reset the timer
  if (inAir) {
    Globals::lastAirTime = currentMillis;
  }

  // If the robot is on the floor for a certain amount of time, leave the box
  if (currentMillis - Globals::lastAirTime > AIR_TIME_INTERVAL) {
    return LEAVE_BOX;
  } else {
    return IDLE;
  }
}

State determineNextStateFromLeaveBox() {
  unsigned long currentMillis = millis();

  if (Globals::linesDetected && Globals::linesTransitionCount > 1) {
    return FOLLOW_SEGMENT;
  } 
  if (Globals::linesDetected && Globals::linesTransitionCount >= 1) {
    if (Globals::turnsDetected == 'S') {
      return FOLLOW_SEGMENT;
    } else if (Globals::turnsDetected == 'L') {
      return TURNING;
    } else {
      return LEAVE_BOX;
    }
  } else {
    return LEAVE_BOX;
  }
}

State determineNextStateFromFollowSegment() {
  unsigned long currentMillis = millis();
  MotorPower motorPower = Globals::motors.getMotorPower();
  if (Globals::distanceTravelled > 26 && !Globals::linesDetected) {
    Globals::motors.disableMotors();
    return TURN_HOME;
  }
  if (motorPower.left != 0 || motorPower.right != 0) {
    Globals::lastMovingTime = currentMillis;
  }
  if ((currentMillis - Globals::lastMovingTime > STOP_TIMEOUT) && (motorPower.left == 0 && motorPower.right == 0)) {
    return STOP;
  }

  if (Globals::lineFollowing.sensorDetected(1) && Globals::lineFollowing.sensorDetected(2) && Globals::lineFollowing.sensorDetected(3) && Globals::turnsDetected != 'L') {
    return FOLLOW_SEGMENT;
  }
  else if (Globals::turnsDetected != 'S' && currentMillis - Globals::turnCooldownTimer > TURN_COOLDOWN_INTERVAL && (Globals::maxHeadingChange < BIG_HEADING_DIFFERENCE_THRESHOLD)) {
    Globals::turnCooldownTimer = currentMillis;
    return TURNING;
  } else {
    return FOLLOW_SEGMENT;
  }
}

State determineNextStateFromTurning() {
  unsigned long currentMillis = millis();
  if (Globals::activeTurn == 'N' && Globals::distanceTravelled > 26) {
    Globals::motors.disableMotors();
    return TURN_HOME;
  }
  else {
    if (currentMillis - Globals::turnTimeoutTimer > TURN_TIMEOUT) {
      Globals::turnTimeoutTimer = currentMillis;
      if (Globals::activeTurn == 'R' && Globals::turnsDetected == 'S' && Globals::lineFollowing.sensorDetected(2) && abs(Globals::headingTurnError) > 0.4){
        return FOLLOW_SEGMENT;
      }
      else if (Globals::activeTurn == 'R' && Globals::turnsDetected == 'L' && abs(Globals::headingTurnError) > 0.4 ){
        return FOLLOW_SEGMENT;
      }

      else if (Globals::activeTurn != 'N' && abs(Globals::headingTurnError) < 0.4 && (Globals::lineFollowing.sensorDetected(2))) {

        return FOLLOW_SEGMENT;
      }

      else if (Globals::activeTurn == 'N' && (Globals::turnsDetected == 'S') && Globals::lineFollowing.sensorDetected(2) && abs(Globals::headingTurnError) < 0.1){
        return FOLLOW_SEGMENT;
      }

      else if ((abs(Globals::headingTurnError) < 0.05 ) && Globals::activeTurn != 'S') {
        return FOLLOW_SEGMENT;
      } else {
        return TURNING;
      }
    }
  }
}

State determineNextStateFromTurnHome() {
  // Until turning error is within an acceptable range, keep turning
  if (abs(Globals::homeTurnError) < 0.0005) {
    return GO_HOME;
  } else {
    return TURN_HOME;
  }
}

State determineNextStateFromGoHome() {
  // when an acceptable distance to origin
  if (Globals::distanceTravelled < 0.35) {
    return STOP;
  } else {
    return GO_HOME;
  }
}

State determineNextStateFromStop() {
  return STOP;
}
