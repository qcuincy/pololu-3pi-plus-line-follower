#include <Arduino.h>
#include "statetransitions.h"
#include "constants.h"
#include "globals.h"
#include "functions.h"

// ---| State entry functions |---
void enterState(State state) {
  updateKinematics();
  beepOff();
  switch (state) {
    case IDLE:
        enterIdle();
        break;
    case LEAVE_BOX:
        enterLeaveBox();
        break;
    case FOLLOW_SEGMENT:
        enterFollowSegment();
        break;
    case TURNING:
        enterTurning();
        break;
    case STOP:
        enterStop();
        break;
    case TURN_HOME:
        enterTurnHome();
        break;
    case GO_HOME:
        enterGoHome();
        break;
  }
  updateWheels();
}

void enterIdle() {
  // Reset the PID controllers
  resetPID();
}

void enterLeaveBox() {
  // Reset the PID controllers
  resetPID();
}

void enterFollowSegment() {
  // Reset the PID controllers
  resetPID();
}


void enterTurning() {
  // Set the active turn to the detected turn
  Globals::activeTurn = Globals::turnsDetected;
  // Set the initial heading to the current heading
  Globals::initialHeading = Globals::currentHeading;

  unsigned long startTime = millis();
  bool isMoving = true;

  while (isMoving) {
    updateKinematics();
    updateLineDetection();
    if (Globals::activeTurn == 'N') {
      if (millis() - startTime < 600) {
        // Give nudge forwards if detected turn is 'N'
        updateKinematics();
        Globals::motors.setMotorPower(15, 15);
        updateWheels();
      } else {
        updateKinematics();
        Globals::motors.disableMotors();
        updateWheels();
        isMoving = false;
      }
    } else if (Globals::activeTurn != 'N' && Globals::activeTurn != 'S') {
      if (millis() - startTime < 400) {
        // Give smaller nudge forwards if detected turn is 'L' or 'R'
        updateKinematics();
        Globals::motors.setMotorPower(15, 15);
        updateWheels();
      } else {
        updateKinematics();
        Globals::motors.disableMotors();
        updateWheels();
        isMoving = false;
      }
    }
    // Update the wheels
    updateWheels();    
  }

  Globals::turnEndPosition[0] = Globals::currentX;
  Globals::turnEndPosition[1] = Globals::currentY;

  // Calculate the distance travelled during the turn
  float dx = Globals::turnEndPosition[0] - Globals::turnStartPosition[0];
  float dy = Globals::turnEndPosition[1] - Globals::turnStartPosition[1];
  float distance = sqrt(dx * dx + dy * dy);
  // Update the path length history
  Globals::turnPathLengths[Globals::turnsTransitionCount % PATH_LENGTH_HISTORY_SIZE] = distance;

  // Reset the PID controllers
  resetPID();

  // Set the headingTurnError to 1 to avoid the condition robot assuming its already turned
  Globals::headingTurnError = 1;
}

void enterStop() {
  // Set the motors to 0
  Globals::motors.setMotorPower(0, 0);
  // Reset the PID controllers
  resetPID();
}

void enterTurnHome() {
  // Set the initial heading to the current heading
  Globals::initialHeading = Globals::currentHeading;
  Globals::homeStartX = Globals::currentX;
  Globals::homeStartY = Globals::currentY;
  // Reset the PID controllers
  resetPID();
}

void enterGoHome() {
  // Set the initial heading to the current heading
  Globals::initialHeading = Globals::currentHeading;
  // Reset the PID controllers
  resetPID();

}

// ---| State exit functions |---
void exitState(State state) {
  updateKinematics();
  Globals::lastState = state;

  beepOn();
  Globals::motors.disableMotors();
  switch (state) {
    case IDLE:
      exitIdle();
      break;
    case LEAVE_BOX:
      exitLeaveBox();
      break;
    case FOLLOW_SEGMENT:
      exitFollowSegment();
      break;
    case TURNING:
      exitTurning();
      break;
    case STOP:
      exitStop();
      break;
    case TURN_HOME:
      exitTurnHome();
      break;
    case GO_HOME:
      exitGoHome();
      break;
  }
  updateWheels();
}

void exitIdle() {
  // Reset the PID controllers
  resetPID();
}

void exitLeaveBox() {
  // Reset the PID controllers
  resetPID();
}

void exitFollowSegment() {
  // Reset the PID controllers
  resetPID();
}

void exitTurning() {
  Globals::turnsTransitionCount++;
  // Update prevTurns array
  Globals::previousTurns[Globals::turnsTransitionCount % TURN_HISTORY_SIZE] = Globals::activeTurn;
  // Set the previous active turn to the current active turn
  Globals::prevActiveTurn = Globals::activeTurn;
  // Reset the PID controllers
  // Set the motors to 0
  Globals::motors.setMotorPower(0, 0);
  // Set the turn cooldown timer
  Globals::turnCooldownTimer = millis();

  Globals::turnStartPosition[0] = Globals::currentX;
  Globals::turnStartPosition[1] = Globals::currentY;
  resetPID();
}

void exitStop() {
  // Reset the PID controllers
  resetPID();
}

void exitTurnHome() {
  // Reset the PID controllers
  resetPID();
}

void exitGoHome() {
  // Reset the PID controllers
  resetPID();
}
