#include <Arduino.h>
#include "constants.h"
#include "globals.h"
#include "functions.h"
#include "stateconditions.h"
#include "statetransitions.h"
#include "stateactions.h"

// Define the state
State Globals::state = IDLE;
State Globals::lastState = IDLE;
DetectionState Globals::lineDetectionState = NOT_DETECTING;
TurnState Globals::turnState = NO_TURN_DETECTED;


void setup() {
  // Initialize all the classes
  initObjects();

  // Initialize the serial communication
  Serial.begin(9600);

  // Initialize the timers
  initTimers();
}

void loop() {
  // Update the kinematics
  updateKinematics();

  // Update the line detection
  updateLineDetection();

  // Determine the next state
  State nextState = determineNextState();

  // If the state has changed, call the exit action
  if (nextState != Globals::state) {
    exitState(Globals::state);
    enterState(nextState);
  }

  // Update the state
  Globals::state = nextState;

  // Call the action for the current state
  executeState(Globals::state);

  // Update the wheels
  updateWheels();

  // Debug print
  debugPrint(true, true, true, false);
}
