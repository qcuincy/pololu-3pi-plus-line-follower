#include <Arduino.h>
#include "constants.h"
#include "globals.h"
#include "functions.h"

#pragma once

// ---| State entry functions |---
void enterState(State state);
void enterIdle(), enterLeaveBox(), enterFollowSegment(), enterTurning(), enterStop(), enterTurnHome(), enterGoHome();

// ---| State exit functions |---
void exitState(State state);
void exitIdle(), exitLeaveBox(), exitFollowSegment(), exitTurning(), exitStop(), exitTurnHome(), exitGoHome();
