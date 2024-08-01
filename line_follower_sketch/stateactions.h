#include <Arduino.h>
#include "constants.h"
#include "globals.h"
#include "functions.h"

#pragma once

// ---| State actions |---
void executeState(State state);

void executeIdle();
void executeLeaveBox();
void executeFollowSegment();
void executeTurning();
void executeStop();
void executeTurnHome();
void executeGoHome();
