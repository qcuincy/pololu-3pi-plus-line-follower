#include <Arduino.h>
#include "constants.h"
#include "globals.h"
#include "functions.h"

#pragma once

State determineNextState();

State determineNextStateFromIdle();
State determineNextStateFromLeaveBox();
State determineNextStateFromFollowSegment();
State determineNextStateFromTurning();
State determineNextStateFromStop();
State determineNextStateFromTurnHome();
State determineNextStateFromGoHome();
