#include <Arduino.h>
#include "functions.h"
#include "globals.h"

double _measurement = 0;
double _prevMeasurement = 0;

// --| Function Definitions |--

// --- Init functions ---
// - Initialize the encoders, motors, PID controllers, and line follower -
void initObjects() {
  setupEncoder0();
  setupEncoder1();

  Globals::motors.initialise();
  
  Globals::headPID.initialise();
  Globals::leftPID.initialise();
  Globals::rightPID.initialise();

  Globals::lineFollowing.initialise();
}

// - Initialize the timers -
void initTimers() {
    Globals::lineDetectionTimer = millis();
    Globals::lineTransitionTimer = millis();
    Globals::turnTransitionTimer = millis();
    Globals::turnCooldownTimer = millis();
    Globals::kinematicsUpdateTimer = millis();
    Globals::wheelsUpdateTimer = millis();
    Globals::motorUpdateTimer = millis();
    Globals::pidUpdateTimer = millis();
    Globals::debugPrintTimer = millis();
    Globals::turnTimeoutTimer = millis();
}

// --- Update functions ---

// - Update the line detection every loop -
void updateLineDetection() {
    unsigned long currentMillis = millis();
    Globals::lineFollowing.update(0);

    if (currentMillis - Globals::lineDetectionTimer >= LINE_DETECTION_INTERVAL) {
        Globals::linesDetected = Globals::lineFollowing.lineDetected();
        Globals::turnsDetected = Globals::lineFollowing.turnDetected(Globals::prevActiveTurn);
        

        bool anySensorDetected = Globals::lineFollowing.anySensorDetected();

        switch (Globals::lineDetectionState) {
            case NOT_DETECTING:
                if (anySensorDetected) {
                    if (Globals::turnsDetected != 'N') {
                        Globals::lineDetectionState = DETECTING;
                    }
                }
                break;

            case DETECTING:
                if (!anySensorDetected && Globals::state != 0) {
                    Globals::turnUpdate = Globals::turnsDetected;

                    // Transition to NOT_DETECTING state and increment line transition count
                    Globals::linesTransitionCount++;
                    
                    Globals::lineDetectionState = NOT_DETECTING;
                }
                break;
        }

        // Reset the timer
        Globals::lineDetectionTimer = currentMillis;
    }
}


// - Update the kinematics every loop -
void updateKinematics() {
    unsigned long currentMillis = millis();

    Globals::kinematics.update();
    
    if (currentMillis - Globals::kinematicsUpdateTimer >= KINEMATICS_UPDATE_INTERVAL) {

        // Update the current kinematics variables
        Globals::currentX = Globals::kinematics.getX();
        Globals::currentY = Globals::kinematics.getY();
        Globals::currentHeading = Globals::kinematics.getHeadingDeg();

        // Update the distance and heading change
        Globals::distanceTravelled = sqrt(pow(Globals::currentX, 2) + pow(Globals::currentY, 2));
        Globals::headingChange = abs(abs(Globals::currentHeading) - abs(Globals::previousHeading));
        // Shift the elements in the array one position to the left
        for (int i = HEADING_HISTORY_SIZE - 1; i > 0; i--) {
            Globals::headingChanges[i] = Globals::headingChanges[i - 1];
            if (Globals::headingChanges[i] > Globals::maxHeadingChange) {
                Globals::maxHeadingChange = Globals::headingChanges[i];
            }
        }

        // Update the previous kinematics variables
        Globals::previousX = Globals::currentX;
        Globals::previousY = Globals::currentY;
        Globals::previousHeading = Globals::currentHeading;

        // Store the current heading change at the beginning of the array
        Globals::headingChanges[0] = Globals::headingChange;

        bool allBelowThreshold = true;
        for (int i = 0; i < RECENT_HEADING_HISTORY_SIZE && i < HEADING_HISTORY_SIZE; i++) {
        if (Globals::headingChanges[i] > SMALL_HEADING_DIFFERENCE_THRESHOLD) {
            allBelowThreshold = false;
            break;
        }
        }

        // Reset maxHeadingChange if all recent changes are below threshold
        if (allBelowThreshold) {
        Globals::maxHeadingChange = 0;
        }

        // Reset the timer
        Globals::kinematicsUpdateTimer = currentMillis;
    }
}

// - Update the wheels after a new PWM has been set -
void updateWheels() {
    unsigned long currentMillis = millis();

    if (currentMillis - Globals::wheelsUpdateTimer >= WHEELS_UPDATE_INTERVAL) {
        Globals::leftWheel.update();
        Globals::rightWheel.update();

        // Reset the timer
        Globals::wheelsUpdateTimer = currentMillis;
    }
}

// - Reset PID controllers -
void resetPID() {
    Globals::headPID.reset();
    Globals::leftPID.reset();
    Globals::rightPID.reset();
    Globals::headTurnPID.reset();
    Globals::leftTurnPID.reset();
    Globals::rightTurnPID.reset();
    Globals::leftSensorPID.reset();
    Globals::rightSensorPID.reset();
    Globals::headSensorPID.reset();
    Globals::leftHomePID.reset();
    Globals::rightHomePID.reset();
    Globals::headHomePID.reset();
}

// --- Debug functions ---
void beepOn() {
    digitalWrite(BUZZER_PIN, HIGH);
}

void beepOff() {
    digitalWrite(BUZZER_PIN, LOW);
}

const char* getStateName(State state) {
    switch (state) {
        case IDLE: return "IDLE";
        case LEAVE_BOX: return "LEAVE_BOX";
        case FOLLOW_SEGMENT: return "FOLLOW_SEGMENT";
        case TURNING: return "TURNING";
        case STOP: return "STOP";
        case TURN_HOME: return "TURN_HOME";
        case GO_HOME: return "GO_HOME";
        default: return "UNKNOWN";
    }
}

void debugPrint(bool currentState, bool lineDetection, bool kinematics, bool pid) {
    unsigned long currentMillis = millis();

    if (currentMillis - Globals::debugPrintTimer >= DEBUG_PRINT_INTERVAL) {
        if (currentState) {
            Serial.print("State: ");
            Serial.print(getStateName(Globals::state));
            Serial.print(" Last State: ");
            Serial.println(getStateName(Globals::lastState));
        }
        if (lineDetection) {
            Serial.print("Lines detected: ");
            Serial.print(Globals::linesTransitionCount);
            Serial.print(" Turn count: ");
            Serial.print(Globals::turnsTransitionCount);
            Serial.print(" Turn: ");
            Serial.print(Globals::turnsDetected);
            Serial.print(" Active turn: ");
            Serial.print(Globals::activeTurn);
            Serial.print(" Prev active turn: ");
            Serial.print(Globals::prevActiveTurn);
            Serial.print(" Path length: ");
            Serial.print(Globals::turnPathLengths[Globals::turnsTransitionCount % TURN_HISTORY_SIZE]);
            Serial.print(" Prev 2 turns: ");
            Serial.print(Globals::previousTurns[0]);
            Serial.print(" ");
            Serial.print(Globals::previousTurns[1]);
            Serial.print(" ");
            Serial.print("Sensor values: ");
            for (int i = 0; i < 5; i++) {
                Serial.print(Globals::lineFollowing.getLineSensorValue(i));
                Serial.print(" ");
            }
            Serial.print("Gradients: ");
            for (int i=0; i < 4; i++) {
            Serial.print(Globals::lineFollowing.getLineSensorValue(i+1) - Globals::lineFollowing.getLineSensorValue(i));
            Serial.print(" ");
            }
            Serial.print("Sensors detected: ");
            for (int i = 0; i < 5; i++) {
                Serial.print(Globals::lineFollowing.sensorDetected(i));
                Serial.print(" ");
            }
            Serial.println();
        }

        if (kinematics) {
            Serial.print("X: ");
            Serial.print(Globals::currentX);
            Serial.print(" Y: ");
            Serial.print(Globals::currentY);
            Serial.print(" Heading: ");
            Serial.print(Globals::currentHeading);
            Serial.print(" Distance Travelled: ");
            Serial.print(Globals::distanceTravelled);
            Serial.print(" Heading Difference: ");
            Serial.print(Globals::headingChange);
            Serial.print(" Max Heading Difference: ");
            Serial.print(Globals::maxHeadingChange);
            Serial.print(" Target heading: ");
            Serial.print(Globals::targetHeading);
            Serial.print(" Current heading: ");
            Serial.print(Globals::currentHeading);
            Serial.print(" Heading error: ");
            Serial.print(Globals::headingTurnError);
            Serial.print(" Home Turn Error: ");
            Serial.print(Globals::homeTurnError);
            Serial.print(" Origin: ");
            Serial.println(Globals::originMeasurement);

        }

        if (pid) {
            Serial.print("Head Feedback: ");
            Serial.print(Globals::headPID.getFeedbackSignal());
            Serial.print(" Left Feedback: ");
            Serial.print(Globals::leftPID.getFeedbackSignal());
            Serial.print(" Right Feedback: ");
            Serial.print(Globals::rightPID.getFeedbackSignal());
            Serial.print(" Head Turn Feedback: ");
            Serial.print(Globals::headTurnPID.getFeedbackSignal());
            Serial.print(" Left Turn Feedback: ");
            Serial.print(Globals::leftTurnPID.getFeedbackSignal());
            Serial.print(" Right Turn Feedback: ");
            Serial.println(Globals::rightTurnPID.getFeedbackSignal());

        }

        // Reset the timer
        Globals::debugPrintTimer = currentMillis;
    }
}

// --- Calculate and constrain functions ---
float constrainPWM(float pwm) {
    if (pwm > MAX_PWM) {
        return MAX_PWM;
    } else if (pwm < -MAX_PWM) {
        return -MAX_PWM;
    } else {
        return pwm;
    }
}

bool checkThreshold(float value, float threshold) {
    return abs(value) < threshold;
}


void driveStraight(float targetAngle) {
    updateKinematics();
    Globals::currentHeading = Globals::kinematics.getHeadingDeg();

    float currentHeading360 = Globals::kinematics.mapAngle360(Globals::currentHeading);
    float targetHeading360 = Globals::kinematics.mapAngle360(targetAngle);
    _measurement = currentHeading360 - targetHeading360;
    _measurement = Globals::kinematics.mapAngle180(_measurement) / 180.0f;
    float demand = 0;

    // Update the PID
    double headFeedback = Globals::headPID.update(demand, _measurement);

    // Set the velocity demands to the feedback signal
    double leftFeedback = Globals::leftPID.update(headFeedback, Globals::leftWheel.getVelocity());
    double rightFeedback = Globals::rightPID.update(headFeedback, Globals::rightWheel.getVelocity());

    double leftPWM = constrainPWM(BIAS_PWM + leftFeedback);
    double rightPWM = constrainPWM(BIAS_PWM - rightFeedback);

    // Set the motors to the output
    Globals::motors.setMotorPower(leftPWM, rightPWM);

    _prevMeasurement = _measurement;

    updateWheels();
}

void followLine() {
    updateKinematics();
    updateLineDetection();
    // Set the measurement to the weighted line sensors measurement
    _measurement = Globals::lineFollowing.weightedMeasurement();
    double demand = 0;

    double headFeedback = Globals::headSensorPID.update(demand, _measurement);

    // Set the velocity demands to the feedback signal
    double leftFeedback = Globals::leftSensorPID.update(headFeedback, Globals::leftWheel.getVelocity());
    double rightFeedback = Globals::rightSensorPID.update(headFeedback, Globals::rightWheel.getVelocity());


    double leftPWM = constrainPWM(LINE_BIAS_PWM + leftFeedback);
    double rightPWM = constrainPWM(LINE_BIAS_PWM - rightFeedback);

    Globals::motors.setMotorPower(leftPWM, rightPWM);
    if (_prevMeasurement * _measurement < 0) {
        resetPID();
    }

    _prevMeasurement = _measurement;
    updateWheels();
}

void determineTarget(char direction) {
    if (direction == 'L') {
        Globals::targetHeading = Globals::initialHeading - 90;
    } else if (direction == 'R') {
        Globals::targetHeading = Globals::initialHeading + 90;
    } else if (direction == 'S') {
        Globals::targetHeading = Globals::initialHeading;
    } else if (direction == 'N') {
        Globals::targetHeading = Globals::initialHeading + 160;
    }
}

void turn(char direction) {
    updateKinematics();
    determineTarget(Globals::activeTurn);

    Globals::currentHeading = Globals::kinematics.getHeadingDeg();


    double targetHeading360 = Globals::kinematics.mapAngle360(Globals::targetHeading);
    double currentHeading360 = Globals::kinematics.mapAngle360(Globals::currentHeading);

    _measurement = currentHeading360 - targetHeading360;
    _measurement = Globals::kinematics.mapAngle180(_measurement) / 180.0f;
    double demand = 0;

    Globals::headingTurnError = _measurement;

    // Update the PID
    double headFeedback = Globals::headTurnPID.update(demand, _measurement);

    // Set the velocity demands to the feedback signal
    double leftFeedback = Globals::leftTurnPID.update(headFeedback, Globals::leftWheel.getVelocity());
    double rightFeedback = Globals::rightTurnPID.update(headFeedback, Globals::rightWheel.getVelocity());


    double leftPWM = constrainPWM(leftFeedback);
    double rightPWM = constrainPWM(-rightFeedback);

    leftPWM = abs(leftPWM) > 15 ? leftPWM : leftPWM * TURN_PWM_SCALE;
    rightPWM = abs(rightPWM) > 15 ? rightPWM : rightPWM * TURN_PWM_SCALE;

    // Set the motors to the output
    Globals::motors.setMotorPower(leftPWM, rightPWM);

    if (_prevMeasurement * _measurement < 0) {
        resetPID();
    }

    _prevMeasurement = _measurement;
    updateWheels();
}

void turnHome() {
    updateKinematics();
    Globals::currentHeading = Globals::kinematics.getHeadingDeg();
    Globals::angleToOrigin = atan2(Globals::currentY, Globals::currentX) * 180.0f / PI;

    double currentHeading360 = Globals::kinematics.mapAngle360(Globals::currentHeading);
    double targetHeading = Globals::kinematics.mapAngle360(Globals::angleToOrigin); // demand [0, 360]
    double diff = currentHeading360 - targetHeading; // error [0, 360]
    _measurement = Globals::kinematics.mapAngle180(diff) / 180.0f; // [0, 360] -> [-180, 180] -> [-1, 1]

    double demand = 0;

    Globals::homeTurnError = _measurement;

    // Update the PID
    double headFeedback, leftFeedback, rightFeedback;
    headFeedback = Globals::headHomePID.update(demand, _measurement);

    leftFeedback = Globals::leftHomePID.update(headFeedback, Globals::leftWheel.getVelocity());
    rightFeedback = Globals::rightHomePID.update(headFeedback, Globals::rightWheel.getVelocity());

    double leftPWM = constrainPWM(leftFeedback);  
    double rightPWM = constrainPWM(-rightFeedback);

    // Set the motors to the output
    Globals::motors.setMotorPower(leftPWM, rightPWM);

    _prevMeasurement = _measurement;

    updateWheels();

}


void goHome() {
    updateKinematics();
    Globals::goHomeDistance = sqrt(pow((Globals::currentX - Globals::homeStartX), 2) + pow((Globals::currentY - Globals::homeStartY), 2));
    Globals::currentHeading = Globals::kinematics.getHeadingDeg();
    Globals::angleToOrigin = atan2(Globals::currentY, Globals::currentX) * 180.0f / PI;

    double currentHeading360 = Globals::kinematics.mapAngle360(Globals::currentHeading);
    double targetHeading = Globals::kinematics.mapAngle360(Globals::angleToOrigin); // demand [0, 360]
    double diff = currentHeading360 - targetHeading; // error [0, 360]
    _measurement = Globals::kinematics.mapAngle180(diff) / 180.0f; // [0, 360] -> [-180, 180] -> [-1, 1]

    double demand = 0;

    Globals::headingTurnError = _measurement;

    // Update the PID
    double headFeedback = Globals::headPID.update(demand, _measurement);

    // Set the velocity demands to the feedback signal
    double leftFeedback = Globals::leftPID.update(headFeedback, Globals::leftWheel.getVelocity());
    double rightFeedback = Globals::rightPID.update(headFeedback, Globals::rightWheel.getVelocity());

    double leftPWM = constrainPWM(BIAS_PWM + leftFeedback);  
    double rightPWM = constrainPWM(BIAS_PWM - rightFeedback);

    // Set the motors to the output
    Globals::motors.setMotorPower(leftPWM, rightPWM);
    
    _prevMeasurement = _measurement;

    updateWheels();
}
