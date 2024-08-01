#include <Arduino.h>

#pragma once

// --- Constant Variables ---
// - Define the pins for the right and left encoders -
#define RIGHT_ENCODER_PIN 0
#define LEFT_ENCODER_PIN 1

#define BUZZER_PIN A7

// - Define the PID constants -
#define KP 14
#define KI 0.01
#define KD 10

#define turnKP 7.5
#define turnKI 0.00085
#define turnKD 10

#define homeKP 5.5
#define homeKI 0.0005
#define homeKD 10

#define sensorKP 2
#define sensorKI 0.003
#define sensorKD 7

// - Define the intervals for the timers -
#define LINE_DETECTION_INTERVAL 0
#define KINEMATICS_UPDATE_INTERVAL 0
#define MOTOR_UPDATE_INTERVAL 0
#define WHEELS_UPDATE_INTERVAL 0
#define PID_UPDATE_INTERVAL 0
#define LINE_TRANSITION_INTERVAL 0
#define TURN_TRANSITION_INTERVAL 0
#define TURN_COOLDOWN_INTERVAL 100
#define DEBUG_PRINT_INTERVAL 250
#define STATE_TIMEOUT 4000
#define AIR_TIME_INTERVAL 1000
#define STOP_TIMEOUT 5000
#define TURN_TIMEOUT 0

#define BIG_HEADING_DIFFERENCE_THRESHOLD 1.5
#define SMALL_HEADING_DIFFERENCE_THRESHOLD 0.2
#define HEADING_HISTORY_SIZE 8
#define RECENT_HEADING_HISTORY_SIZE 4

#define TURN_HISTORY_SIZE 5

#define PATH_LENGTH_HISTORY_SIZE 5

// - Define the PWM constants -
#define MAX_PWM 33
#define BIAS_PWM 15
#define LINE_BIAS_PWM 15
#define TURN_BIAS_PWM 0
#define TURN_PWM_SCALE 1

// - Define the states -
enum State {
  IDLE = 0,
  LEAVE_BOX = 1,
  FOLLOW_SEGMENT = 2,
  TURNING = 3,
  STOP = 4,
  TURN_HOME = 5,
  GO_HOME = 6
};

// - Define the line detection states -
enum DetectionState {
    NOT_DETECTING,
    DETECTING
};

// - Define the turn detection states -
enum TurnState {
    NO_TURN_DETECTED,
    TURN_DETECTED,
};