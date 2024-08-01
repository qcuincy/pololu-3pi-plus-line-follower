#include <Arduino.h>
#include "motors.h"


Motors_c::Motors_c() {};

// Class to operate the motor(s).
void Motors_c::initialise() {
    pinMode(L_PWM_PIN, OUTPUT);
    pinMode(L_DIR_PIN, OUTPUT);
    pinMode(R_PWM_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT);

    // Set initial direction (HIGH/LOW)
    // for the direction pins.
    // ...

    digitalWrite(L_DIR_PIN, LOW);
    digitalWrite(R_DIR_PIN, LOW);

    

    // Set initial power values for the PWM
    // Pins.
    // ...
    analogWrite(L_PWM_PIN, 0);
    analogWrite(R_PWM_PIN, 0);
}

int Motors_c::signOf(float x){
    if (x > 0) {
    return 1;
    } else if (x < 0) {
    return -1;
    } else {
    return 0;
    }
}

void Motors_c::setMotorPower(double left_pwm, double right_pwm ) {
    // Use the signOf() function to get the sign of
    // the power.
    int left_sign = signOf(left_pwm);
    int right_sign = signOf(right_pwm);
    // Use the abs() function to get the magnitude of the power.
    // Use the constrain() function to limit the
    // magnitude to 0-255.
    double left_mag = constrain(abs(left_pwm), 0, 255);
    double right_mag = constrain(abs(right_pwm), 0, 255);




    analogWrite(L_PWM_PIN, left_mag);
    analogWrite(R_PWM_PIN, right_mag);

    digitalWrite(L_DIR_PIN, left_sign == -1 ? FWD : REV);
    digitalWrite(R_DIR_PIN, right_sign == -1 ? FWD : REV);

    L_PWM_VAL = left_sign == -1 ? -left_mag : left_mag;
    R_PWM_VAL = right_sign == -1 ? -right_mag : right_mag;
}

void Motors_c::disableMotors() {
    analogWrite(L_PWM_PIN, 0);
    analogWrite(R_PWM_PIN, 0);
}

bool Motors_c::motorsEnabled() {
    return analogRead(L_PWM_PIN) > 0 || analogRead(R_PWM_PIN) > 0;
}

bool Motors_c::leftMotorEnabled() {
    return analogRead(L_PWM_PIN) > 0;
}

bool Motors_c::rightMotorEnabled() {
    return analogRead(R_PWM_PIN) > 0;
}

void Motors_c::setLeftMotorPower(float left_pwm) {
    setMotorPower(left_pwm, 0);
}

void Motors_c::setRightMotorPower(float right_pwm) {
    setMotorPower(0, right_pwm);
}

void Motors_c::steerForwards(float amt1, float amt2){
    if (amt2 != 0){
    setMotorPower(amt1, amt2);
    } else {
    setMotorPower(amt1, amt1);
    }
}

void Motors_c::steerLeft(float amt){
    setMotorPower(-amt, amt);
}

void Motors_c::steerRight(float amt){
    setMotorPower(amt, -amt);
}

void Motors_c::steerBackwards(float amt1, float amt2) {
    if (amt2 != 0){
    setMotorPower(-amt1, -amt2);
    } else {
    setMotorPower(-amt1, -amt1);
    }
}

// function to return left and right motor power (LPWM, RPWM)
MotorPower Motors_c::getMotorPower() {
    MotorPower motorPower;
    motorPower.left = L_PWM_VAL;
    motorPower.right = R_PWM_VAL;
    return motorPower;
}

// function to return left and right motor direction (LDIR, RDIR)
MotorDirection Motors_c::getMotorDirection() {
    MotorDirection motorDirection;
    motorDirection.left = digitalRead(L_DIR_PIN) == HIGH ? FWD : REV;
    motorDirection.right = digitalRead(R_DIR_PIN) == HIGH ? FWD : REV;
    return motorDirection;
}
