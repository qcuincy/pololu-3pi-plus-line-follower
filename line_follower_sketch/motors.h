#include <Arduino.h>
// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _MOTORS_H
#define _MOTORS_H

# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15

# define FWD HIGH
# define REV LOW

struct MotorPower {
  float left;
  float right;
};

struct MotorDirection {
  int left;
  int right;
};

// Class to operate the motor(s).
class Motors_c {
  public:

    float L_PWM_VAL;  // Add these variables to store the PWM values
    float R_PWM_VAL;  // for the left and right motors.

    // Constructor, must exist.
    Motors_c();
    void initialise();
    int signOf(float x);
    void setMotorPower(double left_pwm, double right_pwm );
    void disableMotors();
    bool motorsEnabled();
    bool leftMotorEnabled();
    bool rightMotorEnabled();
    void setLeftMotorPower(float left_pwm);
    void setRightMotorPower(float right_pwm);
    void steerForwards(float amt1, float amt2 = 0);
    void steerLeft(float amt);
    void steerRight(float amt);
    void steerBackwards(float amt1, float amt2 = 0);
    // function to return left and right motor power (LPWM, RPWM)
    MotorPower getMotorPower();
    // function to return left and right motor direction (LDIR, RDIR)
    MotorDirection getMotorDirection();
    
};



#endif
