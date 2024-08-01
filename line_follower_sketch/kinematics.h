#include <Arduino.h>
#include "encoders.h"
// This #ifndef stops this file
// from being included more than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// The number of encoder counts per revolution of the wheel.
#define ENCODER_COUNTS_PER_REV 12

// Class to track robot position.
class Kinematics_c {
  private:
    float x;
    float y;
    float theta;
    volatile long prev_count_left_encoder;
    volatile long prev_count_right_encoder;
    float wheel_radius;
    float wheel_base;
  public:
    Kinematics_c();
    // Use this function to update your kinematics
    void update();
    float getX() const;
    float getY() const;
    float getHeadingDeg() const;
    float getHeadingRad() const;
    float mapAngle360(float angle) const;
    float mapAngle180(float angle) const;
};

#endif // _KINEMATICS_H
