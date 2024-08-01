#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "kinematics.h"

#ifndef _WHEEL_H
#define _WHEEL_H

class Wheel_c {
  private:
    int encoder_pin;
    long prev_encoder_count;
    long prev_time;
    float rotation_velocity;
    float lpf;
    float alpha;

  public:
    Wheel_c(int encoder_pin);
    void update();
    float getVelocity() const;
    float getEncoderCount() const;
};

#endif // _WHEEL_H
