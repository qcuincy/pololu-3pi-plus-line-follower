#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "kinematics.h"
#include "wheel.h"

Wheel_c::Wheel_c(int encoder_pin) : encoder_pin(encoder_pin), prev_encoder_count(0), prev_time(0), rotation_velocity(0.0), lpf(0.0), alpha(0.2) {};

void Wheel_c::update() {
    long current_encoder_count = readEncoder(encoder_pin);
    long current_time = millis();

    long delta_encoder_count = current_encoder_count - prev_encoder_count;
    long delta_time = current_time - prev_time;

    if (delta_time > 0) { // Avoid division by zero
    rotation_velocity = (float)delta_encoder_count / delta_time;
    }

    // Apply low pass filter
    lpf = (lpf * (1 - alpha)) + (rotation_velocity * alpha);

    prev_encoder_count = current_encoder_count;
    prev_time = current_time;
}

float Wheel_c::getVelocity() const {
    return lpf;
}

float Wheel_c::getEncoderCount() const {
    return prev_encoder_count;
}
