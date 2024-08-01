#include <Arduino.h>
#include "encoders.h"
#include "kinematics.h"
#include "globals.h"


// The number of encoder counts per revolution of the wheel.
#define ENCODER_COUNTS_PER_REV 12

// Class to track robot position.
Kinematics_c::Kinematics_c() : x(0.0f), y(0.0f), theta(0.0f), prev_count_left_encoder(0), prev_count_right_encoder(0), wheel_radius(0.016f), wheel_base(0.043f) {};

void Kinematics_c::update() {
    long current_count_left_encoder = count_left_encoder;
    long current_count_right_encoder = count_right_encoder;

    long delta_count_left_encoder = current_count_left_encoder - prev_count_left_encoder;
    long delta_count_right_encoder = current_count_right_encoder - prev_count_right_encoder;

    prev_count_left_encoder = current_count_left_encoder;
    prev_count_right_encoder = current_count_right_encoder;

    float delta_distance_left = delta_count_left_encoder * 2 * PI * wheel_radius / ENCODER_COUNTS_PER_REV;
    float delta_distance_right = delta_count_right_encoder * 2 * PI * wheel_radius / ENCODER_COUNTS_PER_REV;

    float delta_distance = (delta_distance_left + delta_distance_right) / 2.0f;
    float delta_theta = (delta_distance_right - delta_distance_left) / (wheel_base);
    float theta_rad = theta * PI / 180.0f; // Convert theta to radians

    if (delta_count_left_encoder == delta_count_right_encoder) {
        // The robot is moving in a straight line
        x += delta_distance * cos(theta_rad);
        y += delta_distance * sin(theta_rad);
    } else if (delta_count_left_encoder != delta_count_right_encoder) {
        float delta_theta_rad = delta_theta * PI / 180.0f; // Convert delta theta to radians

        // Check to avoid division by zero
        if (fabs(delta_theta_rad) > 1e-20) {
            // The robot is moving in an arc
            float R = delta_distance / delta_theta_rad; // Radius of the arc
            
            // Calculate the change in position
            float dX = R * sin(delta_theta_rad); // Change in x
            float dY = R * (1 - cos(delta_theta_rad)); // Change in y
            
            // Rotate the changes in x and y by the current angle to get the changes in the global frame
            x += dX * cos(theta_rad) + dY * sin(theta_rad);
            y += dX * sin(theta_rad) - dY * cos(theta_rad);
        } else {
            // The robot is moving in a straight line
            x += delta_distance * cos(theta_rad);
            y += delta_distance * sin(theta_rad);
        }
    }
    
    Globals::angleToOrigin = atan2(y, x) * 180.0f / PI;
    Globals::originMeasurement = mapAngle360(getHeadingDeg()) - mapAngle360(Globals::angleToOrigin);
    Globals::originMeasurement = Globals::kinematics.mapAngle180(Globals::originMeasurement) / 180.0f; // error [-1, 1]

    theta += delta_theta;
}


float Kinematics_c::getX() const {
    return x;
}

float Kinematics_c::getY() const {
    return y;
}

float Kinematics_c::getHeadingDeg() const {
    return theta;
}

float Kinematics_c::getHeadingRad() const {
    // theta is in degrees, convert to radians
    return theta * PI / 180.0f;
}

float Kinematics_c::mapAngle360(float angle) const {
    float angleMod = fmod(angle, 360.0f);
    if (angleMod < 0) angleMod += 360.0f;
        return angleMod;
    }

float Kinematics_c::mapAngle180(float angle) const {
    float angleMod = mapAngle360(angle);
    if (angleMod > 180.0f) angleMod -= 360.0f;
        return angleMod;
    }
