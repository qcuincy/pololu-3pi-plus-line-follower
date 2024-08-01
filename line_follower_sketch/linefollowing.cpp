#include <Arduino.h>
#include "linesensor.h"
#include "linefollowing.h"

float DN2_AIR_THRESHOLD = 0.95;
float DN4_AIR_THRESHOLD = 0.95;

LineSensor_c linesensor;

unsigned long elapsed_time, line_sensor_ts;

int dn1_min = 0, dn1_max = 0;
int dn2_min = 0, dn2_max = 0;
int dn3_min = 0, dn3_max = 0; 
int dn4_min = 0, dn4_max = 0;
int dn5_min = 0, dn5_max = 0;
int dn_mins[5];
int dn_maxs[5];
int dn1_air_min = 0, dn5_air_min = 0;
int dn2_actual_max = 0, dn4_actual_max = 0;
int larger_dn = 0, larger_dn_idx = 0;

LineFollowing_c::LineFollowing_c(bool useADC) {
    this->useADC = useADC;
    for (int i = 0; i < 5; i++) {
        this->prevSensorValues[i] = 0;
        this->detectedSensors[i] = false;
    }
}

void LineFollowing_c::initialise() {
    dn1_min = useADC ? DN1_BLACK_MIN_ADC : DN1_BLACK_MIN;
    dn1_max = useADC ? DN1_BLACK_MAX_ADC : DN1_BLACK_MAX;
    dn2_min = useADC ? DN2_BLACK_MIN_ADC : DN2_BLACK_MIN;
    dn2_max = useADC ? DN2_BLACK_MAX_ADC : DN2_BLACK_MAX;
    dn3_min = useADC ? DN3_BLACK_MIN_ADC : DN3_BLACK_MIN;
    dn3_max = useADC ? DN3_BLACK_MAX_ADC : DN3_BLACK_MAX;
    dn4_min = useADC ? DN4_BLACK_MIN_ADC : DN4_BLACK_MIN;
    dn4_max = useADC ? DN4_BLACK_MAX_ADC : DN4_BLACK_MAX;
    dn5_min = useADC ? DN5_BLACK_MIN_ADC : DN5_BLACK_MIN;
    dn5_max = useADC ? DN5_BLACK_MAX_ADC : DN5_BLACK_MAX;
    dn_mins[0] = dn1_min;
    dn_mins[1] = dn2_min;
    dn_mins[2] = dn3_min;
    dn_mins[3] = dn4_min;
    dn_mins[4] = dn5_min;
    dn_maxs[0] = dn1_max;
    dn_maxs[1] = dn2_max;
    dn_maxs[2] = dn3_max;
    dn_maxs[3] = dn4_max;
    dn_maxs[4] = dn5_max;
    
    dn1_air_min = useADC ? DN1_AIR_MIN_ADC : DN1_AIR_MIN;
    dn5_air_min = useADC ? DN5_AIR_MIN_ADC : DN5_AIR_MIN;
    linesensor.initialise(useADC);

    waitUntilNonZero();

    line_sensor_ts = millis();
}

void LineFollowing_c::waitUntilNonZero() {
    Serial.print("Waiting for non-zero sensor values...");
    while (!allNonZero()) {
        update();
    }
}

void LineFollowing_c::update(unsigned long _ls_update) {
    if (millis() - line_sensor_ts > _ls_update) {
        linesensor.readAllSensors();
        larger_dn = max(ls_value[1], ls_value[3]);
        larger_dn_idx = larger_dn == ls_value[1] ? 1 : 3;
        line_sensor_ts = millis();
    }
}

float LineFollowing_c::weightedMeasurement(bool useMax) {
    float sum, norm_readings[2];

    int sensor_val_2 = useMax ? dn2_max : ls_value[1];
    int sensor_val_4 = useMax ? dn4_max : ls_value[3];
    
    sum = (float)sensor_val_2 + (float)sensor_val_4;
    for (int i = 0; i < 2; i++) {
        norm_readings[i] = (ls_value[i * 2 + 1] / sum) * 2;
    }
    float W = norm_readings[0] - norm_readings[1];

    return W;
}

bool LineFollowing_c::lineDetected() {
    if (!inAir()) {
        detectedSensors[0] = sensorDetected(0);
        detectedSensors[1] = sensorDetected(1);
        detectedSensors[2] = sensorDetected(2);
        detectedSensors[3] = sensorDetected(3);
        detectedSensors[4] = sensorDetected(4);
        if (detectedSensors[1] || detectedSensors[2] || detectedSensors[3]) {
            return true;
        } else {
            if (detectedSensors[0] || detectedSensors[4]) {
                return true;
            } else {
                return false;
            }
        }
    } else {
        return false;
    }
}

bool LineFollowing_c::sensorDetected(int idx) {
    return ls_value[idx] >= dn_mins[idx] && ls_value[idx] <= dn_maxs[idx] ? true : false;
}

bool LineFollowing_c::anySensorDetected() {
    return sensorDetected(0) || sensorDetected(1) || sensorDetected(2) || sensorDetected(3) || sensorDetected(4);
}

bool LineFollowing_c::allNonZero() {
    for (int i = 0; i < 5; i++) {
        if (ls_value[i] == 0) {
            return false;
        }
    }
    return true;
}

bool LineFollowing_c::inAir() {
    float total = (ls_value[0] + ls_value[1] + ls_value[2] + ls_value[3] + ls_value[4]);
    float avg = total / 5;
    float variance = 0;
    for (int i = 0; i < 5; i++) {
        variance += pow(ls_value[i] - avg, 2);
    }
    variance /= 5;
    float standard_deviation = sqrt(variance);

    if (standard_deviation > 100) {
        if (ls_value[1] >= dn2_min && ls_value[2] >= dn3_min && ls_value[3] >= dn4_min) {
            if (ls_value[1] <= dn2_max && ls_value[2] <= dn3_max && ls_value[3] <= dn4_max) {
                if (ls_value[0] <= dn1_min || ls_value[4] <= dn5_min) {
                    return false;
                } else {
                    return true;
                }
            } else {
                return true;
            }
        } else {
            if (ls_value[0] <= dn1_min || ls_value[4] <= dn5_min) {
                return false;
            } else {
                return true;
            }
        }
    } else {
        return false;
    }
}

int LineFollowing_c::getLargerDNidx() {
    return larger_dn_idx;
}


char LineFollowing_c::turnDetected(char previousTurn) {
    int sensorValues[5];
    int gradients[4];
    // Use relaxed match criteria for non-sharp turns
    int numOnSensors = 0;
    for (int i = 0; i < 5; i++) {
        if (sensorDetected(i)) {
            numOnSensors++;
        }
    }
    for (int i = 0; i < 5; i++) {
        sensorValues[i] = getLineSensorValue(i); // Handle absent sensors
    }

    // Calculate gradients between consecutive sensors
    
    for (int i = 0; i < 4; i++) {
        gradients[i] = sensorValues[i + 1] - sensorValues[i];
    }

    if (numOnSensors == 5) {
        return 'S';
    }

    else if (sensorDetected(0)) {
        return 'L';
    } 

    else if (((sensorDetected(4)) && (!sensorDetected(0) && !sensorDetected(1))) && abs(gradients[3]) > SHARP_TURN_GRADIENT_THRESHOLD) {
        return 'R';
    }
    else {
        // Dead end detection and handling
        if (numOnSensors == 0) {
            previousTurn = 'N'; // Backtrack at dead ends
            return 'N';
        } 

        else if ((sensorDetected(1) && sensorDetected(2)) || (sensorDetected(2) && sensorDetected(3))) {
            return 'S';
        }

        // Maintain straight direction if no clear turn detected
        return 'S';
    }
}

// Debugging function
void LineFollowing_c:: printSensorValues() {
    Serial.print(ls_value[0]);
    Serial.print(" ");
    Serial.print(ls_value[1]);
    Serial.print(" ");
    Serial.print(ls_value[2]);
    Serial.print(" ");
    Serial.print(ls_value[3]);
    Serial.print(" ");
    Serial.print(ls_value[4]);
}
