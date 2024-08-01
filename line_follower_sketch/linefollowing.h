#include <Arduino.h>
#include "linesensor.h"

#ifndef _LINEFOLLOWING_H
#define _LINEFOLLOWING_H

# define LINE_SENSOR_UPDATE 0

// Global definition of sensor minimum and maximum values for black (non ADC mode)
# define DN1_BLACK_MIN 1050
# define DN1_BLACK_MAX 2600
# define DN2_BLACK_MIN 1050
# define DN2_BLACK_MAX 2900
# define DN3_BLACK_MIN 1000
# define DN3_BLACK_MAX 2500
# define DN4_BLACK_MIN 1100
# define DN4_BLACK_MAX 3200
# define DN5_BLACK_MIN 1100
# define DN5_BLACK_MAX 3000

// Global definition of sensor minimum and maximum values for black (ADC mode)
# define DN1_BLACK_MIN_ADC 800
# define DN1_BLACK_MAX_ADC 960
# define DN2_BLACK_MIN_ADC 800
# define DN2_BLACK_MAX_ADC 960
# define DN3_BLACK_MIN_ADC 800
# define DN3_BLACK_MAX_ADC 960
# define DN4_BLACK_MIN_ADC 800
# define DN4_BLACK_MAX_ADC 960
# define DN5_BLACK_MIN_ADC 800
# define DN5_BLACK_MAX_ADC 960

# define DN1_AIR_MIN 1500
# define DN5_AIR_MIN 1500
# define DN1_AIR_MIN_ADC 1000
# define DN5_AIR_MIN_ADC 1000

#define SENSOR_THRESHOLD 1000 // Minimum reading to consider sensor "on black line"
#define SHARP_TURN_GRADIENT_THRESHOLD 1000 // Minimum gradient for significant turn
#define SHARP_TURN_GRADIENT_THRESHOLD_LEFT 900 // Minimum gradient for significant turn (left)
#define SOFT_TURN_GRADIENT_THRESHOLD 400 // Minimum gradient for soft turn
#define SOFT_TURN_GRADIENT_THRESHOLD_LEFT 150 // Minimum gradient for soft turn (left)
#define STRAIGHT_LINE_GRADIENT_THRESHOLD 1000 // Maximum gradient for straight line
#define DEAD_END_THRESHOLD 900 // Maximum sensor for dead end

extern float DN2_AIR_THRESHOLD;
extern float DN4_AIR_THRESHOLD;

extern LineSensor_c linesensor;

extern unsigned long elapsed_time, line_sensor_ts;

extern int dn1_min, dn1_max;
extern int dn2_min, dn2_max;
extern int dn3_min, dn3_max; 
extern int dn4_min, dn4_max;
extern int dn5_min, dn5_max;
extern int dn_mins[5];
extern int dn_maxs[5];
extern int dn1_air_min, dn5_air_min;
extern int dn2_actual_max, dn4_actual_max;
extern int larger_dn, larger_dn_idx;

class LineFollowing_c : public LineSensor_c {
  public:
    bool useADC;
    float prevSensorValues[5];
    bool detectedSensors[5];

    LineFollowing_c(bool useADC = false);
    void initialise();
    void waitUntilNonZero();
    void update(unsigned long _ls_update = LINE_SENSOR_UPDATE);
    float weightedMeasurement(bool useMax = false);
    bool lineDetected();
    bool sensorDetected(int idx);
    bool anySensorDetected();
    bool allNonZero();
    bool inAir();
    int getLargerDNidx();
    char turnDetected(char previousTurn);
    void printSensorValues();
};

#endif
