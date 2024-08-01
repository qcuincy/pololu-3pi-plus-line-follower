#include <Arduino.h>
// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

# define EMIT_PIN 11
# define LS_LEFT_PIN A11
# define LS_MIDLEFT_PIN A0
# define LS_MIDDLE_PIN A2
# define LS_MIDRIGHT_PIN A3
# define LS_RIGHT_PIN A4

# define LS_NUM_SENSORS 5

extern int ls_pins[LS_NUM_SENSORS];
extern int ls_value[5];
extern int ls_value_prev[5];

// Class to operate the linesensor(s).
class LineSensor_c {
  public:
  
    bool useADC;
    // Constructor, must exist.
    LineSensor_c();

    void initialise(bool useADC = false);
    void adcInitialise();
    void timedInitialise();
    void readLineSensor(int number);
    void readLineSensorADC(int number);
    void readLineSensorTimed(int number);
    int getLineSensorPin(int number);
    int getLineSensorValue(int number);
    void readAllSensors(bool test=false);
    void readAllSensorsADC();
    void readAllSensorsTimed();
    void readAllSensorsTimedOld();

};


#endif
