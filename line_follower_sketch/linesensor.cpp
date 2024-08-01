#include <Arduino.h>
#include "linesensor.h"

int ls_pins[LS_NUM_SENSORS] = {LS_LEFT_PIN, LS_MIDLEFT_PIN, LS_MIDDLE_PIN, LS_MIDRIGHT_PIN, LS_RIGHT_PIN};
// Global "permanent" store of line sensor values
int ls_value[5];
int ls_value_prev[5];

LineSensor_c::LineSensor_c() : useADC(true) {};

// Class to operate the linesensor(s).
void LineSensor_c::initialise(bool useADC) {
    this->useADC = useADC;
    if (useADC) {
        adcInitialise();
    } else {
        timedInitialise();
    }
};

void LineSensor_c::adcInitialise() {
    // Configure the central line sensor pin as input, and also activate the internal pull-up resistor.
    for (int i = 0; i < LS_NUM_SENSORS; i++) {
        pinMode( ls_pins[i], INPUT_PULLUP );
    }

    // Configure the EMIT pin as output and high.
    pinMode( EMIT_PIN, OUTPUT );
    digitalWrite( EMIT_PIN, HIGH );
};

void LineSensor_c::timedInitialise() {
    pinMode( EMIT_PIN, INPUT );
    for (int i = 0; i < LS_NUM_SENSORS; i++) {
        pinMode( ls_pins[i], INPUT );
    }
};

void LineSensor_c::readLineSensor(int number) {
    // Update previous values
    for (int i = 0; i < LS_NUM_SENSORS; i++) {
        ls_value_prev[i] = ls_value[i];
    }
    if (useADC) {
        readLineSensorADC(number);
    } else {
        readLineSensorTimed(number);
    }
};

void LineSensor_c::readLineSensorADC(int number) {
    ls_value[ number ] = analogRead( ls_pins[ number ] );
};

void LineSensor_c::readLineSensorTimed(int number) {
    float result;

    if( number < 0 || number > 4 ) {
        return;
    }
    

    // Complete the steps referring to the pseudocode block
    // Algorithm 1.
    // The first steps have been done for you.
    // Fix parts labelled ????
    // Some steps are missing - add these.
    pinMode( EMIT_PIN, OUTPUT );
    digitalWrite( EMIT_PIN, HIGH );

    // In this line, we retrieve the pin value
    // stored in the array "ls_pins" at location
    // "number".  So it is like a look-up table.
    // We can think of ls_pins in memory like:
    // Index 0, Index 1, Index 2, Index 3, Index 4
    //[  DN1  ][   DN2 ][  DN3  ][  DN4  ][  DN5  ]
    pinMode( ls_pins[ number ], OUTPUT );
    digitalWrite( ls_pins[ number ], HIGH );

    delayMicroseconds( 10 );

    unsigned long start_time = micros();

    pinMode( ls_pins[ number ], INPUT );

    while( digitalRead( ls_pins[ number ] ) == HIGH ) {
        // Do nothing here (waiting).

        if ( micros() - start_time > 1000000){
            // Serial.println("TIMEOUT");
            break;
        }
    }
    unsigned long end_time = micros();
    unsigned long elapsed_time = end_time - start_time;

    pinMode( EMIT_PIN, INPUT );

    result = (float)elapsed_time;
    ls_value[ number ] = (int)result;
};

int LineSensor_c::getLineSensorPin(int number) {
    if( number < 0 || number > 4 ) {
        return -1;
    }
    return ls_pins[number];
};

int LineSensor_c::getLineSensorValue(int number) {
    if( number < 0 || number > 4 ) {
        return -1;
    }
    return ls_value[number];
};

void LineSensor_c::readAllSensors(bool test) {
    // Update previous values
    for (int i = 0; i < LS_NUM_SENSORS; i++) {
        ls_value_prev[i] = ls_value[i];
    }
    if (useADC) {
        readAllSensorsADC();
    } 
    else {
        if (!test) {
            readAllSensorsTimed();
        } else {
            readAllSensorsTimedOld();
        }
    }
};

void LineSensor_c::readAllSensorsADC() {
    for( int i = 0; i < LS_NUM_SENSORS; i++ ) {
        readLineSensorADC(i);
    }
}

void LineSensor_c::readAllSensorsTimed() {
    for( int i = 0; i < LS_NUM_SENSORS; i++ ) {
        readLineSensorTimed(i);
    }
};

void LineSensor_c::readAllSensorsTimedOld() {
    pinMode( EMIT_PIN, OUTPUT );
    digitalWrite( EMIT_PIN, HIGH );

    for( int i = 0; i < LS_NUM_SENSORS; i++ ) {
        pinMode( ls_pins[ i ], OUTPUT );
        digitalWrite( ls_pins[ i ], HIGH );
    }
    delayMicroseconds( 10 );

    for (int i=0; i<LS_NUM_SENSORS; i++) {
        pinMode( ls_pins[ i ], INPUT );
    }

    unsigned long start_time = micros();

    int sensorsStillReading = LS_NUM_SENSORS;

    while( sensorsStillReading > 0 ) {
        for( int i = 0; i < LS_NUM_SENSORS; i++ ) {
            if( digitalRead( ls_pins[ i ] ) == LOW ) {
                unsigned long end_time = micros();
                unsigned long elapsed_time = end_time - start_time;
                float result = (float)elapsed_time;
                ls_value[ i ] = (int)result;
                sensorsStillReading--;
            }
        }

        if ( micros() - start_time > 1000000){
            break;
        }
    }
    pinMode( EMIT_PIN, INPUT );
};
