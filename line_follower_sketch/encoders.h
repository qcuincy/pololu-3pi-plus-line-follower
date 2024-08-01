#include <Arduino.h>

#ifndef _ENCODERS_H
#define _ENCODERS_H

#define ENCODER_0_A_PIN  7
#define ENCODER_0_B_PIN  23
#define ENCODER_1_A_PIN  26
//#define ENCODER_1_B_PIN Non-standard pin!


// Volatile Global variables used by Encoder ISR.
extern volatile long count_right_encoder; // used by encoder to count the rotation
extern volatile byte state_right_encoder;
extern volatile long count_left_encoder;
extern volatile byte state_left_encoder;

// This ISR handles just Encoder 0
// ISR to read the Encoder0 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.
ISR( INT6_vect );

// This ISR handles just Encoder 1
// ISR to read the Encoder0 Channel A and B pins
// and then look up based on  transition what kind of
// rotation must have occured.
ISR( PCINT0_vect );

/*
   This setup routine enables interrupts for
   encoder1.  The interrupt is automatically
   triggered when one of the encoder pin changes.
   This is really convenient!  It means we don't
   have to check the encoder manually.
*/
void setupEncoder0();
void setupEncoder1();

// Encoder count function
// Returns the Encoder 0 Counts
long readEncoder0();

// Encoder count function
// Returns the Encoder 1 Counts
long readEncoder1();

long readEncoder(int encoder_pin);
#endif
