#ifndef Timer_h
#define Timer_h

#include <Arduino.h>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

//starts timer and sampling
void begin_sampling();

//resets timer
void end_sampling();

//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void);

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
 void tcConfigure(int sampleRate);

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing();

//This function enables TC5 and waits for it to be ready
void tcStartCounter();

//Reset TC5 
void tcReset();

//disable TC5
void tcDisable();

#endif
