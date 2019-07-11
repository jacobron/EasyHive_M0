
/*****************************************************************************
 *
 * https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f 
 *
 */

#include "EasyHive.h"
#include "Timer.h"
#include <Arduino.h>

#define SAMPLE_RATE 4000
#define IN_MIC BAT_VOLT

uint32_t sampleRate = SAMPLE_RATE; //sample rate of the sine wave in Hertz, how many times per second the TC5_Handler() function gets called per second basically
bool state = 0; //just for an example
int audio[SAMPLE_RATE];
long audio_pnt = 0; 

void begin_sampling(){
    SerialUSB.begin(9600);
    tcConfigure(sampleRate); //configure the timer to run at <sampleRate>Hertz
    tcStartCounter(); //starts the timer
}

void end_sampling(){
    tcDisable();
    tcReset();
}

//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void) {
    //YOUR CODE HERE 


    audio[audio_pnt] = analogRead(IN_MIC);
    float audio_vlt = audio[audio_pnt] *  5.0 / 1024;
    SerialUSB.println(audio[audio_pnt]);
    audio_pnt++;
    if(audio_pnt == SAMPLE_RATE){
        audio_pnt = 0;
    }

    state = !state;
    // END OF YOUR CODE
    TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}

/* 
 *  TIMER SPECIFIC FUNCTIONS FOLLOW
 *  you shouldn't change these unless you know what you're doing
 */

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
 void tcConfigure(int sampleRate)
{
    // Enable GCLK for TCC2 and TC5 (timer counter input clock)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
    while (GCLK->STATUS.bit.SYNCBUSY);

    tcReset(); //reset TC5

    // Set Timer counter Mode to 16 bits
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
    // Set TC5 mode as match frequency
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
    //set prescaler and enable TC5
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
    //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
    TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
    while (tcIsSyncing());

    // Configure interrupt request
    NVIC_DisableIRQ(TC5_IRQn);
    NVIC_ClearPendingIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 0);
    NVIC_EnableIRQ(TC5_IRQn);

    // Enable the TC5 interrupt request
    TC5->COUNT16.INTENSET.bit.MC0 = 1;
    while (tcIsSyncing()); //wait until TC5 is done syncing 
} 

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
    return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
    while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset()
{
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (tcIsSyncing());
    while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
    TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (tcIsSyncing());
}
