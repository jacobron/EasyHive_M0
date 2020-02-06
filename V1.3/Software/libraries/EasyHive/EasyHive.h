#ifndef EasyHive_h
#define EasyHive_h

#define EASYHIVE "0.0.1" 

#include <Arduino.h>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//pin assignments
#define CLK_PIN WEIGHT_CLK
#define DATA_PIN WEIGHT_DATA
#define PDWN WEIGHT_PDWN
#define TEMP WEIGHT_TEMP  
#define SELECT WEIGHT_SELECT 

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS SARA_TX_ENABLE

// text conversion functions
#define HEX_CHAR_TO_NIBBLE(c) ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0'))
#define HEX_PAIR_TO_BYTE(h, l) ((HEX_CHAR_TO_NIBBLE(h) << 4) + HEX_CHAR_TO_NIBBLE(l))

// pin definitions
#define powerPin 7 // this is a pin not used on the m0EasyHiveBoard, but needed for library functionality
#define enablePin 7 // this is a pin not used on the m0EasyHiveBoard, but needed for library functionality

#define DEBUG_STREAM SerialUSB
#define DEBUG_STREAM_BAUD 115200
#define MODEM_STREAM Serial1
#define STARTUP_DELAY 5000

// define udp msg length
#define STDSTRINGLEN 30

#define SENSOR_BUFFER_LEN 5

void init_LEDs(void);
void LEDs_off(void);
void LEDs_green(void);
void LEDs_red(void);
void LEDs_blue(void);

void init_RTC(void);
void init_server_data(void);
void init_BoardID(void);
void init_Weight(void);
void startads1232();
void stopads1232();
void init_Temp(void);

float get_Weight(void);
float get_Weight2(void);
long get_Weight_raw(void);

void set_Weight_calib(float val_c, float val_o, float val_cw);
void set_Weight_calib2(float val_c, float val_o, float val_cw);
void get_Weight_calib(float* val_c, float* val_o, float* val_cw2);
void get_Weight_calib2(float* val_c, float* val_o, float* val_cw2);

float get_Temp(void);
float getBatteryVoltage();
long get_time(void);

void enable_USB_in(void);
void serialEvent(void);
void check_USB_in(void);

bool sendMessageThroughUDP(const char param[STDSTRINGLEN]);
void checkMessage(const char msg[STDSTRINGLEN]);
int readNumber(String msg, uint8_t position);

void read_sens_value(float* weight1, float* weight2, float* temp, float* volt, int8_t* signal, long* package, int position);
void safe_sens_value(float weight1, float weight2, float temp, float volt, int8_t signal, long package);
int get_sens_pointer(int offset);

void pinStr( uint32_t ulPin, unsigned strength);

void powerdownfor(int i);
void initSleep();
void systemSleep();

#endif
