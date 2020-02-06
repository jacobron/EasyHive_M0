// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include "EasyHive.h"
#include <Arduino.h>
#include "OneWire.h"
#include "DallasTemperature.h"
#include "ADS1231.h"
#include "FlashStorage.h"
#include "Sodaq_nbIOT.h"
#include "Sodaq_wdt.h"
#include "rtttl.h"
#include "xxtea.h"
#include "base64.h"
#include <stdio.h>
#include <RTCZero.h>

// songs for the buzzer system
char *success = "Superman:d=4,o=6,b=200:8d5,8d5,8d5,8g.5,16p,8g5,2d,8p,8d,8e,8d,8c,1d";
char *rock = "we-rock:d=4,o=6,b=160:32d5,32e5,32f#5,32g5,32a5,32b5,32c#6,32d6,32c#6,32b5,32a5,32g5,32f#5,32e5,32d5";
char *bad = "Death March:d=4,o=5,b=125:c.,c,8c,c.,d#,8d,d,8c,c,8c,2c.";
char *countdown = "Final:d=4,o=5,b=125:16c#6,16b,c#6,f#,p.,16d6,16c#6,8d6,8c#6,b,p.,16d6,16c#6,d6,f#,p.,16b,16a,8b,8a,8g#,8b,a.,16c#6,16b,c#6,";
char *countdown2 = "Final2:f#,p.,16d6,16c#6,8d6,8c#6,b,p.,16d6,16c#6,d6,f#,p.,16b,16a,8b,8a,8g#,8b,a.,16g#,16a,b.,16a,16b,8c#6,8b,8a,";
char* countdown3 = "8g#,f#,d6,2c#.6,16c#6,16d6,16c#6,16b,1c#6,2p";


/* *************** Important Notice: *****************************
you need to specify your own network-options:                    *
* 1. const char *key = "[to be defined]";                        *
* 2. const char* apn = "[CHECK YOUR SIMCARD PROVIDER]";          *
* 3. const char* forceOperator = "5-digits";                        *
* 4. server = {false, "[YOURSERVERIP]", "[YOURSERVERPORT]"};     *
*                                                                *
* ***************************************************************/

// Set the Password for encryption
const char *key = "[to be defined]";

// Set the BoardID variable
int BoardID = 1;

// Set intervals for data logging and data sending [seconds] // due to watchdogtimer they result in multiples of 8
int datasendtime =  60;
int datalogtime = 60;

// Operator variables
const char* apn = "[CHECK YOUR SIMCARD PROVIDER]"; // DE: iot.1nce.net NL: cdt.iot.t-mobile.nl
const char* cdp = "";
unsigned char cid = 1;
const char* forceOperator = "26201"; // optional - depends on SIM / network //26201 Telekom Germany 20416 Telekom NL
const char band = 8;

// Server variables
// define Server struct and init variables
struct Serverdata{
  boolean isSet;
  char ip[16];
  uint16_t port;
} server = {false, "[YOURSERVERIP]", "[YOURSERVERPORT]"};

// RTC Time object
RTCZero rtc;

// Sodaq NBIOT Class init
Sodaq_nbIOT nbiot;

unsigned long temp_value;
unsigned long weight_value;

//float weight_calib = 8388607.0;
//float weight_offset = 4.2;

float weight_calib = 1.0;
float weight_offset = 1.0;
float weight_calib2 = 1.0;
float weight_offset2 = 1.0;

//calibration weights in gram
int calibration_weight = 2000;
int calibration_weight2 = 1000;
int tare_weight = 0;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
//Weight ADS
ADS1231 adc(CLK_PIN,DATA_PIN);

// Flash MEMORY ASSIGNMENT

// Reserve a portion of flash memory to store an "float" variable
// and call it "flash_c".
FlashStorage(flash_c, float);
FlashStorage(flash_o, float);
FlashStorage(flash_cw, float);
FlashStorage(flash_c2, float);
FlashStorage(flash_o2, float);
FlashStorage(flash_cw2, float);
FlashStorage(flash_calibrated, bool);
FlashStorage(flash_calibrated2, bool);
FlashStorage(flash_timeset, bool);
FlashStorage(flash_sendtime,int);
FlashStorage(flash_logtime, int);
FlashStorage(flash_boardid,int);
FlashStorage(flash_boardid_set, bool);
FlashStorage(flash_server, struct Serverdata);




struct EEPROM_field {
  float value;
  char name[10];
};

void enable_USB_in(void){
    // initialize serial:
    SerialUSB.begin(9600);
    // reserve 200 bytes for the inputString:
    inputString.reserve(200);
    return; 
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/

void serialEvent(void) {
    while (SerialUSB.available()) {
        // get the new byte:
        char inChar = (char)SerialUSB.read();
        // add it to the inputString:
        inputString += inChar;
        // if the incoming character is a newline, set a flag so the main loop can
        // do something about it:
        if (inChar == '\n') {
            stringComplete = true;
        }
    }
    return;
}

void check_USB_in(void){
    if (stringComplete) {
        SerialUSB.println(inputString);
        // clear the string:
        inputString = "";
        stringComplete = false;
    }
    return;   
}

void init_LEDs(void){
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);  

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    return; 
}

void LEDs_off(void){
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    return;
}

void LEDs_green(void){
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);
    return;
}

void LEDs_red(void){
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    return;
}

void LEDs_blue(void){
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
    return;
}
void init_RTC(void){
	rtc.begin();
}

long get_time(void){
	long time = rtc.getEpoch();
	// maybe check time here if it makes sense?
	return time;
}

void init_server_data(void){
	// set server_data to standard if not set yet
	struct Serverdata serverbuffer = flash_server.read();
    
    if(serverbuffer.isSet == true){
		SerialUSB.println("server_data is set");
		server = serverbuffer;
	}
	else{
		SerialUSB.println("server_data is not set - initializing with standard data"); 
	}
	 SerialUSB.println(server.ip);
     SerialUSB.println(server.port);
}

void init_BoardID(void){
	// set BoardID to standard if not set yet
	bool boardset = flash_boardid_set.read();

    //needs to be calibrated at least once
    if(boardset == true){
        // 
        BoardID = flash_boardid.read();
        SerialUSB.println("board_id is set");
        SerialUSB.println(BoardID);
    }
    else{
		SerialUSB.println("BoardID is not set yet - setting it to standard");     
        BoardID = 1; 
        SerialUSB.println(BoardID);
    }
}

/**
  Initializes the CPU sleep mode.
*/
void initSleep()
{
	// Set the sleep mode
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  
	// set sleeptime to standard if not set yet
	bool timeset = flash_timeset.read();

	//needs 
	if(timeset == true){ 
		datasendtime = flash_sendtime.read();
		datalogtime = flash_logtime.read();
		SerialUSB.println("time is set");
		SerialUSB.println(datalogtime);
		SerialUSB.println(datasendtime);
	}
	else{
		SerialUSB.println("Logtime is not set yet - setting it to standard");     
		SerialUSB.println(datalogtime);
		SerialUSB.println(datasendtime);
	}
	/*
	SerialUSB.flush();
	SerialUSB.end();
	USBDevice.detach();
	USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; // Disable USB
	*/
}


void init_Temp(void){
    SerialUSB.println("Dallas Temperature IC Control Library Demo");
    // Start up the library
    sensors.begin();
    return; 
}

void init_Weight(void){
    SerialUSB.println("Init ADS1232");
    pinMode(PDWN, OUTPUT);
    digitalWrite(PDWN, HIGH);

    pinMode(TEMP, OUTPUT);
    digitalWrite(TEMP, LOW);
    
    pinMode(SELECT, OUTPUT);
    digitalWrite(SELECT, LOW);

    if (!adc.begin()) {
        Serial.println("Invalid pins bro!");
    }    
    return; 
}

void set_Weight_calib(float val_c, float val_o, float val_cw){

    // Save into "flash_c"
    flash_c.write(val_c);
    // Save into "flash_o"
    flash_o.write(val_o);
    flash_cw.write(val_cw);

    //needs to be calibrated at least once
    flash_calibrated.write(true);

    weight_calib = val_c; 
    weight_offset = val_o; 
    calibration_weight = val_cw;
    return;
}

void set_Weight_calib2(float val_c, float val_o, float val_cw){

    // Save into "flash_c"
    flash_c2.write(val_c);
    // Save into "flash_o"
    flash_o2.write(val_o);
    flash_cw2.write(val_cw);

    //needs to be calibrated at least once
    flash_calibrated2.write(true);

    weight_calib2 = val_c; 
    weight_offset2 = val_o; 
    calibration_weight2 = val_cw;
    return;
}

void get_Weight_calib(float* val_c, float* val_o, float* val_cw){

    // Read the content of "flash_c" and assign it to "weight_calib"
    bool calib = flash_calibrated.read();

    //needs to be calibrated at least once
    if(calib == true){
        // Read the content of "flash_c" and assign it to "weight_calib"
        weight_calib = flash_c.read();
        // Read the content of "flash_o" and assign it to "weight_offset"
        weight_offset = flash_o.read();
        calibration_weight = flash_cw.read();
    }
    else{
        set_Weight_calib(1.0, 1.0, 1000);
        // Read the content of "flash_c" and assign it to "weight_calib"
        weight_calib = flash_c.read();
        // Read the content of "flash_o" and assign it to "weight_offset"
        weight_offset = flash_o.read();
        calibration_weight = flash_cw.read();
    }

    SerialUSB.print("Offset: ");
    SerialUSB.println(weight_offset);
    SerialUSB.print("Calibration: ");
    SerialUSB.println(weight_calib);
      SerialUSB.print("Calibration Weight: ");
    SerialUSB.println(calibration_weight);

    SerialUSB.print("Calibration done?: ");
    SerialUSB.println(calib);

    *val_c = weight_calib;
    *val_o = weight_offset; 
    *val_cw = calibration_weight;
    return; 
}

void get_Weight_calib2(float* val_c, float* val_o, float* val_cw2){

    // Read the content of "flash_c" and assign it to "weight_calib"
    bool calib = flash_calibrated2.read();

    //needs to be calibrated at least once
    if(calib == true){
        // Read the content of "flash_c" and assign it to "weight_calib"
        weight_calib2 = flash_c2.read();
        // Read the content of "flash_o" and assign it to "weight_offset"
        weight_offset2 = flash_o2.read();
        calibration_weight2 = flash_cw2.read();
    }
    else{
        set_Weight_calib2(1.0, 1.0, 1000);
        // Read the content of "flash_c" and assign it to "weight_calib"
        weight_calib2 = flash_c2.read();
        // Read the content of "flash_o" and assign it to "weight_offset"
        weight_offset2 = flash_o2.read();
        calibration_weight2 = flash_cw2.read();
    }

    SerialUSB.print("Offset2: ");
    SerialUSB.println(weight_offset2);
    SerialUSB.print("Calibration2: ");
    SerialUSB.println(weight_calib2);
    SerialUSB.print("Calibration_weight2: ");
    SerialUSB.println(calibration_weight2);

    SerialUSB.print("Calibration2 done?: ");
    SerialUSB.println(calib);

    *val_c = weight_calib2; 
    *val_o = weight_offset2; 
    *val_cw2 = calibration_weight2;
    return; 
}

float get_Temp(void){
    // call sensors.requestTemperatures() to issue a global temperature 
    // request to all devices on the bus
    //SerialUSB.print("Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    //SerialUSB.println("DONE");
    // After we got the temperatures, we can print them here.
    // We use the function ByIndex, and as an example get the temperature from the first sensor only.
    //SerialUSB.print("Temperature for the device 1 (index 0) is: ");
    //SerialUSB.println(sensors.getTempCByIndex(0));  
    return sensors.getTempCByIndex(0); 
}

long get_Weight_raw(void){
	
	
    long val;
    if(adc.getValue(val)){
		
		return val; 
	}
        
}

void startads1232(){
	//start ads1232
	digitalWrite(LC_ON,HIGH);
	digitalWrite(PDWN, HIGH);
	delay(20);
	long fakeval;
	adc.getValue(fakeval);
}

void stopads1232(){
	digitalWrite(LC_ON,LOW);
	digitalWrite(PDWN, LOW);
}

float get_Weight(void){
	
	
	
    // delay(10);

    //Datasheet ADS1232 page 8 -> w = m * val + wzs - calibration_weight

    //m = (calibration_weight / (weight_calib - weight_offset)) 
    //wzs = -m * weight_offset

    long val1;
    if(adc.getValue(val1)){        //this call blocks until a sample is ready!
    //  SerialUSB.print(millis());
    //  SerialUSB.print(",");
    //SerialUSB.println((4.2/8388607.0)*val, 10);  //23 bits of accuracy (24th is sign) with 4.2 volt (measured) AVDD. 2^23 = 8388607 !
    } 
    else{
        SerialUSB.println("Failed to get data");
    }
    //delay(500);
    
    unsigned long w_tare = 0;

    float result = 0; 
    float m = (calibration_weight / (weight_calib - weight_offset));
    float wzs = (-m) * weight_offset;
    result = m * val1 + wzs - tare_weight;
    
    SerialUSB.println("WeightMeasure:");
    SerialUSB.println(val1);
    SerialUSB.println(calibration_weight);
    SerialUSB.println(weight_calib);
    SerialUSB.println(weight_offset);
    
    // result = (val + weight_offset)*weight_calib;
    // weight_offset is val@0g
    // weight_calib is (val@calib-val@0g)*calibration_weight
    
	
    return result; 
}

float get_Weight2(void){
	
	
	
    // delay(10);

    //Datasheet ADS1232 page 8 -> w = m * val + wzs - calibration_weight

    //m = (calibration_weight / (weight_calib - weight_offset)) 
    //wzs = -m * weight_offset

    long val2;
    if(adc.getValue(val2)){        //this call blocks until a sample is ready!
    //  SerialUSB.print(millis());
    //  SerialUSB.print(",");
    //SerialUSB.println((4.2/8388607.0)*val, 10);  //23 bits of accuracy (24th is sign) with 4.2 volt (measured) AVDD. 2^23 = 8388607 !
    } 
    else{
        //SerialUSB.println("Failed to get data");
    }
    //delay(500);
    
    unsigned long w_tare = 0;

    float result = 0; 
    float m = (calibration_weight2 / (weight_calib2 - weight_offset2));
    float wzs = (-m) * weight_offset2;
    result = m * val2 + wzs - tare_weight;
    
    SerialUSB.println("WeightMeasure2:");
    SerialUSB.println(val2);
    SerialUSB.println(calibration_weight2);
    SerialUSB.println(weight_calib2);
    SerialUSB.println(weight_offset2);
    
    // result = (val + weight_offset)*weight_calib;
    // weight_offset is val@0g
    // weight_calib is (val@calib-val@0g)*calibration_weight
    
	
    return result; 
}



float getBatteryVoltage(){
	float R16 = 4700; // see board design files for reference
	float R17 = 10000; 
	float res = 1024;
	float AIN = analogRead(A0);
	DEBUG_STREAM.print("BatteryAIN:");
	DEBUG_STREAM.println(AIN);
	float V = ((R16+R17)/R17)*AIN*3.3/res; // 3,3V is operation voltage of the chip an maximum in
	return V;
}

/******************* NBIOT Functions ********************/

void checkMessage(const char msg[STDSTRINGLEN])
{
	DEBUG_STREAM.println("checking message...");    
	DEBUG_STREAM.println(msg);
	char* buffer;
	strcpy(buffer,msg);
	String data = buffer;
	DEBUG_STREAM.println(data);
	
    if(data == "OK!") // OK!
    {
      DEBUG_STREAM.println("OK! nothing todo");
      play_rtttl(success);
    }
    else if(data.length() > 0) // something was transmitted..
    {
      if(data.indexOf('t') >= 0) // Tare command?
      {
       DEBUG_STREAM.println("tare!");
      }
      if(data.indexOf('W') >= 0) // Weight definition command?
      {
       DEBUG_STREAM.println("Weight");
       uint8_t i = data.indexOf('W')+1;
       int j = 0;
       char weightbuffer[10];
       while(isDigit(data[i]))
       {
        weightbuffer[j] = data[i];
        i++;
        j++;
       }
       weightbuffer[j] = '\0';
       j = atoi(weightbuffer);
       DEBUG_STREAM.println(weightbuffer);
       DEBUG_STREAM.println(j);
       DEBUG_STREAM.println("g");
      }
    }
    else
    {
      play_rtttl(bad);
      DEBUG_STREAM.println("not OK!");
    }
}

bool sendMessageThroughUDP(const char param[STDSTRINGLEN])
{
    DEBUG_STREAM.println();
    DEBUG_STREAM.println("Sending message through UDP");

    int localPort = 16666;
    int socketID = nbiot.createSocket(localPort);

    if (socketID >= 7 || socketID < 0) {
        DEBUG_STREAM.println("Failed to create socket");
        return "SOCKETERROR";
    }

    DEBUG_STREAM.println("Created socket!");

    // const char* strBuffer = "1,50,49";
    const char *text = param;
    size_t len;
    const unsigned char *encrypt_data = (const unsigned char*)xxtea_encrypt(text, strlen(text), key, &len);
    char * base64_data = base64_encode(encrypt_data, len);
    
    const char* strBuffer = base64_data;
    DEBUG_STREAM.println(strBuffer);
    size_t size = strlen(strBuffer);

    // send data
    DEBUG_STREAM.print("sending to: ");
    DEBUG_STREAM.println(server.ip);
    int lengthSent = nbiot.socketSend(socketID, server.ip, server.port, strBuffer); // "195.34.89.241" : 7 is the ublox echo service

    DEBUG_STREAM.print("String length vs sent: ");
    DEBUG_STREAM.print(size);
    DEBUG_STREAM.print(" vs ");
    DEBUG_STREAM.println(lengthSent);
	
	String msg = "";
    // wait for data
    if (nbiot.waitForUDPResponse()) {
        DEBUG_STREAM.println("Received response!");
        while (nbiot.hasPendingUDPBytes()) {
            char data[10];
            
            
            // read two bytes at a time
            SaraN2UDPPacketMetadata p;
            int size = nbiot.socketReceiveHex(data, 2, &p);

            if (size) {
                /*DEBUG_STREAM.write(HEX_PAIR_TO_BYTE(data[0],data[1]));
                // p is a pointer to memory that is owned by nbiot class
                DEBUG_STREAM.println(char(HEX_PAIR_TO_BYTE(data[0],data[1])));
                DEBUG_STREAM.println(p.socketID);
                DEBUG_STREAM.println(p.ip);
                DEBUG_STREAM.println(p.port);
                DEBUG_STREAM.println(p.length);
                DEBUG_STREAM.println(p.remainingLength);
                */
                
                msg += char(HEX_PAIR_TO_BYTE(data[0],data[1]));
            }
            else {
                DEBUG_STREAM.println("Receive failed!");
            }
        }
       DEBUG_STREAM.println("Done checking message");
    }
    else {
        DEBUG_STREAM.println("Timed-out!");
        msg = "timeout";
    }

    nbiot.closeSocket(socketID);
    DEBUG_STREAM.println();
    
   //************* // check message // ********************** //
    
    DEBUG_STREAM.println("checking message...");    
	DEBUG_STREAM.println(msg);
	DEBUG_STREAM.println("message decrypted:");
	
	encrypt_data = (const unsigned char*)base64_decode(msg.c_str(), &len);
	char * decrypt_data =  (char*)xxtea_decrypt(encrypt_data, len, key, &len);
	DEBUG_STREAM.println(decrypt_data);
    msg = decrypt_data;
	
	
    
    if(msg.length() > 0) // something was transmitted..
    {
	  struct Serverdata serverbuffer = server;
	  bool server_altered = false;  // bool flag for server data changing
	  bool interval_altered = false;
	  if(msg == "OK!") // OK!
	  {
      DEBUG_STREAM.println("OK! nothing todo");
	  }
      if(msg.indexOf('a') >= 0) // Tare command for load Cell A
      {
	   DEBUG_STREAM.println("tare A!");
	   digitalWrite(SELECT, LOW); // Select A
	   delay(1000);
	   get_Weight_raw(); // settle it
	   weight_offset = static_cast<float>(get_Weight_raw());
	   DEBUG_STREAM.print("New Weight Offset:");
       DEBUG_STREAM.print(weight_offset);
	   set_Weight_calib(weight_calib,weight_offset,calibration_weight);
	   sendMessageThroughUDP("tared A");
       
      }
      if(msg.indexOf('A') >= 0) // Weight definition for load Cell A
      {
	   digitalWrite(SELECT, LOW); // Select A
	   play_rtttl(countdown);
       DEBUG_STREAM.println("Weight on Load Cell A:");
       calibration_weight = readNumber(msg,msg.indexOf('A')+1);
       DEBUG_STREAM.print(calibration_weight);
       DEBUG_STREAM.println("g");
       
       get_Weight_raw(); // settle it
       weight_calib = static_cast<float>(get_Weight_raw());
       DEBUG_STREAM.print("New Weight Calib:");
       DEBUG_STREAM.print(weight_calib);
       set_Weight_calib(weight_calib,weight_offset,calibration_weight);
       
      }
      if(msg.indexOf('b') >= 0) // Tare command for load Cell B
      {
       DEBUG_STREAM.println("tare B!");
	   digitalWrite(SELECT, HIGH); // Select B
	   delay(1000);
	   DEBUG_STREAM.println("RAW WEIGHT");
	   DEBUG_STREAM.println(get_Weight_raw()); // settle it
	   weight_offset2 = static_cast<float>(get_Weight_raw());
	   DEBUG_STREAM.print("New Weight Offset:");
       DEBUG_STREAM.print(weight_offset2);
	   set_Weight_calib2(weight_calib2,weight_offset2,calibration_weight2);
	   sendMessageThroughUDP("tared B");
      }
      if(msg.indexOf('B') >= 0) // Weight definition for load Cell B
      {

	   digitalWrite(SELECT, HIGH); // Select B
	   play_rtttl(countdown);
       DEBUG_STREAM.println("Weight on Load Cell B:");
       calibration_weight2 = readNumber(msg,msg.indexOf('B')+1);
       DEBUG_STREAM.print(calibration_weight2);
       DEBUG_STREAM.println("g");  
       get_Weight_raw(); // settle it
       weight_calib2 = static_cast<float>(get_Weight_raw());
       DEBUG_STREAM.print("New Weight Calib2:");
       DEBUG_STREAM.print(weight_calib2);
       set_Weight_calib2(weight_calib2,weight_offset2,calibration_weight2);
             }
      if(msg.indexOf('P') >= 0) // New Port to send to
      {   
       uint16_t j = (uint16_t)readNumber(msg,msg.indexOf('P')+1); 
       server.port = j;
       if (server.port != serverbuffer.port){
		   DEBUG_STREAM.print("New Port Number:");
		   DEBUG_STREAM.println(j);
			server_altered = true;
		}
       // TODO: Check if it's the same already and don't write to flash -> prevent too many writings to flash
       // TODO: send test message and if it works keep on going - if not fall back to the old one
      }
      if(msg.indexOf('I') >= 0) // New IP Address to send to
      {
	   DEBUG_STREAM.print("Old IP Address: ");
	   DEBUG_STREAM.println(server.ip);
       DEBUG_STREAM.print("New IP Address: ");
		uint8_t i = msg.indexOf('I')+1;
		uint8_t j = 0;
		char buffer[16];
		while(isDigit(msg[i]) || msg[i] == '.')
		{
			buffer[j] = msg[i];
			i++;
			j++;
		}
		buffer[j] = '\0';
		memcpy(server.ip, buffer, 16);
		DEBUG_STREAM.println(server.ip);
		if (strcmp(server.ip,serverbuffer.ip)){
			server_altered = true;
			DEBUG_STREAM.println("IP change detected!");
		}
		else{
			DEBUG_STREAM.println("already the same");
		}
		// TODO: Check if it's the same IP already and don't write to flash
		// TODO: send test message and if it works keep on going - if not fall back to the old one
		// WATCH OUT IF IP AND PORT CHANGE AT SAME TIME
      }
      
      if(server_altered == true) // if something in serverdata was altered
      {
		// do connection check
		if(sendMessageThroughUDP("check")){
			server.isSet = true;
			flash_server.write(server);
			DEBUG_STREAM.println("changed to new Server address");
			DEBUG_STREAM.println(server.ip);
		}else{
			server = serverbuffer;
			DEBUG_STREAM.println("Connection failed - keep the old address:");
			DEBUG_STREAM.println(server.ip);
		}	
	  }
	  
      if(msg.indexOf('U') >= 0) // Assigning new BoardID
      {
       DEBUG_STREAM.println("New BoardID:");
       int j = readNumber(msg,msg.indexOf('U')+1);
       if(BoardID != j){ // prevent too many writes to flash
			// Save into "flash_boardid"
			BoardID = j;
			flash_boardid.write(BoardID);
			flash_boardid_set.write(true);
			DEBUG_STREAM.print(BoardID);
	   }

      }
      if(msg.indexOf('F') >= 0) // Sending interval in seconds
      {
		int j = readNumber(msg,msg.indexOf('F')+1);
		if(datasendtime != j){
			datasendtime = j;
			DEBUG_STREAM.println("New sendtime");
			interval_altered = true;
		}
      }
      if(msg.indexOf('D') >= 0) // Datalogging interval in seconds
      {
       int j = readNumber(msg,msg.indexOf('D')+1);
	   if(datalogtime != j){
			datalogtime = j;
			DEBUG_STREAM.println("New logtime");
			interval_altered = true;
		}
      }
      if(interval_altered){ // if anything changed in the time setting
		  if(datalogtime > datasendtime) // prevent the bad idea to send more often than to measure
		  {
			  datalogtime = datasendtime;
		  }
		  flash_logtime.write(datalogtime);
		  flash_sendtime.write(datasendtime);
		  flash_timeset.write(true);
		  DEBUG_STREAM.println("New sending and logging interval wrote to flash");
	  }
	  if(msg.indexOf('T') >= 0){ // RTC information sent 
		long j = readNumber(msg,msg.indexOf('T')+1);
		rtc.setEpoch(j); // set RTC time
	  }
	  
      if(msg.indexOf('S') >= 0) // Play a nice song
      {
       play_rtttl(success);
      }
      return 1;
    }
    else
    {
      DEBUG_STREAM.println("not OK!");
      play_rtttl(bad);
	  return 0;
    }
}

int readNumber(String msg, uint8_t position) {
	char buffer[20];
	uint8_t j = 0;
	while(isDigit(msg[position]))
       {
        buffer[j] = msg[position];
        position++;
        j++;
       }
   buffer[j] = '\0';
   return atoi(buffer);
}

//create ringbuffer for sensor data
//could use only 1 pointer if all 3 arrays share the same length

struct Sensordata{
	float weight1[SENSOR_BUFFER_LEN] = {0}; 
	float weight2[SENSOR_BUFFER_LEN] = {0}; 
	float temp[SENSOR_BUFFER_LEN] = {0};
	float volt[SENSOR_BUFFER_LEN] = {0};
	int8_t signal[SENSOR_BUFFER_LEN] = {0};
	long epochtime[SENSOR_BUFFER_LEN] = {0};
	int pointer = -1;
	
} sensorbuffer;


void safe_sens_value(float weight1, float weight2, float temp, float volt, int8_t signal, long epochtime){
	
	sensorbuffer.pointer++;
	if(sensorbuffer.pointer == SENSOR_BUFFER_LEN){
		sensorbuffer.pointer = 0;
	}
	
	sensorbuffer.weight1[sensorbuffer.pointer] = weight1;
	sensorbuffer.weight2[sensorbuffer.pointer] = weight2;
	sensorbuffer.temp[sensorbuffer.pointer] = temp;
	sensorbuffer.volt[sensorbuffer.pointer] = volt;
	sensorbuffer.signal[sensorbuffer.pointer] = signal;
	sensorbuffer.epochtime[sensorbuffer.pointer] = epochtime;

	
	

}

//make sure that position is valid by using get_sens_pointer with offset
void read_sens_value(float* weight1, float* weight2, float* temp, float* volt, int8_t* signal, long* epochtime, int position){
	*weight1 = sensorbuffer.weight1[position]; 
    *weight2 = sensorbuffer.weight2[position]; 
	*temp = sensorbuffer.temp[position]; 
	*volt = sensorbuffer.volt[position];
	*signal = sensorbuffer.signal[position];
	*epochtime = sensorbuffer.epochtime[position];
	
    return; 
}

int get_sens_pointer(int offset){
	int ans;
	
	if(offset == 0)
		return sensorbuffer.pointer;
	else{
		ans = sensorbuffer.pointer + offset; 
		if(ans < 0){
			ans = SENSOR_BUFFER_LEN + ans; 
		}
		else if(ans >= SENSOR_BUFFER_LEN){
			ans = ans - SENSOR_BUFFER_LEN; 
		}
		return ans; 
    }
}

void pinStr( uint32_t ulPin, unsigned strength) // works like pinMode(), but to set drive strength
{
  // Handle the case the pin isn't usable as PIO
  if ( g_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN )
  {
    return ;
  }
  if(strength) strength = 1;      // set drive strength to either 0 or 1 copied
  PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].bit.DRVSTR = strength ;
}


void powerdownfor(int i){
	// Power-Down ADS1232
    digitalWrite(PDWN, LOW);
    
    
	for (int j = 0; j < i/8 ;j++){
		sodaq_wdt_reset(); // resetting the watchdog
		systemSleep();
	}
}



/**
  Powers down all devices and puts the system to deep sleep.
*/
void systemSleep()
{

  __WFI(); // SAMD sleep

}
/** Note: When your board is in sleep, or does not react anymore. You can get it back to life to double press the reset button.
*/
