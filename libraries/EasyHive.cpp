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

// songs for the buzzer system
char *success = "Superman:d=4,o=6,b=200:8d5,8d5,8d5,8g.5,16p,8g5,2d,8p,8d,8e,8d,8c,1d";
char *rock = "we-rock:d=4,o=6,b=160:32d5,32e5,32f#5,32g5,32a5,32b5,32c#6,32d6,32c#6,32b5,32a5,32g5,32f#5,32e5,32d5";
//char *song = "Superman:d=4,o=6,b=200:8d5,8d5,8d5,8g.5,16p,8g5,2d,8p,8d,8e,8d,8c,1d,8p,8d5,8d5,8d5,8g.5,16p,8g5,2d,8d,8d,8e,8c,8g5,8e,2d.,p,8g5,8g5,8g5,2f#.,d.,8g5,8g5,8g5,2f#.,d.,8g5,8g5,8g5,8f#,8e,8f#,2g.,8g5,8g5,8g5,2g.5";
// char *song = "Happy Song:d=4,o=6,b=125:16a#5,16f5,16a#5,16c#,16a#5,16f5,16a#5,16c#,16a#5,16f5,16a#5,16c#,16a#5,16f5,16a#5,16c#,16c#,16g#5,16c#,16f,16c#,16g#5,16c#,16f,16c#,16g#5,16c#,16f,16c#,16g#5,16c#,16f,16f#5,16c#5,16f#5,16a#5,16f#5,16c#5,16f#5,16a#5,16f#5,16c#5,16f#5,16a#5,16f#5,16c#5,16f#5,16a#5,16d#5,16a#4,16d#5,16f#5,16d#5,16a#4,16d#5,16f#5,16f5,16c5,16f5,16a5,16f5,16c5,16f5,16a5,16a#5,8f.5";


// Set the Password for encryption
const char *key = "password";

// Set the BoardID variable
int BoardID = 1;

// Set intervals for data logging and data sending [seconds] // due to watchdogtimer they result in multiples of 8
int datasendtime = 20;
int datalogtime = 2;

// Operator variables
const char* apn = "iot.1nce.net";
const char* cdp = "";
unsigned char cid = 1;
const char* forceOperator = "26201"; // optional - depends on SIM / network
const char band = 8;

// Server variables
// define Server struct and init variables
struct Serverdata{
  boolean isSet;
  char ip[16];
  uint16_t port;
} server = {false, "000.000.000.000", 2222};



// Sodaq NBIOT Class init
Sodaq_nbIOT nbiot;

unsigned long temp_value;
unsigned long weight_value;

//float weight_calib = 8388607.0;
//float weight_offset = 4.2;

float weight_calib = 1.0;
float weight_offset = 1.0;

//calibration weights in gram
const unsigned long calibration_weight = 1000;
const unsigned long tare_weight = 0;

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
FlashStorage(flash_calibrated, bool);
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
}


void init_Temp(void){
    // start serial port
    SerialUSB.begin(9600);
    SerialUSB.println("Dallas Temperature IC Control Library Demo");
    // Start up the library
    sensors.begin();
    return; 
}

void init_Weight(void){
    SerialUSB.begin(9600);
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

void set_Weight_calib(float val_c, float val_o){

    // Save into "flash_c"
    flash_c.write(val_c);
    // Save into "flash_o"
    flash_o.write(val_o);

    //needs to be calibrated at least once
    flash_calibrated.write(true);

    weight_calib = val_c; 
    weight_offset = val_o; 
    return;
}

void get_Weight_calib(float* val_c, float* val_o){
    SerialUSB.begin(9600);

    // Read the content of "flash_c" and assign it to "weight_calib"
    bool calib = flash_calibrated.read();

    //needs to be calibrated at least once
    if(calib == true){
        // Read the content of "flash_c" and assign it to "weight_calib"
        weight_calib = flash_c.read();
        // Read the content of "flash_o" and assign it to "weight_offset"
        weight_offset = flash_o.read();
    }
    else{
        set_Weight_calib(1.0, 1.0);
        // Read the content of "flash_c" and assign it to "weight_calib"
        weight_calib = flash_c.read();
        // Read the content of "flash_o" and assign it to "weight_offset"
        weight_offset = flash_o.read();
    }

    SerialUSB.print("Offset: ");
    SerialUSB.println(weight_offset);
    SerialUSB.print("Calibration: ");
    SerialUSB.println(weight_calib);

    SerialUSB.print("Calibration done?: ");
    SerialUSB.println(calib);

    *val_c = weight_calib; 
    *val_o = weight_offset; 
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
    if(adc.getValue(val))
        return val; 
}

float get_Weight(void){

    //Datasheet ADS1232 page 8 -> w = m * val + wzs - calibration_weight

    //m = (calibration_weight / (weight_calib - weight_offset)) 
    //wzs = -m * weight_offset

    long val;
    if(adc.getValue(val)){        //this call blocks until a sample is ready!
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
    float m = (calibration_weight / (weight_calib - weight_offset));
    float wzs = (-m) * weight_offset;
    result = m * val + wzs - tare_weight;

    return result; 
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
      play_rtttl(success);
	  }
      if(msg.indexOf('a') >= 0) // Tare command for load Cell A
      {
       DEBUG_STREAM.println("tare A!");
      }
      if(msg.indexOf('A') >= 0) // Weight definition for load Cell A
      {
       DEBUG_STREAM.println("Weight on Load Cell A:");
       int j = readNumber(msg,msg.indexOf('A')+1);
       DEBUG_STREAM.print(j);
       DEBUG_STREAM.println("g");
      }
      if(msg.indexOf('b') >= 0) // Tare command for load Cell B
      {
       DEBUG_STREAM.println("tare A!");
      }
      if(msg.indexOf('B') >= 0) // Weight definition for load Cell B
      {
       DEBUG_STREAM.println("Weight on Load Cell B:");
       int j = readNumber(msg,msg.indexOf('B')+1);
       DEBUG_STREAM.print(j);
       DEBUG_STREAM.println("g");
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
      if(msg.indexOf('S') >= 0) // Play a nice song
      {
       play_rtttl(rock);
      }
      return 1;
    }
    else
    {
      DEBUG_STREAM.println("not OK!");
      
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
	int package[SENSOR_BUFFER_LEN] = {0};
	int pointer = -1;
	
} sensorbuffer;


void safe_sens_value(float weight1, float weight2, float temp, float volt, int8_t signal, int package){
	
	sensorbuffer.pointer++;
	if(sensorbuffer.pointer == SENSOR_BUFFER_LEN){
		sensorbuffer.pointer = 0;
	}
	
	sensorbuffer.weight1[sensorbuffer.pointer] = weight1;
	sensorbuffer.weight2[sensorbuffer.pointer] = weight2;
	sensorbuffer.temp[sensorbuffer.pointer] = temp;
	sensorbuffer.volt[sensorbuffer.pointer] = volt;
	sensorbuffer.signal[sensorbuffer.pointer] = signal;
	sensorbuffer.package[sensorbuffer.pointer] = package;

	
	

}

//make sure that position is valid by using get_sens_pointer with offset
void read_sens_value(float* weight1, float* weight2, float* temp, float* volt, int8_t* signal, int* package, int position){
	*weight1 = sensorbuffer.weight1[position]; 
    *weight2 = sensorbuffer.weight2[position]; 
	*temp = sensorbuffer.temp[position]; 
	*volt = sensorbuffer.volt[position];
	*signal = sensorbuffer.signal[position];
	*package = sensorbuffer.package[position];
	
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


void powerdownfor(int i){
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
