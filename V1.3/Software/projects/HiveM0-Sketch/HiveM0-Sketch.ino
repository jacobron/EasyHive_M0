//###################################################
// Include the libraries we need

#include "OneWire.h"
#include "DallasTemperature.h"
#include "ADS1231.h"
#include "Sodaq_nbIOT.h"
#include "Sodaq_wdt.h"
#include "FlashStorage.h"
#include "EasyHive.h"
#include "rtttl.h"

char *startup = "OneMoreT:d=16,o=5,b=250:3a7,4p,3a7,4p,3a7,4p,3b7,4p,3b7,4p";

extern int BoardID;
extern int datalogtime;
extern int datasendtime;
int loopcounter;

extern Sodaq_nbIOT nbiot;
// Server variables
// define Server variables
extern const char* apn;
extern const char* cdp;
extern unsigned char cid;
extern const char* forceOperator; // optional - depends on SIM / network

int8_t SIGNAL;
uint8_t BER;

/*
   The setup function. We only start the sensors here and initialize the board state
*/

// Reserve a portion of flash memory to store an "bool" variable
// and call it "calibrated".
//FlashStorage(test, float);

void setup(void)
{
  play_rtttl(startup);
  tone(BUZZER,4000);
  delay(100);
  noTone(BUZZER);
  
  sodaq_wdt_safe_delay(5000);


  init_LEDs();
  LEDs_red();
  delay(1000);
  LEDs_green();
  delay(1000);
  LEDs_blue();
  delay(1000);
  LEDs_off();

  // init_RTC();
  init_server_data();
  init_BoardID();
  init_Temp();
  init_Weight();

  float w_calib;
  float w_offset;
  float w_cweight;
  get_Weight_calib(&w_calib, &w_offset, &w_cweight);
  SerialUSB.println("calib1");
  SerialUSB.println(w_calib);
  SerialUSB.println(w_offset);

  float w_calib2;
  float w_offset2;
  float w_cweight2;
  get_Weight_calib2(&w_calib2, &w_offset2, &w_cweight2);
  SerialUSB.println("calib2");
  SerialUSB.println(w_calib2);
  SerialUSB.println(w_offset2);


  // TEST
  pinMode(LC_ON, OUTPUT);
  digitalWrite(LC_ON, LOW);

  //this code is needed to setup watchdogtimer and to make MCU sleep
  sodaq_wdt_enable(WDT_PERIOD_8X); // watchdog expires in ~8 seconds
  sodaq_wdt_reset(); // restting the watchdog
  initSleep();


  //param1 = calib_val (at 1000g); param2 = offset_val(at 0g);
  //set_Weight_calib(1622203.0, -329233);
  //set_Weight_calib(12345.0, 42.0);

  //test.write(1.23);

  //enable_USB_in();

  /****** nbiot init *******/


  DEBUG_STREAM.begin(DEBUG_STREAM_BAUD);
  DEBUG_STREAM.println("Initializing and connecting... ");

  MODEM_STREAM.begin(nbiot.getDefaultBaudrate());
  nbiot.setDiag(DEBUG_STREAM);
  nbiot.init(MODEM_STREAM, powerPin, enablePin, -1, cid);

  if (!nbiot.connect(apn, cdp, forceOperator, 8)) {
    DEBUG_STREAM.println("Failed to connect to the modem!");
  }

  // check Signal quality
  nbiot.getRSSIAndBER(&SIGNAL, &BER);

  // read Battery voltage
  float VOLT = getBatteryVoltage();

  //send Message and check answer
  // [FLOAT], [FLOAT], [FLOAT], [FLOAT], [INT8_T], [INTEGER], [LONG]
  // [WEIGHT1],[WEIGHT2],[TEMP],[VOLT],[SIGNALQUALITY],[BOARDID],[PAKETNR/EPOCH]
  String startmsg = "Hello!,,," + String(VOLT) + "," + String(SIGNAL) + "," + String(BoardID);
  if (sendMessageThroughUDP(startmsg.c_str())) {
    DEBUG_STREAM.println("FINE!");
  }
  else
  {
    DEBUG_STREAM.println("NO RESPONSE");
  }

  // DEBUG_STREAM.println(sendMessageThroughUDP("Hello!"));
}

/*
   Main function, get and show the temperature, weight and sleep, send data via NB-IoT
*/
void loop(void)
{
  unsigned long loopstarttime = millis();
  loopcounter++;

  // start loadcellstuff
  digitalWrite(LC_ON, HIGH);
  digitalWrite(PDWN, HIGH);

  float temp_val = get_Temp();
  SerialUSB.print("Temp: ");
  SerialUSB.println(temp_val);
  delay(1000);


  digitalWrite(SELECT, LOW);
  //dummy weight measure for settlement
  get_Weight();
  get_Weight();

  float weight_val1 = get_Weight();
  SerialUSB.print("Weight: ");
  SerialUSB.println(weight_val1);
  delay(500);

  long weight_val_raw1 = get_Weight_raw();
  SerialUSB.print("Weight Raw: ");
  SerialUSB.println(weight_val_raw1, DEC);
  delay(500);

  digitalWrite(SELECT, HIGH); // read the other side
  delay(1000);
  //dummy weight measure for settlement
  get_Weight_raw();
  get_Weight_raw();

  float weight_val2 = get_Weight2();
  SerialUSB.print("Weight2: ");
  SerialUSB.println(weight_val2);

  long weight_val_raw2 = get_Weight_raw();
  SerialUSB.print("Weight Raw: ");
  SerialUSB.println(weight_val_raw2, DEC);
  delay(500);

  // check Signal quality
  nbiot.getRSSIAndBER(&SIGNAL, &BER);

  // read Battery voltage
  float VOLT = getBatteryVoltage();
  //read Time from rtc
  long epochtime = get_time();

  // [FLOAT], [FLOAT], [FLOAT], [FLOAT], [INT8_T], [INTEGER], [LONG]
  // [WEIGHT1],[WEIGHT2],[TEMP],[VOLT],[SIGNALQUALITY],[BOARDID],[Package/EPOCH]

  safe_sens_value(weight_val1, weight_val2, temp_val, VOLT, SIGNAL, epochtime );
  

  // String msg = String(weight_val1) + "," + String(weight_val2) + "," + String(temp_val) + "," + String(VOLT) + "," + String(SIGNAL) + "," + String(BoardID) + "," + String(package) ;

  //float w_calib;
  //float w_offset;
  //get_Weight_calib(&w_calib, &w_offset);
  //SerialUSB.println(w_calib);
  //SerialUSB.println(w_offset);

  if (loopcounter >= datasendtime / datalogtime) {
    if (!nbiot.isConnected()) {
      if (!nbiot.connect(apn, cdp, forceOperator, 8)) {
        DEBUG_STREAM.println("Failed to connect to the modem!");
      }
    }
    else {
      for (int i = 0; i > -loopcounter && i > -SENSOR_BUFFER_LEN; i--) {
        float weight1 = 0;
        float weight2 = 0;
        float temp = 0;
        float volt = 0;
        int8_t csq = 0;
        long epoch = 0;

        int pointer_pos = get_sens_pointer(i);
        read_sens_value(&weight1, &weight2, &temp, &volt, &csq, &epoch, pointer_pos);
        String msg = String(weight1) + "," + String(weight2) + "," + String(temp) + "," + String(volt) + "," + String(csq) + "," + String(BoardID) + "," + String(epoch) ;

        sendMessageThroughUDP(msg.c_str());
        // TODO: Was macht er denn falls Senden schiefläuft? Daten verwerfen?
      }
    }
    loopcounter = 0;
  }
  else {
    SerialUSB.print("logging data...");
    // write data to to the logs

    float weight1 = 0;
    float weight2 = 0;
    float temp = 0;
    float volt = 0;
    int8_t csq = 0;
    long pktnr = 0;

    int pointer_pos = get_sens_pointer(0);
    read_sens_value(&weight1, &weight2, &temp, &volt, &csq, &pktnr, pointer_pos);
    delay(100);
    SerialUSB.println("\n ");
    SerialUSB.print("Temp: ");
    SerialUSB.println(temp);
    SerialUSB.print("Weight1 ");
    SerialUSB.println(weight1);
    SerialUSB.print("Weigh2   ");
    SerialUSB.println(weight2);
    SerialUSB.print("volt ");
    SerialUSB.println(volt);
    SerialUSB.print("signal ");
    SerialUSB.println(csq);
    SerialUSB.print("package ");
    SerialUSB.println(pktnr);
  }
  
  digitalWrite(SELECT, LOW);
  digitalWrite(LC_ON, LOW);
  digitalWrite(PDWN, LOW);

  // calculate sleeptime
  int sleeptime = datalogtime - (millis() - loopstarttime)/1000;
  SerialUSB.print("Sleeptime[s]:");
  SerialUSB.println(sleeptime);
  if(sleeptime > 0) {
    powerdownfor(sleeptime);
  }
    
}
