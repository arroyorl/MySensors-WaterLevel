/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 
 * DESCRIPTION
 * This sketch provides an example how to implement a distance sensor using HC-SR04 
 * http://www.mysensors.org/build/distance
 * 
 * Modified for measuring water tank level (0-100%)
 * Water tank measures 100 cm x 100 cm x 100 cm (1000 liters)
 * minimum level (tank exit pipe) is at 25 cm from bottom
 * maximum level (tank water enter buoy) at 75 cm
 * usable water level 500 lt
 * sketch measures negative % when tank is empty and water below exit pipe
 * 
 * REVISION HISTORY
 * Version 1.0 - Henrik EKblad
 * Version 1.1 - First adapted version by R. Arroyo
 * Version 1.2 - Adapted to AZ board
 *               Added Leds
 *               Added batery level and voltage report
 * Version 1.3 - Battery % calculated by type of battery
 *               BATT_LIFEPO4 for LiFePo4 batteries (3.2 to 3.4 V)
 *               BATT_AAA for 2xAAA batteries (3.0 to 3.2 v)
 * 
 * Version 2.0 - Using sensor DFRobot URM07
 * 
 * Version 2.1 - removed distance maximum (100 cm)
 * 
 * Version 2.2 - increased sleep time to 15 mins
 */


#define SKETCH_VERSION  "2.2"

// Enable debug prints
#define MY_DEBUG

// Enable and select radio type attached
//#define MY_RADIO_RF24
#define MY_RADIO_RFM69

// define battery type
//#define BATT_LIFEPO4
#define BATT_AAA

#ifdef MY_RADIO_RF24
  // CE & CS pins in RF nano
  //#define MY_RF24_CE_PIN 10
  //#define MY_RF24_CS_PIN 9
  //#define MY_RF24_PA_LEVEL (RF24_PA_HIGH)  // PA_MIN=-18dBm, PA_LOW=-12dBm, PA_HIGH=-6dBM, PA_MAX=0dBm
  //#define MY_RF24_CHANNEL (76)  // define RF24 channel 0=ch 1 to 125=ch 126

  // CE & CS pins in RF nano
  //#define MY_RF24_CE_PIN 10
  //#define MY_RF24_CS_PIN 9
#endif

#ifdef MY_RADIO_RFM69
  // For RFM69
  #define MY_RFM69_FREQUENCY RFM69_868MHZ  // Define for frequency setting. Needed if you're radio module isn't 868Mhz (868Mhz is default in lib)
  #define MY_IS_RFM69HW  // Mandatory if you radio module is the high power version (RFM69HW and RFM69HCW), Comment it if it's not the case
  //#define MY_RFM69_NETWORKID 100  // Default is 100 in lib. Uncomment it and set your preferred network id if needed
  //#define RFM69_IRQ_PIN 4  // Default in lib is using D2 for common Atmel 328p (mini pro, nano, uno etc.). Uncomment it and set the pin you're using. Note for Atmel 328p, Mysensors, and regarding Arduino core implementation D2 or D3 are only available. But for advanced mcus like Atmel SAMD (Arduino Zero etc.), Esp8266 you will need to set this define for the corresponding pin used for IRQ
  // #define MY_RFM69_IRQ_NUM 4 // Temporary define (will be removed in next radio driver revision). Needed if you want to change the IRQ pin your radio is connected. So, if your radio is connected to D3/INT1, value is 1 (INT1). For others mcu like Atmel SAMD, Esp8266, value is simply the same as your RFM69_IRQ_PIN
  // #define MY_RFM69_SPI_CS 15 // If using a different CS pin for the SPI bus. Use MY_RFM69_CS_PIN for the development branch.

  // RF69 Tx power control
  #define MY_RFM69_MAX_POWER_LEVEL_DBM (20u)  // 0u=1mW, 10u=10mW, 14u=25mW, 20u=100mW 
  //#define MY_RFM69_ATC_MODE_DISABLED          // disable automatic Tx power control
  //#define MY_RFM69_TX_POWER_DBM (5)           // set TX power with ATC disabled
#endif

// Node ID
#define MY_NODE_ID  81

// Set blinking period (in milliseconds)
#define MY_DEFAULT_LED_BLINK_PERIOD 300

#define MY_DEFAULT_ERR_LED_PIN 4  // Red
#define MY_DEFAULT_TX_LED_PIN 5   // Yellow
#define MY_DEFAULT_RX_LED_PIN 6   // Green

#include <MySensors.h>  
#include <SoftwareSerial.h>
#include "URM07.h"

static const uint8_t FORCE_UPDATE = 4;  // force update 60 (=15 x 4) mins if no changes before
static const uint8_t FORCE_UPDATE_BATTERY = 4;  // send battery level every 60 (=15 x 4) mins

// Batery level pin
int BATTERY_SENSE_PIN = A3;  // select the input pin for the battery sense point
int oldA0Value = 0;

#define CHILD_ID_DIST 0
#define CHILD_ID_LEVEL 1
#define CHILD_ID_BARS 2
#define CHILD_ID_VOL 3
#define CHILD_ID_VOLT 4

#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
unsigned long SLEEP_TIME = 900000; // 15 minutes sleep
uint8_t nNoUpdates;

MyMessage msgDist(CHILD_ID_DIST, V_DISTANCE);
MyMessage msgLevel(CHILD_ID_LEVEL, V_PERCENTAGE);
MyMessage msgBars(CHILD_ID_BARS, V_VAR1);
MyMessage msgVol(CHILD_ID_VOL, V_VOLUME);
MyMessage msgVolt(CHILD_ID_VOLT, V_VOLTAGE);

SoftwareSerial mySoftwareSerial(8,7);  // URM07 serial  Rx, Tx pins

/***********************
 *  Global variables
 ***********************/
int lastDist;
bool metric = true;
uint8_t nNoUpdatesBatt = FORCE_UPDATE_BATTERY;

void setup()  
{ 

  mySoftwareSerial.begin(19200);  //Serial1: Ultrasonic Sensor Communication Serial Port, Buadrate: 19200

  // use the 1.1 V internal reference for Battery level
  #if defined(__AVR_ATmega2560__)
    analogReference(INTERNAL1V1);
  #else
    analogReference(INTERNAL);
  #endif

  metric = getControllerConfig().isMetric;
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("WaterLevel", SKETCH_VERSION);

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_DIST, S_DISTANCE);
  present(CHILD_ID_LEVEL, S_CUSTOM);
  present(CHILD_ID_BARS, S_CUSTOM);
  present(CHILD_ID_VOL, S_WATER);
  present(CHILD_ID_VOLT, S_MULTIMETER);
}

void loop()      
{     
int dist;
int distraw;
int level;
int volume;
int bars;

  distraw = URM07ReadDistance(mySoftwareSerial);
  dist = distraw;
  if ( dist > 100 ) dist = 100;

  if (metric) {
    level = (75 - dist) * 100 / 50;
    volume = (75 - dist) * 500 / 50;  
  } else {
    level = (10 - dist) * 100 / 20;   
    volume = (10 - dist) * 132 / 20;  // Gallons (USA)
  }
  bars = (level * 8) / 100;
  if (bars < 0) bars = 0;
  if (bars > 8) bars = 8;

  Serial.print("Dist: ");
  Serial.print(distraw); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.println(metric?" cm":" in");
  Serial.print("level: ");
  Serial.print(level);
  Serial.print("%, bars: ");
  Serial.println(bars);
  

  if ( (abs(distraw - lastDist) > 1) || nNoUpdates == FORCE_UPDATE) {
      send(msgDist.set(distraw));
      send(msgLevel.set(level));
//      send(msgBars.set(bars));
      send(msgVol.set(volume));
      lastDist = distraw;

      // Reset no updates counter
      nNoUpdates = 0;
  } else {
    // Increase no update counter if the distance stayed the same
    nNoUpdates++;
  }

  // get the battery Voltage
  int A0Value = analogRead(BATTERY_SENSE_PIN);
  #ifdef MY_DEBUG
  Serial.print("A0: ");
  Serial.println(A0Value);
  #endif


#ifdef BATT_LIFEPO4
  // 1M, 470K divider across battery and using internal ADC ref of 1.1V
  // For 3.2/3.4 v battery (max voltage 3.44 V, min voltage 2.8 v)
  // ((1e6+470e3)/470e3)*1.1 = Vmax = 3.44 Volts
  // 3.44/1023 = Volts per bit = 0.003363075
  // voltage divisor is 1 MOhm - 470 kOhm
  // 2.8 v / 0.003363075 = 833,  1023-833=190

  #define BATTERY_ADJ 0.003363075
  int batteryPcnt = ((A0Value-833)*100 ) / 190;
#else
  // 1M, 510K divider across battery and using internal ADC ref of 1.1V
  // For 2xAAA batery (max voltage 3.2 V, min voltage 2.6 v)
  // ((1e6+510e3)/510e3)*1.1 = Vmax = 3.26 Volts
  // 3.26/1023 = Volts per bit = 0.003186706
  // voltage divisor is 1 MOhm - 510 kOhm
  // 2.6 v / 0.003186706 = 815,  1023-815=208

  #define BATTERY_ADJ 0.003186706
  int batteryPcnt = ((A0Value-815)*100 ) / 208;
#endif


  //send battery level every hour
  if ( nNoUpdatesBatt == FORCE_UPDATE_BATTERY) {
    float batteryV  = A0Value * BATTERY_ADJ;

    #ifdef MY_DEBUG
    Serial.print("Battery Voltage: ");
    Serial.print(batteryV);
    Serial.println(" V");
  
    Serial.print("Battery percent: ");
    Serial.print(batteryPcnt);
    Serial.println(" %");
    #endif

    send(msgVolt.set(batteryV, 2));
    sendBatteryLevel(batteryPcnt);
    oldA0Value = A0Value;
    nNoUpdatesBatt = 0;
  } else {
    nNoUpdatesBatt++;
  }

  // sleep to save batery
  sleep(SLEEP_TIME);
}
