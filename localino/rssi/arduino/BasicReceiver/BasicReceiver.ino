/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file BasicReceiver.ino
 * Use this to test simple sender/receiver functionality with two
 * DW1000. Complements the "BasicSender" example sketch.
 * 
 * LT: 
 * Being updated to test accuracy and range limit
 * updating this code to send a pkg number and string

This test will work as follows:
Set up a Jackal to carry a decawave on its back
Plug in the Arduino and wait for it to initialize over USB
Start the corresponding python program, localino_range_limit.py in the top directory
The python program should display packet integrity percentage every second
Move the decawave farther away until packet integrity diminishes
 */

#include <SPI.h>
#include <DW1000.h>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 3; // irq pin for special configuration
const uint8_t PIN_SS = SS; // spi select pin

// DEBUG packet sent status and count
volatile boolean received = false;
volatile boolean error = false;
volatile int16_t numReceived = 0; // todo check int type
String message;

void setup() {
  // DEBUG monitoring
  Serial.begin(9600);
  char c = 'a';

  // wait for the starting condition
  while( c != 's'){
    c = Serial.read();
  }
  Serial.print('s');
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  //  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();

  // We need to adjust this
  DW1000.setDefaults();

  DW1000.setDeviceAddress(6);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY, 5);
  DW1000.commitConfiguration();
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveFailedHandler(handleError);
  DW1000.attachErrorHandler(handleError);
  // start reception
  receiver();
}

void handleReceived() {
  // status change on reception success
  received = true;
}

void handleError() {
  error = true;
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

float get_abs(float num) {
  if (num < 0 )
    return -num;
   else
    return num;
}

String getString(float num) {
  String str;
  if ( num < 100 ) {
    str += '0';
  }
  str += String(num);
  return str;
}

void loop() {
  // enter on confirmation of ISR status change (successfully received)
  if (received) {
    //    numReceived++;
    // get data as string
    DW1000.getData(message);
    Serial.print('*');

    // all in dBm
    float fppower = get_abs( DW1000.getFirstPathPower() );
    float rxpower = get_abs( DW1000.getReceivePower() );
    float sig_q = get_abs( DW1000.getReceiveQuality() );

    Serial.print( getString( fppower ) ); Serial.print('|');
    Serial.print( getString( rxpower ) ); Serial.print('|');
    Serial.print( getString( sig_q ) ); Serial.print('|');
     
//    Serial.print(DW1000.getReceivePower()); Serial.print("|");
//    Serial.println(DW1000.getReceiveQuality()); Serial.print("*");
    
    // Serial.print("Received message ... #"); Serial.println(numReceived);
    //Serial.print("Data is ... "); Serial.println(message);
    //Serial.print("FP power is [dBm] ... "); Serial.println(DW1000.getFirstPathPower());
    //Serial.print("RX power is [dBm] ... "); Serial.println(DW1000.getReceivePower());
    //Serial.print("Signal quality is ... "); Serial.println(DW1000.getReceiveQuality());
    received = false;
  }
}
