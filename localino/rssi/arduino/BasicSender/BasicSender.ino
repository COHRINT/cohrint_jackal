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
const uint8_t PIN_IRQ = 3; // irq pin special configuration
const uint8_t PIN_SS = SS; // spi select pin

// DEBUG packet sent status and count
boolean sent = false;
volatile boolean sentAck = false;
volatile unsigned long delaySent = 0;
int16_t sentNum = 0; // todo check int type
DW1000Time sentTime;

void setup() {
  // DEBUG monitoring
  Serial.begin(9600);

  char c = 'a';

  // wait for the starting condition
  /* while( c != 's'){ */
  /*   c = Serial.read(); */
  /* } */
  //  Serial.println("Sender Ready");

  // Serial.println(F("### DW1000-arduino-sender-test ###"));
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  //  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();

  // we need to adjust htis
  DW1000.setDefaults();

  DW1000.setDeviceAddress(5); // wow can we say who the message is for?
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  DW1000.commitConfiguration();
  //  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  /* char msg[128]; */
  /* DW1000.getPrintableDeviceIdentifier(msg); */
  /* Serial.print("Device ID: "); Serial.println(msg); */
  /* DW1000.getPrintableExtendedUniqueIdentifier(msg); */
  /* Serial.print("Unique ID: "); Serial.println(msg); */
  /* DW1000.getPrintableNetworkIdAndShortAddress(msg); */
  /* Serial.print("Network ID & Device Address: "); Serial.println(msg); */
  /* DW1000.getPrintableDeviceMode(msg); */
  /* Serial.print("Device mode: "); Serial.println(msg); */
  // attach callback for (successfully) sent messages
  DW1000.attachSentHandler(handleSent);
  // start a transmission
  transmitter();
}

void handleSent() {
  // status change on sent success
  sentAck = true;
}

void transmitter() {
  // transmit some data
  //  Serial.print("Transmitting packet ... #"); Serial.println(sentNum);
  DW1000.newTransmit();
  DW1000.setDefaults();
  String msg = "hello"; msg += sentNum; // add 10 1's we'll check the accuracy of this later
  DW1000.setData(msg);
  // delay sending the message for the given amount
  DW1000Time deltaTime = DW1000Time(10, DW1000Time::MILLISECONDS);

  DW1000.setDelay(deltaTime);
  DW1000.startTransmit();
  //  delaySent = millis();
}

void loop() {
  if (!sentAck) {
    return;
  }
  //  Serial.println("Sending next packet now");
  // continue on success confirmation
  // (we are here after the given amount of send delay time has passed)
  sentAck = false;
  // update and print some information about the sent message
  //  Serial.print("ARDUINO delay sent [ms] ... "); Serial.println(millis() - delaySent);
    DW1000Time newSentTime;
    DW1000.getTransmitTimestamp(newSentTime);
  //  Serial.print("Processed packet ... #"); Serial.println(sentNum);
  //  Serial.print("Sent timestamp ... "); Serial.println(newSentTime.getAsMicroSeconds());
  // note: delta is just for simple demo as not correct on system time counter wrap-around
  //  Serial.print("DW1000 delta send time [ms] ... "); Serial.println((newSentTime.getAsMicroSeconds() - sentTime.getAsMicroSeconds()) * 1.0e-3);
  sentTime = newSentTime;
  sentNum++;
  // again, transmit some data
  transmitter();
}
