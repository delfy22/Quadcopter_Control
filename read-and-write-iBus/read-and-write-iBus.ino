/*
  Code to read and write data using iBus protocol over one serial channel on Arduino. As iBus is one wire bidirectional, this could rx and tx to one iBus
  connection by placing a resistor between the rx and tx pins. Or, it could rx from one iBus connection and tx to another.
  Note: On the Arduino Mega, Serial is TX0 and RX0 but is used by the USB serial connection too so will interfere with programming and cannot use the serial
  monitor. Instead, use Serial1, 2, or 3.
  iBus reading adapted from: https://gitlab.com/timwilkinson/FlySkyIBus
  iBus writing adapted from: https://github-mirror.open.netease.com/wdcossey/ppm-to-ibus-serial
*/

#include "FlySkyiBusCombined.h"

#define IBUS_MAXCHANNELS 14  // iBus has a maximum of 14 channels

uint16_t *channel_data= new uint16_t[IBUS_MAXCHANNELS];
uint8_t channel_count = 0;

HardwareSerial& iBus_serial = Serial1; // Choose which serial port the iBus will be communicating over

void setup() {
  pinMode (13, OUTPUT); // On-board LED
  Serial.begin(115200); // Serial Monitor
  iBus.begin(iBus_serial); // Open the iBus serial connection
}

void loop() {
//  channel_data[0] += 1; // Roll control - default = 1500
//  channel_data[1] += 2; // Pitch control - default = 1500
//  channel_data[2] = 0; // Throttle control - default = 1000
//  channel_data[3] = 1500; // Rudder control - default = 1500
//  channel_data[4] = 1000; // Arming control - default = 1000, armed = 1500
//  channel_data[5] = 1000; // Selectable control - default = 1000, second mode at 2000
  channel_count = 6; // Must specify how many pieces of data we're explicitly setting, any others will be set to default value

  iBus.write_one_frame(channel_data, channel_count, iBus_serial); // Send one frame to be written

  iBus.read_loop(); // Request one frame to be read
  for (int i = 0; i < 6; i++) {
    uint16_t data = iBus.readChannel(i); // Read one frame
    channel_data[i] = data;
    switch (i) {
      case 0:
        Serial.print("Roll=");
        break;
      case 1:
        Serial.print("Pitch=");
        break;
      case 2:
        Serial.print("Throttle=");
        break;
      case 3:
        Serial.print("Yaw=");
        break;
      case 4:
        Serial.print("SWC=");
        break;
      case 5:
        Serial.print("SWB=");
        break;
      default:
        break;
    }
    Serial.print(data);
    Serial.print(", ");
  }
  Serial.println();
//  delay(500); // Set a delay between sending frames
}
