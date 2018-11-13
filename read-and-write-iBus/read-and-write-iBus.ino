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
  channel_data[0] = 0; // Data to be written
  channel_data[1] = 0; // Data to be written
}

void loop() {
  channel_data[0] += 1; // Data to be written
  channel_data[1] += 2; // Data to be written
  channel_count = 2; // Must specify how many pieces of data we're explicitly setting, any others will be set to default value

  iBus.write_one_frame(channel_data, channel_count, iBus_serial); // Send one frame to be written

  iBus.read_loop(); // Request one frame to be read
  for (int i = 0; i < 3; i++) {
    uint16_t data = iBus.readChannel(i); // Read one frame
    Serial.println(data);
  }
  Serial.println();
  delay(1000); // Set a delay between sending frames
}
