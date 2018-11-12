/*
 Name:		ppm_to_ibus_serial.ino
 Created:	5/31/2018
 Author:	wdcossey
*/
#include "WriteiBusFrame.h"

#define IBUS_MAXCHANNELS 14                                     // iBus has a maximum of 14 channels

uint16_t *channel_data= new uint16_t[IBUS_MAXCHANNELS];

int time1 = 0;
int time2 = 0;
int timediff = 0;
int incomingByte = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode (13, OUTPUT); // On-board LED
  Serial.begin(115200); // Serial Monitor
  iBus.begin(Serial1);
}

// the loop function runs over and over again until power down or reset
void loop() {

  // Read data from the serial input (ie. keyboard into the serial monitor)
  if(Serial.available() > 0) {
    incomingByte = Serial.parseInt();
    if (incomingByte != 0) {
      Serial.print("Incoming data = ");
      Serial.println(incomingByte);
      channel_data[0] = incomingByte;
      digitalWrite(13, 1-digitalRead(13));
    }
  }

  // Time the WriteSerial operation
  time1 = micros();
	// Write the IBus data to the Serial Port
  iBus.write(channel_data, Serial1);
  time2 = micros();
  timediff = time2 - time1;

  // Display the time taken to WriteSerial
  Serial.print("iBUS loop duration = ");
  Serial.println(timediff);
  
	// Delay before sending next frame
	delay(1000);
}

