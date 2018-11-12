/*
 * Test FlySky IBus interface on an Arduino Mega.
 *  Connect FS-iA6B receiver to Serial1.
 */

 uint8_t data = 0;
 int time1 = 0;
 int time2 = 0;
 int timediff = 0;
 int olddata = 0;
 
#include "FlySkyIBus.h"

void setup() 
{
  Serial.begin(115200);
  IBus.begin(Serial1);
}

void loop() 
{
  time1 = micros();
  IBus.loop();
  time2 = micros();
  timediff = time2 - time1;
  //Serial.print("iBUS loop duration = ");
  //Serial.println(timediff);
  olddata = data;
  data = IBus.readChannel(0);
  
  if(data != olddata) {
    Serial.print("Data = ");
    Serial.println(data);
  }
  //Serial.println(IBus.readChannel(0), HEX);
}
