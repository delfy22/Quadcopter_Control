#include "WriteiBusFrame.h"
#include "FlySkyIBus.h"

#define IBUS_MAXCHANNELS 14  // iBus has a maximum of 14 channels

uint16_t *channel_data= new uint16_t[IBUS_MAXCHANNELS];
uint8_t channel_count = 0;

void setup() {
  pinMode (13, OUTPUT); // On-board LED
  Serial.begin(115200); // Serial Monitor
  iBus.begin(Serial1); //write
  IBus.begin(Serial2); //read
  channel_data[0] = 0;
  channel_data[1] = 0;
}

void loop() {
  channel_data[0] += 1;
  channel_data[1] += 2;
  channel_count = 2;

  iBus.write(channel_data, channel_count, Serial1);

  IBus.loop();
  for (int i = 0; i < 3; i++) {
    uint16_t data = IBus.readChannel(i);
    Serial.println(data);
  }
  Serial.println();
  delay(1000);
}
