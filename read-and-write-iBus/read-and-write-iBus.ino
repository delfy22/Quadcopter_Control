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
#define IBUS_LOWER_LIMIT 1000
#define IBUS_UPPER_LIMIT 2000

uint16_t *channel_data= new uint16_t[IBUS_MAXCHANNELS];
uint8_t channel_count = 0;
uint8_t automated = 0;
uint8_t armed = 0;
uint8_t out_of_bounds[6] = {0};
uint8_t remote_lost = 0;
unsigned long start_time;

HardwareSerial& iBus_serial = Serial1; // Choose which serial port the iBus will be communicating over

void set_safe_outputs (); // function prototype
void set_outputs (uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw, uint16_t arming_sw, uint16_t automated_sw);

void setup() {
  pinMode (13, OUTPUT); // On-board LED
  Serial.begin(115200); // Serial Monitor
  iBus.begin(iBus_serial); // Open the iBus serial connection

  set_safe_outputs(); // Set all outputs to known safe values
  channel_count = 6; // Must specify how many pieces of data we're explicitly setting, any others will be set to default value
  start_time = millis();
}

void loop() {
  iBus.read_loop(); // Request one frame to be read

/***************************************************************************************/
/**************************Safety checks and automation check***************************/
  remote_lost = 0;
  // Check if any channels are out of bounds, this suggests that the controller has been lost
  for (uint8_t i = 0; i < channel_count; i++) {
    if (iBus.readChannel(i) < IBUS_LOWER_LIMIT || iBus.readChannel(i) > IBUS_UPPER_LIMIT) {
      out_of_bounds[i] = 1;
      remote_lost = 1;
    }
    else {
      out_of_bounds[i] = 0;
    }
  }

  // If remote is connected, then check arming (Channel 5 - SWC)
  if (!remote_lost) {
    if (iBus.readChannel(4) > 1100) {
      armed = 1;
      Serial.println("Armed!");
    }
    else {
      armed = 0;
      Serial.println("Unarmed!");
    }
  }

  // If remote is connected, Channel 6 (SWB) is down, and remote is armed then set automated mode, otherwise fly in manual mode
  if (iBus.readChannel(5) > 1500 && !remote_lost && armed) {
    // If we're entering automated mode, take time.
    if (!automated) {
      start_time = millis(); 
    }
    automated = 1;
  }
  else {
    automated = 0;
  }
/***************************************************************************************/

  // If we're in manual mode, pass data read from receiver to the FCU
  if (!automated) {
    for (uint8_t i = 0; i < channel_count; i++) {
      uint16_t read_data = iBus.readChannel(i); // Read one frame
      channel_data[i] = read_data;
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
      Serial.print(read_data);
      Serial.print(", ");
    }
    Serial.println();
  }
  // If in automated mode, calculate our own channel values to send to FCU
  else if(automated) {
    // For automated control
//    channel_data[2] = channel_data[2] + 1;
//    if(channel_data[2] >= 1900) {
//      channel_data[2] = 1000;
//    }
//    Serial.println(channel_data[2]);
      planned_path();
  }
  // If we suspect the controller is lost, set all outputs to known safe values
  if (remote_lost) {
    set_safe_outputs();
    Serial.println("Entering Safe State");
  }
  
  iBus.write_one_frame(channel_data, channel_count, iBus_serial); // Send one frame to be written
  
  delay(1); // Set a delay between sending frames
}


void set_safe_outputs () {
  channel_data[0] = 1500; // Roll control - default = 1500
  channel_data[1] = 1500; // Pitch control - default = 1500
  channel_data[2] = 1000; // Throttle control - default = 1000
  channel_data[3] = 1500; // Rudder control - default = 1500
  channel_data[4] = 1000; // Arming control - default = 1000, armed = 1500
  channel_data[5] = 1000; // Selectable control - default = 1000, second mode at 2000
}

void set_outputs (uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw, uint16_t arming_sw, uint16_t automated_sw) {
  channel_data[0] = roll; // Roll control - default = 1500
  channel_data[1] = pitch; // Pitch control - default = 1500
  channel_data[2] = throttle; // Throttle control - default = 1000
  channel_data[3] = yaw; // Rudder control - default = 1500
  channel_data[4] = arming_sw; // Arming control - default = 1000, armed = 1500
  channel_data[5] = automated_sw; // Selectable control - default = 1000, second mode at 2000
}

void planned_path () {
  unsigned long current_time = millis();
  unsigned long time = current_time - start_time;
  if (time < 4000) {
    set_safe_outputs();
  }
  else if (time < 8000) {
    set_outputs (1500, 1500, 1200, 1500, 1500, 2000);
  }
  else if (time < 12000) {
    set_outputs (1450, 1500, 1200, 1500, 1500, 2000);
  }
  else if (time < 16000) {
    set_outputs (1500, 1500, 1200, 1500, 1500, 2000);
  }
  else if (time < 20000) {
    set_outputs (1550, 1500, 1200, 1500, 1500, 2000);
  }
  else if (time < 24000) {
    set_outputs (1500, 1500, 1200, 1500, 1500, 2000);
  }
  else {
    set_safe_outputs();
  }

  Serial.print("Time is: ");
  Serial.println(time);
}

