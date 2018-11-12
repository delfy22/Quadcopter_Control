/*
 Name:		ppm_to_ibus_serial.ino
 Created:	5/31/2018
 Author:	wdcossey
*/

#define IBUS_FRAME_LENGTH 0x20                                  // iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
#define IBUS_COMMAND40 0x40                                     // Command is always 0x40
#define IBUS_MAXCHANNELS 14                                     // iBus has a maximum of 14 channels

#define IBUS_DEFAULT_VALUE (uint16_t)1500  // Default value for a channel if not used is 1500 (0x05DC)

volatile uint16_t channel_data[IBUS_MAXCHANNELS] = { 0 };	// Create 14 element array of 0s (no. of iBus channels)
volatile uint8_t channel_count = 0; // How many channels we're going to explicitly set
uint8_t index = 0;
int time1 = 0;
int time2 = 0;
int timediff = 0;
int incomingByte = 0;
int display_data = 0;

byte serial_buffer[IBUS_FRAME_LENGTH] = { 0 };
int buffer_index = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode (13, OUTPUT); // On-board LED
  Serial.begin(115200); // Serial Monitor
	Serial1.begin(115200); // Serial channel we're going to send data over (Tx1 - 18 on Mega)
}

void WriteSerial() {
  
  uint16_t ibus_checksum = ((uint16_t)0xFFFF);
  
  	buffer_index = 0;
  
  	// Write the IBus buffer length
  	serial_buffer[buffer_index++] = (byte)IBUS_FRAME_LENGTH;
  	// Write the IBus Command 0x40
  	serial_buffer[buffer_index++] = (byte)IBUS_COMMAND40;
  
  	// Write the IBus channel data to the buffer
  	for (int i = 0; i < min(channel_count, IBUS_MAXCHANNELS); i++) {
  		serial_buffer[buffer_index++] = (byte)(channel_data[i] & 0xFF);
  		serial_buffer[buffer_index++] = (byte)((channel_data[i] >> 8) & 0xFF);
  	}
  
  	// Fill the remaining buffer channels with the default value
  	if (channel_count < IBUS_MAXCHANNELS) {
  		for (int i = 0; i < IBUS_MAXCHANNELS - channel_count; i++) {
  			serial_buffer[buffer_index++] = (byte)(IBUS_DEFAULT_VALUE & 0xFF);
  			serial_buffer[buffer_index++] = (byte)((IBUS_DEFAULT_VALUE >> 8) & 0xFF);
  		}
  	}
  
  	// Calculate the IBus checksum
  	for (int i = 0; i < buffer_index; i++) {
  		ibus_checksum -= (uint16_t)serial_buffer[i];
  	}
  
  	// Write the IBus checksum to the buffer
  	serial_buffer[buffer_index++] = (byte)(ibus_checksum & 0xFF);
  	serial_buffer[buffer_index++] = (byte)((ibus_checksum >> 8) & 0xFF);
  
  	// Display first two pieces of data being sent
//    Serial.println("I'm writing: ");
//    for (int i = 0; i < min(ppm_channel_count, IBUS_MAXCHANNELS); i++) {
//      Serial.println(serial_buffer[i+2]);
//      Serial.println(serial_buffer[i+3]);
//    }
    

  // To debug the frame sent
//  int elementCount = sizeof(serial_buffer) / sizeof(serial_buffer[0]);
//  for(int i = 0; i<elementCount; i++) {
//    Serial.println(serial_buffer[i],HEX);
//  }

    // Write the buffer to the Serial pin
  	Serial1.write(serial_buffer, buffer_index);
  	buffer_index = 0;
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
  channel_count = 1;

  // Time the WriteSerial operation
  time1 = micros();
	// Write the IBus data to the Serial Port
	WriteSerial();
  time2 = micros();
  timediff = time2 - time1;

  // Display the time taken to WriteSerial
//  Serial.print("iBUS loop duration = ");
//  Serial.println(timediff);
  
	// Delay before sending next frame
	delay(1000);
}

