/*
Function to write the iBus protocol over a serial pin.
*/

#include <Arduino.h>
#include "WriteiBusFrame.h"


WriteiBusFrame iBus;

void WriteiBusFrame::begin(HardwareSerial& serial) {
	//serial_pin = serial;
	serial.begin(115200);
}

void WriteiBusFrame::write(uint16_t *channel_data, uint8_t channel_count, HardwareSerial& serial) {

	ibus_checksum = 0xFFFF;

	buffer_index = 0;

	// Write the IBus buffer length
	serial_buffer[buffer_index++] = IBUS_FRAME_LENGTH;
	// Write the IBus Command 0x40
	serial_buffer[buffer_index++] = IBUS_COMMAND40;

	// Write the IBus channel data to the buffer
	for (int i = 0; i < min(channel_count, IBUS_MAXCHANNELS); i++) {
		serial_buffer[buffer_index++] = (channel_data[i] & 0xFF);
		serial_buffer[buffer_index++] = ((channel_data[i] >> 8) & 0xFF);
	}

	// Fill the remaining buffer channels with the default value
	if (channel_count < IBUS_MAXCHANNELS) {
		for (int i = 0; i < IBUS_MAXCHANNELS - channel_count; i++) {
			serial_buffer[buffer_index++] = (IBUS_DEFAULT_VALUE & 0xFF);
			serial_buffer[buffer_index++] = ((IBUS_DEFAULT_VALUE >> 8) & 0xFF);
		}
	}

	// Calculate the IBus checksum
	for (int i = 0; i < buffer_index; i++) {
		ibus_checksum -= (uint16_t)serial_buffer[i];
	}

	// Write the IBus checksum to the buffer
	serial_buffer[buffer_index++] = (ibus_checksum & 0xFF);
	serial_buffer[buffer_index++] = ((ibus_checksum >> 8) & 0xFF);


  // To debug the frame sent
//	  int elementCount = sizeof(serial_buffer) / sizeof(serial_buffer[0]);
//	  for(int i = 0; i<elementCount; i++) {
//	    Serial.println(serial_buffer[i],HEX);
//	  }

	// Write the buffer to the Serial pin
	serial.write(serial_buffer, buffer_index);
	buffer_index = 0;
}
