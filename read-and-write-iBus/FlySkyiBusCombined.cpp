/*
Code to read and write data using iBus protocol over one serial channel on Arduino. As iBus is one wire bidirectional, this could rx and tx to one iBus
connection by placing a resistor between the rx and tx pins. Or, it could rx from one iBus connection and tx to another.
Note: On the Arduino Mega, Serial is TX0 and RX0 but is used by the USB serial connection too so will interfere with programming and cannot use the serial
monitor. Instead, use Serial1, 2, or 3.
iBus reading adapted from: https://gitlab.com/timwilkinson/FlySkyIBus
iBus writing adapted from: https://github-mirror.open.netease.com/wdcossey/ppm-to-ibus-serial
*/

#include <Arduino.h>
#include "FlySkyiBusCombined.h"

FlySkyIBus iBus;

void FlySkyIBus::begin(HardwareSerial& serial)
{
	serial.begin(115200, SERIAL_8N1, 15, 25);
	begin((Stream&)serial);
}

void FlySkyIBus::begin(Stream& stream)
{
	this->stream = &stream;
	this->state = DISCARD;
	this->last = millis();
	this->ptr = 0;
	this->len = 0;
	this->chksum = 0;
	this->lchksum = 0;
}

void FlySkyIBus::read_loop(void)
{
	while (stream->available() > 0)
	{
		uint32_t now = millis();
		if (now - last >= IBUS_TIMEGAP)
		{
			state = GET_LENGTH;
		}
		last = now;

		uint8_t v = stream->read();
		switch (state)
		{
		case GET_LENGTH:
			if (v <= IBUS_FRAME_LENGTH)
			{
				ptr = 0;
				len = v - IBUS_OVERHEAD;
				chksum = 0xFFFF - v;
				state = GET_DATA;
			}
			else
			{
				state = DISCARD;
			}
			break;

		case GET_DATA:
			buffer[ptr++] = v;
			chksum -= v;
			if (ptr == len)
			{
				state = GET_CHKSUML;
			}
			break;

		case GET_CHKSUML:
			lchksum = v;
			state = GET_CHKSUMH;
			break;

		case GET_CHKSUMH:
			// Validate checksum
			if (chksum == (v << 8) + lchksum)
			{
				// Execute command - we only know command 0x40
				switch (buffer[0])
				{
				case IBUS_COMMAND40:
					// Valid - extract channel data
					for (uint8_t i = 1; i < IBUS_MAX_CHANNELS * 2 + 1; i += 2)
					{
						channel[i / 2] = buffer[i] | (buffer[i + 1] << 8);
					}
					break;

				default:
					break;
				}
			}
			state = DISCARD;
			break;

		case DISCARD:
		default:
			break;
		}
	}
}

uint16_t FlySkyIBus::readChannel(uint8_t channelNr)
{
	if (channelNr < IBUS_MAX_CHANNELS)
	{
		return channel[channelNr];  // Return data from one channel
	}
	else
	{
		return 0; //Invalid channel
	}
}

void FlySkyIBus::write_one_frame(uint16_t *channel_data, uint8_t channel_count, HardwareSerial& serial) {

	ibus_checksum = 0xFFFF;

	buffer_index = 0;

	// Write the IBus buffer length
	serial_buffer[buffer_index++] = IBUS_FRAME_LENGTH;
	// Write the IBus Command 0x40
	serial_buffer[buffer_index++] = IBUS_COMMAND40;

	// Write the IBus channel data to the buffer
	for (int i = 0; i < min(channel_count, IBUS_MAX_CHANNELS); i++) {
		//If attempting to send data out of limits then coerce into the limits
		if (channel_data[i] < IBUS_LOWER_LIMIT) {
			channel_data[i] = IBUS_LOWER_LIMIT;
		}
		else if (channel_data[i] > IBUS_UPPER_LIMIT) {
			channel_data[i] = IBUS_UPPER_LIMIT;
		}
		serial_buffer[buffer_index++] = (channel_data[i] & 0xFF);
		serial_buffer[buffer_index++] = ((channel_data[i] >> 8) & 0xFF);
	}

	// Fill the remaining buffer channels with the default value
	if (channel_count < IBUS_MAX_CHANNELS) {
		for (int i = 0; i < IBUS_MAX_CHANNELS - channel_count; i++) {
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

	// Write the buffer to the Serial pin
	serial.write(serial_buffer, buffer_index);
	buffer_index = 0;
}
