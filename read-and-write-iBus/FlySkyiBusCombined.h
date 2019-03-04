/*
	Code to read and write data using iBus protocol over one serial channel on Arduino. As iBus is one wire bidirectional, this could rx and tx to one iBus
	connection by placing a resistor between the rx and tx pins. Or, it could rx from one iBus connection and tx to another.
	Note: On the Arduino Mega, Serial is TX0 and RX0 but is used by the USB serial connection too so will interfere with programming and cannot use the serial
	monitor. Instead, use Serial1, 2, or 3.
	iBus reading adapted from: https://gitlab.com/timwilkinson/FlySkyIBus
	iBus writing adapted from: https://github-mirror.open.netease.com/wdcossey/ppm-to-ibus-serial
*/

#pragma once
#include <inttypes.h>

class HardwareSerial;
class Stream;

class FlySkyIBus {
public:
	void begin(HardwareSerial& serial);
	void begin(Stream& stream);
	void read_loop(void);
	uint16_t readChannel(uint8_t channelNr);
	void write_one_frame(uint16_t *channel_data, uint8_t channel_count, HardwareSerial& serial);

private:
	enum State
	{
		GET_LENGTH,
		GET_DATA,
		GET_CHKSUML,
		GET_CHKSUMH,
		DISCARD,
	};

	static const uint8_t IBUS_FRAME_LENGTH = 0x20;	// iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
	static const uint8_t IBUS_OVERHEAD = 3;	// <len><cmd><data....><chkl><chkh>
	static const uint8_t IBUS_TIMEGAP = 3;	// Packets are received very ~7ms so use ~half that for the gap
	static const uint8_t IBUS_MAX_CHANNELS = 14; // iBus has a maximum of 14 channels
	static const uint8_t IBUS_COMMAND40 = 0x40;	// Command is always 0x40
	static const uint16_t IBUS_DEFAULT_VALUE = 1500;  // Default value for a channel if not used is 1500 (0x05DC)
	static const uint16_t IBUS_LOWER_LIMIT = 1000;	// Set limits for the data sent over iBus 
	static const uint16_t IBUS_UPPER_LIMIT = 1900;	// Set limits for the data sent over iBus 

	uint8_t state;
	Stream* stream;
	uint32_t last;
	uint8_t buffer[IBUS_FRAME_LENGTH];
	uint8_t ptr;
	uint8_t len;
	uint16_t channel[IBUS_MAX_CHANNELS];
	uint16_t chksum;
	uint8_t lchksum;

	uint8_t serial_buffer[IBUS_FRAME_LENGTH] = { 0 };
	uint8_t buffer_index = 0;
  uint16_t ibus_checksum;
};

extern FlySkyIBus iBus;
