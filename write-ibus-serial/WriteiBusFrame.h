/*
Function to write the iBus protocol over a serial pin.
*/

#include <inttypes.h>

class HardwareSerial;
class Stream;

class WriteiBusFrame { // will be merged with FlySkyIBus later
public:
	void begin(HardwareSerial& serial);
	//void begin(Stream& stream);
	void write(uint16_t *channel_data, uint8_t channel_count, HardwareSerial& serial);

private:
	static const uint8_t IBUS_FRAME_LENGTH = 0x20;	// iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
	static const uint8_t IBUS_COMMAND40 = 0x40;	// Command is always 0x40
	static const uint8_t IBUS_MAXCHANNELS = 14;	// iBus has a maximum of 14 channels
	static const uint16_t IBUS_DEFAULT_VALUE = 1500;  // Default value for a channel if not used is 1500 (0x05DC)

	uint8_t serial_buffer[IBUS_FRAME_LENGTH] = { 0 };
	uint8_t buffer_index = 0;
	uint16_t ibus_checksum;

	//HardwareSerial& serial_pin;
};

extern WriteiBusFrame iBus;
