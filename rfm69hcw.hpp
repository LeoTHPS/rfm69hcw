#pragma once

#include <cstdint>

#include <hardware/spi.h>

enum RFM69HCW_LNA
{
	RFM69HCW_LNA_OFF        = -1,
	RFM69HCW_LNA_GAIN_AUTO  = 0b000,
	RFM69HCW_LNA_GAIN_DB_0  = 0b001,
	RFM69HCW_LNA_GAIN_DB_6  = 0b010,
	RFM69HCW_LNA_GAIN_DB_12 = 0b011,
	RFM69HCW_LNA_GAIN_DB_24 = 0b100,
	RFM69HCW_LNA_GAIN_DB_36 = 0b101,
	RFM69HCW_LNA_GAIN_DB_48 = 0b110
};

enum RFM69HCW_POWER
{
	RFM69HCW_POWER_OFF,

	RFM69HCW_POWER_MODE_PA_0 = 0b10000000,
	RFM69HCW_POWER_MODE_PA_1 = 0b01000000,
	RFM69HCW_POWER_MODE_PA_2 = 0b00100000,

	RFM69HCW_POWER_LEVEL_MIN = 0b00000000,
	RFM69HCW_POWER_LEVEL_MAX = 0b00011111
};

enum RFM69HCW_BITRATE
{
	RFM69HCW_BITRATE_1200,
	RFM69HCW_BITRATE_2400,
	RFM69HCW_BITRATE_4800,
	RFM69HCW_BITRATE_9600,
	RFM69HCW_BITRATE_12500,
	RFM69HCW_BITRATE_19200,
	RFM69HCW_BITRATE_25000,
	RFM69HCW_BITRATE_32768,
	RFM69HCW_BITRATE_38400,
	RFM69HCW_BITRATE_50000,
	RFM69HCW_BITRATE_57600,
	RFM69HCW_BITRATE_76800,
	RFM69HCW_BITRATE_100000,
	RFM69HCW_BITRATE_115200,
	RFM69HCW_BITRATE_150000,
	RFM69HCW_BITRATE_153600,
	RFM69HCW_BITRATE_200000,
	RFM69HCW_BITRATE_250000,
	RFM69HCW_BITRATE_300000,

	RFM69HCW_BITRATE_MIN = RFM69HCW_BITRATE_1200,
	RFM69HCW_BITRATE_MAX = RFM69HCW_BITRATE_300000
};

enum RFM69HCW_NETWORK
{
	RFM69HCW_NETWORK_MIN               = 0,
	RFM69HCW_NETWORK_MAX               = 255,

	RFM69HCW_NETWORK_ADDRESS_MIN       = 0,
	RFM69HCW_NETWORK_ADDRESS_MAX       = 254,
	RFM69HCW_NETWORK_ADDRESS_BROADCAST = 255
};

enum RFM69HCW_PREAMBLE
{
	RFM69HCW_PREAMBLE_LENGTH_MIN = 3,
	RFM69HCW_PREAMBLE_LENGTH_MAX = 0xFFFF
};

enum RFM69HCW_FREQUENCY
{
	RFM69HCW_FREQUENCY_MIN = 902000000,
	RFM69HCW_FREQUENCY_MAX = 928000000
};

enum RFM69HCW_MODULATION
{
	RFM69HCW_MODULATION_OOK = 0b01000,
	RFM69HCW_MODULATION_FSK = 0b00000
};

struct rfm69hcw;

typedef bool(*rfm69hcw_receive_callback)(rfm69hcw* r, void* buffer, size_t size, uint8_t address, int8_t rssi, void* param);

rfm69hcw* rfm69hcw_open(spi_inst* spi, uint miso, uint mosi, uint sclk, uint cs, uint irq, uint reset);
void      rfm69hcw_close(rfm69hcw* r);
int       rfm69hcw_get_lna(rfm69hcw* r);
int8_t    rfm69hcw_get_rssi(rfm69hcw* r);
int       rfm69hcw_get_power(rfm69hcw* r);
int       rfm69hcw_get_bitrate(rfm69hcw* r);
uint32_t  rfm69hcw_get_frequency(rfm69hcw* r);
int       rfm69hcw_get_modulation(rfm69hcw* r);
int8_t    rfm69hcw_get_sensitivity(rfm69hcw* r);
uint8_t   rfm69hcw_get_network(rfm69hcw* r);
uint8_t   rfm69hcw_get_network_address(rfm69hcw* r);
bool      rfm69hcw_get_encryption(rfm69hcw* r);
uint16_t  rfm69hcw_get_preamble_length(rfm69hcw* r);
size_t    rfm69hcw_get_payload_capacity(rfm69hcw* r);
bool      rfm69hcw_set_lna(rfm69hcw* r, int value);
bool      rfm69hcw_set_power(rfm69hcw* r, int value);
bool      rfm69hcw_set_bitrate(rfm69hcw* r, int value);
bool      rfm69hcw_set_frequency(rfm69hcw* r, uint32_t value);
bool      rfm69hcw_set_modulation(rfm69hcw* r, int value);
bool      rfm69hcw_set_sensitivity(rfm69hcw* r, int8_t value);
bool      rfm69hcw_set_network(rfm69hcw* r, uint8_t value);
bool      rfm69hcw_set_network_address(rfm69hcw* r, uint8_t value);
bool      rfm69hcw_set_encryption(rfm69hcw* r, bool value);
bool      rfm69hcw_set_encryption_key(rfm69hcw* r, const uint8_t(&value)[16]);
bool      rfm69hcw_set_preamble_length(rfm69hcw* r, uint16_t value);
void      rfm69hcw_reset(rfm69hcw* r);
// @return number of bytes sent
size_t    rfm69hcw_send(rfm69hcw* r, const void* buffer, size_t size);
// @return number of bytes sent
size_t    rfm69hcw_send_to(rfm69hcw* r, const void* buffer, size_t size, uint8_t address);
// @return number of bytes received
size_t    rfm69hcw_receive(rfm69hcw* r, void* buffer, size_t size, uint8_t* address, int8_t* rssi);
// @return number of bytes received
size_t    rfm69hcw_receive_from(rfm69hcw* r, void* buffer, size_t size, uint8_t address, int8_t* rssi);
void      rfm69hcw_receive_while(rfm69hcw* r, void* buffer, size_t size, rfm69hcw_receive_callback callback, void* param);
