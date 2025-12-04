#include "rfm69hcw.hpp"

#include <stdio.h>
#include <string.h>

#include <hardware/gpio.h>
#include <hardware/timer.h>

enum RFM69HCW_MODES
{
	RFM69HCW_MODE_SLEEP       = 0b00000000,
	RFM69HCW_MODE_STANDBY     = 0b00000100,
	RFM69HCW_MODE_RECEIVER    = 0b00010000,
	RFM69HCW_MODE_TRANSMITTER = 0b00001100
};

enum RFM69HCW_REGISTERS
{
	RFM69HCW_RegFifo          = 0x00,
	RFM69HCW_RegOpMode        = 0x01,
	RFM69HCW_RegDataModul     = 0x02,
	RFM69HCW_RegBitrateMsb    = 0x03,
	RFM69HCW_RegBitrateLsb    = 0x04,
	RFM69HCW_RegFdevMsb       = 0x05,
	RFM69HCW_RegFdevLsb       = 0x06,
	RFM69HCW_RegFrfMsb        = 0x07,
	RFM69HCW_RegFrfMid        = 0x08,
	RFM69HCW_RegFrfLsb        = 0x09,
	RFM69HCW_RegOsc1          = 0x0A,
	RFM69HCW_RegAfcCtrl       = 0x0B,
	RFM69HCW_RegReserved0C    = 0x0C,
	RFM69HCW_RegListen1       = 0x0D,
	RFM69HCW_RegListen2       = 0x0E,
	RFM69HCW_RegListen3       = 0x0F,
	RFM69HCW_RegVersion       = 0x10,
	RFM69HCW_RegPaLevel       = 0x11,
	RFM69HCW_RegPaRamp        = 0x12,
	RFM69HCW_RegOcp           = 0x13,
	RFM69HCW_Regerved14       = 0x14,
	RFM69HCW_Regerved15       = 0x15,
	RFM69HCW_Regerved16       = 0x16,
	RFM69HCW_Regerved17       = 0x17,
	RFM69HCW_RegLna           = 0x18,
	RFM69HCW_RegRxBw          = 0x19,
	RFM69HCW_RegAfcBw         = 0x1A,
	RFM69HCW_RegOokPeak       = 0x1B,
	RFM69HCW_RegOokAvg        = 0x1C,
	RFM69HCW_RegOokFix        = 0x1D,
	RFM69HCW_RegAfcFei        = 0x1E,
	RFM69HCW_RegAfcMsb        = 0x1F,
	RFM69HCW_RegAfcLsb        = 0x20,
	RFM69HCW_RegFeiMsb        = 0x21,
	RFM69HCW_RegFeiLsb        = 0x22,
	RFM69HCW_RegRssiConfig    = 0x23,
	RFM69HCW_RegRssiValue     = 0x24,
	RFM69HCW_RegDioMapping1   = 0x25,
	RFM69HCW_RegDioMapping2   = 0x26,
	RFM69HCW_RegIrqFlags1     = 0x27,
	RFM69HCW_RegIrqFlags2     = 0x28,
	RFM69HCW_RegRssiThresh    = 0x29,
	RFM69HCW_RegRxTimeout1    = 0x2A,
	RFM69HCW_RegRxTimeout2    = 0x2B,
	RFM69HCW_RegPreambleMsb   = 0x2C,
	RFM69HCW_RegPreambleLsb   = 0x2D,
	RFM69HCW_RegSyncConfig    = 0x2E,
	RFM69HCW_RegSyncValue1    = 0x2F,
	RFM69HCW_RegSyncValue2    = 0x30,
	RFM69HCW_RegSyncValue3    = 0x31,
	RFM69HCW_RegSyncValue4    = 0x32,
	RFM69HCW_RegSyncValue5    = 0x33,
	RFM69HCW_RegSyncValue6    = 0x34,
	RFM69HCW_RegSyncValue7    = 0x35,
	RFM69HCW_RegSyncValue8    = 0x36,
	RFM69HCW_RegPacketConfig1 = 0x37,
	RFM69HCW_RegPayloadLength = 0x38,
	RFM69HCW_RegNodeAdrs      = 0x39,
	RFM69HCW_RegBroadcastAdrs = 0x3A,
	RFM69HCW_RegAutoModes     = 0x3B,
	RFM69HCW_RegFifoThresh    = 0x3C,
	RFM69HCW_RegPacketConfig2 = 0x3D,
	RFM69HCW_RegAesKey1       = 0x3E,
	RFM69HCW_RegAesKey2       = 0x3F,
	RFM69HCW_RegAesKey3       = 0x40,
	RFM69HCW_RegAesKey4       = 0x41,
	RFM69HCW_RegAesKey5       = 0x42,
	RFM69HCW_RegAesKey6       = 0x43,
	RFM69HCW_RegAesKey7       = 0x44,
	RFM69HCW_RegAesKey8       = 0x45,
	RFM69HCW_RegAesKey9       = 0x46,
	RFM69HCW_RegAesKey10      = 0x47,
	RFM69HCW_RegAesKey11      = 0x48,
	RFM69HCW_RegAesKey12      = 0x49,
	RFM69HCW_RegAesKey13      = 0x4A,
	RFM69HCW_RegAesKey14      = 0x4B,
	RFM69HCW_RegAesKey15      = 0x4C,
	RFM69HCW_RegAesKey16      = 0x4D,
	RFM69HCW_RegTemp1         = 0x4E,
	RFM69HCW_RegTemp2         = 0x4F,
	RFM69HCW_RegTestLna       = 0x58,
	RFM69HCW_RegTestPa1       = 0x5A,
	RFM69HCW_RegTestPa2       = 0x5C,
	RFM69HCW_RegTestDagc      = 0x6F,
	RFM69HCW_RegTestAfc       = 0x71,
	RFM69HCW_RegTest          = 0x50
};

struct rfm69hcw_bitrate
{
	int      i;
	uint32_t rate;
	uint16_t value;
};

#pragma pack(push, 1)
struct rfm69hcw_frame_header
{
	uint8_t size;
	uint8_t destination;
	uint8_t source;
};
#pragma pack(pop)

constexpr uint32_t RFM69HCW_FXOSC        = 32000000;
constexpr double   RFM69HCW_FSTEP        = RFM69HCW_FXOSC / 524288.0;
constexpr uint32_t RFM69HCW_BRF_MIN      = 1200;   // FSK min bitrate
constexpr uint32_t RFM69HCW_BRF_MAX      = 300000; // FSK max bitrate
constexpr uint32_t RFM69HCW_BRO_MIN      = 1200;   // OOK min bitrate
constexpr uint32_t RFM69HCW_BRO_MAX      = 32768;  // OOK max bitrate
constexpr uint32_t RFM69HCW_FDA_MIN      = 600;    // FSK min deviation
constexpr uint32_t RFM69HCW_FDA_MAX      = 300000; // FSK max deviation

constexpr size_t   RFM69HCW_FRAME_CAPACITY     = 255 - (sizeof(rfm69hcw_frame_header) - 1);
constexpr size_t   RFM69HCW_FRAME_CAPACITY_AES = 64  - (sizeof(rfm69hcw_frame_header) - 1);

constexpr rfm69hcw_bitrate RFM69HCW_BITRATES[RFM69HCW_BITRATE_MAX + 1] =
{
	{ RFM69HCW_BITRATE_1200,   1200,   0x682B },
	{ RFM69HCW_BITRATE_2400,   2400,   0x3415 },
	{ RFM69HCW_BITRATE_4800,   4800,   0x1A0B },
	{ RFM69HCW_BITRATE_9600,   9600,   0x0D05 },
	{ RFM69HCW_BITRATE_12500,  12500,  0x0A00 },
	{ RFM69HCW_BITRATE_19200,  19200,  0x0683 },
	{ RFM69HCW_BITRATE_25000,  25000,  0x0500 },
	{ RFM69HCW_BITRATE_32768,  32768,  0x03D1 },
	{ RFM69HCW_BITRATE_38400,  38400,  0x0341 },
	{ RFM69HCW_BITRATE_50000,  50000,  0x0280 },
	{ RFM69HCW_BITRATE_57600,  57600,  0x022C },
	{ RFM69HCW_BITRATE_76800,  76800,  0x01A1 },
	{ RFM69HCW_BITRATE_100000, 100000, 0x0140 },
	{ RFM69HCW_BITRATE_115200, 115200, 0x0116 },
	{ RFM69HCW_BITRATE_150000, 150000, 0x00D5 },
	{ RFM69HCW_BITRATE_153600, 153600, 0x00D0 },
	{ RFM69HCW_BITRATE_200000, 200000, 0x00A1 },
	{ RFM69HCW_BITRATE_250000, 250000, 0x0080 },
	{ RFM69HCW_BITRATE_300000, 300000, 0x006B }
};

struct rfm69hcw
{
	spi_inst* spi;
	uint      spi_cs;

	uint      gpio_irq;
	uint      gpio_reset;

	int8_t    rssi;

	int       lna;
	int       power;
	int8_t    sensitivity;

	int       bitrate;
	uint32_t  frequency;
	int       modulation;
	uint16_t  preamble_length;

	uint8_t   network;
	uint8_t   network_address;

	bool      encryption;
	uint8_t   encryption_key[16];
};

template<typename T>
void      rfm69hcw_read_register(rfm69hcw* r, uint8_t address, T& value)
{
	printf("rfm69hcw_read_register(0x%p, 0x%02X)\n", r, address);

	address &= 0x7F;

	gpio_put(r->spi_cs, false);
	spi_write_blocking(r->spi, &address, 1);
	spi_read_blocking(r->spi, 0, (uint8_t*)&value, sizeof(T));
	gpio_put(r->spi_cs, true);
}
void      rfm69hcw_read_register(rfm69hcw* r, uint8_t address, void* buffer, size_t size)
{
	printf("rfm69hcw_read_register(0x%p, 0x%02X, 0x%p, %lu)\n", r, address, buffer, size);

	address &= 0x7F;

	gpio_put(r->spi_cs, false);
	spi_write_blocking(r->spi, &address, 1);
	spi_read_blocking(r->spi, 0, (uint8_t*)buffer, size);
	gpio_put(r->spi_cs, true);
}
template<typename T>
void      rfm69hcw_write_register(rfm69hcw* r, uint8_t address, T value)
{
	printf("rfm69hcw_write_register(0x%p, 0x%02X, %u)\n", r, address, value);

	address |= 0x80;

	gpio_put(r->spi_cs, false);
	spi_write_blocking(r->spi, &address, 1);
	spi_write_blocking(r->spi, (const uint8_t*)&value, sizeof(T));
	gpio_put(r->spi_cs, true);
}
void      rfm69hcw_write_register(rfm69hcw* r, uint8_t address, const void* buffer, size_t size)
{
	printf("rfm69hcw_write_register(0x%p, 0x%02X, 0x%p, %lu)\n", r, address, buffer, size);

	address |= 0x80;

	gpio_put(r->spi_cs, false);
	spi_write_blocking(r->spi, &address, 1);
	spi_write_blocking(r->spi, (const uint8_t*)buffer, size);
	gpio_put(r->spi_cs, true);
}

void      rfm69hcw_set_mode(rfm69hcw* r, int value)
{
	printf("rfm69hcw_set_mode(0x%p, 0x%p)\n", r, value);

	uint8_t mode;
	rfm69hcw_read_register(r, RFM69HCW_RegOpMode, mode);
	mode &= 0xE3;
	mode |= value;
	rfm69hcw_write_register(r, RFM69HCW_RegOpMode, mode);

	uint8_t flags;

	do
		rfm69hcw_read_register(r, RFM69HCW_RegIrqFlags1, flags);
	while (!(flags & 0b10000000));
}

void      rfm69hcw_fifo_pop(rfm69hcw* r, void* buffer, size_t size)
{
	printf("rfm69hcw_fifo_pop(0x%p, 0x%p, %lu)\n", r, buffer, size);

	rfm69hcw_read_register(r, RFM69HCW_RegFifo, buffer, size);
}
void      rfm69hcw_fifo_push(rfm69hcw* r, const void* buffer, size_t size)
{
	printf("rfm69hcw_fifo_push(0x%p, 0x%p, %lu)\n", r, buffer, size);

	rfm69hcw_write_register(r, RFM69HCW_RegFifo, buffer, size);
}
void      rfm69hcw_fifo_clear(rfm69hcw* r)
{
	printf("rfm69hcw_fifo_clear(0x%p)\n", r);

	rfm69hcw_write_register(r, RFM69HCW_RegIrqFlags2, (uint8_t)0b00010000);
}

rfm69hcw* rfm69hcw_open(spi_inst* spi, uint miso, uint mosi, uint sclk, uint cs, uint irq, uint reset)
{
	printf("rfm69hcw_open(0x%p, %u, %u, %u, %u, %u, %u)\n", spi, miso, mosi, sclk, cs, irq, reset);

	auto r = new rfm69hcw
	{
		.spi             = spi,
		.spi_cs          = cs,

		.gpio_irq        = irq,
		.gpio_reset      = reset,

		.rssi            = -127,

		.lna             = RFM69HCW_LNA_GAIN_AUTO,
		.power           = RFM69HCW_POWER_MODE_PA_0 | RFM69HCW_POWER_LEVEL_MIN,
		.sensitivity     = -114,

		.bitrate         = RFM69HCW_BITRATE_1200,
		.frequency       = RFM69HCW_FREQUENCY_MIN,
		.modulation      = RFM69HCW_MODULATION_FSK,
		.preamble_length = RFM69HCW_PREAMBLE_LENGTH_MIN,

		.network         = RFM69HCW_NETWORK_MIN,
		.network_address = RFM69HCW_NETWORK_ADDRESS_MIN
	};

	gpio_init(cs);
	gpio_set_dir(cs, true);
	gpio_put(cs, true);

	gpio_init(irq);
	gpio_set_dir(irq, false);

	gpio_init(reset);
	gpio_set_dir(reset, true);
	gpio_put(reset, false);

	spi_init(spi, 10000000);
	gpio_set_function(miso, GPIO_FUNC_SPI);
	gpio_set_function(sclk, GPIO_FUNC_SPI);
	gpio_set_function(mosi, GPIO_FUNC_SPI);

	rfm69hcw_reset(r);

	return r;
}
void      rfm69hcw_close(rfm69hcw* r)
{
	printf("rfm69hcw_close(0x%p)\n", r);

	rfm69hcw_set_mode(r, RFM69HCW_MODE_SLEEP);

	spi_deinit(r->spi);
	gpio_deinit(r->spi_cs);
	gpio_deinit(r->gpio_irq);
	gpio_deinit(r->gpio_reset);

	delete r;
}
int       rfm69hcw_get_lna(rfm69hcw* r)
{
	return r->lna;
}
int8_t    rfm69hcw_get_rssi(rfm69hcw* r)
{
	return r->rssi;
}
int       rfm69hcw_get_power(rfm69hcw* r)
{
	return r->power;
}
int       rfm69hcw_get_bitrate(rfm69hcw* r)
{
	return r->bitrate;
}
uint32_t  rfm69hcw_get_frequency(rfm69hcw* r)
{
	return r->frequency;
}
int       rfm69hcw_get_modulation(rfm69hcw* r)
{
	return r->modulation;
}
int8_t    rfm69hcw_get_sensitivity(rfm69hcw* r)
{
	return r->sensitivity;
}
uint8_t   rfm69hcw_get_network(rfm69hcw* r)
{
	return r->network;
}
uint8_t   rfm69hcw_get_network_address(rfm69hcw* r)
{
	return r->network_address;
}
bool      rfm69hcw_get_encryption(rfm69hcw* r)
{
	return r->encryption;
}
uint16_t  rfm69hcw_get_preamble_length(rfm69hcw* r)
{
	return r->preamble_length;
}
size_t    rfm69hcw_get_payload_capacity(rfm69hcw* r)
{
	if (r->encryption)
		return RFM69HCW_FRAME_CAPACITY_AES;

	return RFM69HCW_FRAME_CAPACITY;
}
bool      rfm69hcw_set_lna(rfm69hcw* r, int value)
{
	printf("rfm69hcw_set_lna(0x%p, 0x%p)\n", r, value);

	switch (value)
	{
		case RFM69HCW_LNA_OFF:
			r->lna = value;
			rfm69hcw_write_register(r, RFM69HCW_RegTestLna, (uint8_t)0x1B);
			return true;

		case RFM69HCW_LNA_GAIN_AUTO:
		case RFM69HCW_LNA_GAIN_DB_0:
		case RFM69HCW_LNA_GAIN_DB_6:
		case RFM69HCW_LNA_GAIN_DB_12:
		case RFM69HCW_LNA_GAIN_DB_24:
		case RFM69HCW_LNA_GAIN_DB_36:
		case RFM69HCW_LNA_GAIN_DB_48:
		{
			r->lna = value;

			uint8_t lna;
			rfm69hcw_read_register(r, RFM69HCW_RegLna, lna);
			lna &= 0xF8;
			lna |= value;
			rfm69hcw_write_register(r, RFM69HCW_RegLna, lna);
			rfm69hcw_write_register(r, RFM69HCW_RegTestLna, (uint8_t)0x2D);
		}
		return true;
	}

	return false;
}
bool      rfm69hcw_set_power(rfm69hcw* r, int value)
{
	printf("rfm69hcw_set_power(0x%p, 0x%p)\n", r, value);

	bool pa0      = value & RFM69HCW_POWER_MODE_PA_0;
	bool pa1      = value & RFM69HCW_POWER_MODE_PA_1;
	bool pa2      = value & RFM69HCW_POWER_MODE_PA_2;
	auto pa_level = value & RFM69HCW_POWER_LEVEL_MAX;

	if (pa0 && (pa1 || pa2))
		return false;

	if (pa_level > RFM69HCW_POWER_LEVEL_MAX)
		return false;

	r->power = value;

	rfm69hcw_write_register(r, RFM69HCW_RegPaLevel, (uint8_t)value);

	return true;
}
bool      rfm69hcw_set_bitrate(rfm69hcw* r, int value)
{
	printf("rfm69hcw_set_bitrate(0x%p, 0x%p)\n", r, value);

	if ((value < RFM69HCW_BITRATE_MIN) || (value > RFM69HCW_BITRATE_MAX))
		return false;

	auto br = &RFM69HCW_BITRATES[value];

	switch (r->modulation)
	{
		case RFM69HCW_MODULATION_OOK:
			if ((br->rate < RFM69HCW_BRO_MIN) || (br->rate > RFM69HCW_BRO_MAX))
				return false;
			break;

		case RFM69HCW_MODULATION_FSK:
			if ((br->rate < RFM69HCW_BRF_MIN) || (br->rate > RFM69HCW_BRF_MAX))
				return false;
			break;
	}

	r->bitrate = value;
	rfm69hcw_write_register(r, RFM69HCW_RegBitrateMsb, (uint8_t)((br->value & 0xFF00) >> 8));
	rfm69hcw_write_register(r, RFM69HCW_RegBitrateLsb, (uint8_t)(br->value & 0x00FF));

	return true;
}
bool      rfm69hcw_set_frequency(rfm69hcw* r, uint32_t value)
{
	printf("rfm69hcw_set_frequency(0x%p, %lu)\n", r, value);

	if ((value < RFM69HCW_FREQUENCY_MIN) || (value > RFM69HCW_FREQUENCY_MAX))
		return false;

	r->frequency = value;

	value = (uint32_t)((double)value / RFM69HCW_FSTEP);

	rfm69hcw_write_register(r, RFM69HCW_RegFrfMsb, (uint8_t)((value & 0xFF0000) >> 16));
	rfm69hcw_write_register(r, RFM69HCW_RegFrfMid, (uint8_t)((value & 0x00FF00) >> 8));
	rfm69hcw_write_register(r, RFM69HCW_RegFrfLsb, (uint8_t)(value & 0x0000FF));

	return true;
}
bool      rfm69hcw_set_modulation(rfm69hcw* r, int value)
{
	printf("rfm69hcw_set_modulation(0x%p, 0x%p)\n", r, value);

	switch (value)
	{
		case RFM69HCW_MODULATION_OOK:
		case RFM69HCW_MODULATION_FSK:
		{
			r->modulation = value;

			uint8_t data;
			rfm69hcw_read_register(r, RFM69HCW_RegDataModul, data);
			data &= 0xE7;
			data |= value;
			rfm69hcw_write_register(r, RFM69HCW_RegDataModul, data);

			// TODO: update bitrate and deviation
		}
		return true;
	}

	return false;
}
bool      rfm69hcw_set_sensitivity(rfm69hcw* r, int8_t value)
{
	printf("rfm69hcw_set_sensitivity(0x%p, %i)\n", r, value);

	if (value > 0)
		value = 0;

	r->sensitivity = value;

	rfm69hcw_write_register(r, RFM69HCW_RegRssiThresh, (uint8_t)((int)value * -2));

	return true;
}
bool      rfm69hcw_set_network(rfm69hcw* r, uint8_t value)
{
	printf("rfm69hcw_set_network(0x%p, %u)\n", r, value);

	if ((value < RFM69HCW_NETWORK_MIN) || (value > RFM69HCW_NETWORK_MAX))
		return false;

	r->network = value;

	rfm69hcw_write_register(r, RFM69HCW_RegSyncValue1, value);

	return true;
}
bool      rfm69hcw_set_network_address(rfm69hcw* r, uint8_t value)
{
	printf("rfm69hcw_set_network_address(0x%p, %u)\n", r, value);

	if ((value < RFM69HCW_NETWORK_ADDRESS_MIN) || (value > RFM69HCW_NETWORK_ADDRESS_MAX))
		return false;

	r->network_address = value;

	rfm69hcw_write_register(r, RFM69HCW_RegNodeAdrs, value);

	return true;
}
bool      rfm69hcw_set_encryption(rfm69hcw* r, bool value)
{
	printf("rfm69hcw_set_encryption(0x%p, %s)\n", r, value ? "true" : "false");

	r->encryption = value;

	uint8_t config;
	rfm69hcw_read_register(r, RFM69HCW_RegPacketConfig2, config);
	config &= 0xFE;
	if (value) config |= 1;
	rfm69hcw_write_register(r, RFM69HCW_RegPacketConfig2, config);

	return true;
}
bool      rfm69hcw_set_encryption_key(rfm69hcw* r, const uint8_t(&value)[16])
{
	printf("rfm69hcw_set_encryption_key(0x%p, 0x%p)\n", r, value);

	memcpy(r->encryption_key, value, 16);
	rfm69hcw_write_register(r, RFM69HCW_RegAesKey1, value, 16);

	return true;
}
bool      rfm69hcw_set_preamble_length(rfm69hcw* r, uint16_t value)
{
	printf("rfm69hcw_set_preamble_length(0x%p, %u)\n", r, value);

	if ((value < RFM69HCW_PREAMBLE_LENGTH_MIN) || (value > RFM69HCW_PREAMBLE_LENGTH_MAX))
		return false;

	r->preamble_length = value;
	rfm69hcw_write_register(r, RFM69HCW_RegPreambleMsb, (uint8_t)(value & 0xFF00) >> 8);
	rfm69hcw_write_register(r, RFM69HCW_RegPreambleLsb, (uint8_t)(value & 0x00FF));

	return true;
}
void      rfm69hcw_reset(rfm69hcw* r)
{
	printf("rfm69hcw_reset(0x%p)\n", r);

	gpio_put(r->gpio_reset, true);
	busy_wait_us(100);
	gpio_put(r->gpio_reset, false);
	busy_wait_ms(5);

	rfm69hcw_set_mode(r, RFM69HCW_MODE_STANDBY);

	rfm69hcw_write_register(r, RFM69HCW_RegFifoThresh, (uint8_t)0x8F);
	rfm69hcw_write_register(r, RFM69HCW_RegSyncConfig, (uint8_t)0b10000000);
	rfm69hcw_write_register(r, RFM69HCW_RegSyncValue1, r->network);
	rfm69hcw_write_register(r, RFM69HCW_RegNodeAdrs, r->network_address);
	rfm69hcw_write_register(r, RFM69HCW_RegPacketConfig1, (uint8_t)0b10010100);
	rfm69hcw_write_register(r, RFM69HCW_RegPacketConfig2, (uint8_t)0b00000000);
	rfm69hcw_write_register(r, RFM69HCW_RegBroadcastAdrs, (uint8_t)RFM69HCW_NETWORK_ADDRESS_BROADCAST);

	rfm69hcw_set_lna(r, r->lna);
	rfm69hcw_set_power(r, r->power);
	rfm69hcw_set_bitrate(r, r->bitrate);
	rfm69hcw_set_frequency(r, r->frequency);
	rfm69hcw_set_modulation(r, r->modulation);
	rfm69hcw_set_sensitivity(r, r->sensitivity);
	rfm69hcw_set_preamble_length(r, r->preamble_length);
}
// @return number of bytes sent
size_t    rfm69hcw_send(rfm69hcw* r, const void* buffer, size_t size)
{
	printf("rfm69hcw_send(0x%p, 0x%p, %lu)\n", r, buffer, size);

	return rfm69hcw_send_to(r, buffer, size, RFM69HCW_NETWORK_ADDRESS_BROADCAST);
}
// @return number of bytes sent
size_t    rfm69hcw_send_to(rfm69hcw* r, const void* buffer, size_t size, uint8_t address)
{
	printf("rfm69hcw_send_to(0x%p, 0x%p, %lu, %u)\n", r, buffer, size, address);

	if (auto capacity = rfm69hcw_get_payload_capacity(r); size > capacity)
		size = capacity;

	uint8_t               reg;
	rfm69hcw_frame_header header =
	{
		.size        = (uint8_t)((sizeof(rfm69hcw_frame_header) - 1) + size),
		.destination = address,
		.source      = r->network_address
	};

	rfm69hcw_fifo_push(r, &header, sizeof(rfm69hcw_frame_header));
	if (buffer && size) rfm69hcw_fifo_push(r, buffer, size);

	rfm69hcw_set_mode(r, RFM69HCW_MODE_TRANSMITTER);

	do
		rfm69hcw_read_register(r, RFM69HCW_RegIrqFlags2, reg);
	while (!(reg & 0b00001000)); // PacketSent

	rfm69hcw_set_mode(r, RFM69HCW_MODE_STANDBY);

	return size;
}
// @return number of bytes received
size_t    rfm69hcw_receive(rfm69hcw* r, void* buffer, size_t size, uint8_t* address, int8_t* rssi)
{
	printf("rfm69hcw_receive(0x%p, 0x%p, %lu, 0x%p, 0x%p)\n", r, buffer, size, address, rssi);

	struct context
	{
		int8_t*  rssi;
		uint8_t* address;
		size_t   number_of_bytes_received;
	};

	context c =
	{
		.rssi    = rssi,
		.address = address
	};

	rfm69hcw_receive_while(r, buffer, size, [](rfm69hcw* r, void* buffer, size_t size, uint8_t address, int8_t rssi, void* param) {
		auto c = (context*)param;

		if (c->rssi)    *c->rssi    = rssi;
		if (c->address) *c->address = address;

		c->number_of_bytes_received = size;

		return false;
	}, &c);

	return c.number_of_bytes_received;
}
// @return number of bytes received
size_t    rfm69hcw_receive_from(rfm69hcw* r, void* buffer, size_t size, uint8_t address, int8_t* rssi)
{
	printf("rfm69hcw_receive_from(0x%p, 0x%p, %lu, %u, 0x%p)\n", r, buffer, size, address, rssi);

	uint8_t sender;
	size_t  number_of_bytes_received;

	do
		number_of_bytes_received = rfm69hcw_receive(r, buffer, size, &sender, rssi);
	while (sender != address);

	return number_of_bytes_received;
}
void      rfm69hcw_receive_while(rfm69hcw* r, void* buffer, size_t size, rfm69hcw_receive_callback callback, void* param)
{
	printf("rfm69hcw_receive_while(0x%p, 0x%p, %lu, 0x%p, 0x%p)\n", r, buffer, size, callback, param);

	uint8_t               reg;
	int8_t                rssi;
	rfm69hcw_frame_header header;
	size_t                payload_size;

	rfm69hcw_set_mode(r, RFM69HCW_MODE_RECEIVER);

	rfm69hcw_read_register(r, RFM69HCW_RegPacketConfig2, reg);
	reg |= 0b00000100; // RestartRx
	rfm69hcw_write_register(r, RFM69HCW_RegPacketConfig2, reg);

	do
	{
		do
			rfm69hcw_read_register(r, RFM69HCW_RegIrqFlags2, reg);
		while (!(reg & 0b00000100)); // PayloadReady

		rfm69hcw_set_mode(r, RFM69HCW_MODE_STANDBY);

		rfm69hcw_read_register(r, RFM69HCW_RegRssiValue, reg);
		rssi = -(int8_t)(reg / 2);

		rfm69hcw_fifo_pop(r, &header, sizeof(rfm69hcw_frame_header));
		payload_size = header.size - (sizeof(rfm69hcw_frame_header) - 1);
		rfm69hcw_fifo_pop(r, buffer, payload_size);

		rfm69hcw_set_mode(r, RFM69HCW_MODE_RECEIVER);

		rfm69hcw_read_register(r, RFM69HCW_RegPacketConfig2, reg);
		reg |= 0b00000100; // RestartRx
		rfm69hcw_write_register(r, RFM69HCW_RegPacketConfig2, reg);
	} while (callback(r, buffer, payload_size, header.source, rssi, param));

	rfm69hcw_set_mode(r, RFM69HCW_MODE_STANDBY);
}
