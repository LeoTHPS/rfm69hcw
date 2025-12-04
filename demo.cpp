#include "rfm69hcw.hpp"

#include <pico/time.h>
#include <pico/stdio.h>
#include <pico/stdio_usb.h>
#include <pico/status_led.h>

#include <hardware/gpio.h>

#define RFM69HCW_EN       16 // Power (high = on)
#define RFM69HCW_G0       6  // IRQ
#define RFM69HCW_RST      15 // Reset
#define RFM69HCW_SPI      spi0
#define RFM69HCW_SPI_CS   7
#define RFM69HCW_SPI_SCLK 2
#define RFM69HCW_SPI_MISO 4
#define RFM69HCW_SPI_MOSI 3

#define RFM69HCW_SEND
// #define RFM69HCW_RECV

int main()
{
	stdio_init_all();

	gpio_init(RFM69HCW_EN);
	gpio_set_dir(RFM69HCW_EN, true);
	gpio_put(RFM69HCW_EN, true);

	sleep_ms(1000);

	status_led_init();
	status_led_set_state(false);

	if (auto r = rfm69hcw_open(RFM69HCW_SPI, RFM69HCW_SPI_MISO, RFM69HCW_SPI_MOSI, RFM69HCW_SPI_SCLK, RFM69HCW_SPI_CS, RFM69HCW_G0, RFM69HCW_RST))
	{
		rfm69hcw_set_lna(r, RFM69HCW_LNA_GAIN_AUTO);
		rfm69hcw_set_power(r, RFM69HCW_POWER_MODE_PA_1 | RFM69HCW_POWER_MODE_PA_2 | RFM69HCW_POWER_LEVEL_MAX);
		rfm69hcw_set_bitrate(r, RFM69HCW_BITRATE_115200);
		rfm69hcw_set_frequency(r, 915100000);
		rfm69hcw_set_modulation(r, RFM69HCW_MODULATION_FSK);
		rfm69hcw_set_network(r, RFM69HCW_NETWORK_MIN);
		rfm69hcw_set_network_address(r, RFM69HCW_NETWORK_ADDRESS_MIN);

		uint8_t command = 1;

#if defined(RFM69HCW_SEND)
		while (rfm69hcw_send(r, &command, sizeof(command)))
		{
			status_led_set_state(true);
			sleep_ms(100);
			status_led_set_state(false);

			sleep_ms(2400);
		}
#elif defined(RFM69HCW_RECV)
		rfm69hcw_receive_while(r, &command, sizeof(command), [](rfm69hcw* r, void* buffer, size_t size, uint8_t address, int8_t rssi, void* param) {
			status_led_set_state(true);
			sleep_ms(100);
			status_led_set_state(false);

			return true;
		}, nullptr);
#endif

		rfm69hcw_close(r);
	}

	status_led_deinit();

	gpio_put(RFM69HCW_EN, false);
	gpio_deinit(RFM69HCW_EN);

	stdio_deinit_all();

	return 0;
}
