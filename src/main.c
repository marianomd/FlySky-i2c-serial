/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Deviation is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Deviation.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include <libopencm3/stm32/usart.h>
#include <string.h>
#include "serial_helper.h"

/*
    Code based on https://github.com/flabbergast/libopencm3-ex/tree/master/i2c_eep
*/

static void usart_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

	/* USART1 pins: PA9/TX PA10/RX */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO9 | GPIO10);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	//usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOF);

	/* Set PF3 for LED */
	gpio_mode_setup(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
}

static void i2c_setup(void)
{
	uint32_t i2c;

	/* Enable clocks for I2C2. */
	rcc_periph_clock_enable(RCC_I2C2);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Set alternate functions for the SCL and SDA pins of I2C2. */
	gpio_set_af(GPIOB, GPIO_AF1, GPIO10 | GPIO11);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);

	i2c = I2C2;

	i2c_reset(i2c);

	/* Disable the I2C before changing any configuration. */
	i2c_peripheral_disable(i2c);

	/* setup from libopencm3-examples */
	i2c_enable_analog_filter(i2c);
	i2c_set_digital_filter(i2c, 0);
	i2c_set_speed(i2c, i2c_speed_sm_100k, 8);
	i2c_enable_stretching(i2c);
	i2c_set_7bit_addr_mode(i2c);

	/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(i2c);
}

#define BLOCK_SIZE 8

static void read_block(uint16_t addr)
{
	uint32_t i2c = I2C2;
	uint8_t eeprom_addr = 0x50;

	uint8_t temp[BLOCK_SIZE];
	uint8_t wb[2];

	wb[0] = (uint8_t)(addr >> 8);
	wb[1] = (uint8_t)(addr & 0xFF);

	memset(temp, 0, BLOCK_SIZE);

	// read
	i2c_transfer7(i2c, eeprom_addr, wb, 2, temp, BLOCK_SIZE);

	uint8_t i;
	for (i = 0; i < BLOCK_SIZE; i++)
	{
		phex(temp[i]);
	}
}

int main(void)
{
	rcc_clock_setup_in_hsi_out_48mhz();

	gpio_setup();
	gpio_set(GPIOF, GPIO3); /* LED on */

	usart_setup();

	/* Send a message on USART1. */
	pstr("Leemos bytes de EEPROM:\r\n");

	i2c_setup();

	// write example
	// temp[0] = 0;
	// temp[1] = 0;
	// bytes to write
	// temp[2] = 0x11;
	// temp[3] = 0x22;
	// temp[4] = 0x33;

	// write
	//i2c_transfer7(i2c, eeprom_addr, temp, 5, NULL, 0);

	// uint32_t t;
	// the pause needs to be long enough for the eeprom to finish
	// writing the data above; otherwise the 'read' call below
	// hangs
	// t = 500000;
	// while(t-- > 0)
	// 	__asm__("nop");

	// read example
	// two byte addresses for bigger eeproms (change also transfer7 call!)
	uint16_t block_num = 0;

	for (block_num = 0; block_num < 2048; block_num++)
	{
		read_block(block_num * BLOCK_SIZE);
	}

	pent();

	while (1)
		; /* Halt. */

	return 0;
}
