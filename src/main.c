#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include <libopencm3/stm32/usart.h>
#include <string.h>
#include "serial_helper.h"


// /// @brief  Possible STM32 system reset causes
// typedef enum reset_cause_e
// {
//     RESET_CAUSE_UNKNOWN = 0,
//     RESET_CAUSE_LOW_POWER_RESET,
//     RESET_CAUSE_WINDOW_WATCHDOG_RESET,
//     RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
//     RESET_CAUSE_SOFTWARE_RESET,
//     RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
//     RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
//     RESET_CAUSE_BROWNOUT_RESET,
// } reset_cause_t;

// /// @brief      Obtain the STM32 system reset cause
// /// @param      None
// /// @return     The system reset cause
// reset_cause_t reset_cause_get(void)
// {
//     reset_cause_t reset_cause;

//     if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
//     {
//         reset_cause = RESET_CAUSE_LOW_POWER_RESET;
//     }
//     else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
//     {
//         reset_cause = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
//     }
//     else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
//     {
//         reset_cause = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
//     }
//     else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
//     {
//         reset_cause = RESET_CAUSE_SOFTWARE_RESET; // This reset is induced by calling the ARM CMSIS `NVIC_SystemReset()` function!
//     }
//     else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
//     {
//         reset_cause = RESET_CAUSE_POWER_ON_POWER_DOWN_RESET;
//     }
//     else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
//     {
//         reset_cause = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
//     }
//     // Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to ensure first that the reset cause is 
//     // NOT a POR/PDR reset. See note below. 
//     else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
//     {
//         reset_cause = RESET_CAUSE_BROWNOUT_RESET;
//     }
//     else
//     {
//         reset_cause = RESET_CAUSE_UNKNOWN;
//     }

//     // Clear all the reset flags or else they will remain set during future resets until system power is fully removed.
//     __HAL_RCC_CLEAR_RESET_FLAGS();

//     return reset_cause; 
// }

// // Note: any of the STM32 Hardware Abstraction Layer (HAL) Reset and Clock Controller (RCC) header
// // files, such as "STM32Cube_FW_F7_V1.12.0/Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal_rcc.h",
// // "STM32Cube_FW_F2_V1.7.0/Drivers/STM32F2xx_HAL_Driver/Inc/stm32f2xx_hal_rcc.h", etc., indicate that the 
// // brownout flag, `RCC_FLAG_BORRST`, will be set in the event of a "POR/PDR or BOR reset". This means that a 
// // Power-On Reset (POR), Power-Down Reset (PDR), OR Brownout Reset (BOR) will trip this flag. See the 
// // doxygen just above their definition for the `__HAL_RCC_GET_FLAG()` macro to see this:
// // "@arg RCC_FLAG_BORRST: POR/PDR or BOR reset." <== indicates the Brownout Reset flag will *also* be set in 
// // the event of a POR/PDR. 
// // Therefore, you must check the Brownout Reset flag, `RCC_FLAG_BORRST`, *after* first checking the 
// // `RCC_FLAG_PORRST` flag in order to ensure first that the reset cause is NOT a POR/PDR reset.


// /// @brief      Obtain the system reset cause as an ASCII-printable name string from a reset cause type
// /// @param[in]  reset_cause     The previously-obtained system reset cause
// /// @return     A null-terminated ASCII name string describing the system reset cause
// const char * reset_cause_get_name(reset_cause_t reset_cause)
// {
//     const char * reset_cause_name = "TBD";

//     switch (reset_cause)
//     {
//         case RESET_CAUSE_UNKNOWN:
//             reset_cause_name = "UNKNOWN";
//             break;
//         case RESET_CAUSE_LOW_POWER_RESET:
//             reset_cause_name = "LOW_POWER_RESET";
//             break;
//         case RESET_CAUSE_WINDOW_WATCHDOG_RESET:
//             reset_cause_name = "WINDOW_WATCHDOG_RESET";
//             break;
//         case RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET:
//             reset_cause_name = "INDEPENDENT_WATCHDOG_RESET";
//             break;
//         case RESET_CAUSE_SOFTWARE_RESET:
//             reset_cause_name = "SOFTWARE_RESET";
//             break;
//         case RESET_CAUSE_POWER_ON_POWER_DOWN_RESET:
//             reset_cause_name = "POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR)";
//             break;
//         case RESET_CAUSE_EXTERNAL_RESET_PIN_RESET:
//             reset_cause_name = "EXTERNAL_RESET_PIN_RESET";
//             break;
//         case RESET_CAUSE_BROWNOUT_RESET:
//             reset_cause_name = "BROWNOUT_RESET (BOR)";
//             break;
//     }

//     return reset_cause_name;
// }

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

	/* Set PA15 for LED */
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

	// inicializamos a cero
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

// vim: tabstop=4:shiftwidth=4:noexpandtab
