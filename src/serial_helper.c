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

#include "serial_helper.h"

void phex4(uint8_t c) {
	usart_send_blocking(P_USART, (uint8_t)(c + ((c < 10) ? '0' : 'A' - 10)));
}

void phex(uint8_t c) {
	phex4((c>>4));
	phex4((c&15));
}

void phex16(uint16_t c) {
	phex((uint8_t)(c>>8));
	phex((uint8_t)c);
}

void phex24(uint32_t c) {
	phex16((uint16_t)((c>>8)&0xFFFF));
	phex((uint8_t)c);
}

void phex32(uint32_t c) {
	phex16((uint16_t)(c>>16));
	phex16((uint16_t)(c&0xFFFF));
}

void pent(void) {
	usart_send_blocking(P_USART, '\r');
	usart_send_blocking(P_USART, '\n');
}

void pstr(char *s) {
	uint16_t i = 0;
	while(s[i]) {
		usart_send_blocking(P_USART, (uint8_t)(s[i++]));
	}
}

// vim: tabstop=4:shiftwidth=4:noexpandtab
