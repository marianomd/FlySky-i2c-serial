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

#ifndef SERIAL_HELPER_H
#define SERIAL_HELPER_H

#include <libopencm3/stm32/usart.h>

#define P_USART USART1

void phex4(uint8_t c);
void phex(uint8_t c);
void phex16(uint16_t c);
void phex24(uint32_t c);
void phex32(uint32_t c);
void pent(void);

void pstr(char *s);

#endif /* SERIAL_HELPER_H */