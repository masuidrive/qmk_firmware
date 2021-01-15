/*
Copyright 2021 Yuichiro MASUI <https://github.com/masuidrive>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

/* configuration */
#define MTCH6102_I2C_ADDR 0x25
#define MTCH6102_I2C_TIMEOUT 10000
#define MTCH6102_MATRIX_WIDTH 7
#define MTCH6102_MATRIX_HEIGHT 6
#define MTCH6102_TAPDISTANCE 50
#define MTCH6102_TAPINTERVAL 10

extern void pointing_device_init(void);
extern void pointing_device_task(void);

