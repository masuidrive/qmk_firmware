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
/*
 * Official document for MTCH6102
 * https://ww1.microchip.com/downloads/en/DeviceDoc/40001750A.pdf
 * https://ww1.microchip.com/downloads/jp/DeviceDoc/40001750A_JP.pdf
 */

#include "mtch6102_drive.h"

#include "i2c_master.h"
#include "pointing_device.h"
#include "report.h"
#include "timer.h"

/* I2C commands on MTCH6102 */
#define OP_MODE 0x05
#define NUMBEROFXCHANNELS 0x20
#define NUMBEROFYCHANNELS 0x21
#define TOUCHSTATE 0x10
#define TOUCHX 0x11
#define TOUCHY 0x12
#define TOUCHLSB 0x13
#define GESTURESTATE 0x14
#define GESTUREDIAG 0x15
#define TAPDISTANCE 0x3A

#define OP_MODE_GESTURE 0x03


#define TOUCH_UP -1 /* last_x */
#define POINTER_DISABLE -1 /* last_y */
static int16_t last_x = TOUCH_UP;
static int16_t last_y = POINTER_DISABLE;

/* interval timer */
static uint16_t last_cursor = 0;
const uint8_t cursor_timeout = 10;


/*
uint16_t timer_read(void);
uint32_t timer_read32(void);
uint16_t timer_elapsed(uint16_t last);
*/

/* Read 1 byte from register of MTCH6102 */
static uint8_t i2c_read_register(const uint8_t reg) {
    uint8_t data;
    i2c_readReg(MTCH6102_I2C_ADDR << 1, reg, &data, sizeof(data), MTCH6102_I2C_TIMEOUT);
    return data;
}

/* Write 1 byte from register of MTCH6102 */
static i2c_status_t i2c_write_register(const uint8_t reg, const uint8_t data) {
    return i2c_writeReg(MTCH6102_I2C_ADDR << 1, reg, &data, sizeof(data), MTCH6102_I2C_TIMEOUT);
}

/* Initialize MTCH6102 */
void pointing_device_init(void) {
    last_y = POINTER_DISABLE;

    /* Configure I2C Master */
    i2c_init();
    if(i2c_start(MTCH6102_I2C_ADDR << 1, MTCH6102_I2C_TIMEOUT) != I2C_STATUS_SUCCESS) return;

    /* Configure MTCH6102 */
    if(i2c_write_register(OP_MODE, OP_MODE_GESTURE) != I2C_STATUS_SUCCESS) return;
    if(i2c_write_register(NUMBEROFXCHANNELS, MTCH6102_MATRIX_WIDTH) != I2C_STATUS_SUCCESS) return;
    if(i2c_write_register(NUMBEROFYCHANNELS, MTCH6102_MATRIX_HEIGHT) != I2C_STATUS_SUCCESS) return;
    if(i2c_write_register(TAPDISTANCE, MTCH6102_TAPDISTANCE) != I2C_STATUS_SUCCESS) return;

    last_y = 0;
}

static bool has_report_changed(const report_mouse_t first, const report_mouse_t second) {
    return !(
        (!first.buttons && first.buttons == second.buttons) &&
        (!first.x && first.x == second.x) &&
        (!first.y && first.y == second.y) &&
        (!first.h && first.h == second.h) &&
        (!first.v && first.v == second.v)
    );
}

void pointing_device_task(void) {
    report_mouse_t report = pointing_device_get_report();

    if (timer_elapsed(last_cursor) > cursor_timeout && last_y != POINTER_DISABLE) {
        /* Load pointer data from MTCH */
        const uint8_t touch_state = i2c_read_register(TOUCHSTATE);
        const uint8_t touch_lsb = i2c_read_register(TOUCHLSB);
        const uint16_t touch_x = (i2c_read_register(TOUCHX) << 4) + ((touch_lsb & 0xf0) >> 4);
        const uint16_t touch_y = (i2c_read_register(TOUCHY) << 4) + (touch_lsb & 0x0f);
        const bool touch_event = !!(touch_state & 0x01);
        const bool gesture_event = !!(touch_state & 0x02);

        if (gesture_event) {
            uint8_t gesture_state = i2c_read_register(GESTURESTATE);
            if (gesture_state == 0x10 || gesture_state == 0x11) {
                report.buttons |= MOUSE_BTN1;
            } else {
                report.buttons &= ~MOUSE_BTN1;
            }
        }

        if (touch_event) {
            if (last_x != TOUCH_UP) {
                report.y = (last_x - touch_x);
                report.x = (last_y - touch_y) * -1;
            }
            last_x = touch_x;
            last_y = touch_y;
        }
        else {
            last_x = TOUCH_UP;
            // report.buttons &= ~MOUSE_BTN1;
        }

        last_cursor = timer_read();
    }

    if (has_report_changed(report, pointing_device_get_report())) {
        pointing_device_set_report(report);
        pointing_device_send();
    }
}
