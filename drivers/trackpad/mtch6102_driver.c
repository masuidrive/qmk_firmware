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
#include "wait.h"
#include "action_layer.h"

#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#define delay(us) _delay_us(us)

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

#define OP_MODE_STANDBY 0x00
#define OP_MODE_GESTURE 0x01
#define OP_MODE_TOUCH 0x02
#define OP_MODE_FULL 0x03
#define OP_MODE_RAW 0x04


#define I2C_RETRY_COUNT 20
#define TOUCH_UP -1 /* last_x */
#define POINTER_DISABLE -1 /* last_y */
static double last_x = TOUCH_UP;
static double last_y = POINTER_DISABLE;
static int16_t touchdown_x = -1;
static int16_t touchdown_y = -1;
static uint32_t touchdown_time;

#ifdef MTCH6102_CLICK_ENABLE
static bool mouse_btn1_click_emu = false;
static uint32_t mouse_btn1_timer;
#endif

/* scan interval timer */
static uint16_t last_scan_at = 0;
#define SCAN_INTERVAL 10 // ms

#ifdef MTCH6102_DEBUG
# include "debug.h"
# define DEBUG(fmt, ...) xprintf(fmt, ##__VA_ARGS__)
#else
# define DEBUG(fmt, ...)
#endif

/* Read 1 byte from register of MTCH6102 */
static uint8_t i2c_read_register(const uint8_t reg) {
    uint8_t data;
    i2c_readReg(MTCH6102_I2C_ADDR << 1, reg, &data, 1, MTCH6102_I2C_TIMEOUT);
    return data;
}

/* Write 1 byte from register of MTCH6102 */
static i2c_status_t i2c_write_register(const uint8_t reg, const uint8_t data) {
    return i2c_writeReg(MTCH6102_I2C_ADDR << 1, reg, &data, 1, MTCH6102_I2C_TIMEOUT);
}

/* Initialize MTCH6102 */
void pointing_device_init(void) {
    // Wait MTCH6102 to wake up
    wait_ms(1500);

    /* Configure I2C Master */
    i2c_init();

    /* Configure MTCH6102 */
    for(int i = 0; i < I2C_RETRY_COUNT; ++i) {
        wait_ms(100);
        if(i2c_write_register(OP_MODE, OP_MODE_TOUCH) != I2C_STATUS_SUCCESS) continue;
        if(i2c_write_register(NUMBEROFXCHANNELS, MTCH6102_MATRIX_WIDTH) != I2C_STATUS_SUCCESS) continue;
        if(i2c_write_register(NUMBEROFYCHANNELS, MTCH6102_MATRIX_HEIGHT) != I2C_STATUS_SUCCESS) continue;
        last_y = 0; // not POINTER_DISABLE
        break;
    }
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

#ifdef MTCH6102_CLICK_ENABLE
    if(mouse_btn1_click_emu && timer_elapsed32(mouse_btn1_timer) > MTCH6102_EMU_CLICK_TIME) {
        mouse_btn1_click_emu = false;
        report.buttons &= ~MOUSE_BTN1; // mouse btn1 up
    }
#endif

    //　if (timer_elapsed(last_scan_at) > SCAN_INTERVAL) DEBUG("%d,%d\n",layer_state,last_scan_at);

    if (timer_elapsed(last_scan_at) > SCAN_INTERVAL && last_y != POINTER_DISABLE) {
        last_scan_at = timer_read();

        const uint8_t touch_state = i2c_read_register(TOUCHSTATE);
        const uint8_t touch_lsb = i2c_read_register(TOUCHLSB);
        const double touch_x = ((i2c_read_register(TOUCHX) << 4) + ((touch_lsb & 0xf0) >> 4)) * MTCH6102_PAD_SCALE;
        const double touch_y = ((i2c_read_register(TOUCHY) << 4) + (touch_lsb & 0x0f)) * MTCH6102_PAD_SCALE;
        const bool touch_event = !!(touch_state & 0x01);
        /* Never use MTCH6102 on-chip gesture recognizer */

        if (touch_event) {
            /* set difference current x/y to previously x/y to report.x/y */
            if (last_x != TOUCH_UP) {
                if(layer_state_is(default_layer_state) || layer_state_is(3)) {
                    report.y = (last_x - touch_x);
                    report.x = (last_y - touch_y) * -1;
                } else {
                    report.v = (last_x - touch_x) * MTCH6102_SCROLL_SCALE;
                    report.h = (last_y - touch_y) * MTCH6102_SCROLL_SCALE;// * -1;
                }
            }
            last_x = touch_x;
            last_y = touch_y;

            /* record touchdown time for mouse btn emulation */
            if(touchdown_x < 0) {
                if(layer_state_is(default_layer_state)) {
                    layer_on(3);
                }
                touchdown_x = touch_x;
                touchdown_y = touch_y;
                touchdown_time = timer_read32();
            }
        }
        else { /* if touch up */
            if (last_x != TOUCH_UP && touchdown_x >=0) {
                layer_off(3);
#ifdef MTCH6102_CLICK_ENABLE

                /* Tap within MTCH6102_TAPTIME and MTCH6102_TAPDISTANCE */
                /* then emulate $MTCH6102_EMU_CLICK_TIME ms of mouse button 1 clicks. */
                if(timer_elapsed32(touchdown_time) < MTCH6102_TAPTIME) {
                    const int16_t distance = abs(sqrt(pow(touchdown_x - touch_x, 2.0) + pow(touchdown_y - touch_y, 2.0)));
                    // DEBUG("distance=%d, time=%d\n", distance, timer_elapsed32(touchdown_time));
                    if(distance < MTCH6102_TAPDISTANCE) {
                        mouse_btn1_click_emu = true;
                        mouse_btn1_timer = timer_read32();
                        report.buttons |= MOUSE_BTN1;
                    }
                }
#endif

            }
            // reset vars
            last_x = TOUCH_UP;
            touchdown_x = -1;
            touchdown_y = -1;
        }
    }

    if (has_report_changed(report, pointing_device_get_report())) {
        pointing_device_set_report(report);
        pointing_device_send();
    }
}

/*
Copyright 2021 Yuichiro MASUI <https://twitter.com/masuidrive>
This software is licensed with a Modified BSD License.
*/
