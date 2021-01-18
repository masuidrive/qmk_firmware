/*
MTCH6102 Trackpad I2C chip driver for QMK.

Copyright 2021 Yuichiro MASUI <https://twitter.com/masuidrive>
This software is licensed with a Modified BSD License.
*/

#pragma once

/* Configuration */
#define MTCH6102_I2C_ADDR 0x25
#define MTCH6102_I2C_TIMEOUT 100 // ms
#define MTCH6102_MATRIX_WIDTH 7
#define MTCH6102_MATRIX_HEIGHT 6
#define MTCH6102_PAD_SCALE 1.0
#define MTCH6102_SCROLL_SCALE 0.08
#define MTCH6102_TAPDISTANCE 24 // px
#define MTCH6102_TAPTIME 100 // ms
#define MTCH6102_EMU_CLICK_TIME 10 // ms click emulation time
#define MTCH6102_DEBUG // print debug message

// #define MTCH6102_CLICK_ENABLE // Enable tap to click feature

/* use pointing device call hooks */
extern void pointing_device_init(void);
extern void pointing_device_task(void);
