/*
 * RosettaPad - DualShock 4 (PS4) Controller Driver
 * ================================================
 *
 * Supports DualShock 4 v1 (0x054C:0x05C4) and v2 (0x054C:0x09CC)
 * over Bluetooth (Report ID 0x11) and USB (Report ID 0x01).
 */

#ifndef ROSETTAPAD_DUALSHOCK4_H
#define ROSETTAPAD_DUALSHOCK4_H

#include "controllers/controller_interface.h"

/* ============================================================================
 * DEVICE IDENTIFICATION
 * ============================================================================ */
#define DUALSHOCK4_VID          0x054C   /* Sony */
#define DUALSHOCK4_PID_V1       0x05C4   /* DualShock 4 CUH-ZCT1 */
#define DUALSHOCK4_PID_V2       0x09CC   /* DualShock 4 CUH-ZCT2 */

/* ============================================================================
 * REPORT FORMATS
 * ============================================================================ */
#define DS4_USB_REPORT_ID       0x01
#define DS4_USB_INPUT_SIZE      64
#define DS4_USB_OUTPUT_ID       0x05
#define DS4_USB_OUTPUT_SIZE     32

#define DS4_BT_REPORT_ID        0x11
#define DS4_BT_INPUT_SIZE       78
#define DS4_BT_OUTPUT_SIZE      78

/* Common payload offsets (relative to USB report[0]=0x01; BT 0x11 shifts by +2) */
#define DS4_OFF_LX              1
#define DS4_OFF_LY              2
#define DS4_OFF_RX              3
#define DS4_OFF_RY              4
#define DS4_OFF_BUTTONS1        5    /* D-pad (low nibble) + face buttons */
#define DS4_OFF_BUTTONS2        6    /* Shoulders, triggers, share, options, L3/R3 */
#define DS4_OFF_BUTTONS3        7    /* PS button (bit 0), touchpad click (bit 1), counter */
#define DS4_OFF_L2              8
#define DS4_OFF_R2              9
#define DS4_OFF_GYRO_X          13   /* pitch */
#define DS4_OFF_GYRO_Y          15   /* yaw   */
#define DS4_OFF_GYRO_Z          17   /* roll  */
#define DS4_OFF_ACCEL_X         19
#define DS4_OFF_ACCEL_Y         21
#define DS4_OFF_ACCEL_Z         23
#define DS4_OFF_BATTERY         30   /* bits 0..3 = level, bit 4 = cable connected */
#define DS4_OFF_TOUCHPAD        33   /* touch packet count at [33], finger 0 at [35..38] */

/* Button masks - buttons1 */
#define DS4_BTN1_SQUARE         0x10
#define DS4_BTN1_CROSS          0x20
#define DS4_BTN1_CIRCLE         0x40
#define DS4_BTN1_TRIANGLE       0x80

/* Button masks - buttons2 */
#define DS4_BTN2_L1             0x01
#define DS4_BTN2_R1             0x02
#define DS4_BTN2_L2             0x04
#define DS4_BTN2_R2             0x08
#define DS4_BTN2_SHARE          0x10
#define DS4_BTN2_OPTIONS        0x20
#define DS4_BTN2_L3             0x40
#define DS4_BTN2_R3             0x80

/* Button masks - buttons3 */
#define DS4_BTN3_PS             0x01
#define DS4_BTN3_TOUCHPAD       0x02

/* Touchpad constants */
#define DS4_TOUCHPAD_WIDTH      1920
#define DS4_TOUCHPAD_HEIGHT     943
#define DS4_TOUCH_INACTIVE      0x80

/* ============================================================================
 * CALIBRATION DATA (Feature Report 0x02 over USB, 0x05 over BT)
 * ============================================================================ */
#define DS4_FEAT_CALIB_USB      0x02
#define DS4_FEAT_CALIB_USB_SIZE 37
#define DS4_FEAT_CALIB_BT       0x05
#define DS4_FEAT_CALIB_BT_SIZE  41

#define DS4_ACC_RES_PER_G       8192
#define DS4_ACC_RANGE           (4 * DS4_ACC_RES_PER_G)
#define DS4_GYRO_RES_PER_DEG_S  1024
#define DS4_GYRO_RANGE          (2048 * DS4_GYRO_RES_PER_DEG_S)

typedef struct {
    int16_t bias;
    int sens_numer;
    int sens_denom;
} ds4_axis_calib_t;

typedef struct {
    ds4_axis_calib_t gyro[3];
    ds4_axis_calib_t accel[3];
    int valid;
} ds4_calibration_t;

/* ============================================================================
 * DRIVER INTERFACE
 * ============================================================================ */
const controller_driver_t* dualshock4_get_driver(void);
void dualshock4_register(void);
void dualshock4_parse_dpad(uint8_t buttons1, controller_state_t* out_state);

#endif /* ROSETTAPAD_DUALSHOCK4_H */
