/*
 * RosettaPad - DualShock 4 (PS4) Controller Driver
 * ================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <linux/hidraw.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#include "core/common.h"
#include "controllers/dualshock4/dualshock4.h"

/* ============================================================================
 * CRC32 FOR BLUETOOTH OUTPUT (0xA2 SEED HEADER)
 * ============================================================================ */
static uint32_t ds4_crc32_table[256];
static int ds4_crc32_initialized = 0;

static void ds4_init_crc32_table(void) {
    if (ds4_crc32_initialized) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
        }
        ds4_crc32_table[i] = crc;
    }
    ds4_crc32_initialized = 1;
}

static uint32_t ds4_calc_crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ ds4_crc32_table[(crc ^ data[i]) & 0xFF];
    }
    return ~crc;
}

/* ============================================================================
 * CALIBRATION & BUS STATE
 * ============================================================================ */
static ds4_calibration_t g_ds4_calibration = { .valid = 0 };
static uint32_t g_ds4_bustype = BUS_BLUETOOTH;

static int dualshock4_read_calibration(int fd, uint32_t bustype) {
    uint8_t buf[DS4_FEAT_CALIB_USB_SIZE] = {0};
    buf[0] = DS4_FEAT_CALIB_USB;

    /* Reading calibration over BT also triggers the DS4 to send full 0x11 reports */
    int ret = ioctl(fd, HIDIOCGFEATURE(sizeof(buf)), buf);
    if (ret < 0) {
        printf("[DualShock4] Failed to read calibration: %s\n", strerror(errno));
        g_ds4_calibration.valid = 0;
        return -1;
    }

    int16_t gyro_pitch_bias  = (int16_t)(buf[1] | (buf[2] << 8));
    int16_t gyro_yaw_bias    = (int16_t)(buf[3] | (buf[4] << 8));
    int16_t gyro_roll_bias   = (int16_t)(buf[5] | (buf[6] << 8));

    int16_t gyro_pitch_plus, gyro_pitch_minus;
    int16_t gyro_yaw_plus, gyro_yaw_minus;
    int16_t gyro_roll_plus, gyro_roll_minus;
    int16_t gyro_speed_plus, gyro_speed_minus;
    int16_t acc_x_plus, acc_x_minus;
    int16_t acc_y_plus, acc_y_minus;
    int16_t acc_z_plus, acc_z_minus;

        /*gyro_pitch_plus  = (int16_t)(buf[7]  | (buf[8]  << 8));
        gyro_yaw_plus    = (int16_t)(buf[9]  | (buf[10] << 8));
        gyro_roll_plus   = (int16_t)(buf[11] | (buf[12] << 8));
        gyro_pitch_minus = (int16_t)(buf[13] | (buf[14] << 8));
        gyro_yaw_minus   = (int16_t)(buf[15] | (buf[16] << 8));
        gyro_roll_minus  = (int16_t)(buf[17] | (buf[18] << 8));
        gyro_speed_plus  = (int16_t)(buf[19] | (buf[20] << 8));
        gyro_speed_minus = (int16_t)(buf[21] | (buf[22] << 8));
        acc_x_plus       = (int16_t)(buf[23] | (buf[24] << 8));
        acc_x_minus      = (int16_t)(buf[25] | (buf[26] << 8));
        acc_y_plus       = (int16_t)(buf[27] | (buf[28] << 8));
        acc_y_minus      = (int16_t)(buf[29] | (buf[30] << 8));
        acc_z_plus       = (int16_t)(buf[31] | (buf[32] << 8));
        acc_z_minus      = (int16_t)(buf[33] | (buf[34] << 8));
    } else {*/
        // USB 0x02 calibration interleaves plus/minus per axis
        gyro_pitch_plus  = (int16_t)(buf[7]  | (buf[8]  << 8));
        gyro_pitch_minus = (int16_t)(buf[9]  | (buf[10] << 8));
        gyro_yaw_plus    = (int16_t)(buf[11] | (buf[12] << 8));
        gyro_yaw_minus   = (int16_t)(buf[13] | (buf[14] << 8));
        gyro_roll_plus   = (int16_t)(buf[15] | (buf[16] << 8));
        gyro_roll_minus  = (int16_t)(buf[17] | (buf[18] << 8));
        gyro_speed_plus  = (int16_t)(buf[19] | (buf[20] << 8));
        gyro_speed_minus = (int16_t)(buf[21] | (buf[22] << 8));
        acc_x_plus       = (int16_t)(buf[23] | (buf[24] << 8));
        acc_x_minus      = (int16_t)(buf[25] | (buf[26] << 8));
        acc_y_plus       = (int16_t)(buf[27] | (buf[28] << 8));
        acc_y_minus      = (int16_t)(buf[29] | (buf[30] << 8));
        acc_z_plus       = (int16_t)(buf[31] | (buf[32] << 8));
        acc_z_minus      = (int16_t)(buf[33] | (buf[34] << 8));
    //}

    int speed_2x = gyro_speed_plus + gyro_speed_minus;
    g_ds4_calibration.gyro[0] = (ds4_axis_calib_t){ gyro_pitch_bias, speed_2x * DS4_GYRO_RES_PER_DEG_S, gyro_pitch_plus - gyro_pitch_minus };
    g_ds4_calibration.gyro[1] = (ds4_axis_calib_t){ gyro_yaw_bias,   speed_2x * DS4_GYRO_RES_PER_DEG_S, gyro_yaw_plus - gyro_yaw_minus };
    g_ds4_calibration.gyro[2] = (ds4_axis_calib_t){ gyro_roll_bias,  speed_2x * DS4_GYRO_RES_PER_DEG_S, gyro_roll_plus - gyro_roll_minus };

    int range_x = acc_x_plus - acc_x_minus;
    int range_y = acc_y_plus - acc_y_minus;
    int range_z = acc_z_plus - acc_z_minus;
    g_ds4_calibration.accel[0] = (ds4_axis_calib_t){ acc_x_plus - range_x / 2, 2 * DS4_ACC_RES_PER_G, range_x };
    g_ds4_calibration.accel[1] = (ds4_axis_calib_t){ acc_y_plus - range_y / 2, 2 * DS4_ACC_RES_PER_G, range_y };
    g_ds4_calibration.accel[2] = (ds4_axis_calib_t){ acc_z_plus - range_z / 2, 2 * DS4_ACC_RES_PER_G, range_z };

    for (int i = 0; i < 3; i++) {
        if (g_ds4_calibration.gyro[i].sens_denom == 0) {
            g_ds4_calibration.gyro[i] = (ds4_axis_calib_t){ 0, DS4_GYRO_RANGE, 32767 };
        }
        if (g_ds4_calibration.accel[i].sens_denom == 0) {
            g_ds4_calibration.accel[i] = (ds4_axis_calib_t){ 0, DS4_ACC_RANGE, 32767 };
        }
    }

    g_ds4_calibration.valid = 1;
    printf("[DualShock4] Calibration loaded successfully \n");
    return 0;
}

static inline int32_t ds4_apply_calibration(int16_t raw, const ds4_axis_calib_t* calib) {
    return ((int32_t)(raw - calib->bias) * calib->sens_numer) / calib->sens_denom;
}

/* ============================================================================
 * CONTROLLER INFO
 * ============================================================================ */
static const controller_info_t dualshock4_info = {
    .name = "DualShock 4",
    .manufacturer = "Sony",
    .vendor_id = DUALSHOCK4_VID,
    .product_id = DUALSHOCK4_PID_V2,
    .capabilities = (
        CONTROLLER_CAP_BUTTONS |
        CONTROLLER_CAP_ANALOG_STICKS |
        CONTROLLER_CAP_TRIGGERS |
        CONTROLLER_CAP_RUMBLE |
        CONTROLLER_CAP_MOTION |
        CONTROLLER_CAP_TOUCHPAD |
        CONTROLLER_CAP_LIGHTBAR |
        CONTROLLER_CAP_BATTERY
    ),
    .supports_bluetooth = 1,
    .supports_usb = 1
};

/* ============================================================================
 * DRIVER IMPLEMENTATION
 * ============================================================================ */
static int dualshock4_init(void) {
    ds4_init_crc32_table();
    printf("[DualShock4] Driver initialized\n");
    return 0;
}

static void dualshock4_shutdown(void) {
    printf("[DualShock4] Driver shutdown\n");
}

static int dualshock4_match_device(uint16_t vid, uint16_t pid) {
    return (vid == DUALSHOCK4_VID &&
           (pid == DUALSHOCK4_PID_V1 || pid == DUALSHOCK4_PID_V2));
}

static int dualshock4_find_device(void) {
    DIR* dir = opendir("/dev");
    if (!dir) return -1;

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        if (strncmp(entry->d_name, "hidraw", 6) != 0) continue;

        char path[272];
        snprintf(path, sizeof(path), "/dev/%s", entry->d_name);

        int fd = open(path, O_RDWR | O_NONBLOCK);
        if (fd < 0) continue;

        struct hidraw_devinfo info;
        if (ioctl(fd, HIDIOCGRAWINFO, &info) < 0) {
            close(fd);
            continue;
        }

        if (dualshock4_match_device(info.vendor, info.product)) {
            char name[256] = "";
            ioctl(fd, HIDIOCGRAWNAME(sizeof(name)), name);
            printf("[DualShock4] Found: %s (%s) pid=0x%04X bus=%d\n",
                   name, path, info.product, info.bustype);

            g_ds4_bustype = info.bustype;
            dualshock4_read_calibration(fd, info.bustype);

            closedir(dir);
            return fd;
        }
        close(fd);
    }

    closedir(dir);
    return -1;
}

void dualshock4_parse_dpad(uint8_t buttons1, controller_state_t* out_state) {
    uint8_t dpad = buttons1 & 0x0F;
    switch (dpad) {
        case 0: CONTROLLER_BTN_SET(out_state, BTN_DPAD_UP); break;
        case 1: CONTROLLER_BTN_SET(out_state, BTN_DPAD_UP); CONTROLLER_BTN_SET(out_state, BTN_DPAD_RIGHT); break;
        case 2: CONTROLLER_BTN_SET(out_state, BTN_DPAD_RIGHT); break;
        case 3: CONTROLLER_BTN_SET(out_state, BTN_DPAD_DOWN); CONTROLLER_BTN_SET(out_state, BTN_DPAD_RIGHT); break;
        case 4: CONTROLLER_BTN_SET(out_state, BTN_DPAD_DOWN); break;
        case 5: CONTROLLER_BTN_SET(out_state, BTN_DPAD_DOWN); CONTROLLER_BTN_SET(out_state, BTN_DPAD_LEFT); break;
        case 6: CONTROLLER_BTN_SET(out_state, BTN_DPAD_LEFT); break;
        case 7: CONTROLLER_BTN_SET(out_state, BTN_DPAD_UP); CONTROLLER_BTN_SET(out_state, BTN_DPAD_LEFT); break;
        default: break;
    }
}

static int dualshock4_process_input(const uint8_t* raw_buf, size_t raw_len,
                                    controller_state_t* out_state) {
    const uint8_t* buf = NULL;
    size_t len = 0;

    if (raw_len >= 12 && raw_buf[0] == DS4_BT_REPORT_ID) {
        /* Bluetooth 0x11 report: shift by +2 so buf[1..] matches USB 0x01 layout */
        buf = raw_buf + 2;
        len = raw_len - 2;
    } else if (raw_len >= 10 && raw_buf[0] == DS4_USB_REPORT_ID) {
        buf = raw_buf;
        len = raw_len;
    } else {
        return -1;
    }

    memset(out_state, 0, sizeof(*out_state));

    /* Analog sticks */
    #define DS4_DEADZONE 6
    out_state->left_stick_x  = CONTROLLER_APPLY_DEADZONE(buf[DS4_OFF_LX], DS4_DEADZONE);
    out_state->left_stick_y  = CONTROLLER_APPLY_DEADZONE(buf[DS4_OFF_LY], DS4_DEADZONE);
    out_state->right_stick_x = CONTROLLER_APPLY_DEADZONE(buf[DS4_OFF_RX], DS4_DEADZONE);
    out_state->right_stick_y = CONTROLLER_APPLY_DEADZONE(buf[DS4_OFF_RY], DS4_DEADZONE);

    /* Triggers */
    out_state->left_trigger  = buf[DS4_OFF_L2];
    out_state->right_trigger = buf[DS4_OFF_R2];

    uint8_t buttons1 = buf[DS4_OFF_BUTTONS1];
    uint8_t buttons2 = buf[DS4_OFF_BUTTONS2];
    uint8_t buttons3 = buf[DS4_OFF_BUTTONS3];

    /* D-pad */
    dualshock4_parse_dpad(buttons1, out_state);

    /* Face buttons */
    if (buttons1 & DS4_BTN1_CROSS)    CONTROLLER_BTN_SET(out_state, BTN_SOUTH);
    if (buttons1 & DS4_BTN1_CIRCLE)   CONTROLLER_BTN_SET(out_state, BTN_EAST);
    if (buttons1 & DS4_BTN1_SQUARE)   CONTROLLER_BTN_SET(out_state, BTN_WEST);
    if (buttons1 & DS4_BTN1_TRIANGLE) CONTROLLER_BTN_SET(out_state, BTN_NORTH);

    /* Shoulder & trigger digital bits */
    if (buttons2 & DS4_BTN2_L1) CONTROLLER_BTN_SET(out_state, BTN_L1);
    if (buttons2 & DS4_BTN2_R1) CONTROLLER_BTN_SET(out_state, BTN_R1);
    if (buttons2 & DS4_BTN2_L2) CONTROLLER_BTN_SET(out_state, BTN_L2);
    if (buttons2 & DS4_BTN2_R2) CONTROLLER_BTN_SET(out_state, BTN_R2);

    /* Stick clicks & menu buttons */
    if (buttons2 & DS4_BTN2_L3)      CONTROLLER_BTN_SET(out_state, BTN_L3);
    if (buttons2 & DS4_BTN2_R3)      CONTROLLER_BTN_SET(out_state, BTN_R3);
    if (buttons2 & DS4_BTN2_SHARE)   CONTROLLER_BTN_SET(out_state, BTN_SELECT);
    if (buttons2 & DS4_BTN2_OPTIONS) CONTROLLER_BTN_SET(out_state, BTN_START);

    /* PS Home & Touchpad Click */
    if (buttons3 & DS4_BTN3_PS)       CONTROLLER_BTN_SET(out_state, BTN_HOME);
    if (buttons3 & DS4_BTN3_TOUCHPAD) CONTROLLER_BTN_SET(out_state, BTN_TOUCHPAD);

    /* Motion sensors (present on USB 64-byte and BT 78-byte reports) */
    if (len >= 25) {
        int16_t raw_gyro_x  = (int16_t)(buf[DS4_OFF_GYRO_X]  | (buf[DS4_OFF_GYRO_X + 1]  << 8));
        int16_t raw_gyro_y  = (int16_t)(buf[DS4_OFF_GYRO_Y]  | (buf[DS4_OFF_GYRO_Y + 1]  << 8));
        int16_t raw_gyro_z  = (int16_t)(buf[DS4_OFF_GYRO_Z]  | (buf[DS4_OFF_GYRO_Z + 1]  << 8));
        int16_t raw_accel_x = (int16_t)(buf[DS4_OFF_ACCEL_X] | (buf[DS4_OFF_ACCEL_X + 1] << 8));
        int16_t raw_accel_y = (int16_t)(buf[DS4_OFF_ACCEL_Y] | (buf[DS4_OFF_ACCEL_Y + 1] << 8));
        int16_t raw_accel_z = (int16_t)(buf[DS4_OFF_ACCEL_Z] | (buf[DS4_OFF_ACCEL_Z + 1] << 8));

        if (g_ds4_calibration.valid) {
            out_state->gyro_x  = ds4_apply_calibration(raw_gyro_x,  &g_ds4_calibration.gyro[0]);
            out_state->gyro_y  = ds4_apply_calibration(raw_gyro_y,  &g_ds4_calibration.gyro[1]);
            out_state->gyro_z  = ds4_apply_calibration(raw_gyro_z,  &g_ds4_calibration.gyro[2]);
            out_state->accel_x = ds4_apply_calibration(raw_accel_x, &g_ds4_calibration.accel[0]);
            out_state->accel_y = ds4_apply_calibration(raw_accel_y, &g_ds4_calibration.accel[1]);
            out_state->accel_z = ds4_apply_calibration(raw_accel_z, &g_ds4_calibration.accel[2]);
        } else {
            out_state->gyro_x  = raw_gyro_x;
            out_state->gyro_y  = raw_gyro_y;
            out_state->gyro_z  = raw_gyro_z;
            out_state->accel_x = raw_accel_x;
            out_state->accel_y = raw_accel_y;
            out_state->accel_z = raw_accel_z;
        }
    }

    /* Touchpad (packet 0 starts at buf[35]) */
    if (len >= DS4_OFF_TOUCHPAD + 10) {
        const uint8_t* touch = &buf[DS4_OFF_TOUCHPAD + 2];
        for (int i = 0; i < 2; i++) {
            uint8_t contact = touch[i * 4];
            out_state->touch[i].active = !(contact & DS4_TOUCH_INACTIVE);
            if (out_state->touch[i].active) {
                out_state->touch[i].x = touch[i * 4 + 1] | ((touch[i * 4 + 2] & 0x0F) << 8);
                out_state->touch[i].y = (touch[i * 4 + 2] >> 4) | (touch[i * 4 + 3] << 4);
            }
        }
    }

    /* Battery */
    if (len > DS4_OFF_BATTERY) {
        uint8_t bat = buf[DS4_OFF_BATTERY];
        uint8_t level = bat & 0x0F;
        uint8_t cable = (bat >> 4) & 0x01;
        if (cable) {
            out_state->battery_charging = 1;
            out_state->battery_full = (level >= 11) ? 1 : 0;
            out_state->battery_level = (level >= 10) ? 100 : level * 10;
        } else {
            out_state->battery_charging = 0;
            out_state->battery_full = 0;
            out_state->battery_level = (level >= 10) ? 100 : level * 10;
        }
    }

    out_state->timestamp_ms = time_get_ms();
    return 0;
}

/* ============================================================================
 * OUTPUT REPORTS (RUMBLE & LIGHTBAR)
 * ============================================================================ */
static controller_output_t ds4_last_sent = {
    .rumble_left = 0, .rumble_right = 0,
    .led_r = 255, .led_g = 255, .led_b = 255,
    .player_leds = 0xFF,
};
static int ds4_have_last_sent = 0;

#define DS4_LED_REFRESH_MS 2000
static uint64_t ds4_last_send_ms = 0;

static int dualshock4_write_report(int fd, const controller_output_t* out) {
    if (g_ds4_bustype == BUS_BLUETOOTH) {
        uint8_t report[DS4_BT_OUTPUT_SIZE] = {0};
        report[0] = DS4_BT_REPORT_ID;
        report[1] = 0xC0; /* HID polling rate / L2CAP flags */
        report[2] = 0x00;
        report[3] = 0x07; /* Flags: 0x01 rumble + 0x02 lightbar + 0x04 blink */
        report[6] = out->rumble_right; /* Weak / high-freq motor */
        report[7] = out->rumble_left;  /* Strong / low-freq motor */
        report[8] = out->led_r;
        report[9] = out->led_g;
        report[10] = out->led_b;

        uint8_t crc_buf[75];
        crc_buf[0] = 0xA2;
        memcpy(&crc_buf[1], report, 74);
        uint32_t crc = ds4_calc_crc32(crc_buf, 75);
        report[74] = crc & 0xFF;
        report[75] = (crc >> 8) & 0xFF;
        report[76] = (crc >> 16) & 0xFF;
        report[77] = (crc >> 24) & 0xFF;

        ssize_t written = write(fd, report, sizeof(report));
        return (written == (ssize_t)sizeof(report)) ? 0 : -1;
    } else {
        uint8_t report[DS4_USB_OUTPUT_SIZE] = {0};
        report[0] = DS4_USB_OUTPUT_ID;
        report[1] = 0x07;
        report[4] = out->rumble_right;
        report[5] = out->rumble_left;
        report[6] = out->led_r;
        report[7] = out->led_g;
        report[8] = out->led_b;

        ssize_t written = write(fd, report, sizeof(report));
        return (written == (ssize_t)sizeof(report)) ? 0 : -1;
    }
}

static int dualshock4_send_output(int fd, const controller_output_t* output) {
    if (fd < 0) return -1;

    uint64_t now = time_get_ms();
    int rumble_changed = !ds4_have_last_sent ||
        output->rumble_left  != ds4_last_sent.rumble_left ||
        output->rumble_right != ds4_last_sent.rumble_right;

    int leds_changed = !ds4_have_last_sent ||
        output->led_r != ds4_last_sent.led_r ||
        output->led_g != ds4_last_sent.led_g ||
        output->led_b != ds4_last_sent.led_b ||
        output->player_leds != ds4_last_sent.player_leds;

    int refresh = DS4_LED_REFRESH_MS > 0 && ds4_have_last_sent &&
                  (now - ds4_last_send_ms) >= DS4_LED_REFRESH_MS;

    if (!rumble_changed && !leds_changed && !refresh) {
        return 0;
    }

    if (dualshock4_write_report(fd, output) < 0) {
        return -1;
    }

    ds4_last_sent = *output;
    ds4_have_last_sent = 1;
    ds4_last_send_ms = now;
    return 0;
}

static void dualshock4_on_disconnect(void) {
    printf("[DualShock4] Disconnected\n");
    ds4_have_last_sent = 0;
}

static void dualshock4_enter_low_power(int fd) {
    printf("[DualShock4] Entering low power mode\n");
    controller_output_t off = {0};
    if (fd >= 0) {
        dualshock4_write_report(fd, &off);
        ds4_last_sent = off;
        ds4_have_last_sent = 1;
    }
}

static void dualshock4_power_off(int fd) {
    char uniq[64] = "";
    if (fd < 0 || ioctl(fd, HIDIOCGRAWUNIQ(sizeof(uniq)), uniq) < 0 || uniq[0] == '\0') {
        dualshock4_enter_low_power(fd);
        return;
    }

    bdaddr_t addr;
    if (str2ba(uniq, &addr) < 0) {
        dualshock4_enter_low_power(fd);
        return;
    }

    controller_output_t off = {0};
    dualshock4_write_report(fd, &off);

    int dev_id = hci_get_route(NULL);
    int dd = hci_open_dev(dev_id);
    if (dd < 0) return;

    struct hci_conn_info_req* cr = malloc(sizeof(*cr) + sizeof(struct hci_conn_info));
    if (cr) {
        bacpy(&cr->bdaddr, &addr);
        cr->type = ACL_LINK;
        if (ioctl(dd, HCIGETCONNINFO, (unsigned long)cr) >= 0) {
            hci_disconnect(dd, cr->conn_info->handle, HCI_OE_USER_ENDED_CONNECTION, 1000);
            printf("[DualShock4] Powered off (%s)\n", uniq);
        }
        free(cr);
    }
    hci_close_dev(dd);
    ds4_have_last_sent = 0;
}

/* ============================================================================
 * DRIVER INSTANCE
 * ============================================================================ */
static const controller_driver_t dualshock4_driver = {
    .info = &dualshock4_info,
    .init = dualshock4_init,
    .shutdown = dualshock4_shutdown,
    .find_device = dualshock4_find_device,
    .match_device = dualshock4_match_device,
    .process_input = dualshock4_process_input,
    .send_output = dualshock4_send_output,
    .on_disconnect = dualshock4_on_disconnect,
    .enter_low_power = dualshock4_enter_low_power,
    .power_off = dualshock4_power_off
};

const controller_driver_t* dualshock4_get_driver(void) {
    return &dualshock4_driver;
}

void dualshock4_register(void) {
    controller_register(&dualshock4_driver);
    printf("[DualShock4] Driver registered\n");
}
