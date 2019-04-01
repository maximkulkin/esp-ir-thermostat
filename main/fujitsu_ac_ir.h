#pragma once

#include <stddef.h>

typedef enum {
    fujitsu_ac_model_ARRAH2E = 1,
    fujitsu_ac_model_ARDB1,
} fujitsu_ac_model;


typedef enum {
    ac_cmd_stay_on = 0x00,
    ac_cmd_turn_on = 0x01,
    ac_cmd_turn_off = 0x02,
    ac_cmd_step_horiz = 0x79,
    ac_cmd_step_vert = 0x6C,
} ac_cmd;

typedef enum {
    ac_mode_auto = 0x00,
    ac_mode_cool = 0x01,
    ac_mode_dry = 0x02,
    ac_mode_fan = 0x03,
    ac_mode_heat = 0x04,
} ac_mode;

typedef enum {
    ac_fan_auto = 0x00,
    ac_fan_high = 0x01,
    ac_fan_med = 0x02,
    ac_fan_low = 0x03,
    ac_fan_quiet = 0x04,
} ac_fan;

typedef enum {
    ac_swing_off = 0x00,
    ac_swing_vert = 0x01,
    ac_swing_horiz = 0x02,
    ac_swing_both = 0x03,
} ac_swing;


#define AC_MIN_TEMPERATURE 16
#define AC_MAX_TEMPERATURE 30



typedef struct {
    ac_cmd command;

    ac_mode mode;
    ac_fan fan;
    ac_swing swing;
    uint8_t temperature;
} fujitsu_ac_state_t;


void fujitsu_ac_ir_tx_init(fujitsu_ac_model ac_model);
int fujitsu_ac_ir_send(fujitsu_ac_state_t *state);

