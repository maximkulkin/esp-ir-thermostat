#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "fujitsu_ac_ir.h"
#include <ir/ir.h>
#include <ir/generic.h>



static fujitsu_ac_model model;
static ir_generic_config_t fujitsu_ac_ir_config = {
    .header_mark = 3200,
    .header_space = -1600,

    .bit1_mark = 400,
    .bit1_space = -1200,

    .bit0_mark = 400,
    .bit0_space = -400,

    .footer_mark = 400,
    .footer_space = -8000,
};


void fujitsu_ac_ir_tx_init(fujitsu_ac_model ac_model) {
    ir_tx_init();
    model = ac_model;
}

int fujitsu_ac_ir_send(fujitsu_ac_state_t *state) {
    uint8_t cmd[16];
    size_t cmd_size = 16;
    cmd[0] = 0x14;
    cmd[1] = 0x63;
    cmd[2] = 0x00;
    cmd[3] = 0x10;
    cmd[4] = 0x10;

    switch (state->command) {
    case ac_cmd_turn_off:
    case ac_cmd_step_horiz:
    case ac_cmd_step_vert:
        cmd[5] = state->command;

        if (model == fujitsu_ac_model_ARRAH2E) {
            cmd[6] = ~cmd[5];
            cmd_size = 7;
        } else {
            cmd_size = 6;
        }

        break;
    default:
        switch (model) {
        case fujitsu_ac_model_ARRAH2E:
            cmd[5] = 0xfe;
            break;
        case fujitsu_ac_model_ARDB1:
            cmd[5] = 0xfc;
            break;
        }

        cmd[6] = 9; // size of extended command
        cmd[7] = 0x30;
        cmd[8] = (state->command == ac_cmd_turn_on) | ((state->temperature - AC_MIN_TEMPERATURE) << 4);
        cmd[9] = state->mode | (0 /* timer off */ << 4);
        cmd[10] = state->fan | (state->swing << 4);
        cmd[11] = 0x00; // timer off values
        cmd[12] = 0x00; // timer off/on values
        cmd[13] = 0x00; // timer on values

        uint8_t checksum = 0;

        switch (model) {
        case fujitsu_ac_model_ARRAH2E:
            cmd[14] = 0x20;

            for (int i=7; i < 16 - 1; i++)
                checksum += cmd[i];

            cmd[15] = -checksum;
            cmd_size = 16;
            break;
        case fujitsu_ac_model_ARDB1:
            for (int i=0; i < 14; i++)
                checksum += cmd[i];

            cmd[14] = 0x9B - checksum;
            cmd_size = 15;
            break;
        }
    }

    printf("State: command=%d mode=%d fan=%d swing=%d temperature=%d\n",
           state->command, state->mode, state->fan, state->swing, state->temperature);

    return ir_generic_send(&fujitsu_ac_ir_config, cmd, cmd_size * 8);
}

