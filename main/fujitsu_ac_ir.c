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

    .tolerance = 20,
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

    printf("Sending state: command=%d mode=%d fan=%d swing=%d temperature=%d\n",
           state->command, state->mode, state->fan, state->swing, state->temperature);

    return ir_generic_send(&fujitsu_ac_ir_config, cmd, cmd_size);
}


typedef struct {
    ir_decoder_t decoder;
    ir_decoder_t *generic_decoder;
} fujitsu_ac_ir_decoder_t;


static int fujitsu_ac_ir_decoder_decode(fujitsu_ac_ir_decoder_t *decoder,
                                        int16_t *pulses, uint16_t pulse_count,
                                        void *decode_buffer, uint16_t decode_buffer_size)
{
    if (decode_buffer_size < sizeof(fujitsu_ac_state_t))
        return -2;

    fujitsu_ac_state_t *state = decode_buffer;

    uint8_t cmd[16];
    int cmd_size = decoder->generic_decoder->decode(
        decoder->generic_decoder, pulses, pulse_count, cmd, sizeof(cmd)
    );
    if (cmd_size <= 0)
        return cmd_size;

    if (cmd_size < 6)
        return -1;

    if (cmd[0] != 0x14 || cmd[1] != 0x63 || cmd[2] != 0x00 || cmd[3] != 0x10 || cmd[4] != 0x10)
        return -1;

    switch (cmd[5]) {
    case ac_cmd_turn_off:
    case ac_cmd_step_horiz:
    case ac_cmd_step_vert:
        if ((cmd_size == 7) && (cmd[6] != (~cmd[5] & 0xff))) {
            return -1;
        } else if (cmd_size > 7) {
            return -1;
        }

        state->command = cmd[5];

        break;
    case 0xfe:   // full state model ARRAH2E
    case 0xfc: { // full state model ARDB1
        fujitsu_ac_model model = (cmd[5] == 0xfe) ? fujitsu_ac_model_ARRAH2E : fujitsu_ac_model_ARDB1;
        if (cmd[6] != 9 || cmd[7] != 0x30)
            return -1;

        uint8_t checksum = 0;
        switch (model) {
        case fujitsu_ac_model_ARRAH2E:
            if (cmd[14] != 0x20 || cmd_size != 16)
                return -1;

            for (int i=7; i < 15; i++)
                checksum += cmd[i];

            if (cmd[15] != (-checksum & 0xff))
                return -1;

            break;

        case fujitsu_ac_model_ARDB1:
            if (cmd_size != 15)
                return -1;

            for (int i=0; i < 14; i++)
                checksum += cmd[i];

            if (cmd[14] != ((0x9B - checksum) & 0xff))
                return -1;

            break;
        }

        state->command = cmd[8] && 0xf;
        state->temperature = AC_MIN_TEMPERATURE + (cmd[8] >> 4);
        state->mode = cmd[9] & 0xf;
        state->fan = cmd[10] & 0xf;
        state->swing = cmd[10] >> 4;

        break;
    }
    default:
        return -1;
    }

    printf("Decoded state: command=%d mode=%d fan=%d swing=%d temperature=%d\n",
           state->command, state->mode, state->fan, state->swing, state->temperature);

    return sizeof(fujitsu_ac_state_t);
}


static void fujitsu_ac_ir_decoder_free(fujitsu_ac_ir_decoder_t *decoder) {
    decoder->generic_decoder->free(decoder->generic_decoder);
    free(decoder);
}


ir_decoder_t *fujitsu_ac_ir_make_decoder() {
    fujitsu_ac_ir_decoder_t *decoder = malloc(sizeof(fujitsu_ac_ir_decoder_t));
    if (!decoder)
        return NULL;

    decoder->generic_decoder = ir_generic_make_decoder(&fujitsu_ac_ir_config);
    if (!decoder->generic_decoder) {
        free(decoder);
        return NULL;
    }

    decoder->decoder.decode = (ir_decoder_decode_t) fujitsu_ac_ir_decoder_decode;
    decoder->decoder.free = (ir_decoder_free_t) fujitsu_ac_ir_decoder_free;

    return (ir_decoder_t*) decoder;
}
