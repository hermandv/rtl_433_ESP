/** @file
    Greenhouse sensor node (GH-Node), protocol v1.

    Copyright (C) 2026 Herman de Vries

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/
/**
Greenhouse sensor node (GH-Node), protocol v1.

Specification: development/protocol.md in https://github.com/hermandv/greenhouse

OOK PWM at 433.92 MHz: bit 1 = 500 us pulse, bit 0 = 1000 us pulse, 1000 us gap after each.
8 short preamble pulses, then 17 bytes MSB first. Sent 3 times with an extra 4 ms gap.

Data layout (bytes):

    SS II QQ FF TT TT HH PP PP S1 S1 S2 S2 VV VV BB CC

- S: sync 0xA5
- I: node id
- Q: sequence number, +1 per cycle
- F: flags: 0 LOW_BAT, 1 HEATER_RAN, 2 LISTENING, 3 CMD_ACK, 4 SENSOR_FAULT, 5 REBOOTED
- T: temperature, int16 C x10 (0x7FFF = no value)
- H: humidity % (0xFF = no value)
- P: station pressure, uint16 hPa x10 (0x0000 = no value)
- S1, S2: soil sensors, raw ADC (0xFFFF = no value)
- V: battery mV
- B: box temperature, int8 C (0x7F = no value)
- C: CRC-8, poly 0x31, init 0x00, over bytes 0-15
*/

#include "decoder.h"

#define GH_NODE_BYTES 17
#define GH_NODE_BITS  (GH_NODE_BYTES * 8)

static int gh_node_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const sync[] = {0xA5};
    int result = DECODE_ABORT_LENGTH;

    for (int row = 0; row < bitbuffer->num_rows; ++row) {
        unsigned row_bits = bitbuffer->bits_per_row[row];
        if (row_bits < GH_NODE_BITS) {
            continue;
        }
        unsigned pos = bitbuffer_search(bitbuffer, row, 0, sync, 8);
        if (pos + GH_NODE_BITS > row_bits) {
            continue;
        }
        uint8_t b[GH_NODE_BYTES];
        bitbuffer_extract_bytes(bitbuffer, row, pos, b, GH_NODE_BITS);
        if (crc8(b, GH_NODE_BYTES - 1, 0x31, 0x00) != b[16]) {
            result = DECODE_FAIL_MIC;
            continue;
        }

        int id          = b[1];
        int seq         = b[2];
        int flags       = b[3];
        int temp_raw    = (int16_t)((b[4] << 8) | b[5]);
        int humidity    = b[6];
        int press_raw   = (b[7] << 8) | b[8];
        int soil1       = (b[9] << 8) | b[10];
        int soil2       = (b[11] << 8) | b[12];
        int battery_mv  = (b[13] << 8) | b[14];
        int box_temp    = (int8_t)b[15];

        /* clang-format off */
        data_t *data = data_make(
                "model",             "",            DATA_STRING, "GH-Node",
                "id",                "",            DATA_INT,    id,
                "seq",               "Sequence",    DATA_INT,    seq,
                "flags",             "Flags",       DATA_INT,    flags,
                "battery_ok",        "Battery",     DATA_INT,    !(flags & 0x01),
                "temperature_C",     "Temperature", DATA_COND,   temp_raw != 0x7FFF, DATA_FORMAT, "%.1f C", DATA_DOUBLE, temp_raw * 0.1,
                "humidity",          "Humidity",    DATA_COND,   humidity != 0xFF,   DATA_FORMAT, "%u %%",  DATA_INT,    humidity,
                "pressure_hPa",      "Pressure",    DATA_COND,   press_raw != 0,     DATA_FORMAT, "%.1f hPa", DATA_DOUBLE, press_raw * 0.1,
                "moisture_1",        "Soil 1 raw",  DATA_COND,   soil1 != 0xFFFF,    DATA_INT,    soil1,
                "moisture_2",        "Soil 2 raw",  DATA_COND,   soil2 != 0xFFFF,    DATA_INT,    soil2,
                "battery_mV",        "Battery",     DATA_FORMAT, "%d mV", DATA_INT, battery_mv,
                "box_temperature_C", "Box temp",    DATA_COND,   box_temp != 0x7F,   DATA_FORMAT, "%d C",   DATA_INT,    box_temp,
                "mic",               "Integrity",   DATA_STRING, "CRC",
                NULL);
        /* clang-format on */

        decoder_output_data(decoder, data);
        return 1; // the repeats carry the same reading
    }
    return result;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "seq",
        "flags",
        "battery_ok",
        "temperature_C",
        "humidity",
        "pressure_hPa",
        "moisture_1",
        "moisture_2",
        "battery_mV",
        "box_temperature_C",
        "mic",
        NULL,
};

r_device const gh_node = {
        .name        = "Greenhouse GH-Node sensor",
        .modulation  = OOK_PULSE_PWM,
        .short_width = 500,
        .long_width  = 1000,
        .gap_limit   = 2500,
        .reset_limit = 6000,
        .decode_fn   = &gh_node_decode,
        .fields      = output_fields,
};
