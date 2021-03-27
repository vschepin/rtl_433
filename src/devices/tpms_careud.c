/** @file
    Careud TPMS.

    Copyright (C) 2021 Vladimir Schepin

    based on code
    2019 Andreas Spiess and Christian W. Zuckschwerdt <zany@triq.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

/**
Careud TPMS

http://www.careud.com/

- Frequency: 433.92 MHz
- Pressure: +/- 0.01 bar from 0 bar to 8 bar
- Temperature: +/- 3 C from -40 C to 105 C

Signal is manchester encoded, data XOR encrypted

Data layout (nibbles):

    SS SS KF II TT PP II CC CC

- S: 16 bit sync word, 0x19cf
- K: 4 bit XOR Key
- F: 1 bit Flag - deflation alarm
- F: 1 bit Unknown flag (may be MSB pressure or id bit?)
- F: 1 bit Flag - battery low alarm
- F: 1 bit Unknown flag (may be MSB pressure or id bit?)
- I: 8 bits ID
- T: 8 bit Temperature (deg. C offset by 55)
- P: 8 bit Pressure BAR * 64
- I: 8 bits ID
- C: 16 bit CRC-16/BUYPASS
- The preamble is 55 55 55 a9 (inverted: aa aa aa 56)

TODO: identify most significant pressure bit and one unknown flag bit meaning
*/

#include "decoder.h"
//#define TPMS_CAREUD_SHOW_RAW

static int tpms_careud_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    data_t *data;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    uint8_t d[5];
    uint16_t id;
    char id_str[4 + 1];
    int flags;
    int pressure;
    int temperature;
#ifdef TPMS_CAREUD_SHOW_RAW
    char code_str[9 * 2 + 1];
    char data_str[5 * 2 + 1];
#endif

    bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 72);

    if (packet_bits.bits_per_row[0] < 72) {
        return DECODE_FAIL_SANITY;
    }
    b = packet_bits.bb[0];

    /* Check for sync */
    if ((b[0]<<8 | b[1]) != 0x19cf) {
        return DECODE_FAIL_SANITY;
    }

    /* Check crc */
    uint16_t crc = crc16(&b[2], 7, 0x8005, 0x0000);
    if ( crc != 0) {
        bitrow_printf(b, 7, "%s: sensor bad CRC: %02x -", __func__, crc);
        return DECODE_FAIL_MIC;
    }

    memcpy(d, b+2, sizeof(d));
    /* Some sort of XOR encryption
     * possible wrong but works in test environment
     */
    for(int i = 1; i < 5; i++){
        d[i]^=d[0];
    }
    for(int i = 3; i >= 0; i--){
        d[i]^=d[4];
    }

    id          = (d[1] << 8) | d[4];
    flags       = d[0] & 0x0f;
    temperature = d[2];
    pressure    = d[3];
    sprintf(id_str, "%04x", id);
#ifdef TPMS_CAREUD_SHOW_RAW
    sprintf(code_str, "%02x%02x%02x%02x%02x%02x%02x%02x%02x", b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8]);
    sprintf(data_str, "%02x%02x%02x%02x%02x", d[0], d[1], d[2], d[3], d[4]);
#endif

    /* clang-format off */
    data = data_make(
            "model",            "",                 DATA_STRING, "Careud",
            "type",             "",                 DATA_STRING, "TPMS",
            "id",               "",                 DATA_STRING, id_str,
            "flags",            "",                 DATA_INT, flags,
            "battery",          "",                 DATA_STRING, (flags & 0x02) ? "OK" : "LOW",
            "pressure_BAR",     "Pressure",         DATA_FORMAT, "%.2f BAR", DATA_DOUBLE, (float)pressure / 64,
            "pressure_loss",    "Pressure Loss",    DATA_STRING, (flags & 0x08) ? "OK" : "ALARM",
            "temperature_C",    "Temperature",      DATA_FORMAT, "%d C", DATA_INT, temperature - 55,
#ifdef TPMS_CAREUD_SHOW_RAW
            "pressure_RAW",     "Pressure_RAW",     DATA_INT, pressure,
            "temperature_RAW",  "Temperature_RAW",  DATA_INT, temperature,
            "code",             "",                 DATA_STRING, code_str,
            "data",             "",                 DATA_STRING, data_str,
#endif
            "mic",              "",                 DATA_STRING, "CRC",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** @sa tpms_careud_decode() */
static int tpms_careud_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    // preamble is 55 ... 55 a9 (inverted: aa ... aa 56)
    uint8_t const preamble_pattern[3] = {0x55, 0x55, 0xa9};

    unsigned bitpos = 0;
    int ret         = 0;
    int events      = 0;

    bitbuffer_invert(bitbuffer);

    while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, preamble_pattern, 24)) + 80 <=
            bitbuffer->bits_per_row[0]) {

        ret = tpms_careud_decode(decoder, bitbuffer, 0, bitpos + 16);
        if (ret > 0)
            events += ret;
        bitpos += 2;
    }

    return events > 0 ? events : ret;
}

static char *output_fields[] = {
        "model",
        "type",
        "id",
        "flags",
        "battery",
        "pressure_BAR",
        "pressure_loss",
        "temperature_C",
#ifdef TPMS_CAREUD_SHOW_RAW
        "pressure_RAW",
        "temperature_RAW",
        "code",
        "data",
#endif
        "mic",
        NULL,
};

r_device tpms_careud = {
        .name        = "Careud TPMS",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 52,
        .long_width  = 52,
        .reset_limit = 150,
        .decode_fn   = &tpms_careud_callback,
        .disabled    = 0,
        .fields      = output_fields,
};
