/* FSK 8 byte Manchester encoded TPMS with simple checksum.
 * Seen on Ford Fiesta, Focus, ...
 *
 * Copyright (C) 2017 Christian W. Zuckschwerdt <zany@triq.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Packet nibbles:  IIIIIIII PP TT FF CC
 * I = ID
 * P = likely Pressure
 * T = likely Temperature
 * F = Flags, (46: 87% 1e: 5% 06: 2% 4b: 1% 66: 1% 0e: 1% 44: 1%)
 * C = Checksum, SUM bytes 0 to 6 = byte 7
*/

#include "decoder.h"

// full preamble is 55 55 55 56 (inverted: aa aa aa a9)
static const uint8_t preamble_pattern[2] = { 0xaa, 0xa9 }; // 16 bits

static int tpms_ford_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    data_t *data;
    unsigned int start_pos;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    unsigned id;
    char id_str[9];
    int code;
    char code_str[7];

    start_pos = bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 160);
    // require 64 data bits
    if (start_pos-bitpos < 128) {
        return 0;
    }
    b = packet_bits.bb[0];

    if (((b[0]+b[1]+b[2]+b[3]+b[4]+b[5]+b[6]) & 0xff) != b[7]) {
        return 0;
    }

    id = (unsigned)b[0] << 24 | b[1] << 16 | b[2] << 8 | b[3];
    sprintf(id_str, "%08x", id);

    code = b[4]<<16 | b[5]<<8 | b[6];
    sprintf(code_str, "%06x", code);

    data = data_make(
        "model",        "",     DATA_STRING, "Ford",
        "type",         "",     DATA_STRING, "TPMS",
        "id",           "",     DATA_STRING, id_str,
        "code",         "",     DATA_STRING, code_str,
        "mic",          "",     DATA_STRING, "CHECKSUM",
        NULL);

    decoder_output_data(decoder, data);
    return 1;
}

/* upcoming */

/* Seen on Mini Cabrio (R57)
 * Manufacturer HUF. Should work for BMW Mini R55 R56 R57 R58 R59 R60 R61 X1 X3 X4
 * FSK NRZ 50 us bit width.
 * Preamble 0000288, data: IIIIIIII PP TT UUUUUU CC
 * I : ID (32 bit)
 * P : Pressure scale 2.5 kPa
 * T : Temperature C offset 52
 * U : Unknown (24 bit)
 * C : CRC-8 poly 0x2f init 0x2d
 */

// full preamble is 0000288
static const uint8_t mini_preamble_pattern[2] = {0x02, 0x88}; // 16 bits

static int tpms_mini_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    data_t *data;
    uint8_t b[10];
    int id;
    char id_str[9];
    int pressure;
    int temp_c;
    int code;
    char code_str[7];

    bitbuffer_print(bitbuffer);
    bitbuffer_extract_bytes(bitbuffer, row, bitpos, b, 80);

    if (crc8(b, 10, 0x2f, 0x2d)) {
        //fprintf(stderr, "crc failed %02x vs %02x\n", crc8(b, 9, 0x2f, 0x2f), b[9]);
        //bitbuffer_print(bitbuffer);
        return 0;
    }

    id = b[0] << 24 | b[1] << 16 | b[2] << 8 | b[3];
    sprintf(id_str, "%08x", id);

    pressure = b[4] * 2.5;
    temp_c = b[5] - 52;

    code = b[6] << 16 | b[7] << 8 | b[8];
    sprintf(code_str, "%06x", code);

    data = data_make(
            "model",            "", DATA_STRING, "Mini",
            "type",             "", DATA_STRING, "TPMS",
            "id",               "", DATA_STRING, id_str,
            "pressure_kPa",     "", DATA_INT, pressure,
            "temperature_C",    "", DATA_INT, temp_c,
            "code",             "", DATA_STRING, code_str,
            "mic",              "", DATA_STRING, "CRC",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}

/* other known decoders */
//int schraeder_callback(r_device *decoder, bitbuffer_t *bitbuffer); // 120 us
//int schrader_EG53MA4_callback(r_device *decoder, bitbuffer_t *bitbuffer); // 120 us
int steelmate_callback(r_device *decoder, bitbuffer_t *bitbuffer);
int tpms_citroen_callback(r_device *decoder, bitbuffer_t *bitbuffer);
//int tpms_pmv107j_callback(r_device *decoder, bitbuffer_t *bitbuffer); // 100 us
int tpms_renault_callback(r_device *decoder, bitbuffer_t *bitbuffer);
int tpms_toyota_callback(r_device *decoder, bitbuffer_t *bitbuffer);

/* experimental extra decoders */
static int tpmsverylong_decode(r_device *decoder, bitbuffer_t *bitbuffer);
static int tpmslong_decode(r_device *decoder, bitbuffer_t *bitbuffer);
static int tpmsxor_decode(r_device *decoder, bitbuffer_t *bitbuffer);
static int tpmsraw_decode(r_device *decoder, bitbuffer_t *bitbuffer);

// full preamble is 55 55 55 56 (inverted: aa aa aa a9)
static const uint8_t preamble_pattern_aaa9[2] = {0xaa, 0xa9}; // 16 bits

// full preamble is 99 99 99 9a (inverted: 66 66 66 65)
static const uint8_t preamble_pattern_6665[2] = {0x66, 0x65}; // 16 bits

typedef int (*tpms_parse_fn)(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos);

static int tpms_generic_decode(r_device *decoder, bitbuffer_t *bitbuffer,
        uint8_t const *preamble_pattern, unsigned preamble_len,
        unsigned data_len, tpms_parse_fn tpms_parse);

static int tpms_6665_88_crc16_1021_0288_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos);
static int tpms_6665_88_crc16_1021_f297_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos);
static int tpms_aaa9_72_add_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos);
static int tpms_aaa9_72_crc8_07_xaa_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos);
static int tpms_aaa9_72_crc8_07_x00_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos);
static int tpms_aaa9_80_xor_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos);
static int tpms_aaa9_80_crc8_07_x00_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos);

static int tpms_ford_callback(r_device *decoder, bitbuffer_t *bitbuffer) {
    int row;
    unsigned bitpos;
    int events = 0;

    bitbuffer_t schraeder_bitbuffer = *bitbuffer;
    bitbuffer_t schrader_EG53MA4_bitbuffer = *bitbuffer;
    bitbuffer_t steelmate_bitbuffer = *bitbuffer;
    bitbuffer_t tpms_citroen_bitbuffer = *bitbuffer;
    bitbuffer_t tpms_pmv107j_bitbuffer = *bitbuffer;
    bitbuffer_t tpms_renault_bitbuffer = *bitbuffer;
    bitbuffer_t tpms_toyota_bitbuffer = *bitbuffer;
    bitbuffer_t tpms_mini_bitbuffer = *bitbuffer;

    bitbuffer_invert(bitbuffer);

    for (row = 0; row < bitbuffer->num_rows; ++row) {
        bitpos = 0;
        // Find a preamble with enough bits after it that it could be a complete packet
        while ((bitpos = bitbuffer_search(bitbuffer, row, bitpos,
                (const uint8_t *)&preamble_pattern, 16)) + 144 <=
                bitbuffer->bits_per_row[row]) {
            events += tpms_ford_decode(decoder, bitbuffer, row, bitpos + 16);
            bitpos += 15;
        }
    }

    // check if this is already decoded elsewhere, produces dups, only for research
    //if (!events) events += schraeder_callback(decoder, &schraeder_bitbuffer);
	//if (!events) events += schrader_EG53MA4_callback(decoder, &schrader_EG53MA4_bitbuffer);
	if (!events) events += steelmate_callback(decoder, &steelmate_bitbuffer);
    if (!events) events += tpms_citroen_callback(decoder, &tpms_citroen_bitbuffer);
    //if (!events) events += tpms_pmv107j_callback(decoder, &tpms_pmv107j_bitbuffer);
    if (!events) events += tpms_renault_callback(decoder, &tpms_renault_bitbuffer);
    if (!events) events += tpms_toyota_callback(decoder, &tpms_toyota_bitbuffer);
    if (!events) events += tpms_generic_decode(decoder, &tpms_mini_bitbuffer, mini_preamble_pattern, 16, 40, &tpms_mini_decode);

    /* experimental extra decoders */
    if (!events) events += tpmsverylong_decode(decoder, bitbuffer);
    if (!events) events += tpmslong_decode(decoder, bitbuffer);
    if (!events) events += tpmsxor_decode(decoder, bitbuffer);

    if (!events) events += tpms_generic_decode(decoder, bitbuffer, preamble_pattern_6665, 16, 88, &tpms_6665_88_crc16_1021_0288_parse);
    if (!events) events += tpms_generic_decode(decoder, bitbuffer, preamble_pattern_6665, 16, 88, &tpms_6665_88_crc16_1021_f297_parse);
    if (!events) events += tpms_generic_decode(decoder, bitbuffer, preamble_pattern_aaa9, 16, 72, &tpms_aaa9_72_add_parse);
    if (!events) events += tpms_generic_decode(decoder, bitbuffer, preamble_pattern_aaa9, 16, 72, &tpms_aaa9_72_crc8_07_xaa_parse);
    if (!events) events += tpms_generic_decode(decoder, bitbuffer, preamble_pattern_aaa9, 16, 72, &tpms_aaa9_72_crc8_07_x00_parse);
    if (!events) events += tpms_generic_decode(decoder, bitbuffer, preamble_pattern_aaa9, 16, 80, &tpms_aaa9_80_xor_parse);
    if (!events) events += tpms_generic_decode(decoder, bitbuffer, preamble_pattern_aaa9, 16, 80, &tpms_aaa9_80_crc8_07_x00_parse);

    if (!events) events += tpmsraw_decode(decoder, bitbuffer);

    return events;
}

static char *output_fields[] = {
    "model",
    "type",
    "id",
    "code",
    "mic",
    NULL
};

r_device tpms_ford = {
    .name           = "Ford TPMS",
    .modulation     = FSK_PULSE_PCM,
    .short_width    = 52, // 12-13 samples @250k
    .long_width     = 52, // FSK
    .reset_limit    = 1500, // Maximum gap size before End Of Message [us].
    .decode_fn      = &tpms_ford_callback,
    .disabled       = 0,
    .fields         = output_fields,
};

/* experimental extra decoders */

/* Long TPMS : 10 byte FSK Manchester, CRC

 * Packet nibbles:  FF II II II II TT TT PP PP CC
 * F = flags, (seen: 20, 21, 22, d4, e0, e1, e2, e3)
 * I = id, 32-bit
 * T = Unknown, likely Temperature
 * P = Unknown, likely Pressure
 * C = Checksum, CRC-8 truncated poly 0x07 init 0xaa
 */

// full preamble is 55 55 55 56 (inverted: aa aa aa a9)
static const uint8_t long_preamble_pattern[2] = { 0xaa, 0xa9 }; // 16 bits

static int tpmslong_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos) {
    data_t *data;
    unsigned int start_pos;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    int flags;
    char flags_str[3];
    int id;
    char id_str[9];
    int code;
    char code_str[9];

    start_pos = bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 255);
    //fprintf(stderr, "bitbuffer_manchester_decode from %d is %d\n", bitpos, start_pos-bitpos);
    //bitbuffer_print(&packet_bits);
    // require 80 data bits
    if (start_pos-bitpos < 160) {
        //bitbuffer_print(bitbuffer);
        return 0;
    }
    b = packet_bits.bb[0];

    if (crc8(b, 9, 0x07, 0xaa) != b[9]) {
        //fprintf(stderr, "crc failed %02x vs %02x\n", crc8(b, 9, 0x07, 0x00), b[8]);
        //bitbuffer_print(&packet_bits);
        return 0;
    }

    flags = b[0];
    sprintf(flags_str, "%02x", flags);

    id = b[1]<<24 | b[2]<<16 | b[3]<<8 | b[4];
    sprintf(id_str, "%08x", id);

    code = b[5]<<24 | b[6]<<16 | b[7]<<8 | b[8];
    sprintf(code_str, "%08x", code);

    data = data_make(
            "model",        "",     DATA_STRING, "Long",
            "type",         "",     DATA_STRING, "TPMS",
            "flags",        "",     DATA_STRING, flags_str,
            "id",           "",     DATA_STRING, id_str,
            "code",         "",     DATA_STRING, code_str,
            "mic",          "",     DATA_STRING, "CRC",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}

static int tpmslong_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    int row;
    unsigned bitpos;
    int events = 0;

    // bitbuffer_invert(bitbuffer); // already inverted

    for (row = 0; row < bitbuffer->num_rows; ++row) {
        bitpos = 0;
        // Find a preamble with enough bits after it that it could be a complete packet
        while ((bitpos = bitbuffer_search(bitbuffer, row, bitpos,
                (const uint8_t *)&long_preamble_pattern, 16)) + 160 <=
                bitbuffer->bits_per_row[row]) {
            events += tpmslong_parse(decoder, bitbuffer, row, bitpos + 16);
            bitpos += 15;
        }
    }

    return events;
}

/* Verylong TPMS : Inverted 13 byte FSK Manchester, XOR check
(orig head: 0x3f ff ff 55 55 55 56)
 */

// full preamble is 3f ff ff 55 55 55 56
static const uint8_t verylong_preamble_pattern[2] = { 0xaa, 0xa9 }; // 16 bits

static int tpmsverylong_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos) {
    data_t *data;
    unsigned int start_pos;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    int id;
    char id_str[9];
    int code;
    char code_str[255];
    int i;
    int crc = 0;

    start_pos = bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 255);
    //fprintf(stderr, "bitbuffer_manchester_decode from %d is %d\n", bitpos, start_pos-bitpos);
    //bitbuffer_print(&packet_bits);
    // require 104 data bits
    if (start_pos-bitpos < 208) {
        //bitbuffer_print(bitbuffer);
        return 0;
    }
    b = packet_bits.bb[0];

    int chk = b[0] ^ b[1] ^ b[2] ^ b[3] ^ b[4] ^ b[5] ^ b[6] ^ b[7] ^ b[8] ^ b[9] ^ b[10] ^ b[11];
    if (chk != b[12]) {
        fprintf(stderr, "checksum failed %02x vs %02x\n", chk, b[12]);
        //bitbuffer_print(&packet_bits);
        return 0;
    }

    //bitbuffer_print(&packet_bits);
    //bitbuffer_print(bitbuffer);

    //id = b[0]<<24 | b[1]<<16 | b[2]<<8 | b[3];
    //sprintf(id_str, "%08x", id);
    //
    //code = b[4]<<16 | b[5]<<8 | b[6];
    //sprintf(code_str, "%06x", code);

    sprintf(code_str, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11]);

    data = data_make(
            "model",        "",     DATA_STRING, "Verylong",
            "type",         "",     DATA_STRING, "TPMS",
//            "id",           "",     DATA_STRING, id_str,
            "code",         "",     DATA_STRING, code_str,
            "mic",          "",     DATA_STRING, "CHECKSUM",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}

static int tpmsverylong_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    int row;
    unsigned bitpos;
    int events = 0;

    // bitbuffer_invert(bitbuffer); // already inverted

    for (row = 0; row < bitbuffer->num_rows; ++row) {
        bitpos = 0;
        // Find a preamble with enough bits after it that it could be a complete packet
        while ((bitpos = bitbuffer_search(bitbuffer, row, bitpos,
                (const uint8_t *)&verylong_preamble_pattern, 16)) + 224 <=
                bitbuffer->bits_per_row[row]) {
            events += tpmsverylong_parse(decoder, bitbuffer, row, bitpos + 16);
            bitpos += 15;
        }
    }

    return events;
}

/* FSK 9/10 byte Manchester encoded TPMS with XOR.
 *
9: BMW oder Citroen?
10: (VW Passat, Polo?) Renault!
 */

// full preamble is 55 55 55 56 (inverted: aa aa aa a9)
static const uint8_t xor_preamble_pattern[2] = { 0xaa, 0xa9 }; // 16 bits

static int tpmsxor_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos) {
    data_t *data;
    unsigned int start_pos;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    char id_str[9];
    char code_str[12];

    start_pos = bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 160);
    // require 72 data bits
    if (start_pos-bitpos < 144) {
        return 0;
    }
    b = packet_bits.bb[0];

    int chk = b[0] ^ b[1] ^ b[2] ^ b[3] ^ b[4] ^ b[5] ^ b[6] ^ b[7];
    if (chk != b[8]) {
        return 0;
    }

    sprintf(id_str, "%02x%02x%02x%02x", b[0], b[1], b[2], b[3]);
    if (start_pos-bitpos > 146) {
        sprintf(code_str, "%02x%02x%02x%02x %02x", b[4], b[5], b[6], b[7], b[9]);
    } else {
        sprintf(code_str, "%02x%02x%02x%02x", b[4], b[5], b[6], b[7]);
    }

    data = data_make(
            "model",        "",     DATA_STRING, start_pos-bitpos>146 ? "XOR-10" : "XOR-9",
            "type",         "",     DATA_STRING, "TPMS",
            "id",           "",     DATA_STRING, id_str,
//            "flags",        "",     DATA_STRING, flags_str,
            "len",          "",     DATA_INT, (start_pos-bitpos)/2,
            "code",         "",     DATA_STRING, code_str,
            "mic",          "",     DATA_STRING, "CHECKSUM",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}

static int tpmsxor_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    int row;
    unsigned bitpos;
    int events = 0;

    // bitbuffer_invert(bitbuffer); // already inverted

    for (row = 0; row < bitbuffer->num_rows; ++row) {
        bitpos = 0;
        // Find a preamble with enough bits after it that it could be a complete packet
        while ((bitpos = bitbuffer_search(bitbuffer, row, bitpos,
                (const uint8_t *)&xor_preamble_pattern, 16)) + 160 <=
                bitbuffer->bits_per_row[row]) {
            events += tpmsxor_parse(decoder, bitbuffer, row, bitpos + 16);
            bitpos += 15;
        }
    }

    return events;
}

/*
 * preamble 0x6665, 88 manchester bits, CRC-16  poly=0x1021  init=0x0288
 * preamble 0x6665, 88 manchester bits, CRC-16  poly=0x1021  init=0xf297
 * preamble 0xaaa9, 72 manchester bits, add byte 0 to 7 equals byte 8
 * preamble 0xaaa9, 72 manchester bits, CRC-8, poly=0x07  init=0xaa
 * preamble 0xaaa9, 72 manchester bits, CRC-8, poly=0x07  init=0x00
 * preamble 0xaaa9, 80 manchester bits, xor byte 1 to 8 equals byte 9
 * preamble 0xaaa9, 80 (80-83) manchester bits, CRC-8, poly=0x07, init=0x00
 */

static int tpms_6665_88_crc16_1021_0288_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    data_t *data;
    unsigned int start_pos;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    char id_str[9];
    char code_str[11];

    start_pos = bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 88 * 2);
    if (start_pos-bitpos < 88 * 2) {
        return 0;
    }
    b = packet_bits.bb[0];

    int chk = crc16(b, 11, 0x1021, 0x0288);
    if (chk) {
        return 0;
    }

    sprintf(id_str, "%02x%02x%02x%02x", b[0], b[1], b[2], b[3]);
    sprintf(code_str, "%02x%02x%02x%02x%02x", b[4], b[5], b[6], b[7], b[8]);

    data = data_make(
            "model",        "",     DATA_STRING, "tpms_6665_88_crc16_1021_0288",
            "type",         "",     DATA_STRING, "TPMS",
            "id",           "",     DATA_STRING, id_str,
            "code",         "",     DATA_STRING, code_str,
            "mic",          "",     DATA_STRING, "CRC",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}

static int tpms_6665_88_crc16_1021_f297_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    data_t *data;
    unsigned int start_pos;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    char id_str[9];
    char code_str[11];

    start_pos = bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 88 * 2);
    if (start_pos-bitpos < 88 * 2) {
        return 0;
    }
    b = packet_bits.bb[0];

    int chk = crc16(b, 11, 0x1021, 0xf297);
    if (chk) {
        return 0;
    }

    sprintf(id_str, "%02x%02x%02x%02x", b[0], b[1], b[2], b[3]);
    sprintf(code_str, "%02x%02x%02x%02x%02x", b[4], b[5], b[6], b[7], b[8]);

    data = data_make(
            "model",        "",     DATA_STRING, "tpms_6665_88_crc16_1021_f297",
            "type",         "",     DATA_STRING, "TPMS",
            "id",           "",     DATA_STRING, id_str,
            "code",         "",     DATA_STRING, code_str,
            "mic",          "",     DATA_STRING, "CRC",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}

static int tpms_aaa9_72_add_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    data_t *data;
    unsigned int start_pos;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    char id_str[9];
    char code_str[9];

    start_pos = bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 72 * 2);
    if (start_pos-bitpos < 72 * 2) {
        return 0;
    }
    b = packet_bits.bb[0];

    uint8_t chk = b[0] + b[1] + b[2] + b[3] + b[4] + b[5] + b[6] + b[7];
    if (chk != b[8]) {
        return 0;
    }

    sprintf(id_str, "%02x%02x%02x%02x", b[0], b[1], b[2], b[3]);
    sprintf(code_str, "%02x%02x%02x%02x", b[4], b[5], b[6], b[7]);

    data = data_make(
            "model",        "",     DATA_STRING, "tpms_aaa9_72_add",
            "type",         "",     DATA_STRING, "TPMS",
            "id",           "",     DATA_STRING, id_str,
            "code",         "",     DATA_STRING, code_str,
            "mic",          "",     DATA_STRING, "CHECKSUM",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}

static int tpms_aaa9_72_crc8_07_xaa_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    data_t *data;
    unsigned int start_pos;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    char id_str[9];
    char code_str[9];

    start_pos = bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 72 * 2);
    if (start_pos-bitpos < 72 * 2) {
        return 0;
    }
    b = packet_bits.bb[0];

    int chk = crc8(b, 9, 0x07, 0xaa);
    if (chk) {
        return 0;
    }

    sprintf(id_str, "%02x%02x%02x%02x", b[0], b[1], b[2], b[3]);
    sprintf(code_str, "%02x%02x%02x%02x", b[4], b[5], b[6], b[7]);

    data = data_make(
            "model",        "",     DATA_STRING, "tpms_aaa9_72_crc8_07_xaa",
            "type",         "",     DATA_STRING, "TPMS",
            "id",           "",     DATA_STRING, id_str,
            "code",         "",     DATA_STRING, code_str,
            "mic",          "",     DATA_STRING, "CRC",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}

static int tpms_aaa9_72_crc8_07_x00_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    data_t *data;
    unsigned int start_pos;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    char id_str[9];
    char code_str[12];

    start_pos = bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 72 * 2);
    if (start_pos-bitpos < 72 * 2) {
        return 0;
    }
    b = packet_bits.bb[0];

    int chk = crc8(b, 9, 0x07, 0x00);
    if (chk) {
        return 0;
    }

    sprintf(id_str, "%02x%02x%02x%02x", b[0], b[1], b[2], b[3]);
    sprintf(code_str, "%02x%02x%02x%02x", b[4], b[5], b[6], b[7]);

    data = data_make(
            "model",        "",     DATA_STRING, "tpms_aaa9_72_crc8_07_x00",
            "type",         "",     DATA_STRING, "TPMS",
            "id",           "",     DATA_STRING, id_str,
            "code",         "",     DATA_STRING, code_str,
            "mic",          "",     DATA_STRING, "CRC",
            NULL);

    // actually Renault TPMS
    //decoder_output_data(decoder, data);
    return 1;
}

static int tpms_aaa9_80_xor_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    data_t *data;
    unsigned int start_pos;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    char id_str[9];
    char code_str[11];

    start_pos = bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 80 * 2);
    if (start_pos-bitpos < 80 * 2) {
        return 0;
    }
    b = packet_bits.bb[0];

    int chk = b[1] ^ b[2] ^ b[3] ^ b[4] ^ b[5] ^ b[6] ^ b[7] ^ b[8];
    if (chk != b[9]) {
        return 0;
    }

    sprintf(id_str, "%02x%02x%02x%02x", b[0], b[1], b[2], b[3]);
    sprintf(code_str, "%02x%02x%02x%02x%02x", b[4], b[5], b[6], b[7], b[8]);

    data = data_make(
            "model",        "",     DATA_STRING, "tpms_aaa9_80_xor",
            "type",         "",     DATA_STRING, "TPMS",
            "id",           "",     DATA_STRING, id_str,
            "code",         "",     DATA_STRING, code_str,
            "mic",          "",     DATA_STRING, "CHECKSUM",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}

static int tpms_aaa9_80_crc8_07_x00_parse(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row, unsigned bitpos)
{
    data_t *data;
    unsigned int start_pos;
    bitbuffer_t packet_bits = {0};
    uint8_t *b;
    char id_str[9];
    char code_str[11];

    start_pos = bitbuffer_manchester_decode(bitbuffer, row, bitpos, &packet_bits, 80 * 2);
    if (start_pos-bitpos < 80 * 2) {
        return 0;
    }
    b = packet_bits.bb[0];

    int chk = crc8(b, 10, 0x07, 0x00);
    if (chk) {
        return 0;
    }

    sprintf(id_str, "%02x%02x%02x%02x", b[0], b[1], b[2], b[3]);
    sprintf(code_str, "%02x%02x%02x%02x%02x", b[4], b[5], b[6], b[7], b[8]);

    data = data_make(
            "model",        "",     DATA_STRING, "tpms_aaa9_80_crc8_07_x00",
            "type",         "",     DATA_STRING, "TPMS",
            "id",           "",     DATA_STRING, id_str,
            "code",         "",     DATA_STRING, code_str,
            "mic",          "",     DATA_STRING, "CRC",
            NULL);

    decoder_output_data(decoder, data);
    return 1;
}

/* generic */

static int tpms_generic_decode(r_device *decoder, bitbuffer_t *bitbuffer,
        uint8_t const *preamble_pattern, unsigned preamble_len,
        unsigned data_len, tpms_parse_fn tpms_parse)
{
    int row;
    unsigned bitpos;
    int events = 0;

    // bitbuffer_invert(bitbuffer); // already inverted

    for (row = 0; row < bitbuffer->num_rows; ++row) {
        bitpos = 0;
        // Find a preamble with enough bits after it that it could be a complete packet
        while ((bitpos = bitbuffer_search(bitbuffer, row, bitpos,
                preamble_pattern, preamble_len)) + 2 * data_len <=
                bitbuffer->bits_per_row[row]) {
            events += tpms_parse(decoder, bitbuffer, row, bitpos + preamble_len);
            bitpos += 15;
        }
    }

    return events;
}


/* Raw TPMS
 */

static int tpmsraw_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    int row;
    int events = 0;

    // bitbuffer_invert(bitbuffer); // already inverted

    for (row = 0; row < bitbuffer->num_rows; ++row) {

        if (bitbuffer->bits_per_row[row] < 120) continue;
        int i;
        char code_str[BITBUF_COLS*2 + 10];
        sprintf(code_str, "{%d}  ", bitbuffer->bits_per_row[row]);
        for (i = 0; i < (bitbuffer->bits_per_row[row]+7)/8; ++i) {
            sprintf(&code_str[5+i*2], "%02x", bitbuffer->bb[row][i]);
        }

        data_t *data;
        data = data_make(
                "model",        "",     DATA_STRING, "Raw FSK",
                "type",         "",     DATA_STRING, "TPMS",
                "code",         "",     DATA_STRING, code_str,
                NULL);

        decoder_output_data(decoder, data);
    }

    return events;
}
