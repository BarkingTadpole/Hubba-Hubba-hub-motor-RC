#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DRAGY_NMEA_SENTENCE_MAX 128

typedef struct {
    double latitude_deg;
    double longitude_deg;
    float speed_mps;
    float course_deg;
    float altitude_m;
    float hdop;
    uint32_t utc_time_ms;
    uint32_t date_ddmmyy;
    uint32_t valid_sentence_count;
    uint32_t checksum_error_count;
    uint32_t parse_error_count;
    uint8_t satellites;
    bool fix_valid;
    bool position_valid;
    bool speed_valid;
    bool course_valid;
    bool altitude_valid;
} dragy_nmea_fix_t;

typedef struct {
    char sentence[DRAGY_NMEA_SENTENCE_MAX];
    size_t length;
    dragy_nmea_fix_t fix;
} dragy_nmea_parser_t;

void dragy_nmea_init(dragy_nmea_parser_t *parser);
bool dragy_nmea_feed(dragy_nmea_parser_t *parser, uint8_t byte);
