#include "dragy_nmea.h"

#include <math.h>
#include <stddef.h>

#define CHECK(condition)            \
    do {                            \
        if (!(condition)) {         \
            __builtin_trap();       \
        }                           \
    } while (0)

static bool feed_sentence(dragy_nmea_parser_t *parser, const char *sentence)
{
    bool updated = false;
    for (size_t index = 0; sentence[index] != '\0'; index++) {
        if (dragy_nmea_feed(parser, (uint8_t)sentence[index])) updated = true;
    }
    return updated;
}

static void test_rmc_and_gga_fix(void)
{
    dragy_nmea_parser_t parser;
    dragy_nmea_init(&parser);
    CHECK(feed_sentence(&parser,
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n"));
    CHECK(parser.fix.fix_valid);
    CHECK(parser.fix.position_valid);
    CHECK(fabs(parser.fix.latitude_deg - 48.1173) < 0.00001);
    CHECK(fabs(parser.fix.longitude_deg - 11.5166667) < 0.00001);
    CHECK(parser.fix.speed_valid);
    CHECK(fabsf(parser.fix.speed_mps - 11.5236f) < 0.001f);
    CHECK(parser.fix.course_valid);
    CHECK(fabsf(parser.fix.course_deg - 84.4f) < 0.01f);
    CHECK(parser.fix.utc_time_ms == 45319000);
    CHECK(parser.fix.date_ddmmyy == 230394);

    CHECK(feed_sentence(&parser,
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n"));
    CHECK(parser.fix.fix_valid);
    CHECK(parser.fix.satellites == 8);
    CHECK(fabsf(parser.fix.hdop - 0.9f) < 0.01f);
    CHECK(parser.fix.altitude_valid);
    CHECK(fabsf(parser.fix.altitude_m - 545.4f) < 0.01f);
    CHECK(parser.fix.valid_sentence_count == 2);
}

static void test_vtg_and_talker_variants(void)
{
    dragy_nmea_parser_t parser;
    dragy_nmea_init(&parser);
    CHECK(feed_sentence(&parser,
        "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48\n"));
    CHECK(parser.fix.speed_valid);
    CHECK(fabsf(parser.fix.speed_mps - 2.8333f) < 0.001f);
    CHECK(parser.fix.course_valid);
    CHECK(fabsf(parser.fix.course_deg - 54.7f) < 0.01f);

    CHECK(feed_sentence(&parser,
        "$GNRMC,092204.999,A,4250.5589,S,14718.5084,E,0.02,31.66,200520,,,A*5C\n"));
    CHECK(parser.fix.position_valid);
    CHECK(parser.fix.latitude_deg < 0.0);
    CHECK(parser.fix.longitude_deg > 0.0);
}

static void test_rejects_bad_checksum_and_recovers(void)
{
    dragy_nmea_parser_t parser;
    dragy_nmea_init(&parser);
    CHECK(!feed_sentence(&parser,
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*00\n"));
    CHECK(parser.fix.checksum_error_count == 1);
    CHECK(parser.fix.valid_sentence_count == 0);
    CHECK(feed_sentence(&parser,
        "$GPRMC,123519,V,,,,,,,230394,,,N*51\n"));
    CHECK(!parser.fix.fix_valid);
    CHECK(!parser.fix.position_valid);
    CHECK(parser.fix.valid_sentence_count == 1);
}

static void test_rejects_invalid_vtg_after_prior_fix(void)
{
    dragy_nmea_parser_t parser;
    dragy_nmea_init(&parser);
    CHECK(feed_sentence(&parser,
        "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48\n"));
    CHECK(!feed_sentence(&parser,
        "$GPVTG,,T,,M,,N,,K*4E\n"));
    CHECK(parser.fix.valid_sentence_count == 1);
    CHECK(parser.fix.parse_error_count == 1);
}

int main(void)
{
    test_rmc_and_gga_fix();
    test_vtg_and_talker_variants();
    test_rejects_bad_checksum_and_recovers();
    test_rejects_invalid_vtg_after_prior_fix();
    return 0;
}
