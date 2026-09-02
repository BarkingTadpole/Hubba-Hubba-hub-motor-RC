#include "dragy_nmea.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NMEA_MAX_FIELDS 24
#define KNOTS_TO_MPS 0.514444444f

static int hex_value(char value)
{
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    return -1;
}

static bool parse_double(const char *text, double *value)
{
    if (text == NULL || text[0] == '\0') return false;
    char *end = NULL;
    double parsed = strtod(text, &end);
    if (end == text || *end != '\0' || !isfinite(parsed)) return false;
    *value = parsed;
    return true;
}

static bool parse_float(const char *text, float *value)
{
    double parsed = 0.0;
    if (!parse_double(text, &parsed) || parsed < -3.4e38 || parsed > 3.4e38) {
        return false;
    }
    *value = (float)parsed;
    return true;
}

static bool parse_unsigned(const char *text, uint32_t maximum, uint32_t *value)
{
    if (text == NULL || text[0] == '\0') return false;
    char *end = NULL;
    unsigned long parsed = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || parsed > maximum) return false;
    *value = (uint32_t)parsed;
    return true;
}

static bool parse_coordinate(const char *text,
                             const char *hemisphere,
                             bool latitude,
                             double *degrees)
{
    double raw = 0.0;
    if (!parse_double(text, &raw) || raw < 0.0 || hemisphere == NULL ||
        hemisphere[0] == '\0' || hemisphere[1] != '\0') {
        return false;
    }

    double whole_degrees = floor(raw / 100.0);
    double minutes = raw - whole_degrees * 100.0;
    double maximum = latitude ? 90.0 : 180.0;
    if (minutes < 0.0 || minutes >= 60.0 || whole_degrees > maximum ||
        (whole_degrees == maximum && minutes > 0.0)) {
        return false;
    }

    char direction = hemisphere[0];
    if (latitude ? (direction != 'N' && direction != 'S')
                 : (direction != 'E' && direction != 'W')) {
        return false;
    }

    double result = whole_degrees + minutes / 60.0;
    if (direction == 'S' || direction == 'W') result = -result;
    *degrees = result;
    return true;
}

static bool parse_time_ms(const char *text, uint32_t *time_ms)
{
    double raw = 0.0;
    if (!parse_double(text, &raw) || raw < 0.0 || raw >= 240000.0) return false;
    uint32_t hours = (uint32_t)(raw / 10000.0);
    raw -= hours * 10000.0;
    uint32_t minutes = (uint32_t)(raw / 100.0);
    double seconds = raw - minutes * 100.0;
    if (hours > 23 || minutes > 59 || seconds < 0.0 || seconds >= 60.0) {
        return false;
    }
    *time_ms = (uint32_t)lround((hours * 3600.0 + minutes * 60.0 + seconds) * 1000.0);
    return true;
}

static bool sentence_type_is(const char *type, const char *suffix)
{
    size_t type_length = strlen(type);
    size_t suffix_length = strlen(suffix);
    return type_length >= suffix_length &&
           strcmp(type + type_length - suffix_length, suffix) == 0;
}

static size_t split_fields(char *payload, char *fields[NMEA_MAX_FIELDS])
{
    size_t count = 0;
    char *field = payload;
    while (count < NMEA_MAX_FIELDS) {
        fields[count++] = field;
        char *comma = strchr(field, ',');
        if (comma == NULL) break;
        *comma = '\0';
        field = comma + 1;
    }
    return count;
}

static bool parse_rmc(char *fields[NMEA_MAX_FIELDS],
                      size_t count,
                      dragy_nmea_fix_t *fix)
{
    if (count < 10 || (fields[2][0] != 'A' && fields[2][0] != 'V')) return false;
    parse_time_ms(fields[1], &fix->utc_time_ms);
    fix->fix_valid = fields[2][0] == 'A';

    double latitude = 0.0;
    double longitude = 0.0;
    fix->position_valid = fix->fix_valid &&
                          parse_coordinate(fields[3], fields[4], true, &latitude) &&
                          parse_coordinate(fields[5], fields[6], false, &longitude);
    if (fix->position_valid) {
        fix->latitude_deg = latitude;
        fix->longitude_deg = longitude;
    }

    float speed_knots = 0.0f;
    fix->speed_valid = fix->fix_valid && parse_float(fields[7], &speed_knots) &&
                       speed_knots >= 0.0f;
    if (fix->speed_valid) fix->speed_mps = speed_knots * KNOTS_TO_MPS;

    float course = 0.0f;
    fix->course_valid = fix->fix_valid && parse_float(fields[8], &course) &&
                        course >= 0.0f && course <= 360.0f;
    if (fix->course_valid) fix->course_deg = course;
    parse_unsigned(fields[9], 311299, &fix->date_ddmmyy);
    return true;
}

static bool parse_gga(char *fields[NMEA_MAX_FIELDS],
                      size_t count,
                      dragy_nmea_fix_t *fix)
{
    if (count < 10) return false;
    uint32_t quality = 0;
    if (!parse_unsigned(fields[6], 9, &quality)) return false;
    parse_time_ms(fields[1], &fix->utc_time_ms);
    fix->fix_valid = quality > 0;

    double latitude = 0.0;
    double longitude = 0.0;
    fix->position_valid = fix->fix_valid &&
                          parse_coordinate(fields[2], fields[3], true, &latitude) &&
                          parse_coordinate(fields[4], fields[5], false, &longitude);
    if (fix->position_valid) {
        fix->latitude_deg = latitude;
        fix->longitude_deg = longitude;
    }

    uint32_t satellites = 0;
    if (parse_unsigned(fields[7], 255, &satellites)) {
        fix->satellites = (uint8_t)satellites;
    }
    parse_float(fields[8], &fix->hdop);
    fix->altitude_valid = fix->fix_valid && parse_float(fields[9], &fix->altitude_m);
    return true;
}

static bool parse_vtg(char *fields[NMEA_MAX_FIELDS],
                      size_t count,
                      dragy_nmea_fix_t *fix)
{
    if (count < 9) return false;
    bool parsed = false;
    float course = 0.0f;
    if (parse_float(fields[1], &course) && course >= 0.0f && course <= 360.0f) {
        fix->course_deg = course;
        fix->course_valid = true;
        parsed = true;
    }
    float speed_kmh = 0.0f;
    if (parse_float(fields[7], &speed_kmh) && speed_kmh >= 0.0f) {
        fix->speed_mps = speed_kmh / 3.6f;
        fix->speed_valid = true;
        parsed = true;
    }
    return parsed;
}

static bool parse_sentence(dragy_nmea_parser_t *parser)
{
    char *star = strchr(parser->sentence, '*');
    if (parser->sentence[0] != '$' || star == NULL ||
        star[1] == '\0' || star[2] == '\0' || star[3] != '\0') {
        parser->fix.parse_error_count++;
        return false;
    }

    uint8_t checksum = 0;
    for (char *cursor = parser->sentence + 1; cursor < star; cursor++) {
        checksum ^= (uint8_t)*cursor;
    }
    int high = hex_value(star[1]);
    int low = hex_value(star[2]);
    if (high < 0 || low < 0 || checksum != (uint8_t)((high << 4) | low)) {
        parser->fix.checksum_error_count++;
        return false;
    }

    *star = '\0';
    char *fields[NMEA_MAX_FIELDS] = {0};
    size_t count = split_fields(parser->sentence + 1, fields);
    bool parsed = false;
    if (count > 0 && sentence_type_is(fields[0], "RMC")) {
        parsed = parse_rmc(fields, count, &parser->fix);
    } else if (count > 0 && sentence_type_is(fields[0], "GGA")) {
        parsed = parse_gga(fields, count, &parser->fix);
    } else if (count > 0 && sentence_type_is(fields[0], "VTG")) {
        parsed = parse_vtg(fields, count, &parser->fix);
    }

    if (parsed) {
        parser->fix.valid_sentence_count++;
    } else if (count > 0 && (sentence_type_is(fields[0], "RMC") ||
                            sentence_type_is(fields[0], "GGA") ||
                            sentence_type_is(fields[0], "VTG"))) {
        parser->fix.parse_error_count++;
    }
    return parsed;
}

void dragy_nmea_init(dragy_nmea_parser_t *parser)
{
    if (parser != NULL) memset(parser, 0, sizeof(*parser));
}

bool dragy_nmea_feed(dragy_nmea_parser_t *parser, uint8_t byte)
{
    if (parser == NULL) return false;
    if (byte == '$') {
        parser->sentence[0] = '$';
        parser->length = 1;
        return false;
    }
    if (parser->length == 0) return false;
    if (byte == '\r') return false;
    if (byte == '\n') {
        parser->sentence[parser->length] = '\0';
        parser->length = 0;
        return parse_sentence(parser);
    }
    if (byte < 0x20 || byte > 0x7e || parser->length + 1 >= sizeof(parser->sentence)) {
        parser->length = 0;
        parser->fix.parse_error_count++;
        return false;
    }
    parser->sentence[parser->length++] = (char)byte;
    return false;
}
