#include "config_backup.h"
#include "independent_steering.h"
#include "steering_limit.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    uint8_t data[CONFIG_BACKUP_SIZE], copy[CONFIG_BACKUP_SIZE];
    configuration_bundle_t b, roundtrip;
    FILE *file = fopen("tests/host/fixtures/config_backup.rcconfig", "rb");
    assert(file && fread(data,1,sizeof(data),file) == sizeof(data));
    assert(fgetc(file) == EOF); fclose(file);
    assert(config_backup_decode(data,sizeof(data),&b));
    assert(b.throttle.neutral_us == 1514 && b.throttle.loaded_from_nvs && b.arm.loaded_from_nvs);
    assert(b.drive.steering.wheel_calibration[0].count == 3);
    assert(b.drive.steering.wheel_calibration[1].count == 5);
    assert(b.drive.steering_limit.points[1].limit_tenths_percent == 650);
    assert(config_backup_encode(&b,copy,sizeof(copy)) && memcmp(copy,data,sizeof(data)) == 0);
    assert(config_backup_decode(copy,sizeof(copy),&roundtrip));
    for (size_t i = 0; i < sizeof(data); i++) {
        copy[i] ^= 1;
        assert(!config_backup_decode(copy,sizeof(copy),&roundtrip));
        copy[i] ^= 1;
    }
    assert(!config_backup_decode(data,sizeof(data)-1,&roundtrip));
    assert(!config_backup_decode(data,sizeof(data)+1,&roundtrip));
    assert(!config_backup_decode(NULL,sizeof(data),&roundtrip));
    configuration_bundle_t invalid = b;
    invalid.drive.rpm_pulses_per_revolution = 0; assert(!config_backup_encode(&invalid,copy,sizeof(copy)));
    invalid = b; invalid.throttle.deadband_us = 151; assert(!configuration_bundle_valid(&invalid));
    invalid = b; invalid.arm.stop_2_us = invalid.arm.run_us; assert(!configuration_bundle_valid(&invalid));
    invalid = b; invalid.drive.steering.toe_tenths_deg = 850; assert(!configuration_bundle_valid(&invalid));
    b.input.loaded_from_nvs = b.throttle.loaded_from_nvs = b.arm.loaded_from_nvs = false;
    assert(config_backup_encode(&b,copy,sizeof(copy)) && config_backup_decode(copy,sizeof(copy),&roundtrip));
    assert(!roundtrip.input.loaded_from_nvs && !roundtrip.throttle.loaded_from_nvs && !roundtrip.arm.loaded_from_nvs);
    puts("Configuration backup: Python/zlib golden file, full calibration roundtrip, corrupt/truncated files and validation passed.");
}
