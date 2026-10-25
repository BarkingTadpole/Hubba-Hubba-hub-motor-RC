"""Compile production JSON formatters and validate their actual output and bounds."""
from pathlib import Path
import json
import os
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
wifi = (ROOT/'main/wifi_control.c').read_text()
def function(source, name):
    m = re.search(r'^(?:static )?[\w *]+\b'+name+r'\([^;]*?\)\n\{.*?^\}', source, re.M|re.S)
    assert m, name
    return m.group()
base = r'''
#include <assert.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "powertrain_controller.h"
#include "independent_steering.h"
#include "drivetrain_control.h"
#define ESP_LOGE(...) ((void)0)
static powertrain_remote_snapshot_t sample;
static bool powertrain_get_remote_snapshot_stub(powertrain_remote_snapshot_t *s) {*s=sample;return true;}
#define powertrain_get_remote_snapshot powertrain_get_remote_snapshot_stub
static void setup(void) {
    independent_steering_defaults(&sample.config.steering);
    sample.config.steering.trim_us[0]=450;sample.config.steering.trim_us[1]=-450;
    sample.config.steering.linkage=(steering_linkage_limits_t){900,655};
    sample.config.steering.toe_tenths_deg=-300;sample.config.tank_limit_percent=100;
    for(int side=0;side<2;side++) {
        wheel_angle_calibration_t *cal=&sample.config.steering.wheel_calibration[side];
        cal->count=9;
        int center=1500+sample.config.steering.trim_us[side];
        for(int i=0;i<9;i++) cal->points[i]=(wheel_angle_point_t){
            i<=4 ? 2000+(center-2000)*i/4 : center+(1000-center)*(i-4)/4, -300+i*75};
    }
    sample.wheel_capture_sequence=UINT32_MAX;
    sample.config.drivetrain_mode=DRIVETRAIN_TANK;
    sample.config.steering.point_count=8;
    for(int i=0;i<8;i++) sample.config.steering.points[i]=(ackermann_point_t){i*400,1000-i*100};
    sample.steering_calibration=(steering_calibration_t){.left_us=2050,.center_us=1518,.right_us=987,.deadband_us=30};
    sample.steering_servo_us[0]=1800;sample.steering_servo_us[1]=1300;
    sample.rpm.rpm[0]=1234;sample.rpm.rpm[1]=2345;sample.rpm.valid[0]=true;sample.rpm.valid[1]=true;
    sample.vehicle_speed_mps=12;sample.vehicle_speed_held=true;sample.ackermann_strength_percent=25;
    sample.steering_rejected_spikes=UINT32_MAX;sample.captured_at_us=INT64_MAX;
    memset(sample.calibration_message,'"',sizeof(sample.calibration_message)-1);
    memset(sample.calibration_prompt,'\\',sizeof(sample.calibration_prompt)-1);
}
'''
json_types = re.search(r'typedef struct \{\n    char \*data;.*?\} json_builder_t;',wifi,re.S).group()
json_code=base+json_types+'\n'+ '\n'.join(function(wifi,f) for f in [
    'bool_json','finite_number','json_escape_string','calibration_kind_name',
    'sample_age_ms','json_append','append_config','format_telemetry','format_config_response',
    'parse_command_body'])+r'''
int main(void) {
    setup();char frame[8192],config[3072],small[8];
    char command[256];
    const char *valid = " {\"command\": \"config reverse 20\"} ";
    assert(parse_command_body(valid,strlen(valid),command,sizeof(command)));
    assert(strcmp(command,"config reverse 20")==0);
    assert(!parse_command_body(valid,strlen(valid),small,sizeof(small)));
    const char *invalid[] = {
        "{}", "{\"command\":\"\"}", "{\"command\":\"config reverse 20\"} extra",
        "{\"command\":\"arm\\nconfig reverse 20\"}",
        "{\"command\":\"config reverse 20\",\"extra\":1}",
        "{\"command\":\"config reverse 20\""
    };
    for(size_t i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++)
        assert(!parse_command_body(invalid[i],strlen(invalid[i]),command,sizeof(command)));
    assert(format_telemetry(frame,sizeof(frame))>0);
    assert(format_config_response(config,sizeof(config))>0);
    assert(format_telemetry(small,sizeof(small))==-1);
    assert(format_config_response(small,sizeof(small))==-1);
    puts(frame);puts(config);
}
'''
with tempfile.TemporaryDirectory() as tmp:
    temp=Path(tmp);(temp/'esp_err.h').write_text('typedef int esp_err_t;\n')
    src=temp/'json.c';src.write_text(json_code);exe=temp/'json'
    subprocess.run([*shlex.split(os.environ.get('CC','cc')),'-std=c11','-Wall','-Wextra','-Werror','-I',str(temp),
                    '-I',str(ROOT/'main/include'),str(src),str(ROOT/'main/independent_steering.c'),
                    str(ROOT/'main/drivetrain_control.c'),'-lm','-o',str(exe)],check=True)
    output=subprocess.check_output([str(exe)],text=True).splitlines()
    telemetry,config=map(json.loads,output)
    assert telemetry['protocol']==3 and telemetry['drive_mode']=='TANK'
    assert telemetry['config']['steering_toe_deg']==-30 and telemetry['config']['tank_limit_percent']==100
    assert telemetry['config']['steering_toe_range']=={'min_deg': -30, 'max_deg': 22.9}
    assert [wheel['wheel'] for wheel in telemetry['rpm']['wheels']]==['RL','RR']
    assert set(telemetry['rc'])=={'throttle','steering','arm'}
    assert all(key not in telemetry for key in ['tv','imu','dragy','logging'])
    assert telemetry['steering']['left_us']==1800 and telemetry['steering']['right_us']==1300
    assert len(config['config']['ackermann_strength_points'])==8 and config['config']==telemetry['config']
    assert config['config']['steering_geometry']['wheelbase_mm']==445
    assert config['config']['servos'][0]['left_angle_deg']==30
    assert all(len(s['wheel_points'])==9 for s in config['config']['servos'])
    assert telemetry['calibration']['wheel_capture_sequence']==4294967295
    assert [servo['trim_us'] for servo in config['config']['servos']]==[450,-450]
    assert config['config']['steering_input']['deadband_us']==30
    assert config['config']['steering_linkage']=={'inside_deg':90, 'outside_deg':65.5}
    assert 'logging_rate_hz' not in config['config'] and 'logging' not in config
    fixture=json.loads((ROOT/'tests/host/fixtures/monitor_state.json').read_text())
    assert telemetry.keys()==fixture.keys()
    for section in ['rc','steering','rpm','config','calibration']:
        assert telemetry[section].keys()==fixture[section].keys(), section
    for actual,expected in zip(telemetry['config']['servos'],fixture['config']['servos']):
        assert actual.keys()==expected.keys()
    assert config['config']['steering_linkage'].keys()==fixture['config']['steering_linkage'].keys()
    assert config['config']['steering_toe_range'].keys()==fixture['config']['steering_toe_range'].keys()
    print(f'Wire formats passed: {len(output[0])}-byte JSON, complete curves, bounded buffers, HTTP command bodies and browser fixture.')
