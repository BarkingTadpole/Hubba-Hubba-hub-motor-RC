"""Exercise production backup HTTP handlers against bounded simulated uploads."""
from pathlib import Path
import os
import re
import shlex
import struct
import subprocess
import tempfile
import zlib

ROOT = Path(__file__).resolve().parents[2]
wifi = (ROOT / 'main/wifi_control.c').read_text()

def function(name):
    match = re.search(r'^(?:static )?[\w *]+\b' + name + r'\([^;]*?\)\n\{.*?^\}', wifi, re.M | re.S)
    assert match, name
    return match.group()

code = r'''
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "powertrain_controller.h"
#include "drivetrain_control.h"
#define ESP_OK 0
#define ESP_FAIL -1
#define HTTPD_SOCK_ERR_TIMEOUT -7
typedef struct {int content_len;} httpd_req_t;
static uint8_t good[CONFIG_BACKUP_SIZE], input[CONFIG_BACKUP_SIZE], output[CONFIG_BACKUP_SIZE];
static size_t input_size, offset, binary_size;
static int timeout_count, reads, imports;
static bool allowed;
static char status[64], response[512], disposition[128];
static int httpd_req_recv(httpd_req_t *r,char *buffer,size_t size) {
    (void)r;reads++;
    if(timeout_count>0){timeout_count--;return HTTPD_SOCK_ERR_TIMEOUT;}
    if(offset==input_size)return 0;
    size_t chunk=input_size-offset;if(chunk>17)chunk=17;if(chunk>size)chunk=size;
    memcpy(buffer,input+offset,chunk);offset+=chunk;return (int)chunk;
}
static int send_json_message(httpd_req_t *r,const char *s,bool ok,const char *message) {
    (void)r;(void)ok;snprintf(status,sizeof(status),"%s",s);snprintf(response,sizeof(response),"%s",message);return 0;
}
static int send_json(httpd_req_t *r,const char *s,const char *body,size_t length) {
    (void)r;assert(length<sizeof(response));memcpy(response,body,length);response[length]=0;
    snprintf(status,sizeof(status),"%s",s);return 0;
}
static int httpd_resp_set_type(httpd_req_t *r,const char *type) {(void)r;assert(!strcmp(type,"application/octet-stream"));return 0;}
static int httpd_resp_set_hdr(httpd_req_t *r,const char *key,const char *value) {
    (void)r;if(!strcmp(key,"Content-Disposition"))snprintf(disposition,sizeof(disposition),"%s",value);return 0;
}
static int httpd_resp_send(httpd_req_t *r,const char *data,size_t size) {
    (void)r;assert(size==sizeof(output));memcpy(output,data,size);binary_size=size;return 0;
}
bool powertrain_export_configuration(uint8_t *data,size_t size) {
    if(!allowed)return false;
    assert(size==sizeof(good));memcpy(data,good,size);return true;
}
bool powertrain_import_configuration(const uint8_t *data,size_t size) {
    configuration_bundle_t b;assert(config_backup_decode(data,size,&b));imports++;return allowed;
}
static void reset(void) {
    memcpy(input,good,sizeof(input));input_size=sizeof(input);offset=0;timeout_count=reads=imports=0;
    binary_size=0;allowed=true;status[0]=response[0]=disposition[0]=0;
}
'''
code += '\n'.join(function(name) for name in [
    'bool_json', 'backup_get_handler', 'receive_backup', 'backup_preview_handler', 'backup_import_handler'])

# Recompute CRC to verify semantic validation, not just corruption detection.
golden = (ROOT / 'tests/host/fixtures/config_backup.rcconfig').read_bytes()
mutants = []
for offset, value in [(16, 8), (40, 256), (42, 256), (48, 2), (60, 2), (138, 10), (216, 1), (224, 1001)]:
    data = bytearray(golden)
    struct.pack_into('<H', data, offset, value)
    struct.pack_into('<I', data, 12, zlib.crc32(data[16:]))
    mutants.append('{' + ','.join(str(v) for v in data) + '}')
code += '\nstatic const uint8_t invalid[][CONFIG_BACKUP_SIZE]={' + ',\n'.join(mutants) + '};\n'
code += r'''
int main(int argc,char **argv) {
    assert(argc==2);FILE *f=fopen(argv[1],"rb");assert(f && fread(good,1,sizeof(good),f)==sizeof(good));fclose(f);
    httpd_req_t request={CONFIG_BACKUP_SIZE};reset();
    timeout_count=2;assert(backup_preview_handler(&request)==0 && !strcmp(status,"200 OK"));
    assert(strstr(response,"\"left_wheel_points\":3") && strstr(response,"\"right_wheel_points\":5"));
    const int sizes[]={-1,0,249,251,INT_MAX};
    for(size_t i=0;i<sizeof(sizes)/sizeof(sizes[0]);i++) {
        reset();request.content_len=sizes[i];backup_import_handler(&request);
        assert(!strcmp(status,"400 Bad Request") && reads==0 && imports==0);
    }
    request.content_len=CONFIG_BACKUP_SIZE;
    reset();input_size--;backup_import_handler(&request);assert(!strcmp(status,"400 Bad Request") && imports==0);
    reset();timeout_count=3;backup_import_handler(&request);assert(reads==3 && imports==0);
    reset();input[100]^=1;backup_import_handler(&request);assert(!strcmp(status,"400 Bad Request") && imports==0);
    for(size_t i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++) {
        reset();memcpy(input,invalid[i],sizeof(input));backup_import_handler(&request);
        assert(!strcmp(status,"400 Bad Request") && imports==0);
    }
    reset();allowed=false;backup_import_handler(&request);assert(!strcmp(status,"409 Conflict") && imports==1);
    reset();backup_import_handler(&request);assert(!strcmp(status,"200 OK") && imports==1);
    reset();allowed=false;backup_get_handler(&request);assert(!strcmp(status,"409 Conflict") && binary_size==0);
    reset();backup_get_handler(&request);assert(binary_size==sizeof(good) && !memcmp(good,output,sizeof(good)));
    assert(strstr(disposition,".rcconfig"));
    puts("Backup HTTP: download, preview, bounded/partial/timed-out uploads, semantic rejection and import dispatch passed.");
}
'''
with tempfile.TemporaryDirectory() as directory:
    temp = Path(directory)
    (temp / 'esp_err.h').write_text('typedef int esp_err_t;\n')
    source = temp / 'http.c'
    source.write_text(code)
    executable = temp / 'backup_http'
    subprocess.run([*shlex.split(os.environ.get('CC', 'cc')), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(temp), '-I', str(ROOT / 'main/include'), str(source),
                    *[str(ROOT / 'main' / f'{name}.c') for name in
                      ['config_backup', 'independent_steering', 'steering_limit', 'drivetrain_control']],
                    '-lm', '-o', str(executable)], check=True)
    subprocess.run([str(executable), str(ROOT / 'tests/host/fixtures/config_backup.rcconfig')], check=True)
