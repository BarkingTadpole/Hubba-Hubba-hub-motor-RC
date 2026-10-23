"""Exercise real NVS config loading/saving against a deterministic in-memory NVS stub."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory() as tmp:
    temp=Path(tmp)
    (temp/'esp_err.h').write_text('''#pragma once
    typedef int esp_err_t;
    #define ESP_OK 0
    #define ESP_ERR_INVALID_ARG 1
    #define ESP_ERR_NVS_NO_FREE_PAGES 2
    #define ESP_ERR_NVS_NEW_VERSION_FOUND 3
    #define ESP_ERR_NVS_NOT_FOUND 4
    ''')
    header='''#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
typedef int nvs_handle_t;
typedef int nvs_open_mode_t;
#define NVS_READONLY 0
#define NVS_READWRITE 1
esp_err_t nvs_open(const char *, nvs_open_mode_t, nvs_handle_t *);
void nvs_close(nvs_handle_t);
esp_err_t nvs_commit(nvs_handle_t);
esp_err_t nvs_get_blob(nvs_handle_t,const char *,void *,size_t *);
esp_err_t nvs_set_blob(nvs_handle_t,const char *,const void *,size_t);
'''
    for kind in ['u8','u16']:
        typ={'u8':'uint8_t','u16':'uint16_t'}[kind]
        header+=f'esp_err_t nvs_get_{kind}(nvs_handle_t,const char *,{typ} *);\n'
        header+=f'esp_err_t nvs_set_{kind}(nvs_handle_t,const char *,{typ});\n'
    (temp/'nvs.h').write_text(header)
    (temp/'nvs_flash.h').write_text('#include "esp_err.h"\nesp_err_t nvs_flash_init(void);\nesp_err_t nvs_flash_erase(void);\n')
    code=r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "nvs.h"
#include "config_store.h"
#include "independent_steering.h"
static independent_steering_config_t stored,pending;
static bool available,legacy,blob_present,old_present,commit_fail,pending_blob;
static uint16_t old_blob[24] = {1,2100,1520,1100,900,1480,1900,2,0,800,1000,600};
static uint16_t cal_values[4] = {2050,1518,987,30}, pending_cal[4];
static size_t stored_size;
static unsigned erased;
int nvs_flash_init(void){return ESP_OK;}
int nvs_flash_erase(void){erased++;return ESP_OK;}
int nvs_open(const char *name,int mode,int *handle){
    (void)name;(void)mode;*handle=1;pending_blob=false;pending=stored;
    memcpy(pending_cal,cal_values,sizeof(cal_values));return available?0:4;
}
void nvs_close(int handle){(void)handle;}
int nvs_commit(int handle){
    (void)handle;if(commit_fail)return 5;
    if(pending_blob){stored=pending;stored_size=sizeof(stored);blob_present=true;}
    memcpy(cal_values,pending_cal,sizeof(cal_values));return 0;
}
int nvs_get_u8(int h,const char *key,uint8_t *v){(void)h;if(legacy && !strcmp(key,"rev_limit")){*v=12;return 0;}return 4;}
static int cal_index(const char *key){
    const char *keys[]={"st_left","st_center","st_right","st_dead"};
    for(int i=0;i<4;i++)if(!strcmp(key,keys[i]))return i;
    return -1;
}
int nvs_get_u16(int h,const char *key,uint16_t *v){
    (void)h;if(!legacy)return 4;
    int index=cal_index(key);if(index>=0){*v=cal_values[index];return 0;}
    const char *keys[]={"rx_throttle","rx_neutral","rx_reverse","rx_deadband","arm_run","arm_stop","arm_stop2","rpm_ppr"};
    uint16_t vals[]={978,1514,2044,80,983,2049,1515,7};
    for(unsigned i=0;i<sizeof(vals)/sizeof(vals[0]);i++)if(!strcmp(key,keys[i])){*v=vals[i];return 0;}return 4;
}
int nvs_set_u8(int h,const char *k,uint8_t v){(void)h;(void)k;(void)v;return 0;}
int nvs_set_u16(int h,const char *k,uint16_t v){(void)h;int i=cal_index(k);if(i>=0)pending_cal[i]=v;return 0;}
int nvs_get_blob(int h,const char *k,void *v,size_t *len){
    (void)h;
    if(!strcmp(k,"dual_steer_v1")){
        if(!old_present)return 4;
        if(*len<sizeof(old_blob))return 6;
        memcpy(v,old_blob,sizeof(old_blob));*len=sizeof(old_blob);return 0;
    }
    assert(!strcmp(k,"dual_steer_v2"));if(!blob_present)return 4;
    if(*len<stored_size)return 6;memcpy(v,&stored,stored_size);*len=stored_size;return 0;
}
int nvs_set_blob(int h,const char *k,const void *v,size_t len){
    (void)h;assert(!strcmp(k,"dual_steer_v2"));assert(len==sizeof(pending));memcpy(&pending,v,len);pending_blob=true;return 0;
}
int main(void){
    drive_config_t config;config_store_load_drive(&config);
    assert(!config.loaded_from_nvs && independent_steering_config_valid(&config.steering));
    assert(config.steering.points[0].strength_tenths_percent==0);
    available=true;legacy=true;config_store_load_drive(&config);
    assert(config.reverse_limit_percent==12 && config.rpm_pulses_per_revolution==7);
    throttle_calibration_t throttle;arm_calibration_t arm;steering_calibration_t input;
    config_store_load_throttle(&throttle);config_store_load_arm(&arm);config_store_load_steering(&input);
    assert(throttle.loaded_from_nvs && arm.loaded_from_nvs && input.loaded_from_nvs);
    input.deadband_us=0;input.center_us=1520;
    assert(config_store_save_steering(&input)==0);
    config_store_load_steering(&input);assert(input.deadband_us==0 && input.center_us==1520 && input.loaded_from_nvs);
    input.deadband_us=151;assert(config_store_save_steering(&input)==ESP_ERR_INVALID_ARG);
    // Old PWM endpoints migrate; 80%/60% reductions never become geometric strength.
    old_present=true;config_store_load_drive(&config);
    assert(config.steering.servo[0].center_us==1520 && config.steering.servo[1].left_us==900);
    assert(config.steering.points[0].strength_tenths_percent==0 && config.steering.points[1].strength_tenths_percent==0);
    old_blob[2]=2300;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);old_blob[2]=1520;
    config.steering.points[0].strength_tenths_percent=637;
    config.steering.servo[1]=(steering_servo_config_t){1000,1490,2000,450,350};
    config.steering.geometry=(steering_geometry_t){4500,3150,1100,290,280};
    assert(config_store_save_drive(&config)==0);config_store_load_drive(&config);
    assert(config.steering.points[0].strength_tenths_percent==637 && config.steering.servo[1].center_us==1490);
    assert(config.steering.geometry.wheelbase_tenths_mm==4500 && config.steering.servo[1].left_tenths_deg==450);
    independent_steering_config_t valid=stored;
    stored.point_count=9;config_store_load_drive(&config);assert(config.steering.point_count==2);
    stored=valid;stored.points[1].speed_tenths_kmh=0;config_store_load_drive(&config);assert(config.steering.points[0].strength_tenths_percent==0);
    stored=valid;stored.version=99;config_store_load_drive(&config);assert(config.steering.version==2);
    stored=valid;stored.geometry.track_tenths_mm=0;config_store_load_drive(&config);assert(config.steering.geometry.track_tenths_mm==3200);
    stored=valid;stored.servo[1].left_tenths_deg=280;config_store_load_drive(&config);assert(config.steering.servo[1].center_us==1500);
    stored=valid;stored.servo[1].center_us=2201;config_store_load_drive(&config);assert(config.steering.servo[1].center_us==1500);
    stored=valid;stored.points[1].strength_tenths_percent=65535;config_store_load_drive(&config);assert(config.steering.servo[1].center_us==1500);
    stored=valid;stored_size=sizeof(stored)-1;config_store_load_drive(&config);assert(config.steering.servo[1].center_us==1500);
    // Bad V2 must use defaults, not resurrect V1 settings.
    assert(config.steering.servo[0].center_us==1500);
    stored_size=sizeof(stored);config_store_load_drive(&config);commit_fail=true;
    config.steering.points[0].strength_tenths_percent=100;
    assert(config_store_save_drive(&config)!=0);config_store_load_drive(&config);
    assert(config.steering.points[0].strength_tenths_percent==637);
    input=(steering_calibration_t){.left_us=2100,.center_us=1530,.right_us=1000,.deadband_us=20};
    assert(config_store_save_steering(&input)!=0);config_store_load_steering(&input);assert(input.center_us==1520 && input.deadband_us==0);
    config.steering.point_count=0;assert(config_store_save_drive(&config)==ESP_ERR_INVALID_ARG);
    assert(erased==0);
    puts("NVS passed: v1 endpoint migration/curve reset, v2 round-trip, geometry, input calibration, invalid blobs, failed commits, no erase.");
}
'''
    source=temp/'test.c';source.write_text(code);exe=temp/'test'
    subprocess.run([os.environ.get('CC','cc'),'-std=c11','-Wall','-Wextra','-Werror','-I',tmp,
                    '-I',str(ROOT/'main/include'),str(source),str(ROOT/'main/config_store.c'),
                    str(ROOT/'main/independent_steering.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
