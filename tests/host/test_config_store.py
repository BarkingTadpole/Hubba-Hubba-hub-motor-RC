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
static bool available,legacy,blob_present,v5_present,v4_present,v3_present,v2_present,old_present,commit_fail,pending_blob;
static uint16_t v5_blob[38];
static size_t v5_size = sizeof(v5_blob);
static uint16_t v4_blob[37];
static size_t v4_size = sizeof(v4_blob);
static int stored_mode = -1, stored_tank = -1, pending_mode, pending_tank;
static uint16_t v3_blob[35];
static size_t v3_size = sizeof(v3_blob);
static uint16_t v2_blob[33] = {2,2100,1520,1100,400,350,900,1480,1900,350,400,
    4500,3100,1080,300,280,3,0,800,100,600,1000,200};
static size_t v2_size = sizeof(v2_blob);
static uint16_t old_blob[24] = {1,2100,1520,1100,900,1480,1900,2,0,800,1000,600};
static uint16_t cal_values[4] = {2050,1518,987,30}, pending_cal[4];
static size_t stored_size;
static unsigned erased,blob_writes;
int nvs_flash_init(void){return ESP_OK;}
int nvs_flash_erase(void){erased++;return ESP_OK;}
int nvs_open(const char *name,int mode,int *handle){
    (void)name;(void)mode;*handle=1;pending_blob=false;pending=stored;
    pending_mode=stored_mode;pending_tank=stored_tank;
    memcpy(pending_cal,cal_values,sizeof(cal_values));return available?0:4;
}
void nvs_close(int handle){(void)handle;}
int nvs_commit(int handle){
    (void)handle;if(commit_fail)return 5;
    if(pending_blob){stored=pending;stored_size=sizeof(stored);blob_present=true;}
    stored_mode=pending_mode;stored_tank=pending_tank;
    memcpy(cal_values,pending_cal,sizeof(cal_values));return 0;
}
int nvs_get_u8(int h,const char *key,uint8_t *v){
    (void)h;
    if(!strcmp(key,"drive_mode") && stored_mode>=0){*v=stored_mode;return 0;}
    if(!strcmp(key,"tank_limit") && stored_tank>=0){*v=stored_tank;return 0;}
    if(legacy && !strcmp(key,"rev_limit")){*v=12;return 0;}return 4;
}
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
int nvs_set_u8(int h,const char *k,uint8_t v){
    (void)h;
    if(!strcmp(k,"drive_mode"))pending_mode=v;
    if(!strcmp(k,"tank_limit"))pending_tank=v;
    return 0;
}
int nvs_set_u16(int h,const char *k,uint16_t v){(void)h;int i=cal_index(k);if(i>=0)pending_cal[i]=v;return 0;}
int nvs_get_blob(int h,const char *k,void *v,size_t *len){
    (void)h;
    if(!strcmp(k,"dual_steer_v1")){
        if(!old_present)return 4;
        if(*len<sizeof(old_blob))return 6;
        memcpy(v,old_blob,sizeof(old_blob));*len=sizeof(old_blob);return 0;
    }
    if(!strcmp(k,"dual_steer_v2")){
        if(!v2_present)return 4;
        if(*len<v2_size)return 6;
        memcpy(v,v2_blob,v2_size);*len=v2_size;return 0;
    }
    if(!strcmp(k,"dual_steer_v3")){
        if(!v3_present)return 4;
        if(*len<v3_size)return 6;
        memcpy(v,v3_blob,v3_size);*len=v3_size;return 0;
    }
    if(!strcmp(k,"dual_steer_v4")){
        if(!v4_present)return 4;
        if(*len<v4_size)return 6;
        memcpy(v,v4_blob,v4_size);*len=v4_size;return 0;
    }
    if(!strcmp(k,"dual_steer_v5")){
        if(!v5_present)return 4;
        if(*len<v5_size)return 6;
        memcpy(v,v5_blob,v5_size);*len=v5_size;return 0;
    }
    assert(!strcmp(k,"dual_steer_v6"));if(!blob_present)return 4;
    if(*len<stored_size)return 6;memcpy(v,&stored,stored_size);*len=stored_size;return 0;
}
int nvs_set_blob(int h,const char *k,const void *v,size_t len){
    (void)h;assert(!strcmp(k,"dual_steer_v6"));assert(len==152 && len==sizeof(pending));memcpy(&pending,v,len);pending_blob=true;blob_writes++;return 0;
}
int main(void){
    drive_config_t config;config_store_load_drive(&config);
    assert(!config.loaded_from_nvs && independent_steering_config_valid(&config.steering));
    assert(config.tank_limit_percent==25 && config.drivetrain_mode==DRIVETRAIN_AWD && config.steering.toe_tenths_deg==0);
    assert(config.steering.points[0].strength_tenths_percent==0);
    assert(config.steering.linkage.inside_tenths_deg==900 && config.steering.linkage.outside_tenths_deg==680);
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
    assert(config.steering.trim_us[0]==0 && config.steering.trim_us[1]==0 && !blob_present);
    assert(config.steering.linkage.inside_tenths_deg==900 && config.steering.linkage.outside_tenths_deg==680);
    old_blob[2]=2300;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);old_blob[2]=1520;
    // V2 retains every calibration, geometry and curve field; both trims start zero.
    v2_present=true;config_store_load_drive(&config);
    assert(config.steering.version==6 && config.steering.trim_us[0]==0 && config.steering.trim_us[1]==0);
    assert(config.steering.linkage.inside_tenths_deg==900 && config.steering.linkage.outside_tenths_deg==680);
    assert(memcmp((char *)&config.steering+2,v2_blob+1,sizeof(v2_blob)-2)==0);
    assert(!blob_present && blob_writes==0); // Loading never saves migration.
    v2_size--;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v2_size++;
    v2_size++;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v2_size--;
    v2_blob[0]=99;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v2_blob[0]=2;
    v2_blob[12]=0;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v2_blob[12]=3100;
    v2_blob[16]=9;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v2_blob[16]=3;
    v2_blob[18]=1001;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v2_blob[18]=800;
    assert(blob_writes==0); // Corrupt v2 does not restore stale v1 endpoints either.
    // V3 retains every field including signed trims and adds only default limits.
    memcpy(v3_blob,v2_blob,sizeof(v2_blob));v3_blob[0]=3;v3_blob[33]=80;v3_blob[34]=(uint16_t)-90;
    v3_present=true;config_store_load_drive(&config);
    assert(config.steering.version==6 && config.steering.trim_us[0]==80 && config.steering.trim_us[1]==-90);
    assert(memcmp((char *)&config.steering+2,v3_blob+1,sizeof(v3_blob)-2)==0);
    assert(config.steering.linkage.inside_tenths_deg==900 && config.steering.linkage.outside_tenths_deg==680);
    assert(!blob_present && blob_writes==0);
    v3_size--;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v3_size++;
    v3_size++;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v3_size--;
    v3_blob[0]=99;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v3_blob[0]=3;
    v3_blob[33]=INT16_MAX;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v3_blob[33]=80;
    assert(blob_writes==0); // Invalid v3 cannot resurrect valid v2/v1 settings.
    // V4 keeps every existing byte except version; toe defaults to zero.
    memcpy(v4_blob,v3_blob,sizeof(v3_blob));v4_blob[0]=4;v4_blob[35]=855;v4_blob[36]=642;
    v4_present=true;config_store_load_drive(&config);
    assert(config.steering.version==6 && config.steering.toe_tenths_deg==0);
    assert(memcmp((char *)&config.steering+2,v4_blob+1,sizeof(v4_blob)-2)==0);
    assert(!blob_present && blob_writes==0);
    v4_size--;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v4_size++;
    v4_size++;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v4_size--;
    v4_blob[0]=99;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v4_blob[0]=4;
    v4_blob[35]=901;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v4_blob[35]=855;
    assert(blob_writes==0);
    // V5 retains toe and all prior values, but has no measured wheel points.
    memcpy(v5_blob,v4_blob,sizeof(v4_blob));v5_blob[0]=5;v5_blob[37]=15;
    v5_present=true;config_store_load_drive(&config);
    assert(config.steering.version==6 && config.steering.toe_tenths_deg==15);
    assert(config.steering.wheel_calibration[0].count==0 && config.steering.wheel_calibration[1].count==0);
    assert(memcmp((char *)&config.steering+2,v5_blob+1,sizeof(v5_blob)-2)==0 && blob_writes==0);
    v5_size--;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v5_size++;
    v5_size++;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v5_size--;
    v5_blob[0]=99;config_store_load_drive(&config);assert(config.steering.servo[0].center_us==1500);v5_blob[0]=5;
    config_store_load_drive(&config);
    config.drivetrain_mode=DRIVETRAIN_TANK;config.tank_limit_percent=37;
    config.steering.toe_tenths_deg=-300;
    config.steering.points[0].strength_tenths_percent=637;
    config.steering.servo[1]=(steering_servo_config_t){1000,1490,2000,450,350};
    config.steering.geometry=(steering_geometry_t){4500,3150,1100,290,280};
    config.steering.trim_us[0]=120;config.steering.trim_us[1]=-60;
    config.steering.linkage=(steering_linkage_limits_t){855,642};
    assert(config_store_save_drive(&config)==0);config_store_load_drive(&config);
    assert(config.drivetrain_mode==DRIVETRAIN_TANK && config.tank_limit_percent==37 && config.steering.toe_tenths_deg==-300);
    stored_mode=99;stored_tank=101;config_store_load_drive(&config);
    assert(config.drivetrain_mode==DRIVETRAIN_AWD && config.tank_limit_percent==25);
    stored_mode=DRIVETRAIN_TANK;stored_tank=37;config_store_load_drive(&config);
    assert(config.steering.trim_us[0]==120 && config.steering.trim_us[1]==-60);
    assert(config.steering.points[0].strength_tenths_percent==637 && config.steering.servo[1].center_us==1490);
    assert(config.steering.geometry.wheelbase_tenths_mm==4500 && config.steering.servo[1].left_tenths_deg==450);
    assert(config.steering.linkage.inside_tenths_deg==855 && config.steering.linkage.outside_tenths_deg==642);
    independent_steering_config_t valid=stored;
    stored.point_count=9;config_store_load_drive(&config);assert(config.steering.point_count==2);
    stored=valid;stored.points[1].speed_tenths_kmh=0;config_store_load_drive(&config);assert(config.steering.points[0].strength_tenths_percent==0);
    stored=valid;stored.version=99;config_store_load_drive(&config);assert(config.steering.version==6);
    stored=valid;stored.toe_tenths_deg=350;config_store_load_drive(&config);assert(config.steering.toe_tenths_deg==350);
    stored=valid;stored.toe_tenths_deg=-350;config_store_load_drive(&config);assert(config.steering.toe_tenths_deg==-350);
    stored=valid;stored.toe_tenths_deg=351;config_store_load_drive(&config);assert(config.steering.toe_tenths_deg==0);
    stored=valid;stored.toe_tenths_deg=-351;config_store_load_drive(&config);assert(config.steering.toe_tenths_deg==0);
    stored=valid;stored.linkage.inside_tenths_deg=0;config_store_load_drive(&config);assert(config.steering.toe_tenths_deg==0);
    stored=valid;stored.linkage.inside_tenths_deg=901;config_store_load_drive(&config);assert(config.steering.linkage.inside_tenths_deg==900);
    stored=valid;stored.linkage.outside_tenths_deg=UINT16_MAX;config_store_load_drive(&config);assert(config.steering.linkage.outside_tenths_deg==680);
    stored=valid;stored.trim_us[0]=INT16_MAX;config_store_load_drive(&config);assert(config.steering.trim_us[0]==0 && config.steering.servo[0].center_us==1500);
    stored=valid;stored.trim_us[1]=INT16_MIN;config_store_load_drive(&config);assert(config.steering.trim_us[1]==0 && config.steering.servo[1].center_us==1500);
    stored=valid;stored.geometry.track_tenths_mm=0;config_store_load_drive(&config);assert(config.steering.geometry.track_tenths_mm==3200);
    stored=valid;stored.servo[1].left_tenths_deg=280;config_store_load_drive(&config);assert(config.steering.servo[1].center_us==1500);
    stored=valid;stored.servo[1].center_us=2501;config_store_load_drive(&config);assert(config.steering.servo[1].center_us==1500);
    stored=valid;stored.points[1].strength_tenths_percent=65535;config_store_load_drive(&config);assert(config.steering.servo[1].center_us==1500);
    stored=valid;stored_size=sizeof(stored)-1;config_store_load_drive(&config);assert(config.steering.servo[1].center_us==1500);
    // Bad V6 must use defaults, not resurrect V5, V4, V3, V2 or V1 settings.
    assert(config.steering.servo[0].center_us==1500);
    stored_size=sizeof(stored);config_store_load_drive(&config);commit_fail=true;
    config.steering.points[0].strength_tenths_percent=100;
    config.steering.trim_us[0]=25;config.steering.trim_us[1]=-35;
    config.steering.linkage=(steering_linkage_limits_t){0,900};
    config.steering.toe_tenths_deg=0;config.tank_limit_percent=60;config.drivetrain_mode=DRIVETRAIN_FWD;
    assert(config_store_save_drive(&config)!=0);config_store_load_drive(&config);
    assert(config.steering.points[0].strength_tenths_percent==637);
    assert(config.steering.trim_us[0]==120 && config.steering.trim_us[1]==-60);
    assert(config.steering.linkage.inside_tenths_deg==855 && config.steering.linkage.outside_tenths_deg==642);
    assert(config.steering.toe_tenths_deg==-300 && config.tank_limit_percent==37 && config.drivetrain_mode==DRIVETRAIN_TANK);
    input=(steering_calibration_t){.left_us=2100,.center_us=1530,.right_us=1000,.deadband_us=20};
    assert(config_store_save_steering(&input)!=0);config_store_load_steering(&input);assert(input.center_us==1520 && input.deadband_us==0);
    config.steering.point_count=0;assert(config_store_save_drive(&config)==ESP_ERR_INVALID_ARG);
    config.steering=valid;config.steering.trim_us[0]=531;
    assert(config_store_save_drive(&config)==ESP_ERR_INVALID_ARG);
    config.steering=valid;config.steering.trim_us[1]=-441;
    assert(config_store_save_drive(&config)==ESP_ERR_INVALID_ARG);
    config.steering=valid;config.steering.linkage.inside_tenths_deg=901;
    assert(config_store_save_drive(&config)==ESP_ERR_INVALID_ARG);
    config.steering=valid;config.steering.linkage.outside_tenths_deg=901;
    assert(config_store_save_drive(&config)==ESP_ERR_INVALID_ARG);
    // Expanded servo range and large legal trims survive persistence unchanged.
    commit_fail=false;config.steering=valid;
    config.steering.toe_tenths_deg=0;
    config.steering.servo[0]=(steering_servo_config_t){2500,550,500,300,300};
    config.steering.servo[1]=(steering_servo_config_t){2500,2450,500,300,300};
    config.steering.trim_us[0]=1900;config.steering.trim_us[1]=-1900;
    config.steering.linkage=(steering_linkage_limits_t){0,900};
    assert(config_store_save_drive(&config)==ESP_OK);config_store_load_drive(&config);
    assert(config.steering.servo[0].left_us==2500 && config.steering.servo[0].right_us==500);
    assert(config.steering.trim_us[0]==1900 && config.steering.trim_us[1]==-1900);
    assert(config.steering.linkage.inside_tenths_deg==0 && config.steering.linkage.outside_tenths_deg==900);
    stored.servo[0].right_us=499;config_store_load_drive(&config);assert(config.steering.servo[0].right_us==1000);
    stored.servo[0].right_us=500;stored.servo[1].left_us=2501;
    config_store_load_drive(&config);assert(config.steering.servo[1].left_us==2000);
    assert(erased==0);
    config.steering=valid;config.tank_limit_percent=101;
    assert(config_store_save_drive(&config)==ESP_ERR_INVALID_ARG);
    config.tank_limit_percent=25;config.drivetrain_mode=99;
    assert(config_store_save_drive(&config)==ESP_ERR_INVALID_ARG);
    config.drivetrain_mode=DRIVETRAIN_AWD;
    independent_steering_defaults(&config.steering);
    config.steering.wheel_calibration[0]=(wheel_angle_calibration_t){5,{{2000,-300},{1750,-100},{1500,0},{1200,80},{1000,300}}};
    config.steering.wheel_calibration[1]=(wheel_angle_calibration_t){3,{{2000,-300},{1500,0},{1000,300}}};
    assert(config_store_save_drive(&config)==ESP_OK);config_store_load_drive(&config);
    assert(config.steering.wheel_calibration[0].count==5 && config.steering.wheel_calibration[1].count==3);
    assert(config.steering.wheel_calibration[0].points[3].angle_tenths_deg==80);
    valid=stored;
    stored.wheel_calibration[0].count=10;config_store_load_drive(&config);
    assert(config.steering.wheel_calibration[0].count==0);
    stored=valid;stored.wheel_calibration[0].points[2].pulse_us=1501;config_store_load_drive(&config);
    assert(config.steering.wheel_calibration[0].count==0);
    stored=valid;stored.wheel_calibration[0].points[1].pulse_us=1000;config_store_load_drive(&config);
    assert(config.steering.wheel_calibration[0].count==0);
    stored=valid;config_store_load_drive(&config);commit_fail=true;
    config.steering.wheel_calibration[0].count=0;
    assert(config_store_save_drive(&config)!=ESP_OK);config_store_load_drive(&config);
    assert(config.steering.wheel_calibration[0].count==5 && erased==0);
    puts("NVS passed: read-only v1-v5 migration, v6 measured wheel curves/toe/trim/linkage, tank mode/limit, invalid blobs, failed commits, no erase.");
}
'''
    source=temp/'test.c';source.write_text(code);exe=temp/'test'
    subprocess.run([os.environ.get('CC','cc'),'-std=c11','-Wall','-Wextra','-Werror','-I',tmp,
                    '-I',str(ROOT/'main/include'),str(source),str(ROOT/'main/config_store.c'),
                    str(ROOT/'main/independent_steering.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
