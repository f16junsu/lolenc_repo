#include "core.h"
#include "module.h"
#include <cmath>
#include <stdlib.h>

extern TTL jd_5;
extern TTL jd_6;
extern TTL jb_0;
extern TTL jb_2;
extern TTL jb_3;
extern TTL jb_1;
extern TTL jb_7;
extern TTL jb_6;
extern TTL jb_8;
extern TTL jb_5;
extern TTL jc_0;
extern TTL jc_3;
extern TTL jc_4;
extern TTL jc_7;

extern TTL jc_1;
extern TTL jd_3;
extern TTL jd_4;
extern TTL jc_6;
extern TTL jc_5;
extern TTL jd_1;
extern TTL jd_2;
extern TTL jd_7;
extern TTL jd_8;

extern InputController ja_1;
extern InputController ja_2;
extern InputController ja_3;
extern InputController ja_4;
extern InputController ja_5;
extern InputController ja_6;
extern InputController ja_7;
extern InputController ja_8;
extern InputController ja_9;

extern SwitchController dds_sw_0;
extern SwitchController dds_sw_1;
extern SwitchController dds_sw_2;
extern SwitchController dds_sw_3;
extern SwitchController dds_sw_4;
extern SwitchController dds_sw_5;
extern SwitchController dds_sw_6;
extern SwitchController dds_sw_7;

extern DDS dds_07;

int main(){
    ArgInt edge_num(0,0,100,1,"HI");
    int64_t edge_num_val = edge_num.get_value();

    reset_module();
    xil_printf("hello world\r\n");

    delay(8000);
    jb_0.on();
    dds_07.set_config(1.0, 10000000, 0, 0);

    for (int i = 0; i < 100; i++){
        dds_sw_7.on(0);
        delay_coarse(100);
        dds_sw_7.off(0);
        delay_coarse(100);
    }

    auto_start();
}