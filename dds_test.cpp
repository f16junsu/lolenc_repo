#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "xparameters.h"
#include "sleep.h"
#include <time.h>
#include <cmath>

extern DDS IND1_1;
extern DDS IND1_2;
extern DDS IND2_1;
extern DDS IND2_2;
extern DDS IND3_1;
extern DDS IND3_2;
extern DDS IND4_1;
extern DDS IND4_2;
extern DDS IND5_1;
extern DDS IND5_2;

extern SwitchController IND1_sw;
extern SwitchController IND2_sw;
extern SwitchController IND3_sw;
extern SwitchController IND4_sw;
extern SwitchController IND5_sw;


int main(void){
    reset_module();
    init();

    delay(1000);
    IND1_sw.set_dds();
    IND2_sw.set_dds();
    IND3_sw.set_dds();
    IND4_sw.set_dds();
    IND5_sw.set_dds();

    IND1_1.set_config(0.5, 200 * MHz, 0.0, 0);
    IND1_2.set_config(0.5, 200 * MHz, 0.0, 0);
    IND2_1.set_config(0.5, 200 * MHz, 0.0, 0);
    IND2_2.set_config(0.5, 200 * MHz, 0.0, 0);
    IND3_1.set_config(0.5, 200 * MHz, 0.0, 0);
    IND3_2.set_config(0.5, 200 * MHz, 0.0, 0);
    IND4_1.set_config(0.5, 200 * MHz, 0.0, 0);
    IND4_2.set_config(0.5, 200 * MHz, 0.0, 0);
    IND5_1.set_config(0.5, 200 * MHz, 0.0, 0);
    IND5_2.set_config(0.5, 200 * MHz, 0.0, 0);
}