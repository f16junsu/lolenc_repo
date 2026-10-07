#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "utility.h"
#include <cmath>
#include <stdlib.h>
#include "sleep.h"

extern SwitchController IND1_sw;
extern SwitchController IND2_sw;
extern SwitchController IND3_sw;
extern SwitchController IND4_sw;
extern SwitchController IND5_sw;

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

extern TTL jb_0;

ArgInt ION1_IND1_ON(0, 0, 1, 1, "AU");
ArgInt ION2_IND2_ON(0, 0, 1, 1, "AU");
ArgInt ION3_IND3_ON(0, 0, 1, 1, "AU");
ArgInt ION4_IND4_ON(0, 0, 1, 1, "AU");
ArgInt ION5_IND5_ON(0, 0, 1, 1, "AU");

ArgInt phase(0, 0, 24, 1, "AU");
ArgFloat IND1_1_power(0.5, 0.0, 1.0, 0.1, 4, "AU");
ArgFloat IND1_2_power(0.5, 0.0, 1.0, 0.1, 4, "AU");
ArgFloat IND2_1_power(0.5, 0.0, 1.0, 0.1, 4, "AU");
ArgFloat IND2_2_power(0.5, 0.0, 1.0, 0.1, 4, "AU");
ArgFloat IND3_1_power(0.5, 0.0, 1.0, 0.1, 4, "AU");
ArgFloat IND3_2_power(0.5, 0.0, 1.0, 0.1, 4, "AU");
ArgFloat IND4_1_power(0.5, 0.0, 1.0, 0.1, 4, "AU");
ArgFloat IND4_2_power(0.5, 0.0, 1.0, 0.1, 4, "AU");
ArgFloat IND5_1_power(0.5, 0.0, 1.0, 0.1, 4, "AU");
ArgFloat IND5_2_power(0.5, 0.0, 1.0, 0.1, 4, "AU");

int64_t IND1_ON_val = ION1_IND1_ON.get_value();
int64_t IND2_ON_val = ION2_IND2_ON.get_value();
int64_t IND3_ON_val = ION3_IND3_ON.get_value();
int64_t IND4_ON_val = ION4_IND4_ON.get_value();
int64_t IND5_ON_val = ION5_IND5_ON.get_value();

int64_t phase_val = phase.get_value();
double IND1_1_power_val = IND1_1_power.get_value();
double IND2_1_power_val = IND2_1_power.get_value();
double IND3_1_power_val = IND3_1_power.get_value();
double IND4_1_power_val = IND4_1_power.get_value();
double IND5_1_power_val = IND5_1_power.get_value();
double IND1_2_power_val = IND1_2_power.get_value();
double IND2_2_power_val = IND2_2_power.get_value();
double IND3_2_power_val = IND3_2_power.get_value();
double IND4_2_power_val = IND4_2_power.get_value();
double IND5_2_power_val = IND5_2_power.get_value();

int64_t IND_freq = 50;

int main(){
    // reset_module();

    init();
    delay(1000);

    IND1_sw.set_dds();
    IND2_sw.set_dds();
    IND3_sw.set_dds();
    IND4_sw.set_dds();
    IND5_sw.set_dds();


    delay(1000);
    jb_0.on();
    IND1_1.set_config(IND1_1_power_val, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND1_2.set_config(IND1_2_power_val, (int64_t)(IND_freq * MHz), 0.0, 0);

    IND2_1.set_config(IND2_1_power_val, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND2_2.set_config(IND2_2_power_val, (int64_t)(IND_freq * MHz), 0.0, 0);

    IND3_1.set_config(IND3_1_power_val, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND3_2.set_config(IND3_2_power_val, (int64_t)(IND_freq * MHz), 0.0, 0);

    IND4_1.set_config(IND4_1_power_val, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND4_2.set_config(IND4_2_power_val, (int64_t)(IND_freq * MHz), 0.0, 0);

    IND5_1.set_config(IND5_1_power_val, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND5_2.set_config(IND5_2_power_val, (int64_t)(IND_freq * MHz), 0.0, 0);

    delay(1000);
    IND1_1.reset_phase();
    IND1_2.reset_phase();
    IND2_1.reset_phase();
    IND2_2.reset_phase();
    IND3_1.reset_phase();
    IND3_2.reset_phase();
    IND4_1.reset_phase();
    IND4_2.reset_phase();
    IND5_1.reset_phase();
    IND5_2.reset_phase();
    delay(1000);


    if (IND1_ON_val == 1) {
        IND1_sw.on(0);

        // delay_coarse(50);
        // IND1_1.set_config(IND1_1_power_val, (int64_t)(IND_freq / 3 * MHz), 0.0, 0);
        // delay_coarse(50);
        // IND1_1.set_config(IND1_1_power_val, (int64_t)(IND_freq * MHz), 0.0, 0);

        // IND1_1.initialize_phase();
        // delay_coarse(50);
        // IND1_1.set_config(IND1_1_power_val, (int64_t)(IND_freq / 3 * MHz), 0.0, 0);
        // delay_coarse(50);
        // IND1_1.sync_set_freq((int64_t)(IND_freq * MHz));
    }
    else {
        IND1_sw.off(0); ////
    }
    if (IND2_ON_val == 1) {
        IND2_sw.on(0);
    }
    else {
        IND2_sw.off(0);
    }
    if (IND3_ON_val == 1) {
        IND3_sw.on(0);
    }
    else {
        IND3_sw.off(0);
    }
    if (IND4_ON_val == 1) {
        delay_coarse(-100);
        IND4_sw.on(0);
    }
    else {
        IND4_sw.off(0);
    }
    if (IND5_ON_val == 1) {
        IND5_sw.on(0);
    }
    else {
        IND5_sw.off(0);
    }

    delay_coarse(1000);
    jb_0.off();
    auto_start();

    xil_printf("IND dds set\r\n");
}