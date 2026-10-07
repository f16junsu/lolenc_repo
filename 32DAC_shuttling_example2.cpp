#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "xparameters.h"
#include "sleep.h"

#define LENGTH 11

extern DAC dac_controller_0;
extern DAC dac_controller_1;

int main(){
    reset_module();
    init();


    /* Make DAC32 instance */
    DAC32 dac32_controller(&dac_controller_0, &dac_controller_1);

    /* Setting waveforms */
    VoltageSet z_plus({0, 0, 0, 0, -0.01, -0.01, 0, 0, 0.01, 0.01, 0, 0, 0, 0, 0, 0});
    VoltageSet set0[LENGTH];
    VoltageSet set1[LENGTH];
    set0[0] = VoltageSet({
        0,
        0,
        0,
        0,
        -0.137,
        -0.137,
        -0.16,
        -0.16,
        -0.143,
        -0.143,
        0,
        0,
        0,
        0,
        0,
        -0.198
    });
    set1[0] = VoltageSet({
        0,
        0,
        0,
        0,
        -0.137,
        -0.137,
        -0.16,
        -0.16,
        -0.143,
        -0.143,
        0,
        0,
        0,
        0,
        0,
        -0.234
    });
    for (int i = 1; i < 6; i++){
        double j = i;
        set0[i] = set0[i-1] + z_plus;
        set1[i] = set1[i-1] + z_plus;
    }
    for (int i = 6; i < 11; i++){
        set0[i] = set0[i-1] - z_plus;
        set0[i] = set0[i-1] - z_plus;
    }
    dac32_controller.save_waveform(0, set0, set1, LENGTH);



    /* Initialize DAC */
    xil_printf("DAC32 Initialization\r\n");
    dac32_controller.initialize();


    /* Play waveforms */
    xil_printf("DAC32 Play waveform\r\n");
    dac32_controller.play_waveform(0, 125000000);


    /* Start the experiment */
    auto_start();
    usleep(10000);

}