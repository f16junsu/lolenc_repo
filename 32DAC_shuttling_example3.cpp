#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "xparameters.h"
#include "sleep.h"

#define LENGTH 17

extern DAC dac_controller_0;
extern DAC dac_controller_1;

int main(){
    reset_module();
    init();


    /* Make DAC32 instance */
    DAC32 dac32_controller(&dac_controller_0, &dac_controller_1);

    /* Setting waveforms */
    VoltageSet z_plus({0, 0, 0, 0, -0.01, -0.01, 0, 0, 0.01, 0.01, 0, 0, 0, 0, 0, 0});
    VoltageSet well0({
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
    VoltageSet well1({
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
    VoltageSet set0[LENGTH];
    VoltageSet set1[LENGTH];
    for (int i = 0; i < LENGTH; i++){
        set0[i] = well0;
        set1[i] = well1;
    }
    for (int i = 0; i < 16; i++){
        set0[i][i] = set0[i][i] - 0.05;
        // set1[16 + i][i] = set1[16 + i][i] - 0.05;
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