#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "xparameters.h"
#include "sleep.h"

#define LENGTH 1

extern DAC dac_controller_0;
extern DAC dac_controller_1;

int main(){
    reset_module();
    init();


    /* Make DAC32 instance */
    DAC32 dac32_controller(&dac_controller_0, &dac_controller_1);

    /* Setting waveforms */
    VoltageSet set0[LENGTH];
    VoltageSet set1[LENGTH];
    VoltageSet set00[LENGTH];
    VoltageSet set11[LENGTH];
    set0[0] = VoltageSet({
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    });
    set1[0] = VoltageSet({
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    });
    set00[0] = VoltageSet({
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1
    });
    set11[0] = VoltageSet({
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1,
        0.1
    });
    set0[0] += 0.1;
    set1[0] -= 0.1;
    dac32_controller.save_waveform(0, set0, set1, LENGTH);

    /* Initialize DAC */
    xil_printf("DAC32 Initialization\r\n");
    dac32_controller.initialize();

    /* Play waveforms */
    xil_printf("DAC32 Play waveform\r\n");
    dac32_controller.play_waveform(0);


    /* Start the experiment */
    auto_start();
    usleep(10000);

}