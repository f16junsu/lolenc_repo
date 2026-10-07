#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "xparameters.h"
#include "sleep.h"

#define LENGTH 3

extern DAC dac_controller_0;
extern DAC dac_controller_1;
extern LDAC jd_7;
extern LDAC jd_8;


int main(){
    reset_module();
    init();


    /* Setting LDAC inside DAC */
    dac_controller_0.set_ldac(&jd_7);
    dac_controller_1.set_ldac(&jd_8);

    /* Setting waveforms */
    double waveform0_0[LENGTH * 16] = {
        // set1
        0,
        0,
        0,
        0,
        -0.14,
        -0.14,
        -0.16,
        -0.16,
        -0.14,
        -0.14,
        0,
        0,
        0,
        0,
        0,
        0,
        // set2
        0,
        0,
        0,
        0,
        -0.1,
        -0.1,
        -0.16,
        -0.16,
        -0.18,
        -0.18,
        0,
        0,
        0,
        0,
        0,
        0,
        // set3
        0,
        0,
        0,
        0,
        -0.14,
        -0.14,
        -0.16,
        -0.16,
        -0.14,
        -0.14,
        0,
        0,
        0,
        0,
        0,
        0
    };
    double waveform1_0[LENGTH * 16] = {
        // set1
        0,
        0,
        0,
        -0.14,
        -0.14,
        -0.16,
        -0.16,
        -0.14,
        -0.14,
        0,
        0,
        0,
        0,
        0,
        -0.2,
        -0.2,
        // set2
        0,
        0,
        0,
        -0.1,
        -0.1,
        -0.16,
        -0.16,
        -0.18,
        -0.18,
        0,
        0,
        0,
        0,
        0,
        -0.2,
        -0.2,
        // set3
        0,
        0,
        0,
        -0.14,
        -0.14,
        -0.16,
        -0.16,
        -0.14,
        -0.14,
        0,
        0,
        0,
        0,
        0,
        -0.2,
        -0.2
    };
    dac_controller_0.set_waveform(0, waveform0_0, LENGTH);
    dac_controller_1.set_waveform(0, waveform1_0, LENGTH);

    // double waveform0_1[LENGTH * 16] = {
    //     0,
    //     0,
    //     0,
    //     0,
    //     -0.14,
    //     -0.14,
    //     -0.16,
    //     -0.16,
    //     -0.14,
    //     -0.14,
    //     0,
    //     0,
    //     0,
    //     0,
    //     0,
    //     0
    // };

    // double waveform1_1[LENGTH * 16] = {
    //     0,
    //     0,
    //     0,
    //     -0.14,
    //     -0.14,
    //     -0.16
    //     -0.1640,
    //     -0.14,
    //     -0.14,
    //     0,
    //     0,
    //     0,
    //     0,
    //     0,
    //     -0.2,
    //     -0.2
    // };
    // dac_controller_0.set_waveform(1, waveform0_1, LENGTH);
    // dac_controller_1.set_waveform(1, waveform1_1, LENGTH);



    /* Initialize DAC */
    delay_coarse(10);
    dac_controller_0.initialize();
    dac_controller_1.initialize();
    delay_coarse(10);
    dac_controller_0.dac_disable_ldac();
    dac_controller_1.dac_disable_ldac();
    delay_coarse(4000);
    dac_controller_0.dac_set_spi_config(0, 0, 0, 0, 0, 0, 0, 0, 0);
    dac_controller_1.dac_set_spi_config(0, 0, 0, 0, 0, 0, 0, 0, 0);
    delay_coarse(4000);
    dac_controller_0.dac_set_pwr_down(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    dac_controller_1.dac_set_pwr_down(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    delay_coarse(4000);
    dac_controller_0.dac_set_range0(0b1100, 0b1100, 0b1100, 0b1100);
    dac_controller_1.dac_set_range0(0b1100, 0b1100, 0b1100, 0b1100);
    delay_coarse(4000);
    dac_controller_0.dac_set_range1(0b1100, 0b1100, 0b1100, 0b1100);
    dac_controller_1.dac_set_range1(0b1100, 0b1100, 0b1100, 0b1100);
    delay_coarse(4000);
    dac_controller_0.dac_set_range2(0b1100, 0b1100, 0b1100, 0b1100);
    dac_controller_1.dac_set_range2(0b1100, 0b1100, 0b1100, 0b1100);
    delay_coarse(4000);
    dac_controller_0.dac_set_range3(0b1100, 0b1100, 0b1100, 0b1100);
    dac_controller_1.dac_set_range3(0b1100, 0b1100, 0b1100, 0b1100);





    /* Play waveforms */
    dac_controller_0.play_waveform(0, 125000000);
    dac_controller_1.play_waveform(0, 125000000);




    /* Start the experiment */
    auto_start();
    usleep(10000);

}