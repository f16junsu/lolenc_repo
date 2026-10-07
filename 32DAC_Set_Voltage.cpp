#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "xparameters.h"
#include "sleep.h"

extern DAC dac_controller_0;
extern DAC dac_controller_1;
extern TTL AOM_MAIN;
extern TTL AOM_SPAM;
extern TTL AOM_DOPDET;

int main(){
    reset_module();
    init();


    /* Make DAC32 instance */
    DAC32 dac32_controller(&dac_controller_0, &dac_controller_1);

    /* Setting waveforms */
    Waveform waveform_trap, waveform_storage, waveform_double, waveform_intermediate, waveform_blow;
    VoltageSet set_trap_0, set_trap_l, set_trap_r, set_storage, set_double, set_intermediate, set00, set01, set02, set03, set04, set05;

    set_trap_0 = VoltageSet({
    0,
    0,
    0,
    0,
    -0.14,
    -0.14,
    -0.16, // center
    -0.16, // center
    -0.14,
    -0.14,
    0,
    0,
    0, 
    0,
    0,
    -0.2
    });
    
    set_trap_l = VoltageSet({
    0,
    0,
    0,
    0,
    0,
    0,
    -10, // center
    -10, // center
    0,
    0,
    0,
    0,
    0, 
    0,
    0,
    -2.1
    });

    set_trap_r = VoltageSet({
    0,
    0,
    0,
    0,
    0,
    0,
    -10, // center
    -10, // center
    0,
    0,
    0,
    0,
    0, 
    0,
    0,
    -3.1
    });

    set_storage = VoltageSet({
    0,
    0,
    0,
    0,
    0,
    0,
    0, // center
    0, // center
    -0.0,
    -0.0,
    -0.0,
    -0.0,
    -0.39, // E13
    -0.75, // E14
    0,
    -0.2
    });

    set_double = VoltageSet({
    0,
    0,
    0,
    0,
    0,
    0,
    -0.6, // center
    -0.6, // center
    0.0,
    0.3,
    0.3,
    0.0,
    -0.6, // E13
    -0.6,
    0,
    -0.2
    });

    VoltageSet set_double_l = VoltageSet({
    0,
    0,
    0,
    0,
    0,
    0,
    -0.6, // center
    -0.6, // center
    0.0,
    0.3,
    0.3,
    0.0,
    -0.6, // E13
    -0.6,
    0,
    -0.22
    });

    VoltageSet set_double_r = VoltageSet({
    0,
    0,
    0,
    0,
    0,
    0,
    -0.6, // center
    -0.6, // center
    0.0,
    0.3,
    0.3,
    0.0,
    -0.6, // E13
    -0.6,
    0,
    -0.22
    });

    VoltageSet set_blow = VoltageSet({
    0,
    0,
    0,
    0,
    0,
    0,
    10, // center
    10, // center
    0.0,
    0.0,
    0.0,
    0.0,
    0.0, // E13
    0.0,
    0,
    10.0
    });

    // waveform_trap.add_set(set_trap_l, set_trap_r);
    waveform_trap.add_set(set_trap_0, set_trap_0);
    waveform_storage.add_set(set_storage, set_storage);
    // waveform_double.add_set(set_double, set_double);
    waveform_double.add_set(set_double_l, set_double_r);
    waveform_intermediate.add_set(set_intermediate, set_intermediate);
    waveform_blow.add_set(set_blow, set_blow);


    Waveform double_to_inter = Waveform::create_linear_interpolation(
        set_double, set_double, set_intermediate, set_intermediate, 100
    );
    Waveform inter_to_storage = Waveform::create_linear_interpolation(
        set_intermediate, set_intermediate, set_storage, set_storage, 100
    );

    Waveform storage_to_double = Waveform::create_linear_interpolation(
        set_storage, set_storage, set_double, set_double, 50
    );

    Waveform inter_to_double = Waveform::create_linear_interpolation(
        set_intermediate, set_intermediate, set_double, set_double, 50
    );

    Waveform double_to_storage = Waveform::create_linear_interpolation(
        set_double, set_double, set_storage, set_storage, 50
    );


    Waveform full_waveform = double_to_inter + inter_to_storage + storage_to_double;

    dac32_controller.save_waveform(0, waveform_trap);
    dac32_controller.save_waveform(1, waveform_double);
    dac32_controller.save_waveform(2, waveform_intermediate);
    dac32_controller.save_waveform(3, waveform_storage);
    // dac32_controller.save_waveform(4, double_to_inter);
    // dac32_controller.save_waveform(5, inter_to_storage);
    // dac32_controller.save_waveform(6, storage_to_double);
    // dac32_controller.save_waveform(7, full_waveform);
    dac32_controller.save_waveform(8, waveform_blow);

    /* Initialize DAC */
    xil_printf("DAC32 Initialization\r\n");
    dac32_controller.initialize();


    /* Play waveforms */
    xil_printf("DAC32 Play waveform\r\n");
    delay_coarse(10);
    AOM_MAIN.on();
    AOM_SPAM.on();
    AOM_DOPDET.on();
    
    dac32_controller.play_waveform(0, 12500000);
    // dac32_controller.play_waveform(1, 12500000);
    //dac32_controller.play_waveform(2, 12500000);
    // dac32_controller.play_waveform(3, 12500000);
    // dac32_controller.play_waveform(4, 12500000);
    //dac32_controller.play_waveform(5, 12500000);
    // dac32_controller.play_waveform(8, 12500000);

    /* Start the experiment */
    auto_start();
    usleep(1000);

}