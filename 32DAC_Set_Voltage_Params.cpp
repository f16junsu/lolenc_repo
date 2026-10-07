#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "xparameters.h"
#include "sleep.h"
#include "utility.h"

#define LENGTH 1

extern DAC dac_controller_0;
extern DAC dac_controller_1;

ArgInt x(0, -1000, 1000, 1,"AU");
ArgInt y(0, -1000, 1000, 1,"AU");
ArgInt z(0, -1000, 1000, 1,"AU");
ArgInt sym(0, -1000, 1000, 1,"AU");
ArgInt asym(0, -1000, 1000, 1,"AU");
ArgInt t(0, -1000, 1000, 1,"AU");

int64_t x_val = x.get_value();
int64_t y_val = y.get_value();
int64_t z_val = z.get_value();
int64_t sym_val = sym.get_value();
int64_t asym_val = asym.get_value();
int64_t t_val = t.get_value();

int main(){
    reset_module();
    init();


    /* Make DAC32 instance */
    DAC32 dac32_controller(&dac_controller_0, &dac_controller_1);

    // VoltageSet base_voltage({
    //     -0.08,
    //     -0.06,
    //     -0.05,
    //     -0.03,
    //     -0.05,
    //     -0.05,
    //     -0.036,
    //     -0.036,
    //     -0.05,
    //     -0.05,
    //     -0.03,
    //     -0.05,
    //     -0.06,
    //     -0.08,
    //     0,
    //     -0.2
    // });

    // VoltageSet base_voltage({
    //     0.0,
    //     0.0,
    //     0.0,
    //     0.0,
    //     -0.14,
    //     -0.14,
    //     -0.16,
    //     -0.16,
    //     -0.14,
    //     -0.14,
    //     0.0,
    //     0.0,
    //     0.0,
    //     0.0,
    //     0,
    //     -0.2
    // });

    VoltageSet base_voltage({
        0.0,
        0.0,
        -0.00,
        -0.00,
        -0.02,
        -0.02,
        -0.03,
        -0.03,
        -0.02,
        -0.02,
        -0.00,
        -0.00,
        0.0,
        0.0,
        0,
        -0.2
    });

    Waveform wf = create_waveform_from_controls(
        base_voltage,
        x_val,
        y_val,
        z_val,
        sym_val,
        asym_val,
        t_val
    );
    dac32_controller.save_waveform(0, wf);


    /* Initialize DAC */
    xil_printf("DAC32 Initialization\r\n");
    dac32_controller.initialize();


    /* Play waveforms */
    xil_printf("DAC32 Play waveform\r\n");
    dac32_controller.play_waveform(0, 12500000);


    /* Start the experiment */
    auto_start();
    usleep(10000);

}