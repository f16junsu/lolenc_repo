#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "utility.h"
#include <cmath>
#include <stdlib.h>
#include "sleep.h"

extern DAC dac_controller_0;
extern DAC dac_controller_1;

// default , min, max
ArgFloat IND3_power(1.0, 0.0, 1.0, 0.1, 4, "AU");
ArgInt step(200, 50, 100000, 1, "ns");
ArgInt num_steps(30, 1, 100, 1, "AU");
ArgInt shots(200, 0, 300, 50, "AU");
ArgInt num_angles(100, 1, 1000, 1, "AU");

double IND3_power_val = IND3_power.get_value();
int64_t step_val = step.get_value();
int64_t num_steps_val = num_steps.get_value();
int64_t shots_val = shots.get_value();
int64_t num_angles_val = num_angles.get_value();

DatasetFloatList result_list("Raman_Rabi after rotation_single", true);
int64_t result_data = 0;

// start control parameters
double base_x = 40;
double base_y = 0.0;
double base_z = 0.0;
double base_sym = 0;
double base_asym = -162.0;
double base_t = 52.0;

int main(){
    reset_module();
    xil_printf("RFSoC Start\r\n");

    DAC32 dac32_controller(&dac_controller_0, &dac_controller_1);
    Waveform wf;

    VoltageSet base_voltage({
        0.0,
        0.0,
        0.0,
        0.0,
        -0.1,
        -0.1,
        -0.2,
        -0.2,//
        -0.2,
        -0.1,
        -0.1,
        0.0,
        0.0,
        0.0,
        0.0,
        -0.1
    });

    init();
    dac32_controller.initialize();
    auto_start();
    sleep(1);

    // Set the individual beam AOM frequency
    // delay(DDS_set_time);
    // IND3_1.set_config(IND3_power_val, (int64_t)(IND_freq * MHz), 0.0, 0);
    // delay(delay_offset);
    wf = create_waveform_from_controls(
                base_voltage,
                base_x,
                base_y,
                base_z,
                base_sym,
                base_asym,
                base_t
    );
    dac32_controller.save_waveform(0, wf);
    VoltageSet start0 = wf.get_dac0(0);
    VoltageSet start1 = wf.get_dac1(0);
    wf = create_waveform_from_controls(
                base_voltage,
                (double) 0,
                base_y,
                base_z,
                base_sym,
                (double) 0,
                base_t
    );
    VoltageSet end0 = wf.get_dac0(0);
    VoltageSet end1 = wf.get_dac1(0);
    wf = Waveform::create_linear_interpolation(start0, start1, end0, end1, num_angles_val);
    dac32_controller.save_waveform(1, wf);

    for(int i=0; i < num_steps_val; i++){
        for(int shot = 0; shot < shots_val; shot++){
            SPAM_Reset_PMT_SINGLE();
            /*
             * Start of Experiment
             */
            // init();

            // Set rotated dc potential

            init();

            dac32_controller.play_waveform(0, 0, false);
            // auto_start();
            // sleep(0.0001);

            // init();
            // Set IND frequency
            delay(DDS_set_time);
            IND3_1.set_config(IND3_power_val, (int64_t)(IND_freq * MHz), 0.0, 0);
            delay(delay_offset);

            // Doppler cooling
            SPAM_Doppler();
            RAMAN_GLB.set(1);  // Pre-turn on
            delay(Doppler_duration * ms);

            dac32_controller.play_waveform(1, 0, false);
            delay(100 * delay_offset); // 1 us delay

            // Initialization
            SPAM_Init();
            delay(Init_duration * ns);
            SPAM_Off();
            delay(AOM_Main_falltime * us);

            // Operation: Rabi flopping
            RAMAN_GLB.set(1); // Global beam on
            IND3_sw.on(0); // Individual beam on
            delay(i * step_val * ns + delay_offset);

            RAMAN_GLB.set(0); // Global beam off
            IND3_sw.off(0); // Individual beam off
            delay(Raman_falltime * us);

            // Detection
            SPAM_Detect_SINGLE();

            // Terminate experiment
            SPAM_Doppler();

            auto_start();

            /*
            * Set TTL signal to 0
            */

            result_data = SPAM_Read_PMT_SINGLE();

            result_list.append_list(3, (double)(result_data), (double)i, (double)shot);
        }
    }
}