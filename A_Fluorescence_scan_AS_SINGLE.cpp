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
ArgInt scan_dim_A(0, 0, 500, 1,"AU");
ArgInt scan_dim_S(10, 0, 500, 1,"AU");
ArgFloat scale_A(0.0, 0.0, 100.0, 0.1, 2, "AU");
ArgFloat scale_S(0.0, 0.0, 100.0, 0.1, 2, "AU");

int64_t scan_dim_A_val = 0; // = scan_dim_A.get_value();
int64_t scan_dim_S_val = 5; // = scan_dim_S.get_value();
double scale_A_val = 0.0; //  scale_A.get_value();
double scale_S_val = 0.0; // scale_S.get_value();

double base_x = -120.0;
double base_y = 0.0;
double base_z = 0.0;
double base_sym = 0.0;
double base_asym = 122.0;
double base_t = 160.0;

ArgInt shots(100, 0, 300, 50, "AU");

int64_t shots_val = 5; //  shots.get_value();

DatasetFloatList result_list("Fluorescence_scan_single", true);
int64_t result_data = 0;

int main(){
    reset_module();
    xil_printf("RFSoC Start\r\n");

    DAC32 dac32_controller(&dac_controller_0, &dac_controller_1);
    Waveform wf;

    VoltageSet base_voltage({
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
        -0.2
    });

    init();
    dac32_controller.initialize();
    auto_start();
    sleep(1);

    for(int j=-(scan_dim_S_val); j < (scan_dim_S_val) + 1; j++){
        for(int i=-(scan_dim_A_val); i < (scan_dim_A_val) + 1; i++){
            wf = create_waveform_from_controls(
                base_voltage,
                base_x,
                base_y,
                base_z,
                (double)(j * scale_S_val + base_sym),
                (double)(i * scale_A_val + base_asym),
                base_t
            );
            // init();
            dac32_controller.save_waveform(0, wf);
            // dac32_controller.play_waveform(0, 4000);
            // auto_start();
            // sleep(0.01);

            for(int shot = 0; shot < shots_val; shot++){
                SPAM_Reset_PMT_SINGLE();
                /*
                * Start of Experiment
                */
                // Initialize experiment
                init();
                dac32_controller.play_waveform(0, 12500);

                // Doppler cooling
                SPAM_Doppler();
                delay(Doppler_duration * ms);

                // Detection
                SPAM_Detect_SINGLE();

                // Doppler cooling
                SPAM_Doppler();

                auto_start();

                result_data = SPAM_Read_PMT_SINGLE();
                result_list.append_list(4, (double)result_data, (double)i, (double)j, (double)shot);
            }
        }
    }

    init();
    wf = create_waveform_from_controls(base_voltage, base_x, base_y, base_z, base_sym, base_asym, base_t);
    dac32_controller.save_waveform(0, wf);
    dac32_controller.play_waveform(0, 12500000);
    auto_start();
}