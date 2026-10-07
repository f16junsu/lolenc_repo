#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "utility.h"
#include <cmath>
#include <stdlib.h>

// default , min, max
ArgInt ION1_IND1_ON(0, 0, 1, 1, "AU");
ArgInt ION2_IND2_ON(0, 0, 1, 1, "AU");
ArgInt ION3_IND3_ON(1, 0, 1, 1, "AU");
ArgInt ION4_IND4_ON(0, 0, 1, 1, "AU");
ArgInt ION5_IND5_ON(0, 0, 1, 1, "AU");

ArgFloat IND1_power(1.0, 0.0, 1.0, 0.1, 4, "AU");
ArgFloat IND2_power(1.0, 0.0, 1.0, 0.1, 4, "AU");
ArgFloat IND3_power(1.0, 0.0, 1.0, 0.1, 4, "AU");
ArgFloat IND4_power(1.0, 0.0, 1.0, 0.1, 4, "AU");
ArgFloat IND5_power(1.0, 0.0, 1.0, 0.1, 4, "AU");

ArgFloat scan_center_freq(0.0, -30.0, 30.0, 0.05, 6, "MHz"); // value relative to 200 MHz
ArgFloat scan_step_freq(0.1, 0.0, 100.0, 1.0, 3, "kHz");
ArgInt scan_span_freq(10, 1, 100000, 1, "Au");
ArgInt scan_pulse(20, 1, 1000, 20, "us");
ArgInt scan_range(0, -1, 1, 1, "AU");

ArgInt shots(100, 0, 300, 50, "AU");

int64_t IND1_ON_val = ION1_IND1_ON.get_value();
int64_t IND2_ON_val = ION2_IND2_ON.get_value();
int64_t IND3_ON_val = ION3_IND3_ON.get_value();
int64_t IND4_ON_val = ION4_IND4_ON.get_value();
int64_t IND5_ON_val = ION5_IND5_ON.get_value();

double IND1_power_val = IND1_power.get_value();
double IND2_power_val = IND2_power.get_value();
double IND3_power_val = IND3_power.get_value();
double IND4_power_val = IND4_power.get_value();
double IND5_power_val = IND5_power.get_value();

double scan_center_freq_val = scan_center_freq.get_value();
double scan_step_freq_val = scan_step_freq.get_value();
int64_t scan_span_freq_val = scan_span_freq.get_value();
int64_t scan_pulse_val = scan_pulse.get_value();
int64_t scan_range_val = scan_range.get_value();

int64_t shots_val = shots.get_value();

DatasetFloatList result_list_1("Sideband_scan_chain, ION1-PMT3", true);
DatasetFloatList result_list_2("Sideband_scan_chain, ION2-PMT4", true);
DatasetFloatList result_list_3("Sideband_scan_chain, ION3-PMT5", true);
DatasetFloatList result_list_4("Sideband_scan_chain, ION4-PMT6", true);
DatasetFloatList result_list_5("Sideband_scan_chain, ION5-PMT7", true);
int64_t result_data_1 = 0;
int64_t result_data_2 = 0;
int64_t result_data_3 = 0;
int64_t result_data_4 = 0;
int64_t result_data_5 = 0;
PMT_val_CHAIN result_counts;

int main(){
    reset_module();
    xil_printf("RFSoC Start\r\n");

    int begin = 0;
    int end = 0;

    if (scan_range_val == -1) {
        begin = -scan_span_freq_val;
        end = 0;
    }
    if (scan_range_val == 0) {
        begin = -scan_span_freq_val;
        end = scan_span_freq_val;
    }
    if (scan_range_val == 1) {
        begin = 0;
        end = scan_span_freq_val;
    }

    for(int i=begin; i < end + 1; i++){
        for(int shot = 0; shot < shots_val; shot++){
            // xil_printf("shot num: %d \r\n", i * shots_val + shot);
            SPAM_Reset_PMT_CHAIN();
            /*
             * Start of Experiment
             */
            // Initialize experiment
            init();

            delay(DDS_set_time);
            if (IND1_ON_val == 1) {
                IND1_1.set_config(IND1_power_val, (int64_t)(IND_freq * MHz) + (int64_t)((scan_center_freq_val * MHz) + (i * scan_step_freq_val * kHz)), 0.0, 0);
            }
            if (IND2_ON_val == 1) {
                IND2_1.set_config(IND2_power_val, (int64_t)(IND_freq * MHz) + (int64_t)((scan_center_freq_val * MHz) + (i * scan_step_freq_val * kHz)), 0.0, 0);
            }
            if (IND3_ON_val == 1) {
                IND3_1.set_config(IND3_power_val, (int64_t)(IND_freq * MHz) + (int64_t)((scan_center_freq_val * MHz) + (i * scan_step_freq_val * kHz)), 0.0, 0);
            }
            if (IND4_ON_val == 1) {
                IND4_1.set_config(IND4_power_val, (int64_t)(IND_freq * MHz) + (int64_t)((scan_center_freq_val * MHz) + (i * scan_step_freq_val * kHz)), 0.0, 0);
            }
            if (IND5_ON_val == 1) {
                IND5_1.set_config(IND5_power_val, (int64_t)(IND_freq * MHz) + (int64_t)((scan_center_freq_val * MHz) + (i * scan_step_freq_val * kHz)), 0.0, 0);
            }

            // Doppler cooling
            SPAM_Doppler();
            RAMAN_GLB.set(1); // Pre-turn on
            delay(Doppler_duration * ms);

            // Initialization
            SPAM_Init();
            delay(Init_duration * ns);
            SPAM_Off();
            delay(AOM_Main_falltime * us);

            // Operation: Rabi flopping
            RAMAN_GLB.set(1); // Global beam on
            if (IND1_ON_val == 1) {
                IND1_sw.on(0);
            }
            if (IND2_ON_val == 1) {
                IND2_sw.on(0);
            }
            if (IND3_ON_val == 1) {
                IND3_sw.on(0);
            }
            if (IND4_ON_val == 1) {
                IND4_sw.on(0);
            }
            if (IND5_ON_val == 1) {
                IND5_sw.on(0);
            }
            delay(scan_pulse_val * us);

            RAMAN_GLB.set(0); // Global beam off
            if (IND1_ON_val == 1) {
                IND1_sw.off(0);
            }
            if (IND2_ON_val == 1) {
                IND2_sw.off(0);
            }
            if (IND3_ON_val == 1) {
                IND3_sw.off(0);
            }
            if (IND4_ON_val == 1) {
                IND4_sw.off(0);
            }
            if (IND5_ON_val == 1) {
                IND5_sw.off(0);
            }
            delay(Raman_falltime * us);

            // Detection
            SPAM_Detect_CHAIN();

            // Termination
            SPAM_Doppler();

            if (i == begin && shot == 0) {
                xil_printf("CHAIN before first auto_start\r\n");
            }
            auto_start();
            if (i == begin && shot == 0) {
                xil_printf("CHAIN after first auto_start\r\n");
            }

            result_counts = SPAM_Read_PMT_CHAIN();

            result_data_1 = result_counts.count_1;
            result_data_2 = result_counts.count_2;
            result_data_3 = result_counts.count_3;
            result_data_4 = result_counts.count_4;
            result_data_5 = result_counts.count_5;

            result_list_1.append_list(3, (double)(result_data_1), (double)i, (double)shot);
            result_list_2.append_list(3, (double)(result_data_2), (double)i, (double)shot);
            result_list_3.append_list(3, (double)(result_data_3), (double)i, (double)shot);
            result_list_4.append_list(3, (double)(result_data_4), (double)i, (double)shot);
            result_list_5.append_list(3, (double)(result_data_5), (double)i, (double)shot);
        }
    }
}
