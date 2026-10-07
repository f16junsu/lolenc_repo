#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "utility.h"
#include <cmath>
#include <stdlib.h>
#include "sleep.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif
#define WAVE_LENGTH         (2560000)
extern WaveCacheController awg;

const double f11 = 0.815587;
const double f12 = 0.832943;
const double f13 = 0.847913;
const double f14 = 0.860929;
const double f15 = 0.871637;
const double f16 = 0.880333;
const double f17 = 0.885902;

const double f21 = 1.050483;
const double f22 = 1.063843;
const double f23 = 1.075622;
const double f24 = 1.085665;
const double f25 = 1.100806;
const double f26 = 1.094129;
const double f27 = 1.105699;

// Sisyphus pulse parameters
ArgInt Sis_pulse_duration(200, 0, 10000, 200, "us");
ArgInt Sis_iteration(50, 0, 500, 10, "AU");

int64_t Sis_pulse_duration_val = Sis_pulse_duration.get_value();
int64_t Sis_iteration_val = Sis_iteration.get_value();
double mw_pi_time = 29.0; // us

// Gate scan parameters
ArgInt step(5000, 1, 100000, 1, "ns");
ArgInt num_steps(30, 1, 1000, 1, "AU");
ArgInt shots(200, 0, 300, 50, "AU");
ArgInt q1_gate(0, 0, 1, 1, "AU");
ArgInt q2_gate(0, 0, 1, 1, "AU");
ArgInt q3_gate(0, 0, 1, 1, "AU");
ArgInt q4_gate(0, 0, 1, 1, "AU");
ArgInt q5_gate(0, 0, 1, 1, "AU");

// SBC powers
ArgFloat q1_sbc_power(1.0, 0.0, 1.0, 0.05, 4, "AU");
ArgFloat q2_sbc_power(1.0, 0.0, 1.0, 0.05, 4, "AU");
ArgFloat q3_sbc_power(1.0, 0.0, 1.0, 0.05, 4, "AU");
ArgFloat q4_sbc_power(1.0, 0.0, 1.0, 0.05, 4, "AU");
ArgFloat q5_sbc_power(1.0, 0.0, 1.0, 0.05, 4, "AU");

// Gate powers
ArgFloat q1_gate_power(1.0, 0.0, 1.0, 0.05, 4, "AU");
ArgFloat q2_gate_power(1.0, 0.0, 1.0, 0.05, 4, "AU");
ArgFloat q3_gate_power(1.0, 0.0, 1.0, 0.05, 4, "AU");
ArgFloat q4_gate_power(1.0, 0.0, 1.0, 0.05, 4, "AU");
ArgFloat q5_gate_power(1.0, 0.0, 1.0, 0.05, 4, "AU");

// Gate detunings (kHz)
ArgFloat q1_gate_detuning(0.0, 0.0, 2000.0, 0.5, 4, "kHz");
ArgFloat q2_gate_detuning(0.0, 0.0, 2000.0, 0.5, 4, "kHz");
ArgFloat q3_gate_detuning(0.0, 0.0, 2000.0, 0.5, 4, "kHz");
ArgFloat q4_gate_detuning(0.0, 0.0, 2000.0, 0.5, 4, "kHz");
ArgFloat q5_gate_detuning(0.0, 0.0, 2000.0, 0.5, 4, "kHz");

int64_t step_val = step.get_value();
int64_t num_steps_val = num_steps.get_value();
int64_t shots_val = shots.get_value();

int64_t q1_gate_val = q1_gate.get_value();
int64_t q2_gate_val = q2_gate.get_value();
int64_t q3_gate_val = q3_gate.get_value();
int64_t q4_gate_val = q4_gate.get_value();
int64_t q5_gate_val = q5_gate.get_value();

double q1_sbc_power_val = q1_sbc_power.get_value();
double q2_sbc_power_val = q2_sbc_power.get_value();
double q3_sbc_power_val = q3_sbc_power.get_value();
double q4_sbc_power_val = q4_sbc_power.get_value();
double q5_sbc_power_val = q5_sbc_power.get_value();

double q1_gate_power_val = q1_gate_power.get_value();
double q2_gate_power_val = q2_gate_power.get_value();
double q3_gate_power_val = q3_gate_power.get_value();
double q4_gate_power_val = q4_gate_power.get_value();
double q5_gate_power_val = q5_gate_power.get_value();

double q1_gate_detuning_val = q1_gate_detuning.get_value();
double q2_gate_detuning_val = q2_gate_detuning.get_value();
double q3_gate_detuning_val = q3_gate_detuning.get_value();
double q4_gate_detuning_val = q4_gate_detuning.get_value();
double q5_gate_detuning_val = q5_gate_detuning.get_value();
// Results
DatasetFloatList result_list_1("2Q gate 7-ION AWG, Q1", true);
DatasetFloatList result_list_2("2Q gate 7-ION AWG, Q2", true);
DatasetFloatList result_list_3("2Q gate 7-ION AWG, Q3", true);
DatasetFloatList result_list_4("2Q gate 7-ION AWG, Q4", true);
DatasetFloatList result_list_5("2Q gate 7-ION AWG, Q5", true);
DatasetFloatList* result_lists[5] = {
    &result_list_1, &result_list_2, &result_list_3, &result_list_4, &result_list_5
};

int64_t result_datas[5] = {0, 0, 0, 0, 0};
double gate_detunings[5] = {q1_gate_detuning_val, q2_gate_detuning_val, q3_gate_detuning_val, q4_gate_detuning_val, q5_gate_detuning_val};
double gate_powers[5] = {q1_gate_power_val, q2_gate_power_val, q3_gate_power_val, q4_gate_power_val, q5_gate_power_val};
PMT_val_CHAIN result_counts;

void reset_all_dds();
void sisyphus_cooling(double cooling_duration, int64_t n_iterations, double mw_pi_time);
void sbc_sequence();
void set_pulse(double detuning_1, double detuning_2, double detuning_3, double detuning_4, double detuning_5,
               int64_t pulse_duration, int64_t iteration);
void single_sbc_pulse(double detuning_1, double detuning_2, double detuning_3, double detuning_4, double detuning_5,
                      int64_t pulse_duration);
void set_pulse_switch(double detuning_11, double detuning_12, double detuning_13, double detuning_14, double detuning_15,
                      double detuning_21, double detuning_22, double detuning_23, double detuning_24, double detuning_25,
                      int64_t pulse_duration_1, int64_t pulse_duration_2, int64_t iteration);
void configure_gate_dds();
void run_gate_pulse(double pulse_time_ns);

int main() {
    reset_module();
    xil_printf("RFSoC Start\r\n");

    int16_t wave_data[WAVE_LENGTH] = {0};
    int64_t valid_channels[5] = {q1_gate_val, q2_gate_val, q3_gate_val, q4_gate_val, q5_gate_val};

    for (int64_t ion = 0; ion < 5; ion++) {
        if (!valid_channels[ion]) {
            continue;
        }
        double freq_rsb = IND_freq * MHz - gate_detunings[ion] *kHz;
        double freq_bsb = IND_freq * MHz + gate_detunings[ion] *kHz;
        printf("Ion %d: RSB freq = %f MHz, BSB freq = %f MHz\r\n", ion + 1, freq_rsb / MHz, freq_bsb / MHz);
        for (int64_t i = 0; i < WAVE_LENGTH; i++) {
            double angle_rsb = (2.0 * M_PI * ((double)i) * freq_rsb) / ((double)(2e9));
            double angle_bsb = (2.0 * M_PI * ((double)i) * freq_bsb) / ((double)(2e9));
            wave_data[i] = static_cast<int16_t>(gate_powers[ion] * (0x3fff * sin(angle_rsb) + 0x3fff * sin(angle_bsb)));
        }

        awg.write_wave_data(wave_data, WAVE_LENGTH, ion);
    }


    // init();
    // delay(delay_offset);
    // INIT_DDS();
    // auto_start();
    // sleep(1);

    init();
    delay(DDS_set_time);
    MW_1.set_config(MW_power_amp, (int64_t)(MW_ssb_frequency * MHz + MW_ssb_shift * kHz), 0.0, 0);
    MW_sw.off(0);
    delay(delay_offset);

    for (int i = 0; i < num_steps_val; i++) {
        double pulse_time = (double)i * step_val; // ns

        for (int shot = 0; shot < shots_val; shot++) {
            SPAM_Reset_PMT_CHAIN();

            /*
             * Start of Experiment
             */
            init();

            delay_coarse(1500);
            reset_all_dds();

            // Doppler cooling
            SPAM_Doppler();
            RAMAN_GLB.set(1); // Pre-turn on
            // delay(Doppler_duration * ms);
            delay(32);
            SPAM_Off();

            // Sisyphus cooling
            sisyphus_cooling(Sis_pulse_duration_val, Sis_iteration_val, mw_pi_time);

            // Initialization
            SPAM_Init();
            delay(Init_duration * ns);
            SPAM_Off();
            // delay(AOM_Main_falltime * us);
            delay(32);

            // Sideband cooling
            delay(DDS_set_time);
            // sbc_sequence();

            // Reset before gate
            reset_all_dds();

            // Gate DDS set to AWG mode
            configure_gate_dds();
            delay(8);

            // Gate pulse
            run_gate_pulse(pulse_time);

            // Detection
            // SPAM_Detect_CHAIN();

            // Terminate experiment
            SPAM_Doppler();

            auto_start();

            // result_counts = SPAM_Read_PMT_CHAIN();
            // result_datas[0] = result_counts.count_1;
            // result_datas[1] = result_counts.count_2;
            // result_datas[2] = result_counts.count_3;
            // result_datas[3] = result_counts.count_4;
            // result_datas[4] = result_counts.count_5;
            // for (int idx = 0; idx < 5; idx++) {
            //     if (valid_channels[idx]) {
            //         result_lists[idx]->append_list(3, (double)(result_datas[idx]), pulse_time / 1000, (double)shot);
            //     }
            // }
        }
    }
}

void reset_all_dds() {
    delay(DDS_set_time);

    IND1_sw.set_dds();
    IND2_sw.set_dds();
    IND3_sw.set_dds();
    IND4_sw.set_dds();
    IND5_sw.set_dds();

    IND1_1.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND1_2.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND2_1.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND2_2.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND3_1.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND3_2.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND4_1.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND4_2.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND5_1.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND5_2.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);

    delay(delay_offset);

    IND1_sw.off(0);
    IND2_sw.off(0);
    IND3_sw.off(0);
    IND4_sw.off(0);
    IND5_sw.off(0);

    delay(delay_offset);
}

void sisyphus_cooling(double cooling_duration, int64_t n_iterations, double mw_pi_time) {
    for (int i = 0; i < n_iterations; i++) {
        AOM_SIS_1.set(1);
        AOM_SIS_2.set(1);
        delay(cooling_duration * us + delay_offset);

        AOM_SIS_1.set(0);
        AOM_SIS_2.set(0);
        delay(AOM_Sisyphus_falltime * us);

        if (i != (n_iterations - 1)) {
            SPAM_Init();
            delay(Init_duration * ns);
            SPAM_Off();
            delay(AOM_Main_falltime * us);

            MW_sw.on(0);
            delay(mw_pi_time * us + delay_offset);
            MW_sw.off(0);
            delay(MW_falltime * us);
        }
    }
}

void sbc_sequence() {
    delay(10);

    // #1
    set_pulse_switch(
        f16, f14, f15, f11, f17,
        f26, f24, f25, f21, f27,
        5000, 7000, 25
    );

    // #2
    set_pulse_switch(
        f13, f12, f11, f17, f14,
        f23, f22, f21, f27, f24,
        7000, 7000, 25
    );

    // #3
    set_pulse_switch(
        f16, f14, f15, f11, f17,
        f26, f24, f25, f21, f27,
        11000, 11000, 25
    );
}

void set_pulse(double detuning_1, double detuning_2, double detuning_3, double detuning_4, double detuning_5,
               int64_t pulse_duration, int64_t iteration) {
    for (int i = 0; i < iteration; i++) {
        single_sbc_pulse(detuning_1, detuning_2, detuning_3, detuning_4, detuning_5, pulse_duration);
    }
}

void set_pulse_switch(double detuning_11, double detuning_12, double detuning_13, double detuning_14, double detuning_15,
                      double detuning_21, double detuning_22, double detuning_23, double detuning_24, double detuning_25,
                      int64_t pulse_duration_1, int64_t pulse_duration_2, int64_t iteration) {
    for (int i = 0; i < iteration; i++) {
        single_sbc_pulse(detuning_11, detuning_12, detuning_13, detuning_14, detuning_15, pulse_duration_1);
        single_sbc_pulse(detuning_21, detuning_22, detuning_23, detuning_24, detuning_25, pulse_duration_2);
    }
}

void single_sbc_pulse(double detuning_1, double detuning_2, double detuning_3, double detuning_4, double detuning_5,
                      int64_t pulse_duration) {
    IND1_1.set_config(q1_sbc_power_val, (int64_t)((IND_freq - detuning_1) * MHz), 0.0, 0);
    IND2_1.set_config(q2_sbc_power_val, (int64_t)((IND_freq - detuning_2) * MHz), 0.0, 0);
    IND3_1.set_config(q3_sbc_power_val, (int64_t)((IND_freq - detuning_3) * MHz), 0.0, 0);
    IND4_1.set_config(q4_sbc_power_val, (int64_t)((IND_freq - detuning_4) * MHz), 0.0, 0);
    IND5_1.set_config(q5_sbc_power_val, (int64_t)((IND_freq - detuning_5) * MHz), 0.0, 0);

    IND1_2.set_config(0.0, (int64_t)((IND_freq - detuning_1) * MHz), 0.0, 0);
    IND2_2.set_config(0.0, (int64_t)((IND_freq - detuning_2) * MHz), 0.0, 0);
    IND3_2.set_config(0.0, (int64_t)((IND_freq - detuning_3) * MHz), 0.0, 0);
    IND4_2.set_config(0.0, (int64_t)((IND_freq - detuning_4) * MHz), 0.0, 0);
    IND5_2.set_config(0.0, (int64_t)((IND_freq - detuning_5) * MHz), 0.0, 0);

    delay(delay_offset);

    RAMAN_GLB.set(1); // kept on for pre-turn on
    IND1_sw.on(0);
    IND2_sw.on(0);
    IND3_sw.on(0);
    IND4_sw.on(0);
    IND5_sw.on(0);

    delay(pulse_duration * ns + delay_offset);

    IND1_sw.off(0);
    IND2_sw.off(0);
    IND3_sw.off(0);
    IND4_sw.off(0);
    IND5_sw.off(0);

    delay(Raman_falltime * us);

    SPAM_Init();
    delay(Init_duration * ns);
    SPAM_Off();
    delay(AOM_Main_falltime * us);
}

void configure_gate_dds() {

    IND1_sw.set_awg();
    IND2_sw.set_awg();
    IND3_sw.set_awg();
    IND4_sw.set_awg();
    IND5_sw.set_awg();
    // delay(delay_offset * ns);

    if (q1_gate_val) {
        awg.set_awg(1, 0, WAVE_LENGTH >> 4, 1400);
    }
    if (q2_gate_val) {
        awg.set_awg(2, 1, WAVE_LENGTH >> 4, 1300);
    }
    if (q3_gate_val) {
        awg.set_awg(3, 2, WAVE_LENGTH >> 4, 1200);
    }
    if (q4_gate_val) {
        awg.set_awg(4, 3, WAVE_LENGTH >> 4, 1100);
    }
    if (q5_gate_val) {
        awg.set_awg(5, 4, WAVE_LENGTH >> 4, 1000);
    }
}

void run_gate_pulse(double pulse_time_ns) {
    RAMAN_GLB.set(1); // GLB beam on

    if (q1_gate_val) IND1_sw.on(0);
    if (q2_gate_val) IND2_sw.on(0);
    if (q3_gate_val) IND3_sw.on(0);
    if (q4_gate_val) IND4_sw.on(0);
    if (q5_gate_val) IND5_sw.on(0);

    delay(pulse_time_ns * ns + delay_offset);

    RAMAN_GLB.set(0); // GLB beam off
    IND1_sw.off(0);
    IND2_sw.off(0);
    IND3_sw.off(0);
    IND4_sw.off(0);
    IND5_sw.off(0);

    delay(Raman_falltime * us);
}