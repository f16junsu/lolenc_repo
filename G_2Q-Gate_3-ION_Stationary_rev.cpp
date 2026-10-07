#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "utility.h"
#include <cmath>
#include <stdlib.h>
#include "sleep.h"

# define NUM_IONS         (3)

const double f11 = 1.285304;
const double f12 = 1.397370;
const double f13 = 1.471907;

const double f21 = 1.527301;
const double f22 = 1.623078;
const double f23 = 1.687925;

// Sisyphus pulse parameters
ArgInt Sis_pulse_duration(200, 0, 10000, 200, "us");
ArgInt Sis_iteration(0, 0, 500, 10, "AU");

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

// SBC powers
ArgFloat q1_sbc_power(1.0, 0.0, 1.0, 0.05, 4, "AU");
ArgFloat q2_sbc_power(1.0, 0.0, 1.0, 0.05, 4, "AU");
ArgFloat q3_sbc_power(1.0, 0.0, 1.0, 0.05, 4, "AU");

// Gate powers
ArgFloat q1r_gate_power(1.0, 0.0, 1.0, 0.01, 4, "AU");
ArgFloat q2r_gate_power(1.0, 0.0, 1.0, 0.01, 4, "AU");
ArgFloat q3r_gate_power(1.0, 0.0, 1.0, 0.01, 4, "AU");
ArgFloat q1b_gate_power(1.0, 0.0, 1.0, 0.01, 4, "AU");
ArgFloat q2b_gate_power(1.0, 0.0, 1.0, 0.01, 4, "AU");
ArgFloat q3b_gate_power(1.0, 0.0, 1.0, 0.01, 4, "AU");

// Gate detunings (kHz)
ArgFloat q1_gate_detuning(0.0, 0.0, 2000.0, 0.5, 4, "kHz");
ArgFloat q2_gate_detuning(0.0, 0.0, 2000.0, 0.5, 4, "kHz");
ArgFloat q3_gate_detuning(0.0, 0.0, 2000.0, 0.5, 4, "kHz");

ArgFloat f_sym(0.0, -100.0, 100.0, 100.0, 4, "kHz");
ArgFloat f_asym(0.0, -100.0, 100.0, 100.0, 4, "kHz");
ArgFloat amp_power(1.0, 0.0, 10.0, 0.01, 4, "AU");
ArgFloat imb_power(0.0, -1.0, 1.0, 0.01, 4, "AU");

int64_t step_val = step.get_value();
int64_t num_steps_val = num_steps.get_value();
int64_t shots_val = shots.get_value();

int64_t q1_gate_val = q1_gate.get_value();
int64_t q2_gate_val = q2_gate.get_value();
int64_t q3_gate_val = q3_gate.get_value();

double q1_sbc_power_val = q1_sbc_power.get_value();
double q2_sbc_power_val = q2_sbc_power.get_value();
double q3_sbc_power_val = q3_sbc_power.get_value();

double q1r_gate_power_val = q1r_gate_power.get_value();
double q2r_gate_power_val = q2r_gate_power.get_value();
double q3r_gate_power_val = q3r_gate_power.get_value();

double q1b_gate_power_val = q1b_gate_power.get_value();
double q2b_gate_power_val = q2b_gate_power.get_value();
double q3b_gate_power_val = q3b_gate_power.get_value();

double q1_gate_detuning_val = q1_gate_detuning.get_value();
double q2_gate_detuning_val = q2_gate_detuning.get_value();
double q3_gate_detuning_val = q3_gate_detuning.get_value();

double f_sym_val = f_sym.get_value();
double f_asym_val = f_asym.get_value();
double amp_power_val = amp_power.get_value();
double imb_power_val = imb_power.get_value();

ArgInt enable_sbc(1, 0, 1, 1, "AU");
int64_t enable_sbc_val = enable_sbc.get_value();

// Results
DatasetFloatList result_list_1("2Q gate 3-ION, Q1", true);
DatasetFloatList result_list_2("2Q gate 3-ION, Q2", true);
DatasetFloatList result_list_3("2Q gate 3-ION, Q3", true);
DatasetFloatList* result_lists[NUM_IONS] = {
    &result_list_1, &result_list_2, &result_list_3
};

int64_t result_data = 0;
int64_t result_datas[NUM_IONS] = {0, 0, 0};
PMT_val_CHAIN result_counts;

int64_t pmt_channel = 3; // default to PMT3 for single ion case

void reset_all_dds();
void sisyphus_cooling(double cooling_duration, int64_t iteration, double mw_pi_time);
void sbc_sequence();
void set_pulse(double detuning_1, double detuning_2, double detuning_3,
               int64_t pulse_duration, int64_t iteration);
void single_sbc_pulse(double detuning_1, double detuning_2, double detuning_3,
                      int64_t pulse_duration);
void set_pulse_switch(double detuning_11, double detuning_12, double detuning_13,
                      double detuning_21, double detuning_22, double detuning_23,
                      int64_t pulse_duration_1, int64_t pulse_duration_2, int64_t iteration);
bool check_single_or_chain(int64_t q1, int64_t q2, int64_t q3);
int64_t get_valid_single_pmt(int64_t q1, int64_t q2, int64_t q3);
int get_result_index_from_pmt(int64_t pmt_channel);

void configure_gate_dds();
void run_gate_pulse(double pulse_time_ns);

bool is_single;
int64_t valid_channels[NUM_IONS] = {q1_gate_val, q2_gate_val, q3_gate_val};


void pre_experiment(){
    reset_module();
    xil_printf("RFSoC Start\r\n");

    is_single = check_single_or_chain(q1_gate_val, q2_gate_val, q3_gate_val);

    if (is_single) {
        pmt_channel = get_valid_single_pmt(q1_gate_val, q2_gate_val, q3_gate_val);
    }

    init();

    delay(delay_offset);
    INIT_DDS();

    delay(DDS_set_time);
    MW_1.set_config(MW_power_amp, (int64_t)(MW_ssb_frequency * MHz + MW_ssb_shift * kHz), 0.0, 0);
    MW_sw.off(0);
    delay(delay_offset);

    set_potential_to_operation();

    auto_start();
    usleep(1000);
}

void post_experiment(){
    init();
    doppler_stage1();
    set_potential_to_idle();
    auto_start();

    xil_printf("End of the experiment\r\n");
}

void pre_shot(){
    if (is_single) {
        SPAM_Reset_ITH_PMT_SINGLE(pmt_channel);
    } else {
        SPAM_Reset_PMT_CHAIN();
    }

    init();
    reset_all_dds();
    delay(DDS_set_time);
}

void post_shot(){
    doppler_stage1();
    auto_start();
}

int main() {
    pre_experiment();

    for (int i = 0; i < num_steps_val; i++) {
        double pulse_time = (double)i * step_val; // ns

        for (int shot = 0; shot < shots_val; shot++) {
            pre_shot();

            // Doppler cooling
            two_stage_doppler();
            RAMAN_GLB.set(1); // Pre-turn on
            SPAM_Off();

            // Sisyphus cooling
            sisyphus_cooling(Sis_pulse_duration_val, Sis_iteration_val, mw_pi_time);

            // Initialization
            SPAM_Init();
            delay(Init_duration * ns);
            SPAM_Off();

            // Operation: sideband cooling
            if (enable_sbc_val) {
                delay(DDS_set_time);
                sbc_sequence();
            }

            // Reset before gate
            reset_all_dds();

            // Gate DDS setup
            configure_gate_dds();

            // Gate pulse
            run_gate_pulse(pulse_time);

            // Detection
            if (is_single) {
                SPAM_Detect_ITH_SINGLE(pmt_channel);
            } else {
                SPAM_Detect_CHAIN();
            }

            // Terminate experiment
            post_shot();

            if (is_single) {
                result_data = SPAM_Read_ITH_PMT_SINGLE(pmt_channel);
                int idx = get_result_index_from_pmt(pmt_channel);
                result_lists[idx]->append_list(3, (double)(result_data), pulse_time / 1000, (double)shot);
            } else {
                result_counts = SPAM_Read_PMT_CHAIN();
                result_datas[0] = result_counts.count_2;
                result_datas[1] = result_counts.count_3;
                result_datas[2] = result_counts.count_4;
                for (int idx = 0; idx < NUM_IONS; idx++) {
                    if (valid_channels[idx]) {
                        result_lists[idx]->append_list(3, (double)(result_datas[idx]), pulse_time / 1000, (double)shot);
                    }
                }
            }
        }
    }
    post_experiment();
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

    set_pulse_switch(
        f13, f13, f13,
        f23, f23, f23,
        22000, 6000, 20
    );

    set_pulse_switch(
        f12, f11, f13,
        f22, f21, f23,
        20000, 6000, 30
    );
    set_pulse_switch(
        f12, f11, f13,
        f23, f23, f23,
        20000, 20000, 5
    );
}

void set_pulse(double detuning_1, double detuning_2, double detuning_3,
               int64_t pulse_duration, int64_t iteration) {
    for (int i = 0; i < iteration; i++) {
        single_sbc_pulse(detuning_1, detuning_2, detuning_3, pulse_duration);
    }
}

void set_pulse_switch(double detuning_11, double detuning_12, double detuning_13,
                      double detuning_21, double detuning_22, double detuning_23,
                      int64_t pulse_duration_1, int64_t pulse_duration_2, int64_t iteration) {
    for (int i = 0; i < iteration; i++) {
        single_sbc_pulse(detuning_11, detuning_12, detuning_13, pulse_duration_1);
        single_sbc_pulse(detuning_21, detuning_22, detuning_23, pulse_duration_2);
        single_sbc_pulse(detuning_21, detuning_22, detuning_23, pulse_duration_2);
        single_sbc_pulse(detuning_21, detuning_22, detuning_23, pulse_duration_2);
    }
}

void single_sbc_pulse(double detuning_1, double detuning_2, double detuning_3,
                      int64_t pulse_duration) {
    IND2_1.set_config(q1_sbc_power_val, (int64_t)((IND_freq - detuning_1) * MHz), 0.0, 0);
    IND3_1.set_config(q2_sbc_power_val, (int64_t)((IND_freq - detuning_2) * MHz), 0.0, 0);
    IND4_1.set_config(q3_sbc_power_val, (int64_t)((IND_freq - detuning_3) * MHz), 0.0, 0);
    RAMAN_GLB.set(1); // GLB beam on
    IND2_sw.on(0);
    IND3_sw.on(0);
    IND4_sw.on(0);

    delay(pulse_duration * ns + delay_offset);

    IND2_sw.off(0);
    IND3_sw.off(0);
    IND4_sw.off(0);
    delay(Raman_falltime * us);

    // Initialization
    SPAM_Init();
    delay(Init_duration * ns);
    SPAM_Off();
    delay(AOM_Main_falltime * us);
}

void reset_all_dds() {
    delay(DDS_set_time);
    IND2_1.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND2_2.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND3_1.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND3_2.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND4_1.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);
    IND4_2.set_config(0.0, (int64_t)(IND_freq * MHz), 0.0, 0);

    delay(delay_offset);

    IND2_sw.off(0);
    IND3_sw.off(0);
    IND4_sw.off(0);

    delay(delay_offset);
}

void configure_gate_dds() {
    if (q1_gate_val == 1) {
        IND2_1.set_config(q1r_gate_power_val * (1 - imb_power_val) * amp_power_val, (int64_t)(IND_freq * MHz - (q1_gate_detuning_val + f_sym_val) * kHz + f_asym_val * kHz), 0.0, 0);
        IND2_2.set_config(q1b_gate_power_val * (1 + imb_power_val) * amp_power_val, (int64_t)(IND_freq * MHz + (q1_gate_detuning_val + f_sym_val) * kHz + f_asym_val * kHz), 0.0, 0);
    }
    if (q2_gate_val == 1) {
        IND3_1.set_config(q2r_gate_power_val * (1 - imb_power_val) * amp_power_val, (int64_t)(IND_freq * MHz - (q2_gate_detuning_val + f_sym_val) * kHz + f_asym_val * kHz), 0.0, 0);
        IND3_2.set_config(q2b_gate_power_val * (1 + imb_power_val) * amp_power_val, (int64_t)(IND_freq * MHz + (q2_gate_detuning_val + f_sym_val) * kHz + f_asym_val * kHz), 0.0, 0);
    }
    if (q3_gate_val == 1) {
        IND4_1.set_config(q3r_gate_power_val * (1 - imb_power_val) * amp_power_val, (int64_t)(IND_freq * MHz - (q3_gate_detuning_val + f_sym_val) * kHz + f_asym_val * kHz), 0.0, 0);
        IND4_2.set_config(q3b_gate_power_val * (1 + imb_power_val) * amp_power_val, (int64_t)(IND_freq * MHz + (q3_gate_detuning_val + f_sym_val) * kHz + f_asym_val * kHz), 0.0, 0);
    }
    delay(DDS_set_time);
    INIT_DDS();
    delay(DDS_set_time);
}

void run_gate_pulse(double pulse_time_ns) {
    RAMAN_GLB.set(1); // GLB beam on

    if (q1_gate_val == 1) IND2_sw.on(0);
    if (q2_gate_val == 1) IND3_sw.on(0);
    if (q3_gate_val == 1) IND4_sw.on(0);
    delay(pulse_time_ns * ns + delay_offset);

    RAMAN_GLB.set(0); // GLB beam off
    IND2_sw.off(0);
    IND3_sw.off(0);
    IND4_sw.off(0);
    delay(Raman_falltime * us);
}

bool check_single_or_chain(int64_t q1, int64_t q2, int64_t q3) {
    int64_t sum = q1 + q2 + q3;
    return (sum == 1) ? true : false; // true for single, false for chain
}

int64_t get_valid_single_pmt(int64_t q1, int64_t q2, int64_t q3) {
    if (q1) return 2;
    else if (q2) return 3;
    else if (q3) return 4;
    else return 3;
}

int get_result_index_from_pmt(int64_t pmt_channel) {
    if (pmt_channel == 2) return 0;
    else if (pmt_channel == 3) return 1;
    else if (pmt_channel == 4) return 2;
    else return 1;
}