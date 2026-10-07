#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "utility.h"
#include <cmath>
#include <limits>
#include <stdlib.h>
#include <stdio.h>
#include "sleep.h"

extern DDS MW_1;
extern DDS MW_2;
extern DDS IND1_1;
extern DDS IND1_2;
extern DDS IND2_1;
extern DDS IND2_2;
extern DDS IND3_1;
extern DDS IND3_2;
extern DDS dds_07;
extern DDS dds_17;



extern SwitchController MW_sw;
extern SwitchController IND1_sw;
extern SwitchController IND2_sw;
extern SwitchController IND3_sw;
extern SwitchController dds_sw_7;

extern RamanPID_Controller raman_pid_controller;

extern TTL jb_0;

static constexpr uint64_t RAMAN_PID_ADC_NCO_FREQ_HZ = 370378400ULL;
static constexpr uint64_t RAMAN_PID_FABRIC_DDC_FREQ_HZ = 10000000ULL;
static constexpr uint64_t RAMAN_PID_HARMONIC_REF_FREQ_HZ =
    RAMAN_PID_ADC_NCO_FREQ_HZ - RAMAN_PID_FABRIC_DDC_FREQ_HZ;

int main(){
    reset_module();
    xil_printf("RFSoC Start\r\n");
    xil_printf("Raman PID Test\r\n");

    // set_rfdc_nco_freq_mhz_ex(RFDC_NCO_TILE_ADC, 0U, 0U, 370.3784);
    // sleep(2);

    init();
    delay(1000);

    // IND2_sw.set_dds();
    // IND3_sw.set_dds();
    // delay(16);
    // IND2_sw.on(0);
    // IND3_sw.on(0);
    // delay(320);

    // IND2_1.set_config(1, (int64_t)(IND_freq * MHz), 0.0, 0);
    // IND3_1.set_config(1, (int64_t)(IND_freq * 1 * MHz), 0.0, 0);
    // delay(160);
    // IND3_1.initialize_phase();
    // delay(256);
    // IND3_1.set_config(1, (int64_t)(IND_freq * 1.33 * MHz), 0.0, 0);
    // delay(160);
    // IND3_1.set_config(1, (int64_t)(IND_freq * 1 * MHz), 0.0, 0);
    // delay(16);
    // IND3_1.reset_phase();
    // delay(160);
    // IND3_1.sync_set_freq((int64_t)(IND_freq * 1 * MHz));


    delay(1000);
    // IND2_sw.off(0);
    // IND3_sw.off(0);


    // dds_sw_7.set_dds();
    // MW_sw.set_dds();
    // delay(1000);


    // MW_1.set_config(1, (int64_t)(360.377123 * MHz), 0.0, 0);
    // dds_07.set_config(1, (int64_t)(360.377412 * MHz), 0.0, 0);
    // dds_17.set_config(1, (int64_t)(360.377123 * MHz), 0.0, 0);
    // dds_07.set_phase(0.25);

    // IND1_1.set_config(1, (int64_t)(200 * MHz), 0.0, 0);
    delay(1000);



    // MW_1.reset_phase();
    // dds_07.reset_phase();
    // dds_17.reset_phase();
    // MW_1.initialize_phase();
    // dds_07.initialize_phase();
    // delay(1000);

    // for (int i = 0; i < 21; i++){
    //     dds_07.set_config(1, (int64_t)((360.36 + i * 0.0001) * MHz), 0.0, 0);
    //     delay(1000 * us);
    // }
    // dds_07.set_freq((int64_t)(243 * MHz));
    // dds_07.set_config(1, (int64_t)(243 * MHz), 0.0, 0);
    // MW_1.set_config(1, (int64_t)(243 * MHz), 0.0, 0);
    // delay(1000);
    // MW_1.sync_set_freq((int64_t)(200 * MHz));
    // dds_07.sync_set_freq((int64_t)(200 * MHz));

    // MW_sw.on(0);
    // dds_sw_7.on(0);

    // MW_sw.off(0);
    dds_sw_7.off(0);
    dds_sw_7.off(0);

    // IND1_sw.on(0);
    // jb_0.on();

    // jb_0.off();
    // raman_pid_controller.set_ADC_NCO_freq(RAMAN_PID_ADC_NCO_FREQ_HZ);
    delay(1000);

    // raman_pid_controller.set_timings(21, 4, 0, 1, 4); // last arg: signed CORDIC shift offset
    // long double current_nco_hz = 0.0L;

    // if (raman_pid_controller.try_read_current_ADC_NCO_freq_hz(current_nco_hz)) {
    //     printf("Current ADC NCO frequency: %.6Lf Hz\r\n", current_nco_hz);
    // } else {
    //     raman_pid_controller.print_status();
    // }
    // delay(1000);
    // raman_pid_controller.set_cordic_input_shift(15);
    // raman_pid_controller.set_cordic_shift_offset(-1); // offset from base shift 17
    // raman_pid_controller.set_comp_order(100, 100);
    // raman_pid_controller.lock_PID();
    // raman_pid_controller.unlock_PID();
    // raman_pid_controller.set_cordic_shift_offset(8);
    // delay(1000);

    auto_start();

    sleep(1);
    // printf("ADC NCO: %llu Hz, harmonic reference: %llu Hz\r\n",
    //        (unsigned long long)RAMAN_PID_ADC_NCO_FREQ_HZ,
    //        (unsigned long long)RAMAN_PID_HARMONIC_REF_FREQ_HZ);
    // for (int i = 0; i < 2160000; i++){
    for (int i = 0; i < 10; i++){

        // __uint128_t power_snapshot =
        // raman_pid_controller.read_block_power_snapshot();

        // uint64_t power_raw =
        //     static_cast<uint64_t>(power_snapshot);

        // uint64_t power_status =
        //     static_cast<uint64_t>(power_snapshot >> 64);

        // bool power_valid = (power_status & 0x1ULL) != 0;
        // uint32_t power_seq =
        //     static_cast<uint32_t>(power_status >> 32);

        // if (power_valid) {
        //     printf("Power: %llu, seq: %u\r\n",
        //         (unsigned long long)power_raw,
        //         (unsigned int)power_seq);
        // } else {
        //     printf("Power: invalid/timeout\r\n");
        // }

        uint64_t harmonic_est = raman_pid_controller.read_harmonic_est_word();
        __uint128_t harmonic_est_microhz =
            (static_cast<__uint128_t>(harmonic_est & MASK48BIT) *
             static_cast<__uint128_t>(SAMPLING_FREQ) *
             static_cast<__uint128_t>(1000000)) /
            (static_cast<__uint128_t>(1) << 48);
        printf("Harmonic Estimate: %llu.%06llu Hz, t: %d ms\r\n",
               (unsigned long long)(
                   static_cast<uint64_t>(harmonic_est_microhz / 1000000)
               ),
               (unsigned long long)(
                   static_cast<uint64_t>(harmonic_est_microhz % 1000000)
               ),
               (i * 10));
        usleep(10000);
    }

    // raman_pid_controller.print_status();
}
