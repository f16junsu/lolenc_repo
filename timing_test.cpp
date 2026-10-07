#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "utility.h"
#include <cmath>
#include <limits>
#include <stdlib.h>
#include "sleep.h"

#define WAVE_LENGTH 2944 << 6

extern TTL jb_0;
extern SwitchController dds_sw_0;
extern SwitchController dds_sw_1;
extern SwitchController dds_sw_2;
extern WaveCacheController wave_cache_controller;

// default , min, max
ArgInt shots(1, 0, 20, 5, "AU");
int64_t shots_val = shots.get_value();
ArgInt period(50, 0, 1000, 5, "ns");
int64_t period_val = period.get_value();
ArgInt num(10, 0, 100, 1, "AU");
int64_t num_val = period.get_value();
// ArgFloat pulse_width_ns_arg(
//     1.0,
//     0.0,
//     static_cast<double>(WAVE_LENGTH) / 16.0,
//     1.0 / 16.0,
//     4,
//     "ns"
// );
// double pulse_width_ns_val = pulse_width_ns_arg.get_value();

DatasetFloatList result_list("Timing Test", true);
int64_t result_data = 0;

int16_t pulse[WAVE_LENGTH] = {0};
int16_t square_pulse[WAVE_LENGTH] = {0};

inline int64_t f(double t){
    const double a = 2.4;
    const double b = 0.4;
    const double c = 12062;
    double value = ((t-a)*(t-a) - a*a) * (std::exp(-b * t)) * c;
    return (int64_t)value;
}

inline int64_t g(int sample, int64_t pulse_sample_count){
    if(sample < 0 || pulse_sample_count <= 0){
        return 0;
    }

    return (sample < pulse_sample_count)
        ? static_cast<int64_t>(std::numeric_limits<int16_t>::min())
        : 0;
}

// int16_t zeros[WAVE_LENTGH] = {0};

int main(){
    reset_module();
    xil_printf("RFSoC Start\r\n");
    // wave_cache_controller.reset();
    // // init();

    // prepare pulse data
    for (int t = 0; t < (period_val * 2 * num_val); t += (period_val * 2)){
        for(int sample = 0; sample < 100; sample++){
            const int offset = 0;
            pulse[sample + offset + t] = f((double)sample / 2);
        }
    }
    // for(int sample = 0; sample < 100; sample++){
    //     const int offset = 0;
    //     pulse[sample + offset] = f((double)sample / 2);
    // }


    // wave_cache_controller.write_wave_data(zeros, WAVE_LENGTH, 0);
    wave_cache_controller.write_wave_data(pulse, WAVE_LENGTH, 0);

    for(int shot = 0; shot < shots_val; shot++){
        init();

        delay_coarse(10);
        dds_sw_0.set_awg();
        dds_sw_1.set_awg();
        dds_sw_2.set_awg();

        delay_coarse(10);
        dds_sw_0.on(1);
        dds_sw_1.on(1);
        dds_sw_2.on(1);

        delay_coarse(2000);
        wave_cache_controller.set_awg(1, 0, WAVE_LENGTH >> 4, 1500);
        wave_cache_controller.set_awg(2, 0, WAVE_LENGTH >> 4, 1400);
        jb_0.on();

        delay_coarse(1000);
        jb_0.off();

        auto_start();
    }

    return 0;
}
