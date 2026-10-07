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


DatasetFloatList result_list("pmt_counts", true);

static double random_uniform_open(){
    return ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
}

static int64_t sample_poisson(double mean){
    const double threshold = std::exp(-mean);
    int64_t count = -1;
    double product = 1.0;

    do{
        count++;
        product *= random_uniform_open();
    } while(product > threshold);

    return count;
}


int main(){
    reset_module();

    xil_printf("Random PMT Test\r\n");
    init();
    delay(1000);

    for(int shot = 0; shot < 1024; shot++){
        SPAM_Reset_PMT_SINGLE();
                /*
                * Start of Experiment
                */
                // Initialize experiment
        init();
        // dac32_controller.play_waveform(0, 12500);

                // Doppler cooling
        SPAM_Doppler();
        delay(Doppler_duration * ms);

                // Detection
        SPAM_Detect_SINGLE();

                // Doppler cooling
        SPAM_Doppler();

        auto_start();

        int64_t result_data = SPAM_Read_PMT_SINGLE();

        const bool ghz_111 = (rand() % 2) == 1;
        int64_t result_data1 = ghz_111 ? sample_poisson(10.0) : 0;
        int64_t result_data2 = ghz_111 ? sample_poisson(10.0) : 0;
        int64_t result_data3 = ghz_111 ? sample_poisson(10.0) : 0;
        result_list.append_list(3, (double)result_data1, (double)result_data2, (double)result_data3);
    }
    result_list.append_list(3, (double)-1.0, (double)-1.0, (double)-1.0);
}
