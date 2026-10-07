#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "utility.h"
#include <cmath>
#include <stdlib.h>
#include <unistd.h>

extern TTL AOM_MAIN;
extern SwitchController IND1_sw;

ArgInt step(1, 1, 10000, 1, "ns");
ArgInt num_steps(1, 1, 1000, 1, "AU");
ArgInt shots(1, 0, 10000, 1, "AU");

int64_t step_val = step.get_value();
int64_t num_steps_val = num_steps.get_value();
int64_t shots_val = shots.get_value();

DatasetFloatList result_list("result", true);
int64_t result_data = 0;

int main(){
    reset_module();
    xil_printf("RFSoC Start\r\n");

    for(int i=0; i < num_steps_val; i++){
        /*
        * Start of Experiment
        */
        IND1_sw.reset();
        sleep(0.5);
        init();

        for(int shot = 0; shot < shots_val; shot++){
           delay_coarse(2);
           AOM_MAIN.on();
           delay_coarse(2);
           AOM_MAIN.off();
            // delay_coarse(2);
            // IND1_sw.on(0);
            // delay_coarse(2);
            // IND1_sw.off(0);
        }
        auto_start();
        sleep(1);
    }
    xil_printf("Experiment Finished\r\n");
}
