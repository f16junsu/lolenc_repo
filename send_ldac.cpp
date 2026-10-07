#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "xparameters.h"
#include "sleep.h"
#include <time.h>
#include <cmath>


extern LDAC jd_6;
extern LDAC jd_7;

int main(){
    xil_printf("DAC controller test \r\n");
    reset_module();
    init();

    // DAC send ldac signal test 1
    delay_coarse(100);
    jd_6.trigger();
    delay_coarse(60);
    jd_7.trigger();
    xil_printf("LDAC signal sent\r\n");


    auto_start();

    usleep(1000);
}