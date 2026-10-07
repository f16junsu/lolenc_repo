#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "xparameters.h"
#include "sleep.h"
#include <time.h>
#include <cmath>


extern TTL jb_7;

int main(){
    xil_printf("DAC controller test \r\n");
    reset_module();
    init();

    delay_coarse(100);
    jb_7.off();


    auto_start();
    usleep(1000);
}