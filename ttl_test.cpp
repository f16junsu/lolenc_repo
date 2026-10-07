#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "xparameters.h"
#include "sleep.h"
#include <time.h>
#include <cmath>

extern TTL jd_6;
extern TTL jb_7;

int main(){
    xil_printf("TTL test \r\n");
    reset_module();
    init();

    // TTL send signal test 1
    delay_coarse(100);
    for (int i = 0; i < 30; i++){
        jb_7.on();
        delay_coarse(1);
        jb_7.off();
        delay_coarse(1);
    }

    xil_printf("TTL signal sent\r\n");

    auto_start();

    usleep(1000);
}