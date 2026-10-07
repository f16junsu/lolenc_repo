#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "xparameters.h"
#include "sleep.h"
#include <time.h>
#include <cmath>

extern DAC dac_controller_0;
extern DAC dac_controller_1;

int main(){
    xil_printf("DAC controller test \r\n");
    reset_module();
    init();

    delay_coarse(10);
    dac_controller_0.initialize();
    dac_controller_1.initialize();
    xil_printf("DAC controllers initialized\r\n");


    delay_coarse(4000);
    dac_controller_0.dac_set_spi_config(0, 0, 0, 0, 0, 0, 0, 0, 0);
    dac_controller_1.dac_set_spi_config(0, 0, 0, 0, 0, 0, 0, 0, 0);
    xil_printf("DAC set spi config\r\n");

    delay_coarse(4000);
    dac_controller_0.dac_set_pwr_down(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
    dac_controller_1.dac_set_pwr_down(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
    xil_printf("DAC set pwr down\r\n");

    xil_printf("32DAC_PWD before auto_start\r\n");
    auto_start();
    xil_printf("32DAC_PWD after auto_start\r\n");
    xil_printf("32DAC_PWD returning\r\n");
    // end_experiment();
}
