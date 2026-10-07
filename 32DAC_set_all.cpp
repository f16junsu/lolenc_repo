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

ArgInt chip_select(0, 0, 1, 1, "");
ArgInt channel(1, 1, 16, 1, "");
ArgFloat voltage(0.0, -19.9, 20.0, 0.01, 2, "V");

int cs = chip_select.get_value();
int ch = channel.get_value();
double vol = voltage.get_value();


int main(){
    xil_printf("DAC controller test \r\n");
    reset_module();
    init();

    delay_coarse(10);
    dac_controller_0.initialize();
    dac_controller_1.initialize();
    delay_coarse(4000);
    xil_printf("DAC controllers initialized\r\n");


    delay_coarse(4000);
    dac_controller_0.dac_set_spi_config(0, 0, 0, 0, 0, 0, 0, 0, 0);
    dac_controller_1.dac_set_spi_config(0, 0, 0, 0, 0, 0, 0, 0, 0);
    xil_printf("DAC set spi config\r\n");

    delay_coarse(4000);
    dac_controller_0.dac_set_range0(0b1100, 0b1100, 0b1100, 0b1100);
    dac_controller_1.dac_set_range0(0b1100, 0b1100, 0b1100, 0b1100);
    delay_coarse(4000);
    dac_controller_0.dac_set_range1(0b1100, 0b1100, 0b1100, 0b1100);
    dac_controller_1.dac_set_range1(0b1100, 0b1100, 0b1100, 0b1100);
    delay_coarse(4000);
    dac_controller_0.dac_set_range2(0b1100, 0b1100, 0b1100, 0b1100);
    dac_controller_1.dac_set_range2(0b1100, 0b1100, 0b1100, 0b1100);
    delay_coarse(4000);
    dac_controller_0.dac_set_range3(0b1100, 0b1100, 0b1100, 0b1100);
    dac_controller_1.dac_set_range3(0b1100, 0b1100, 0b1100, 0b1100);
    delay_coarse(4000);

    delay_coarse(4000);
    dac_controller_0.dac_set_pwr_down(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    dac_controller_1.dac_set_pwr_down(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);



    for (int i = 0; i < 16; i++){
        delay_coarse(4000);
        dac_controller_0.dac_set_voltage(i, vol);
        dac_controller_1.dac_set_voltage(i, vol);
    }

    auto_start();

    usleep(500);
}