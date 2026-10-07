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

ArgFloat DAC0_1(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_2(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_3(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_4(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_5(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_6(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_7(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_8(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_9(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_10(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_11(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_12(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_13(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_14(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_15(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC0_16(0.0, -19.9, 20.0, 0.1, 4, "V");

ArgFloat DAC1_1(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_2(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_3(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_4(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_5(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_6(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_7(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_8(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_9(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_10(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_11(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_12(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_13(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_14(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_15(0.0, -19.9, 20.0, 0.1, 4, "V");
ArgFloat DAC1_16(0.0, -19.9, 20.0, 0.1, 4, "V");

double dac0[16] = {
    DAC0_1.get_value(),
    DAC0_2.get_value(),
    DAC0_3.get_value(),
    DAC0_4.get_value(),
    DAC0_5.get_value(),
    DAC0_6.get_value(),
    DAC0_7.get_value(),
    DAC0_8.get_value(),
    DAC0_9.get_value(),
    DAC0_10.get_value(),
    DAC0_11.get_value(),
    DAC0_12.get_value(),
    DAC0_13.get_value(),
    DAC0_14.get_value(),
    DAC0_15.get_value(),
    DAC0_16.get_value()
};
double dac1[16] = {
    DAC1_1.get_value(),
    DAC1_2.get_value(),
    DAC1_3.get_value(),
    DAC1_4.get_value(),
    DAC1_5.get_value(),
    DAC1_6.get_value(),
    DAC1_7.get_value(),
    DAC1_8.get_value(),
    DAC1_9.get_value(),
    DAC1_10.get_value(),
    DAC1_11.get_value(),
    DAC1_12.get_value(),
    DAC1_13.get_value(),
    DAC1_14.get_value(),
    DAC1_15.get_value(),
    DAC1_16.get_value()
};


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
    dac_controller_0.dac_set_pwr_down(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    dac_controller_1.dac_set_pwr_down(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    //

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
    for (int i = 0; i < 16; i++) {
        delay_coarse(4000);
        dac_controller_0.dac_set_voltage(i, dac0[i]);
        dac_controller_1.dac_set_voltage(i, dac1[i]);
    }

    auto_start();

    usleep(1000);
}