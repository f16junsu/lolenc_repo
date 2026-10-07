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
ArgFloat DAC1_voltage(0.0, -20.0, 20.0, 0.1, 1, "V");
ArgFloat DAC2_voltage(0.0, -20.0, 20.0, 0.1, 1, "V");
double vol1 = DAC1_voltage.get_value();
double vol2 = DAC2_voltage.get_value();

int main(){
    xil_printf("DAC controller test \r\n");
    reset_module();
    init();

    delay_coarse(10);
    dac_controller_0.initialize();
    dac_controller_1.initialize();
    xil_printf("DAC controllers initialized\r\n");


    delay_coarse(300);
    dac_controller_0.dac_set_spi_config(0, 0, 0, 0, 0, 0, 0, 0, 0);
    dac_controller_1.dac_set_spi_config(0, 0, 0, 0, 0, 0, 0, 0, 0);
    xil_printf("DAC set spi config\r\n");
    delay_coarse(300);
    dac_controller_0.dac_set_voltage(0, vol1);

    //delay_coarse(300);
    //dac_controller_0.dac_set_pwr_down(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);
    //dac_controller_1.dac_set_pwr_down(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    //dac_controller_0.dac_set_pwr_down(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
    //delay_coarse(300);
    //dac_controller_0.dac_set_range0(0b1100, 0b1100, 0b1100, 0b1100);
    //dac_controller_1.dac_set_range0(0b1100, 0b1100, 0b1100, 0b1100);
    //delay_coarse(300);
    //dac_controller_0.dac_set_range1(0b1100, 0b1100, 0b1100, 0b1100);
    //dac_controller_1.dac_set_range1(0b1100, 0b1100, 0b1100, 0b1100);
    //delay_coarse(300);
    //dac_controller_0.dac_set_range2(0b1100, 0b1100, 0b1100, 0b1100);
    //dac_controller_1.dac_set_range2(0b1100, 0b1100, 0b1100, 0b1100);
    //delay_coarse(300);
    //dac_controller_0.dac_set_range3(0b1100, 0b1100, 0b1100, 0b1100);
    //dac_controller_1.dac_set_range3(0b1100, 0b1100, 0b1100, 0b1100);

    //delay_coarse(300);
    //dac_controller_0.dac_disable_ldac();
    //dac_controller_1.dac_disable_ldac();


    //for (int i = 0; i < 16; i++) {
    //    delay_coarse(300);
    //    dac_controller_0.dac_set_voltage(i, vol1);
    //    dac_controller_1.dac_set_voltage(i, vol2);
    //    xil_printf("DAC1-%d voltage: %dV", i, vol1);
    //    xil_printf("DAC2-%d voltage: %dV\r\n", i, vol2);
   // }
    //delay_coarse(4000);
    //dac_controller_0.dac_set_voltage(0, 0);
    //dac_controller_1.dac_set_voltage(0, 0);
    // for (int i = 0; i < 16; i++) {
    //     delay_coarse(4000);
    //     dac_controller_0.dac_set_voltage(i, i);
    //     xil_printf("DAC1-%d voltage: %fV DAC2-%d voltage: %fV\r\n", i, vol1, i, vol2);
    // }

    // // DAC voltage set test 1
    // delay_coarse(10);
    // dac_controller_0.dac_set_voltage(0, 1);
    // xil_printf("DAC channel 2 voltage set to 1V\r\n");

    // // DAC voltage set test 2
    // delay_coarse(300);
    // dac_controller_0.dac_set_voltage(1, 2);
    // xil_printf("DAC channel 15 voltage set to 2.5V\r\n");

    // // DAC set spi config test
    // delay_coarse(300);
    // dac_controller_0.dac_set_spi_config(1, 0, 1, 0, 1, 0, 0, 0, 0);
    // xil_printf("DAC set spi config\r\n");

    // DAC enable ldac test
    // delay_coarse(4000);
    // dac_controller_0.dac_enable_ldac();
    // dac_controller_1.dac_enable_ldac();
    // xil_printf("DAC LDAC enabled\r\n");

    // for (int i = 0; i < 16; i++) {
    //     delay_coarse(4000);
    //     dac_controller_0.dac_set_voltage(i, 0);
    //     dac_controller_1.dac_set_voltage(i, 0);
    //     xil_printf("DAC channel %d voltage set to 0V\r\n", i);
    // }

    auto_start();

    usleep(1000);
}