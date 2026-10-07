#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"

int main(){
    int64_t value=0;

    Adder adder("adder", "127.0.0.4", 8080);

    //while(true){
        value = (int64_t)adder.add(50.0,20.0);
        xil_printf("Adder Test\r\n");
    //}
    adder.set_value(88.0);
    value = (int64_t)adder.get_value();
    adder.print_value();
    xil_printf("added value :%d\r\n", value);
}