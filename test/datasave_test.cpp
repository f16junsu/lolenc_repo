#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include <cmath>
#include <stdlib.h>

extern TTL jb_1;

int main(){
    DatasetFloatList data_list("data_list",true);
    uint64_t random = 0;

    xil_printf("Data Save Test\r\n");
    for(int i = 0 ; i < 100; i++){
        for( int j = 0 ; j < 10; j++){
            random = rand() % 100;
            double t = 0.01 * i;
            if( random < 50.0*sin(600*t) + 50.0){
                // Count, Time, Shot
                data_list.append_list(3,(double)(random)+200.0,(double)i,(double)j);
            }
            else{
                // Count, Time, Shot
                data_list.append_list(3,(double)(random),(double)i,(double)j);
            }
        }
    }

    delay_coarse(1);
    jb_1.off();
    delay_coarse(1);
    jb_1.off();
    auto_start();
}