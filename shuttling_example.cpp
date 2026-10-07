#include "core.h"
#include "module.h"
#include "dataset.h"
#include "peripheral.h"
#include "xparameters.h"
#include "sleep.h"

#define WAVE_LENGTH 3

extern DAC dac_controller_0;
extern DAC dac_controller_1;
extern LDAC dac0_ldac;
extern LDAC dac1_ldac;


int main(){
    reset_module();
    init();


    /* Make DAC32 instance */
    DAC32 dac32_controller(&dac_controller_0, &dac_controller_1);

    /* Setting waveforms */

    // 테스트 1: 기본 Waveform 생성 및 설정
    xil_printf("=== Test 1: Basic Waveform Creation ===\r\n");
    Waveform test_wave1; // 기본 Wavefrom 변수 정의
    // 설정할 VoltageSet 변수 정의(DAC0, DAC1)
    VoltageSet v0 = VoltageSet({
        0,
        0,
        0,
        -0.14,
        -0.14,
        -0.14,
        -0.15,
        -0.15,
        -0.16,
        -0.15,
        -0.15,
        -0.14,
        -0.14,
        -0.14,
        0,
        -0.2
    });
    VoltageSet v1 = VoltageSet({
        0,
        0,
        0,
        -0.14,
        -0.14,
        -0.14,
        -0.15,
        -0.15,
        -0.16,
        -0.15,
        -0.15,
        -0.14,
        -0.14,
        -0.14,
        0,
        -0.2
    });
    // add_set을 통해 LENGTH 신경쓰지 않고 추가할 수 있음.
    test_wave1.add_set(v0, v1);
    test_wave1.add_set(v0, v1);
    test_wave1.add_set(v0, v1);
    test_wave1.add_set(VoltageSet(), VoltageSet()); // remove test를 위한 0으로 초기화된 VoltageSet 추가
    test_wave1.add_set(v0, v1);
    test_wave1.add_set(v0, v1);
    // 6개 세트 추가

    test_wave1.remove_set(3); // 결과적으로 5개 세트가 남음.
    test_wave1.insert_set(3, VoltageSet(), VoltageSet()); // 3번째 세트 자리에 0으로 초기화된 VoltageSet 추가

    // DAC32에 파형 저장
    dac32_controller.save_waveform(0, test_wave1);
    xil_printf("Waveform 0 saved with 32 channels, 6 sets\r\n");




    // 테스트 1.5: 일차원 배열로부터 Waveform 생성
    xil_printf("=== Test 1.5: Waveform from 1D Arrays ===\r\n");

    // 48개 요소 (3세트 x 16채널) 일차원 배열 생성
    double vol0[48] = {0};  // DAC0 배열
    double vol1[48] = {0};  // DAC1 배열

    // 배열 초기화 (간단한 패턴)
    for (int i = 0; i < 48; i++) {
        vol0[i] = 0.1 * i;           // 0V, 0.1V, 0.2V, ..., 4.7V
        vol1[i] = 0.05 * i + 0.5;    // 0.5V, 0.55V, 0.6V, ..., 2.9V
    }

    // 일차원 배열로부터 Waveform 생성
    Waveform array_wave(vol0, vol1);

    // DAC32에 파형 저장
    dac32_controller.save_waveform(4, array_wave);
    xil_printf("Waveform 4 saved from 1D arrays (3 sets, 32 channels)\r\n");

    // 테스트 1.6: VoltageSet 배열로부터 Waveform 생성
    xil_printf("=== Test 1.6: Waveform from VoltageSet Arrays ===\r\n");
    VoltageSet arr0[2], arr1[2];
    arr0[0] = VoltageSet({
        0,
        0,
        0,
        -0.14,
        -0.14,
        -0.14,
        -0.15,
        -0.15,
        -0.16,
        -0.15,
        -0.15,
        -0.14,
        -0.14,
        -0.14,
        0,
        -0.2
    });
    arr1[0] = arr0[0];
    arr0[1] = VoltageSet({
        0,
        0,
        0,
        -0.1,
        -0.1,
        -0.1,
        -0.15,
        -0.1,
        0.1,
        -0.1,
        -0.15,
        -0.1,
        -0.1,
        -0.1,
        0,
        -0.2
    });
    arr1[1] = arr0[1];

    Waveform vs_wave(arr0, arr1);
    dac32_controller.save_waveform(5, vs_wave);
    xil_printf("Waveform 5 saved from VoltageSet arrays (2 sets, 32 channels)\r\n");


    // 테스트 2: Linear Interpolation
    xil_printf("=== Test 2: Linear Interpolation ===\r\n");
    VoltageSet start_dac0, start_dac1, end_dac0, end_dac1;

    // 시작점: 모든 채널 0V
    start_dac0.set_all(0.0);
    start_dac1.set_all(0.0);

    // 끝점: 모든 채널 3V
    end_dac0.set_all(3.0);
    end_dac1.set_all(3.0);

    Waveform linear_wave = Waveform::create_linear_interpolation(
        start_dac0, start_dac1, end_dac0, end_dac1, 10
    );

    dac32_controller.save_waveform(1, linear_wave);
    xil_printf("Linear interpolation waveform saved (0V to 3V, 10 steps)\r\n");

    // 테스트 3: Waveform 연결 (Concatenation)
    xil_printf("=== Test 3: Waveform Concatenation ===\r\n");
    Waveform wave_part1(3);
    Waveform wave_part2(3);

    // 첫 번째 부분: 낮은 전압
    for (int i = 0; i < 3; i++) {
        wave_part1.set_all_channels(i, 1.0);
    }

    // 두 번째 부분: 높은 전압
    for (int i = 0; i < 3; i++) {
        wave_part2.set_all_channels(i, 4.0);
    }

    // 두 파형 연결
    Waveform combined_wave = wave_part1 + wave_part2;
    dac32_controller.save_waveform(2, combined_wave);
    xil_printf("Combined waveform saved (3 sets @ 1V + 3 sets @ 4V)\r\n");

    // 테스트 4: 특정 채널별 설정
    xil_printf("=== Test 4: Channel-specific Configuration ===\r\n");
    Waveform channel_test_wave(4);

    // 각 세트별로 다른 패턴 설정
    for (int set = 0; set < 4; set++) {
        for (int ch = 0; ch < 32; ch++) {
            double voltage = (set + 1) * 0.5 + (ch * 0.05);  // 세트와 채널에 따른 전압
            channel_test_wave.set_voltage(set, ch, voltage);
        }
    }

    dac32_controller.save_waveform(3, channel_test_wave);
    xil_printf("Channel-specific waveform saved\r\n");

    // 테스트 5: 파형 정보 출력
    xil_printf("=== Test 5: Waveform Information ===\r\n");
    for (int i = 0; i < 4; i++) {
        dac32_controller.print_waveform_info(i);
        xil_printf("\r\n");
    }


    // From here PL logic starts
    /* Initialize DAC */
    dac32_controller.initialize();

    /* Play waveforms - 실제 하드웨어 테스트 */
    xil_printf("=== Hardware Test: Playing Waveforms ===\r\n");

    xil_printf("Playing basic waveform (0)...\r\n");
    dac32_controller.play_waveform(0, 125000000);


    xil_printf("Playing linear interpolation waveform (1)...\r\n");
    dac32_controller.play_waveform(1, 125000000);  // 선형 보간 파형 재생


    xil_printf("Playing combined waveform (2)...\r\n");
    dac32_controller.play_waveform(2, 125000000);  // 결합된 파형 재생


    xil_printf("Playing 1D array-based waveform (4)...\r\n");
    dac32_controller.play_waveform(4, 125000000);  // 1D array로부터 생성된 파형 재생


    dac32_controller.play_waveform(5, 125000000);  // VoltageSet 배열로부터 생성된 파형 재생

    // 테스트 8: 모든 파형 정보 출력
    xil_printf("=== Final Status: All Waveforms ===\r\n");
    dac32_controller.print_all_waveforms_info();


    /* Start the experiment */
    auto_start();
    usleep(10000);

}