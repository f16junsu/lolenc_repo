실제 카메라: RAW 프레임과 detection 결과를 함께 확인하는 시험

FLARE_camera_single_test.cpp: jb_0에 HIGH 1 us 펄스를 딱 한 번 출력.
FLARE_camera_repeat_test.cpp: 같은 시험을 10번 수행. 각 shot의 RESULT,
자동 ACK, ZCU102 ready 복귀를 확인하고 추가로 1초 기다린 후 다음 펄스.
오류가 생기면 다음 펄스를 보내지 않고 중단한다.

두 cpp와 FLARE_camera_settings.h, FLARE_camera_test_common.h를 같은 폴더에 둔다.
일반 lolenc 실험처럼 cpp 하나만 선택해 실행한다. device_config.cpp는 현재
정의된 flare_controller와 jb_0를 사용하며 별도의 하드웨어/IP 빌드는 필요 없다.
FPGA는 Aurora가 포함된 버전을 사용해야 한다.
컴파일할 lolenc 소스/BSP도 FLARE 브랜치에 맞춰야 한다. PLNCO_line_trigger
브랜치에는 FLAREController가 없으므로 이 실험을 그대로 컴파일할 수 없다.
시작할 때 FLARE와 해당 TTL 그룹의 이전 명령 FIFO를 비운다. init()만으로는
TTL의 미실행 명령이 지워지지 않기 때문이다. FIFO만 비우며 출력 리셋은 하지 않는다.

카메라 준비
- 외부 트리거의 상승 에지 한 번당 한 프레임. Free run을 중지하고 촬영 대기.
- HIGH 1 us가 실제 카메라의 트리거 설정에서 수락되는지 확인.
- 512 x 512, binning 1 x 1, Camera Link 출력 활성화.
- 노출과 판독 설정은 카메라 SDK에서 설정한다. 이 코드는 바꾸지 않는다.
- 현재 FPGA의 SOF/frame timeout은 각각 기본 1초다. 아주 긴 노출은 피한다.
- jb_0는 AOM_MAIN과 동일한 실제 TTL 출력이다. 이번 시험에서는 카메라
  트리거에 연결한다. 다른 TTL 비트는 실제 하드웨어 출력값을 읽어서 유지한다.
- Pulse와 ARM_TRIGGER는 같은 RTIO tick에 놓인다. Aurora 수신까지의 지연은
  있으므로 ARMED_ACK를 기다린 뒤 촬영하는 프로토콜로 해석하지 않는다.

ZCU102 RAW 경로
- Linux driver와 zcu102-image-daemon이 실제 camera 모드로 동작해야 한다.
- /dev/zcu102-ion-camera0, inject_mode=N, auto_refill=Y, stride=1024 확인.
- 이번 코드는 STREAM_STOP 후 RAW_REQUIRED로 촬영한다. STREAM_STOP은
  free-running 저장 경로를 중지하며, shot의 RAW_REQUIRED 저장은 수행한다.
- PC 수신기는 viewer가 실행하는 receiver 하나만 사용한다.

PC viewer 시작
수신기의 새 --shot-metadata 옵션은 RAW 옆에 .raw.json을 저장한다.
CRC를 검증한 수신 header와 payload CRC, geometry, shot/frame ID가 들어간다.
이 옵션이 없는 기존 receiver로는 아래 PC 도구가 CRC 근거를 확인할 수 없다.
기존 viewer를 종료한 뒤 새 receiver 경로를 지정해 다시 시작한다.
single과 repeat은 각각 새로운 저장 폴더를 사용한다.

예시 (FLARE 프로젝트 루트에서 실행; ZCU102_IP는 실제 주소로 변경):

  mkdir C:\shots\flare_single_20261010
  python viewer/start_viewer.py --host ZCU102_IP --save-dir C:\shots\flare_single_20261010 --receiver sw/build/flare-image-receiver-camera-test.exe --shot-metadata

별도의 PC 터미널에서 (lolenc_repo 폴더):

  python FLARE_camera_verify.py --shots 1 --shot-dir C:\shots\flare_single_20261010 --results-root results

도구가 READY를 출력하고 viewer가 ZCU102에 연결된 상태에서 lolenc GUI의
FLARE_camera_single_test.cpp를 실행한다. HDF5가 생성되고 RAW가 저장되면
자동으로 대응시켜 PASS/FAIL과 JSON 보고서를 출력한다. 기다리는 한도는 300초.

10회 시험은 새 viewer 저장 폴더를 사용하고 위 PC 명령의 --shots를 10으로
바꾼 다음 FLARE_camera_repeat_test.cpp를 실행한다.

이미 완료한 HDF5 파일을 명시해서 다시 확인할 수도 있다:

  python FLARE_camera_verify.py --shots 1 --shot-dir C:\shots\flare_single_20261010 --h5 results\날짜\경로\실험결과.h5

판별 설정
FLARE_camera_settings.h의 기본값은 ROI (100,200), 8 x 8, threshold=5000,
confidence margin=1000, background 보정 OFF, ion 1개다. 실제 이온 위치에
맞춰 수정한다. ROI 좌표는 중심이 아니라 좌상단이며 전체 프레임은 유지한다.
threshold는 이 ROI의 픽셀 합과 비교하는 값이다. 현재 설정은 통신/계산 검증용이고
밝음/어두움 경계를 실제 이온에 대해 교정한 값이 아니다.

검증 범위
보드 DETECTION PASS:
- CONFIG_STATUS 정상, token 일치, ready 확인.
- ARMED_ACK accepted/raw_enabled=1/raw_reason=0.
- RESULT shot/token 일치, code=0, errors=0, frame/classifier/config_current=1.
- ion_count=1, valid_mask=1. state/low_conf/saturation은 실제 값으로 기록.
- RESULT sequence를 자동 ACK하고 결과/shot 카운터가 한 번 증가, ready 복귀.

PC FLARE camera PASS (최종 판정):
- 같은 shot ID의 RAW 하나. 중복·누락을 허용하지 않는다.
- header CRC 검증 근거, shot payload CRC 존재 및 저장 파일 CRC 재검증.
- valid frame, error=0, 512 x 512 x 16-bit, 524288 bytes.
- RAW에서 같은 ROI의 정수 픽셀 합을 구해 FPGA state 및 low_conf와 비교.
  state=1 iff sum>=threshold; low_conf=1 iff |sum-threshold|<margin.
- saturation_mask는 기록한다. 실제 FPGA 빌드의 포화 검출 설정은 별도이므로
  이 시험에서 카메라의 물리적 포화 수준을 검증했다고 판단하지 않는다.

현재 daemon의 RAW header는 metadata_partial=1이며 config/timestamp 필드는
실제 값이 없는 0이다. 이 값을 Aurora config token과 비교하지 않는다.
RAW와 RESULT는 shot ID로 대응시키고 config token은 Aurora 및 설정 데이터셋에서
확인한다. CRC와 계산 일치는 PC 저장까지의 일관성을 검증하며 Camera Link의
픽셀 방향/물리적 이온 위치는 viewer 화면과 실제 영상으로 함께 확인한다.

Dataset 스키마 (모든 값은 fixed-width double; 64-bit payload는 lo32/hi32 분리)
flare_camera_summary [board_pass,mask,failed_phase,requested,attempted,passed,token,raw_policy]
flare_camera_settings [token,ion_count,bg_enable,scale_q20,sw,sh,bw,bh,dx,dy,margin,x,y,threshold]
flare_camera_results [index,shot_id,token,result_sequence,state,valid,low_conf,saturation,
  errors,result_code,ion_count,config_current,classifier_complete,frame_valid,armed_ok,
  raw_enabled,ARM_us,RESULT_us,auto_ack_delta,board_pass]
flare_camera_records [phase,type,flags,words,entries,sequence,W0lo,W0hi,...W5hi]
flare_camera_exchanges [phase,kind,mask,records,ARM_us,RESULT_us,command_tx_delta,
  rx_delta,auto_ack_delta,tx_drop,record_drop,next_tx_sequence]
flare_camera_ttl [baseaddr,channel,pulse_ns,extra_gap_us,raw_policy]

mask: 1 mapping/virtual queue, 2 link, 4 timeout, 8 record shape,
16 protocol/duplicate, 32 local errors/counters, 64 config, 128 peer ready/counters,
256 ARM/raw reservation, 512 RESULT, 1024 automatic ACK, 2048 TTL/override, 4096 host IPC.
ARM/RESULT_us는 PS polling start 기준이며 200 us RTIO 예약과 polling 지연을 포함한다.
실제 trigger-to-frame 또는 hardware latency 그 자체로 해석하지 않는다.

성공 후 새 ROI 설정이 active로 남고 stream은 stopped, jb_0는 LOW로 남는다.
실패 때 protocol record를 데이터셋에 남긴다. 자동 재촬영/RESET/광범위한
peer error clear는 하지 않는다. RAW 수신/검증 도구도 기존 자료를 덮어쓰지 않는다.
