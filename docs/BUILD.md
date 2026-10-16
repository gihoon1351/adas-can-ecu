# 빌드 및 검증 안내

## 구성별 시작점

| 대상 | 소스 | 준비 방법 |
|---|---|---|
| Vision RTL | `hardware/vision/sources_1/` | 해당 Zynq 하드웨어 프로젝트에 소스와 필요한 IP를 연결 |
| Vision C | `firmware/vision/main.c` | 대응 Vision XSA로 Vitis 플랫폼을 만든 뒤 C 소스 추가 |
| VCU RTL | `hardware/vcu/vcu.srcs/` | Basys3와 원본 BD·커스텀 IP 구성을 확인 |
| VCU C | `firmware/vcu/main.c` | MicroBlaze 플랫폼/BSP를 생성하고 인터럽트·주소 정의 확인 |
| Motor | `firmware/motor1/`, `firmware/motor2/` | STM32CubeIDE에서 IOC와 소스, 링커 스크립트를 이용해 구성 |

Vivado/Vitis 2020.2와 원본 STM32CubeIDE 프로젝트 환경을 기준으로 합니다. 생성된 BIT·ELF·캐시·플랫폼 바이너리는 이 소스 정리본에 넣지 않았습니다. 특히 최신 Vision 폴더에는 전체 하드웨어 프로젝트/XSA가 없으므로 이 저장소만으로 원클릭 전체 재빌드를 보장하지 않습니다.

## 보드 연결 후 확인 순서

1. CAN 트랜시버, 전원과 접지, 종단저항 및 비트 타이밍을 확인합니다.
2. 각 ECU의 메모리 맵과 실제 연결된 IP 주소·인터럽트 번호를 BSP와 맞춥니다.
3. Vision 결과 0x301과 하트비트 0x300의 ID/DLC·바이트 구성을 확인합니다.
4. VCU 수신 메일박스, RX 인터럽트 해제, 소프트웨어 큐 해석 순서로 관측합니다.
5. VCU의 Motor 명령과 상태 응답을 확인합니다. 함수의 송신 요청 수락은 실제 CAN ACK 완료와 구분합니다.

이번 정리에서는 보드 업로드 및 기능 시뮬레이션을 실행하지 않았습니다.
