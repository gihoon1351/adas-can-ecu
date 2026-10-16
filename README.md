# CAN Controller 및 CNN 기반 ADAS 차량 제어 시스템

Vision ECU의 차선·CNN 인식 정보를 VCU로 전달하고, VCU가 Motor ECU에 제어 명령을 전달하는 분산 ECU 시스템입니다.

| 항목 | 내용 |
|---|---|
| 작성자 | 최기훈 |
| 수행 기간 | 2026.09.08 ~ 2026.10.14 |
| 프로젝트 형태 | 팀 프로젝트 |
| 담당 범위 | CAN 프로토콜 RTL, AXI4-Lite 레지스터 맵, C 함수 인터페이스 및 인식 결과 연동 |
| 주요 기술 | Verilog · SystemVerilog · C · AXI4-Lite · CAN 2.0A · MicroBlaze · Zynq PS |

## 저장소 구성

| 경로 | 내용 |
|---|---|
| `hardware/vision/` | 최신 Vision ECU RTL |
| `hardware/vcu/` | VCU CAN·Safety Domain·주변장치 RTL 및 BD |
| `firmware/vision/` | visionecu1013 C 소스 |
| `firmware/vcu/` | MicroBlaze VCU C 소스 |
| `firmware/motor1/, firmware/motor2/` | Motor ECU 및 보존된 HAL 소스 |
| `docs/` | 구조·실행 안내와 원본 대응표 |

## 자료 범위와 확인 상태

이 저장소는 프로젝트 원본에서 RTL, 펌웨어, 테스트벤치와 필요한 설정 자료를 추려 정리한 소스 배포본입니다. 생성 캐시, 실행 바이너리, 과거 백업과 중복 BSP는 제외했습니다. 원본 소스의 로직은 변경하지 않았습니다.

정리 과정에서는 파일 구성과 참조 경로를 점검했습니다. 이번 정리에서 Vivado 합성·구현, VCS 시뮬레이션, 보드 동작을 새로 실행한 것은 아닙니다. 프로젝트 수행 당시 결과와 현재 소스의 재실행 결과는 구분합니다.

- [실행 및 검증 안내](docs/BUILD.md)
- [파일 구성과 원본 대응](docs/SOURCE_MAP.csv)
- [구조 및 주요 동작](docs/ARCHITECTURE.md)
- [포함된 제3자 코드 안내](THIRD_PARTY.md)
