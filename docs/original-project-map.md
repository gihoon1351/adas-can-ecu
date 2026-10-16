# Project map

| 구성요소 | 프로젝트 위치 | 최종 산출물 |
|---|---|---|
| Motor 1 | `PROJECTS/motor1` | `RELEASE/MOTOR/motor1.elf` |
| Motor 2 | `PROJECTS/motor2` | `RELEASE/MOTOR/motor2.elf` |
| VCU hardware | `PROJECTS/vcu` | `RELEASE/VCU/vcu_design_wrapper.bit`, `.xsa` |
| VCU firmware | `PROJECTS/vitis/vcu_app` | `RELEASE/VCU/vcu_app.elf` |
| VCU platform | `PROJECTS/vitis/vcu_platform` | platform export 내부 |
| Vision hardware | `PROJECTS/sobel/sobel_test` | `RELEASE/VISION/sobel_test_dual_roi.bit`, `.xsa` |
| Vision firmware | `PROJECTS/vitis/ycbc` | `RELEASE/VISION/ycbc.elf` |
| Vision platform | `PROJECTS/vitis/ycbcr` | platform export 내부 |
| 업로드/튜닝 도구 | `PROJECTS/tools`, `TOOLS` | 해당 없음 |
| 측정 자료 | `TESTS/measurements`, `TESTS/tuning` | 해당 없음 |

Vivado의 생성 캐시와 과거 실험 프로젝트는 활성 프로젝트에서 제외했습니다. 현재 프로젝트에 필요한 `*.srcs`, `*.gen`, `*.ip_user_files`, 블록 디자인 및 제약파일은 포함합니다.

