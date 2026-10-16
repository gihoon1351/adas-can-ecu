#ifndef CAN_HW_H
#define CAN_HW_H

#include <stdbool.h>
#include <stdint.h>

/* CAN HAL: PS(CPU)가 PL CAN IP의 AXI4-Lite 레지스터를 접근하는 계층.
 * 아래 주소는 260926 block diagram의 CAN IP 기준이다. 다른 XSA를 쓰면
 * 실제 연결 주소를 다시 확인해야 한다. CAN 프레임의 의미는 app/에서 정한다. */
typedef struct {
    /* volatile: 각 접근을 실제 버스 읽기/쓰기로 수행하도록 컴파일러에 지시.
     * BTR/CR/TX는 쓰기, SR/RX는 읽기 용도이다. */
    volatile uint32_t btr;         /* +0x00: [31:24]BRP [23:16]TSEG1 [15:8]TSEG2 [7:0]SJW */
    volatile uint32_t cr;          /* +0x04: bit1 리셋 펄스, bit0 송신 요청 펄스 */
    volatile uint32_t sr;          /* +0x08: bit0 BUS_OFF, bit1 TX_ERROR, bit2 RX_IRQ, bit3 TX_READY */
    volatile uint32_t tx_mailbox;  /* +0x0c: [16:13]DLC [12:2]ID [1]IDE [0]RTR */
    volatile uint32_t tx_data0;    /* +0x10: data[0]~data[3], 낮은 바이트부터 */
    volatile uint32_t tx_data1;    /* +0x14: data[4]~data[7], 낮은 바이트부터 */
    volatile uint32_t rx_mailbox;  /* +0x18: 수신 프레임 메타데이터 */
    volatile uint32_t rx_data0;    /* +0x1c: 수신 data[0]~data[3] */
    volatile uint32_t rx_data1;    /* +0x20: 수신 data[4]~data[7] */
} CanHwRegisters;

/* CAN 비트 타이밍 값. 이 IP에서는 1 TQ = (BRP+1)개의 입력 클록,
 * 1 CAN bit = (1+TSEG1+TSEG2) TQ로 설정한다. */
typedef struct {
    uint8_t brp, tseg1, tseg2, sjw;
} CanHwTiming;

/* 이 HAL은 11-bit 표준 ID의 데이터 프레임만 송신한다. DLC는 0~8. */
typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} CanFrame;

/* CAN IP 한 개의 레지스터 시작 주소를 기억한다. */
typedef struct {
    CanHwRegisters *regs;
} CanHw;

/* app이 호출할 수 있는 HAL 송신 함수 형식: context에는 CanHw *를 전달. */
typedef bool (*CanSend)(void *context, const CanFrame *frame);

/* 입력 클록 76.92308 MHz, {BRP,TSEG1,TSEG2,SJW}={10,11,2,2}이면
 * 76.92308 MHz / ((10+1)*(1+11+2)) = 약 499.5 kbit/s.
 * IP에 리셋 명령을 쓰고 비트 타이밍을 설정한다. */
bool CanHw_Init(CanHw *can, uintptr_t baseaddr, CanHwTiming timing);
/* 비차단 송신: 준비 상태 확인 후 단일 TX mailbox에 적고 송신 요청.
 * true = IP에 요청을 적음. 실제 버스 송신 성공/상대 노드 ACK 보장은 아님.
 * false = 인자 오류, mailbox 사용 중, bus-off 또는 TX 오류 상태. */
bool CanHw_Send(void *context, const CanFrame *frame);
/* Non-blocking receive. Returns true after one pending standard data frame
 * has been copied and the RX interrupt has been acknowledged. */
bool CanHw_Receive(CanHw *can, CanFrame *frame);
/* 원본 SR 반환. NULL이면 0이므로 이때의 0을 정상 상태로 해석하지 말 것. */
uint32_t CanHw_Status(const CanHw *can);

#endif
