#include "can_hw.h"
#include <stddef.h>

/* CR을 쓰면 RTL이 해당 비트를 1클록 명령 펄스로 바꾼다.
 * 송신할 때는 CR=1, 리셋할 때는 CR=2를 각각 쓴다. */
#define CAN_CR_SW_RESET 0x2U
#define CAN_CR_TX_VALID 0x1U
/* SR은 RTL이 갱신하는 상태 레지스터. READY는 '다음 요청 가능'이라는 뜻. */
#define CAN_SR_BUS_OFF  0x1U
#define CAN_SR_ERROR    0x2U
#define CAN_SR_RX_IRQ   0x4U
#define CAN_SR_TX_READY 0x8U

/* 부팅 시 1회 호출. baseaddr은 AXI4-Lite CAN IP의 시작 주소.
 * BRP는 RTL의 카운터가 5비트이므로 0~31만 허용한다. */
bool CanHw_Init(CanHw *can, uintptr_t baseaddr, CanHwTiming timing)
{
    /* NULL 주소, 정렬되지 않은 주소, 성립하지 않는 비트 타이밍은 거부. */
    if (can == NULL || baseaddr == (uintptr_t)0 || (baseaddr & 3U) != 0U ||
        timing.tseg1 == 0U || timing.tseg2 == 0U || timing.sjw == 0U ||
        timing.sjw > timing.tseg2 || timing.brp > 31U)
        return false;
    can->regs = (CanHwRegisters *)baseaddr;
    /* 코어의 진행 중인 송수신 상태를 초기화한 다음 타이밍을 설정한다.
     * SW reset은 AXI 슬레이브 전체의 레지스터 초기화와 다르다. */
    can->regs->cr = CAN_CR_SW_RESET;
    can->regs->btr = ((uint32_t)timing.brp << 24) |
                     ((uint32_t)timing.tseg1 << 16) |
                     ((uint32_t)timing.tseg2 << 8) | timing.sjw;
    return true;
}

uint32_t CanHw_Status(const CanHw *can)
{
    /* 수신 인터럽트(bit2)는 여기서는 해석하지 않고 SR 전체를 돌려준다. */
    return (can == NULL || can->regs == NULL) ? 0U : can->regs->sr;
}

/* app에서 등록하는 송신 콜백. 이 HAL은 ID/길이/데이터만 싣는다.
 * 프레임의 CRC, bit stuffing, arbitration, ACK 검사는 CAN RTL 담당. */
bool CanHw_Send(void *context, const CanFrame *frame)
{
    CanHw *can = (CanHw *)context;
    uint32_t s, lo = 0U, hi = 0U;
    uint8_t i;
    /* 현재 코드는 11비트 표준 ID, 데이터 0~8바이트만 지원. */
    if (can == NULL || can->regs == NULL || frame == NULL ||
        frame->id > 0x7ffU || frame->dlc > 8U)
        return false;
    s = CanHw_Status(can);
    /* IP가 이전 프레임을 처리 중이거나 오류 중이면 레지스터를 덮지 않는다.
     * 이 함수에는 SW 대기 루프/FIFO가 없다. app이 다음 poll에서 재시도. */
    if ((s & CAN_SR_TX_READY) == 0U ||
        (s & (CAN_SR_BUS_OFF | CAN_SR_ERROR)) != 0U)
        return false;
    /* 예: bytes={0x80,0x02,0x03,0x28,0x64}, DLC=5이면
     * lo=0x28030280, hi=0x00000064. 쓰지 않는 바이트는 0. */
    for (i = 0U; i < frame->dlc; ++i) {
        if (i < 4U) lo |= (uint32_t)frame->data[i] << (8U * i);
        else hi |= (uint32_t)frame->data[i] << (8U * (i - 4U));
    }
    /* MAILBOX: DLC와 표준 ID 설정. IDE/RTR=0 -> 표준 데이터 프레임.
     * DATA를 모두 쓰고 마지막에 CR을 쓰면 RTL이 완성된 값을 캡처. */
    can->regs->tx_mailbox = ((uint32_t)frame->dlc << 13) |
                            ((uint32_t)frame->id << 2); /* standard data */
    can->regs->tx_data0 = lo;
    can->regs->tx_data1 = hi;
    can->regs->cr = CAN_CR_TX_VALID;
    /* 여기의 true는 요청 수락만 의미한다. ACK/송신 완료는 이후 RTL에서
     * 결정되며, 이 IP의 RX IRQ도 송신 완료 IRQ가 아니다. */
    return true;
}

bool CanHw_Receive(CanHw *can, CanFrame *frame)
{
    uint32_t mailbox, lo, hi;
    uint8_t i;
    if (can == NULL || can->regs == NULL || frame == NULL ||
        (CanHw_Status(can) & CAN_SR_RX_IRQ) == 0U)
        return false;

    mailbox = can->regs->rx_mailbox;
    lo = can->regs->rx_data0;
    hi = can->regs->rx_data1;
    frame->id = (uint16_t)((mailbox >> 2U) & 0x7ffU);
    frame->dlc = (uint8_t)((mailbox >> 13U) & 0x0fU);
    if (((mailbox & 0x3U) != 0U) || frame->dlc > 8U) {
        can->regs->sr = CAN_SR_RX_IRQ;
        return false;
    }
    for (i = 0U; i < 8U; ++i) {
        frame->data[i] = (i < 4U) ?
            (uint8_t)(lo >> (8U * i)) :
            (uint8_t)(hi >> (8U * (i - 4U)));
    }
    can->regs->sr = CAN_SR_RX_IRQ;
    return true;
}
