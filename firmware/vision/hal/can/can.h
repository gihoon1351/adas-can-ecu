#ifndef SRC_HAL_HAL_CAN_H_
#define SRC_HAL_HAL_CAN_H_

#include <stdbool.h>
#include <stdint.h>
#include "xparameters.h"

/* =========================================================
 * Register Map
 * CAN_BTR (0x00): Bit timing configuration.
 * CAN_CR (0x04): Control commands.
 * CAN_SR (0x08): Controller status.
 * CAN_TX_MAILBOX (0x0C): Transmit identifier and frame control.
 * CAN_TX_DATA0/1 (0x10/0x14): Transmit payload bytes 0-7.
 * CAN_RX_MAILBOX (0x18): Received identifier and frame control.
 * CAN_RX_DATA0/1 (0x1C/0x20): Received payload bytes 0-7.
 * ========================================================= */

typedef struct
{
	volatile uint32_t CAN_BTR;        // 0x00
	volatile uint32_t CAN_CR;         // 0x04
	volatile uint32_t CAN_SR;         // 0x08
	volatile uint32_t CAN_TX_MAILBOX; // 0x0C
	volatile uint32_t CAN_TX_DATA0;   // 0x10
	volatile uint32_t CAN_TX_DATA1;   // 0x14
	volatile uint32_t CAN_RX_MAILBOX; // 0x18
	volatile uint32_t CAN_RX_DATA0;   // 0x1C
	volatile uint32_t CAN_RX_DATA1;   // 0x20

} CAN_TypeDef_t;

/* =========================================================
 * Base Address
 * ========================================================= */

#define CAN_BASEADDR XPAR_CAN_CONTROLLER_V1_0_0_BASEADDR
#define CAN0 ((CAN_TypeDef_t *)(uintptr_t)CAN_BASEADDR)

/* =========================================================
 * Status Codes
 * ========================================================= */

typedef enum
{
	CAN_STATUS_OK = 0,
	CAN_STATUS_BUSY,
	CAN_STATUS_INVALID_PARAMETER,
	CAN_STATUS_NO_DATA,
	CAN_STATUS_ERROR
} CAN_Status_t;

/* =========================================================
 * Protocol Limits
 * ========================================================= */

#define CAN_STANDARD_ID_MAX 0x7FFU
#define CAN_DLC_MAX 8U

/* =========================================================
 * CAN Frame
 * ide is currently limited to standard identifiers.
 * ========================================================= */

typedef struct
{
	uint16_t id;
	uint8_t dlc;
	bool rtr;
	bool ide;
	uint8_t data[CAN_DLC_MAX];
} CAN_Frame_t;

/* =========================================================
 * CAN_BTR
 * ========================================================= */

#define CAN_SJW_POS 0U
#define CAN_TSEG2_POS 8U
#define CAN_TSEG1_POS 16U
#define CAN_BRP_POS 24U

#define CAN_SJW_MASK (0xFFU << CAN_SJW_POS)
#define CAN_TSEG2_MASK (0xFFU << CAN_TSEG2_POS)
#define CAN_TSEG1_MASK (0xFFU << CAN_TSEG1_POS)
#define CAN_BRP_MASK (0xFFU << CAN_BRP_POS)

/* =========================================================
 * CAN_CR
 * ========================================================= */

#define CAN_TX_VALID (1U << 0)
#define CAN_SW_RESET (1U << 1)

/* =========================================================
 * CAN_SR
 * ========================================================= */

#define CAN_BUS_OFF (1U << 0)
#define CAN_ERROR (1U << 1)
#define CAN_RX_INT (1U << 2)
#define CAN_TX_READY (1U << 3)

/* =========================================================
 * CAN_TX_MAILBOX
 * ========================================================= */

#define CAN_TX_RTR (1U << 0)
#define CAN_TX_IDE (1U << 1)

#define CAN_TX_ID_POS 2U
#define CAN_TX_DLC_POS 13U

#define CAN_TX_ID_MASK (0x7FFU << CAN_TX_ID_POS)
#define CAN_TX_DLC_MASK (0xFU << CAN_TX_DLC_POS)

/* =========================================================
 * CAN_TX_DATA0
 * ========================================================= */

#define CAN_TX_DATA0_POS 0U
#define CAN_TX_DATA1_POS 8U
#define CAN_TX_DATA2_POS 16U
#define CAN_TX_DATA3_POS 24U

#define CAN_TX_DATA0_MASK (0xFFU << CAN_TX_DATA0_POS)
#define CAN_TX_DATA1_MASK (0xFFU << CAN_TX_DATA1_POS)
#define CAN_TX_DATA2_MASK (0xFFU << CAN_TX_DATA2_POS)
#define CAN_TX_DATA3_MASK (0xFFU << CAN_TX_DATA3_POS)

/* =========================================================
 * CAN_TX_DATA1
 * ========================================================= */

#define CAN_TX_DATA4_POS 0U
#define CAN_TX_DATA5_POS 8U
#define CAN_TX_DATA6_POS 16U
#define CAN_TX_DATA7_POS 24U

#define CAN_TX_DATA4_MASK (0xFFU << CAN_TX_DATA4_POS)
#define CAN_TX_DATA5_MASK (0xFFU << CAN_TX_DATA5_POS)
#define CAN_TX_DATA6_MASK (0xFFU << CAN_TX_DATA6_POS)
#define CAN_TX_DATA7_MASK (0xFFU << CAN_TX_DATA7_POS)

/* =========================================================
 * CAN_RX_MAILBOX
 * ========================================================= */

#define CAN_RX_RTR (1U << 0)
#define CAN_RX_IDE (1U << 1)

#define CAN_RX_ID_POS 2U
#define CAN_RX_DLC_POS 13U

#define CAN_RX_ID_MASK (0x7FFU << CAN_RX_ID_POS)
#define CAN_RX_DLC_MASK (0xFU << CAN_RX_DLC_POS)

/* =========================================================
 * CAN_RX_DATA0
 * ========================================================= */

#define CAN_RX_DATA0_POS 0U
#define CAN_RX_DATA1_POS 8U
#define CAN_RX_DATA2_POS 16U
#define CAN_RX_DATA3_POS 24U

#define CAN_RX_DATA0_MASK (0xFFU << CAN_RX_DATA0_POS)
#define CAN_RX_DATA1_MASK (0xFFU << CAN_RX_DATA1_POS)
#define CAN_RX_DATA2_MASK (0xFFU << CAN_RX_DATA2_POS)
#define CAN_RX_DATA3_MASK (0xFFU << CAN_RX_DATA3_POS)

/* =========================================================
 * CAN_RX_DATA1
 * ========================================================= */

#define CAN_RX_DATA4_POS 0U
#define CAN_RX_DATA5_POS 8U
#define CAN_RX_DATA6_POS 16U
#define CAN_RX_DATA7_POS 24U

#define CAN_RX_DATA4_MASK (0xFFU << CAN_RX_DATA4_POS)
#define CAN_RX_DATA5_MASK (0xFFU << CAN_RX_DATA5_POS)
#define CAN_RX_DATA6_MASK (0xFFU << CAN_RX_DATA6_POS)
#define CAN_RX_DATA7_MASK (0xFFU << CAN_RX_DATA7_POS)

/* =========================================================
 * CAN Bit Timing Configuration
 * ========================================================= */

typedef struct
{
	uint8_t brp;
	uint8_t tseg1;
	uint8_t tseg2;
	uint8_t sjw;

} CAN_BTR_t;

/* =========================================================
 * Initialization
 * CAN_SoftwareReset: Issue a software reset command.
 * CAN_Initialization: Reset the controller and apply bit timing.
 * ========================================================= */

void CAN_SoftwareReset(CAN_TypeDef_t *const can);
void CAN_Initialization(CAN_TypeDef_t *can, const CAN_BTR_t *btr);

/* =========================================================
 * Transmit
 * CAN_IsTxReady: Check whether the transmit mailbox is available.
 * CAN_SetTxMailbox: Configure a standard-ID transmit frame.
 * CAN_SetTxData: Copy payload bytes into the transmit registers.
 * CAN_SendFrame: Validate and start transmission of a frame.
 * CAN_TxStart: Set the transmit-valid control bit.
 * ========================================================= */

bool CAN_IsTxReady(const CAN_TypeDef_t *const can);
CAN_Status_t CAN_SetTxMailbox(CAN_TypeDef_t *const can, const CAN_Frame_t *const frame);
CAN_Status_t CAN_SetTxData(CAN_TypeDef_t *const can, const CAN_Frame_t *const frame);
CAN_Status_t CAN_SendFrame(CAN_TypeDef_t *const can, const CAN_Frame_t *const frame);
void CAN_TxStart(CAN_TypeDef_t *const can);

/* =========================================================
 * Receive
 * CAN_IsRxPending: Check whether a receive interrupt is pending.
 * CAN_ClearRxInterrupt: Clear the receive interrupt status.
 * CAN_GetRxMailbox: Decode the received identifier and control fields.
 * CAN_GetRxData: Copy received payload bytes into a frame.
 * CAN_ReadFrame: Read and acknowledge one pending frame.
 * ========================================================= */

bool CAN_IsRxPending(const CAN_TypeDef_t *const can);
void CAN_ClearRxInterrupt(CAN_TypeDef_t *const can);
CAN_Status_t CAN_GetRxMailbox(const CAN_TypeDef_t *const can, CAN_Frame_t *const frame);
CAN_Status_t CAN_GetRxData(const CAN_TypeDef_t *const can, CAN_Frame_t *const frame);
CAN_Status_t CAN_ReadFrame(CAN_TypeDef_t *const can, CAN_Frame_t *const frame);

/* =========================================================
 * Error Status
 * CAN_IsError: Check the controller error status bit.
 * CAN_IsBusOff: Check whether the controller is bus-off.
 * ========================================================= */

bool CAN_IsError(const CAN_TypeDef_t *const can);
bool CAN_IsBusOff(const CAN_TypeDef_t *const can);

#endif /* SRC_HAL_HAL_CAN_H_ */
