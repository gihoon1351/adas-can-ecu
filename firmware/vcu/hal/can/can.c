#include "can.h"
#include <stddef.h>

/* =========================================================
 * Bit Timing
 * ========================================================= */

static uint32_t CAN_MakeBrp(uint8_t brp)
{
	return (((uint32_t)brp << CAN_BRP_POS) & CAN_BRP_MASK);
}

static uint32_t CAN_MakeTseg1(uint8_t tseg1)
{
	return (((uint32_t)tseg1 << CAN_TSEG1_POS) & CAN_TSEG1_MASK);
}

static uint32_t CAN_MakeTseg2(uint8_t tseg2)
{
	return (((uint32_t)tseg2 << CAN_TSEG2_POS) & CAN_TSEG2_MASK);
}

static uint32_t CAN_MakeSjw(uint8_t sjw)
{
	return (((uint32_t)sjw << CAN_SJW_POS) & CAN_SJW_MASK);
}

static void CAN_SetBitTiming(CAN_TypeDef_t *const can, const CAN_BTR_t *btr)
{
	uint32_t value;

	value = CAN_MakeBrp(btr->brp);
	value |= CAN_MakeTseg1(btr->tseg1);
	value |= CAN_MakeTseg2(btr->tseg2);
	value |= CAN_MakeSjw(btr->sjw);

	can->CAN_BTR = value;
}

/* =========================================================
 * Initialization
 * ========================================================= */

void CAN_SoftwareReset(CAN_TypeDef_t *const can)
{
	can->CAN_CR = CAN_SW_RESET;
}

void CAN_Initialization(CAN_TypeDef_t *can, const CAN_BTR_t *btr)
{
	if ((can == NULL) || (btr == NULL)) {
		return;
	}

	CAN_SoftwareReset(can);

	CAN_SetBitTiming(can, btr);
}

void CAN_TxStart(CAN_TypeDef_t *const can)
{
	can->CAN_CR = CAN_TX_VALID;
}

/* =========================================================
 * Status
 * ========================================================= */

bool CAN_IsTxReady(const CAN_TypeDef_t *const can)
{
	uint32_t status;

	status = can->CAN_SR;

	return ((status & CAN_TX_READY) != 0U);
}

bool CAN_IsError(const CAN_TypeDef_t *const can)
{
	uint32_t status;

	status = can->CAN_SR;

	return ((status & CAN_ERROR) != 0U);
}

bool CAN_IsBusOff(const CAN_TypeDef_t *const can)
{
	uint32_t status;

	status = can->CAN_SR;

	return ((status & CAN_BUS_OFF) != 0U);
}

uint16_t CAN_GetTEC(const CAN_TypeDef_t *const can)
{
	if (can == NULL) {
		return 0U;
	}
	return (uint16_t)((can->CAN_SR & CAN_TEC_MASK) >> CAN_TEC_POS);
}

uint8_t CAN_GetLastErrorState(const CAN_TypeDef_t *const can)
{
	if (can == NULL) {
		return 0U;
	}
	return (uint8_t)((can->CAN_SR & CAN_LAST_ERROR_STATE_MASK) >> CAN_LAST_ERROR_STATE_POS);
}

/* =========================================================
 * Transmit
 * ========================================================= */

static uint32_t CAN_MakeTxId(uint16_t id)
{
	return (((uint32_t)id << CAN_TX_ID_POS) & CAN_TX_ID_MASK);
}

static uint32_t CAN_MakeTxDlc(uint8_t dlc)
{
	return (((uint32_t)dlc << CAN_TX_DLC_POS) & CAN_TX_DLC_MASK);
}

static uint32_t CAN_MakeTxIde(bool ide)
{
	uint32_t value = 0U;

	if (ide != false) {
		value = CAN_TX_IDE;
	}

	return value;
}

static uint32_t CAN_MakeTxRtr(bool rtr)
{
	uint32_t value = 0U;

	if (rtr != false) {
		value = CAN_TX_RTR;
	}

	return value;
}

CAN_Status_t CAN_SetTxMailbox(CAN_TypeDef_t *const can, const CAN_Frame_t *const frame)
{
	uint32_t value;

	if ((can == NULL) || (frame == NULL)) {
		return CAN_STATUS_INVALID_PARAMETER;
	}

	if ((frame->id > CAN_STANDARD_ID_MAX) || (frame->dlc > CAN_DLC_MAX)) {
		return CAN_STATUS_INVALID_PARAMETER;
	}

	if (frame->ide != false) {
		return CAN_STATUS_INVALID_PARAMETER;
	}

	value = CAN_MakeTxId(frame->id);
	value |= CAN_MakeTxDlc(frame->dlc);
	value |= CAN_MakeTxIde(frame->ide);
	value |= CAN_MakeTxRtr(frame->rtr);

	can->CAN_TX_MAILBOX = value;

	return CAN_STATUS_OK;
}

static uint32_t CAN_MakeTxDataByte(uint8_t data, uint8_t position)
{
	return ((uint32_t)data << position);
}

CAN_Status_t CAN_SetTxData(CAN_TypeDef_t *const can, const CAN_Frame_t *const frame)
{
	uint32_t data0 = 0U;
	uint32_t data1 = 0U;

	if ((can == NULL) || (frame == NULL)) {
		return CAN_STATUS_INVALID_PARAMETER;
	}

	if (frame->dlc > CAN_DLC_MAX) {
		return CAN_STATUS_INVALID_PARAMETER;
	}

	if (frame->rtr == false) {

		if (frame->dlc > 0U) {
			data0 |= CAN_MakeTxDataByte(frame->data[0], CAN_TX_DATA0_POS);
		}

		if (frame->dlc > 1U) {
			data0 |= CAN_MakeTxDataByte(frame->data[1], CAN_TX_DATA1_POS);
		}

		if (frame->dlc > 2U) {
			data0 |= CAN_MakeTxDataByte(frame->data[2], CAN_TX_DATA2_POS);
		}

		if (frame->dlc > 3U) {
			data0 |= CAN_MakeTxDataByte(frame->data[3], CAN_TX_DATA3_POS);
		}

		if (frame->dlc > 4U) {
			data1 |= CAN_MakeTxDataByte(frame->data[4], CAN_TX_DATA4_POS);
		}

		if (frame->dlc > 5U) {
			data1 |= CAN_MakeTxDataByte(frame->data[5], CAN_TX_DATA5_POS);
		}

		if (frame->dlc > 6U) {
			data1 |= CAN_MakeTxDataByte(frame->data[6], CAN_TX_DATA6_POS);
		}

		if (frame->dlc > 7U) {
			data1 |= CAN_MakeTxDataByte(frame->data[7], CAN_TX_DATA7_POS);
		}
	}

	can->CAN_TX_DATA0 = data0;
	can->CAN_TX_DATA1 = data1;

	return CAN_STATUS_OK;
}

CAN_Status_t CAN_SendFrame(CAN_TypeDef_t *const can, const CAN_Frame_t *const frame)
{
	CAN_Status_t status;

	if ((can == NULL) || (frame == NULL)) {
		return CAN_STATUS_INVALID_PARAMETER;
	}

	if ((frame->id > CAN_STANDARD_ID_MAX) || (frame->dlc > CAN_DLC_MAX)) {
		return CAN_STATUS_INVALID_PARAMETER;
	}

	if (frame->ide != false) {
		return CAN_STATUS_INVALID_PARAMETER;
	}

	if ((CAN_IsBusOff(can) != false) || (CAN_IsError(can) != false)) {
		return CAN_STATUS_ERROR;
	}

	if (CAN_IsTxReady(can) == false) {
		return CAN_STATUS_BUSY;
	}

	status = CAN_SetTxMailbox(can, frame);

	if (status != CAN_STATUS_OK) {
		return status;
	}

	status = CAN_SetTxData(can, frame);

	if (status != CAN_STATUS_OK) {
		return status;
	}

	CAN_TxStart(can);

	return CAN_STATUS_OK;
}

/* =========================================================
 * Receive
 * ========================================================= */

bool CAN_IsRxPending(const CAN_TypeDef_t *const can)
{
	uint32_t status;

	status = can->CAN_SR;

	return ((status & CAN_RX_INT) != 0U);
}

void CAN_ClearRxInterrupt(CAN_TypeDef_t *const can)
{
	can->CAN_SR = CAN_RX_INT;
}

CAN_Status_t CAN_GetRxMailbox(const CAN_TypeDef_t *const can, CAN_Frame_t *const frame)
{
	uint32_t mailbox;
	uint8_t dlc;

	if ((can == NULL) || (frame == NULL)) {
		return CAN_STATUS_INVALID_PARAMETER;
	}

	mailbox = can->CAN_RX_MAILBOX;

	dlc = (uint8_t)((mailbox & CAN_RX_DLC_MASK) >> CAN_RX_DLC_POS);

	if (dlc > CAN_DLC_MAX) {
		return CAN_STATUS_ERROR;
	}

	frame->id = (uint16_t)((mailbox & CAN_RX_ID_MASK) >> CAN_RX_ID_POS);

	frame->dlc = dlc;

	frame->ide = ((mailbox & CAN_RX_IDE) != 0U);

	frame->rtr = ((mailbox & CAN_RX_RTR) != 0U);

	if (frame->ide != false) {
		return CAN_STATUS_ERROR;
	}

	return CAN_STATUS_OK;
}

static uint8_t CAN_GetRxDataByte(uint32_t value, uint32_t mask, uint8_t position)
{
	return (uint8_t)((value & mask) >> position);
}

CAN_Status_t CAN_GetRxData(const CAN_TypeDef_t *const can, CAN_Frame_t *const frame)
{
	uint32_t data0;
	uint32_t data1;
	uint8_t index;

	if ((can == NULL) || (frame == NULL)) {
		return CAN_STATUS_INVALID_PARAMETER;
	}

	if (frame->dlc > CAN_DLC_MAX) {
		return CAN_STATUS_ERROR;
	}

	data0 = can->CAN_RX_DATA0;
	data1 = can->CAN_RX_DATA1;

	for (index = 0U; index < CAN_DLC_MAX; index++) {
		frame->data[index] = 0U;
	}

	if (frame->rtr == false) {
		if (frame->dlc > 0U) {
			frame->data[0] = CAN_GetRxDataByte(data0, CAN_RX_DATA0_MASK, CAN_RX_DATA0_POS);
		}

		if (frame->dlc > 1U) {
			frame->data[1] = CAN_GetRxDataByte(data0, CAN_RX_DATA1_MASK, CAN_RX_DATA1_POS);
		}

		if (frame->dlc > 2U) {
			frame->data[2] = CAN_GetRxDataByte(data0, CAN_RX_DATA2_MASK, CAN_RX_DATA2_POS);
		}

		if (frame->dlc > 3U) {
			frame->data[3] = CAN_GetRxDataByte(data0, CAN_RX_DATA3_MASK, CAN_RX_DATA3_POS);
		}

		if (frame->dlc > 4U) {
			frame->data[4] = CAN_GetRxDataByte(data1, CAN_RX_DATA4_MASK, CAN_RX_DATA4_POS);
		}

		if (frame->dlc > 5U) {
			frame->data[5] = CAN_GetRxDataByte(data1, CAN_RX_DATA5_MASK, CAN_RX_DATA5_POS);
		}

		if (frame->dlc > 6U) {
			frame->data[6] = CAN_GetRxDataByte(data1, CAN_RX_DATA6_MASK, CAN_RX_DATA6_POS);
		}

		if (frame->dlc > 7U) {
			frame->data[7] = CAN_GetRxDataByte(data1, CAN_RX_DATA7_MASK, CAN_RX_DATA7_POS);
		}
	}

	return CAN_STATUS_OK;
}

CAN_Status_t CAN_ReadFrame(CAN_TypeDef_t *const can, CAN_Frame_t *const frame)
{
	CAN_Status_t status;

	if ((can == NULL) || (frame == NULL)) {
		return CAN_STATUS_INVALID_PARAMETER;
	}

	if (CAN_IsRxPending(can) == false) {
		return CAN_STATUS_NO_DATA;
	}

	status = CAN_GetRxMailbox(can, frame);

	if (status != CAN_STATUS_OK) {
		CAN_ClearRxInterrupt(can);
		return status;
	}

	status = CAN_GetRxData(can, frame);

	CAN_ClearRxInterrupt(can);

	if (status != CAN_STATUS_OK) {
		return status;
	}

	return CAN_STATUS_OK;
}
