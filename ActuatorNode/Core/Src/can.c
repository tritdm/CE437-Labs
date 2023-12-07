#include "can.h"

volatile uint8_t CANDataRcvFlag = 0;
uint8_t CANRxBuffer[CAN_DATA_LENGTH];
CAN_RxHeaderTypeDef CANRxHeader;

void CAN_Transmit(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *pHeader,
        		const uint8_t aData[], uint32_t *pTxMailbox)
{
	HAL_CAN_AddTxMessage(hcan, pHeader, aData, pTxMailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CANRxHeader, CANRxBuffer) != HAL_OK)
	{
		Error_Handler();
	}

	if (CANRxHeader.StdId == CAN_RX_STD_ID)
	{
		CANDataRcvFlag = 1;
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CANRxHeader, CANRxBuffer) != HAL_OK)
	{
		Error_Handler();
	}

	if (CANRxHeader.StdId == CAN_RX_STD_ID)
	{
		CANDataRcvFlag = 1;
	}
}
