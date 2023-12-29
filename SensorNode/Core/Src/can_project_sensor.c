#include "can_project_sensor.h"

extern uint8_t CANRxBuffer[];
extern CAN_RxHeaderTypeDef CANRxHeader;


void CANSensorTransmit(CAN_HandleTypeDef *hcan, CANSensorData* controlData)
{
	uint8_t CANTxControl[CAN_DATA_LENGTH];
	CAN_TxHeaderTypeDef CANTxHeaderControl;
	uint32_t CANTxMailboxesRequest = CAN_TX_MAILBOX1;

	CANTxHeaderControl.StdId = CAN_PROJECT_ACTUATOR_STDID;
	CANTxHeaderControl.IDE 	 = CAN_ID_STD;
	CANTxHeaderControl.RTR 	 = CAN_RTR_DATA;
	CANTxHeaderControl.DLC 	 = CAN_DATA_LENGTH;

	CANTxControl[CAN_DATA_SEQ_IDX] 			= (controlData->sequence >> 8) & 0xff;
	CANTxControl[CAN_DATA_SEQ_IDX+1] 		= (controlData->sequence) & 0xff;
	CANTxControl[CAN_SENSOR_DATA_PRI_IDX] 	= controlData->priority;
	CANTxControl[CAN_SENSOR_DATA_SPEED_IDX]	= controlData->speed;
	CANTxControl[CAN_SENSOR_DATA_DIRECT_IDX]= controlData->direction;
//	CANTxControl[4]		= UNUSED_DATA;
//	CANTxControl[5]		= UNUSED_DATA;
//	CANTxControl[6]		= UNUSED_DATA;
//	CANTxControl[7]		= UNUSED_DATA;

	CAN_Transmit(hcan, &CANTxHeaderRequest, CANTxRequest, &CANTxMailboxesRequest);
	HAL_GPIO_TogglePin(GPIO_Port, LEDR_Pin);
}

void CANResponseCheck()
{

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CANRxHeader, CANRxBuffer) != HAL_OK)
	{
		Error_Handler();
	}

	if (CANRxHeader.StdId == CAN_PROJECT_ACTUATOR_STDID)
	{
		CANDataRcvFlag = 1;
	}
}
