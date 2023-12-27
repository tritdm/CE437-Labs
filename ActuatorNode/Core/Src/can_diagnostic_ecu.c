#include "can_diagnostic_ecu.h"
#include "can_node_1.h"
#include "gpio.h"

extern uint8_t CANRxBuffer[];
extern CAN_RxHeaderTypeDef CANRxHeader;
uint8_t CANDiagnosticRequestRcvFlag = 0;
uint8_t seed[4], key[16];

uint8_t readDataByIdentifierResponse(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *CANRxHeader, uint8_t CANRxData[])
{
	uint8_t CAN_PCI = CANRxData[0] >> 4;
	uint8_t CAN_SID = CANRxData[1];
	if ((CAN_PCI_SINGLE_FRAME != CAN_PCI) || (READ_DATA_BY_ID_SID != CAN_SID)) return 0;

	uint8_t CAN_DL = CANRxData[0] & 0x0f;
	uint16_t CAN_DID = (CANRxData[2] << 8) | (CANRxData[3] << 0);
	if (CAN_DL != 0x03)
	{
		CANNegativeResponse(hcan, CAN_PCI, READ_INVALID_LENGTH);
		return 1;
	}
//	if (CAN_DID != 0x0123)
//	{
//	}

	uint8_t CANTxResponse[READ_POSITIVE_RESPONSE_LENGTH];
	CAN_TxHeaderTypeDef CANTxHeaderResponse;
	uint32_t CANTxMailboxesResponse = CAN_TX_MAILBOX1;

	CANTxHeaderResponse.StdId 	= CAN_DIAGNOSTIC_RESPONSE_ID;
	CANTxHeaderResponse.IDE 	= CAN_ID_STD;
	CANTxHeaderResponse.RTR 	= CAN_RTR_DATA;
	CANTxHeaderResponse.DLC 	= READ_POSITIVE_RESPONSE_LENGTH;

	CANTxResponse[0] = CAN_SID + POSITIVE_RESPONSE_SID_ADD;
	CANTxResponse[1] = CANRxData[2];
	CANTxResponse[2] = CANRxData[3];
	CANTxResponse[3] = CANRxHeader->StdId >> 8;
	CANTxResponse[4] = CANRxHeader->StdId & 0xff;
	CANTxResponse[5] = UNUSED_DATA;
	CANTxResponse[6] = UNUSED_DATA;
	CANTxResponse[7] = UNUSED_DATA;

	CAN_Transmit(hcan, &CANTxHeaderResponse, CANTxResponse, &CANTxMailboxesResponse);

	HAL_GPIO_TogglePin(ACTUATOR_GPIO_PORT, LEDB_Pin);
	HAL_GPIO_TogglePin(ACTUATOR_GPIO_PORT, LEDR_Pin);
	HAL_GPIO_TogglePin(ACTUATOR_GPIO_PORT, LEDG_Pin);

	return 1;
}

uint8_t writeDataByIdentifierResponse(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *CANRxHeader, uint8_t CANRxData[])
{
	uint8_t CAN_PCI = CANRxData[0] >> 4;
	uint8_t CAN_SID = CANRxData[1];
	if ((CAN_PCI_SINGLE_FRAME != CAN_PCI) || (WRITE_DATA_BY_ID_SID != CAN_SID)) return 0;

	uint8_t CAN_DL = CANRxData[0] & 0x0f;
	uint16_t CAN_DID = (CANRxData[2] << 8) | (CANRxData[3] << 0);
	if (CAN_DL < 0x03)
	{
		CANNegativeResponse(hcan, CAN_PCI, WRITE_INVALID_LENGTH);
		return 1;
	}
	if (CAN_DID != 0x0123)
	{
		CANNegativeResponse(hcan, CAN_PCI, WRITE_DID_NOT_SUPPORT);
		return 1;
	}

	uint8_t CANTxResponse[WRITE_POSITIVE_RESPONSE_LENGTH];
	CAN_TxHeaderTypeDef CANTxHeaderResponse;
	uint32_t CANTxMailboxesResponse = CAN_TX_MAILBOX1;

	CANTxHeaderResponse.StdId 	= CAN_DIAGNOSTIC_RESPONSE_ID;
	CANTxHeaderResponse.IDE 	= CAN_ID_STD;
	CANTxHeaderResponse.RTR 	= CAN_RTR_DATA;
	CANTxHeaderResponse.DLC 	= WRITE_POSITIVE_RESPONSE_LENGTH;

	CANTxResponse[0] = CAN_SID + POSITIVE_RESPONSE_SID_ADD;
	CANTxResponse[1] = CANRxData[2];
	CANTxResponse[2] = CANRxData[3];
	CANTxResponse[3] = UNUSED_DATA;
	CANTxResponse[4] = UNUSED_DATA;
	CANTxResponse[5] = UNUSED_DATA;
	CANTxResponse[6] = UNUSED_DATA;
	CANTxResponse[7] = UNUSED_DATA;

	CAN_Transmit(hcan, &CANTxHeaderResponse, CANTxResponse, &CANTxMailboxesResponse);

	HAL_GPIO_TogglePin(ACTUATOR_GPIO_PORT, LEDB_Pin);
	HAL_GPIO_TogglePin(ACTUATOR_GPIO_PORT, LEDR_Pin);
	HAL_GPIO_TogglePin(ACTUATOR_GPIO_PORT, LEDG_Pin);

	return 1;
}

void securityAccessSeedGenerate(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *CANRxHeader, uint8_t CANRxData[])
{
	uint8_t CAN_PCI = CANRxData[0] >> 4;
	uint8_t CAN_SID = CANRxData[1];

	if ((CAN_PCI_SINGLE_FRAME != CAN_PCI) || (SECURITY_ACCESS_SID != CAN_SID)) return 0;

	uint8_t CANTxResponse[SECURITY_ACCESS_SEED_GEN_LENGTH];
	CAN_TxHeaderTypeDef CANTxHeaderResponse;
	uint32_t CANTxMailboxesResponse = CAN_TX_MAILBOX1;

	CANTxHeaderResponse.StdId 	= CAN_DIAGNOSTIC_RESPONSE_ID;
	CANTxHeaderResponse.IDE 	= CAN_ID_STD;
	CANTxHeaderResponse.RTR 	= CAN_RTR_DATA;
	CANTxHeaderResponse.DLC 	= SECURITY_ACCESS_SEED_GEN_LENGTH;

	CANTxResponse[0] = CAN_SID + POSITIVE_RESPONSE_SID_ADD;
	CANTxResponse[1] = SEED_LEVEL;
	srand(time(NULL));
	/* Generate SEED */
	CANTxResponse[2] = rand() % 256;
	CANTxResponse[3] = rand() % 256;
	CANTxResponse[4] = rand() % 256;
	CANTxResponse[5] = rand() % 256;
	CANTxResponse[6] = UNUSED_DATA;
	CANTxResponse[7] = UNUSED_DATA;

	CAN_Transmit(hcan, &CANTxHeaderResponse, CANTxResponse, &CANTxMailboxesResponse);

	/* Save SEED and calculate KEY*/
	seed[0] = CANTxResponse[2];
	seed[1] = CANTxResponse[3];
	seed[2] = CANTxResponse[4];
	seed[3] = CANTxResponse[5];

	key[0] 	= seed[0] ^ seed[1];
	key[1] 	= seed[1] + seed[2];
	key[2] 	= seed[2] ^ seed[3];
	key[3] 	= seed[3] + seed[0];

	key[4] 	= seed[0] | seed[1];
	key[5] 	= seed[1] + seed[2];
	key[6] 	= seed[2] | seed[3];
	key[7] 	= seed[3] + seed[1];

	key[8] 	= seed[0] & seed[1];
	key[9] 	= seed[1] ^ seed[2];
	key[10] = seed[2] & seed[3];
	key[11] = seed[3] ^ seed[0];

	key[12] = seed[0] - seed[1];
	key[13] = seed[1] ^ seed[2];
	key[14] = seed[2] - seed[3];
	key[15] = seed[3] ^ seed[0];

	return 1;
}

void flowControlResponse(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *CANRxHeader, uint8_t CANRxData[])
{

}

uint8_t CANNegativeResponse(CAN_HandleTypeDef *hcan, uint8_t CANPCI, uint8_t CANNegativeResponseCode)
{
	uint8_t CANTxResponse[NEGATIVE_RESPONSE_LENGTH];
	CAN_TxHeaderTypeDef CANTxHeaderResponse;
	uint32_t CANTxMailboxesResponse = CAN_TX_MAILBOX1;

	CANTxHeaderResponse.StdId 	= CAN_DIAGNOSTIC_RESPONSE_ID;
	CANTxHeaderResponse.IDE 	= CAN_ID_STD;
	CANTxHeaderResponse.RTR 	= CAN_RTR_DATA;
	CANTxHeaderResponse.DLC 	= NEGATIVE_RESPONSE_LENGTH;

	CANTxResponse[0] 	= NEGATIVE_RESPONSE;
	CANTxResponse[1] 	= CANPCI;
	CANTxResponse[2]	= CANNegativeResponseCode;
	CANTxResponse[3] 	= UNUSED_DATA;
	CANTxResponse[4] 	= UNUSED_DATA;
	CANTxResponse[5] 	= UNUSED_DATA;
	CANTxResponse[6] 	= UNUSED_DATA;
	CANTxResponse[7] 	= UNUSED_DATA;

	CAN_Transmit(hcan, &CANTxHeaderResponse, CANTxResponse, &CANTxMailboxesResponse);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CANRxHeader, CANRxBuffer) != HAL_OK)
	{
		Error_Handler();
	}

	if (CANRxHeader.StdId == CAN_DIAGNOSTIC_REQUEST_ID)
	{
		CANDiagnosticRequestRcvFlag = 1;
	}
}
