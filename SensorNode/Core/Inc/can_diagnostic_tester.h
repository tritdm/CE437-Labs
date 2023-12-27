#ifndef __CAN_DIAGNOSTIC_H__
#define __CAN_DIAGNOSTIC_H__

#include "main.h"

#define CAN_DIAGNOSTIC_REQUEST_ID		0x712
#define CAN_DIAGNOSTIC_RESPONSE_ID		0x7a2

#define CAN_PCI_SINGLE_FRAME 			0x00
#define CAN_PCI_FIRST_FRAME				0x01
#define CAN_PCI_CONSECUTIVE_FRAME		0x02
#define CAN_PCI_FLOW_CONTROL			0x03

#define READ_WRITE_REQUEST_LENGTH		0x04
#define UNUSED_DATA						0x55

#define NEGATIVE_RESPONSE				0x7f
#define NEGATIVE_RESPONSE_LENGTH 		0x03
#define POSITIVE_RESPONSE_SID_ADD		0x40
#define READ_DATA_BY_ID_REQUEST_SID 	0x22
#define READ_DATA_BY_ID_RESPONSE_SID 	0x62
#define READ_DATA_BY_ID_RECORD			0x0123
#define READ_POSITIVE_RESPONSE_LENGTH	0x05
#define WRITE_DATA_BY_ID_REQUEST_SID	0x2e
#define WRITE_DATA_BY_ID_RESPONSE_SID	0x6e
#define WRITE_DATA_BY_ID_RECORD			0x0123
#define WRITE_POSITIVE_RESPONSE_LENGTH	0x01
#define SECURITY_ACCESS_REQUEST_SID		0x27
#define SECURITY_ACCESS_RESPONSE_SID	0x67
#define CAN_SEED_REQUEST_LENGTH	 		0x02
#define CAN_UNLOCK_REQUEST_LENGTH	 	0x12
#define SEED_LEVEL						0x01
#define KEY_LEVEL						0x02
//#define SERCU_POSITIVE_RESPONSE_LENGTH	0x01

#define READ_INVALID_LENGTH				0x13
#define WRITE_INVALID_LENGTH			0x13
#define WRITE_DID_NOT_SUPPORT			0x31
#define SECURITY_INVALID_LENGTH			0x13
#define SECURITY_INVALID_KEY			0x35

void writeShortDataByIdentifierRequest(CAN_HandleTypeDef *hcan);

void readDataByIdenfierRequest(CAN_HandleTypeDef *hcan);

void writeDataByIdenfierResponseCheck(uint8_t CANRxBuffer[]);

void readDataByIdenfierResponseCheck(uint8_t CANRxBuffer[]);

void securityAccessSeedRequest(CAN_HandleTypeDef *hcan);

uint8_t securityAccessSeedResponseCheck(uint8_t CANRxBuffer[]);

void securityRemainKeySend(CAN_HandleTypeDef *hcan);

uint8_t flowControlCheck(uint8_t CANRxData[]);

void securityAccessUnlockRequest(CAN_HandleTypeDef *hcan);

#endif /* __CAN_DIAGNOSTIC_H__ */
