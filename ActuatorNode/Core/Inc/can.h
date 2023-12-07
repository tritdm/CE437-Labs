#ifndef __CAN_H__
#define __CAN_H__

#include "main.h"

#define CAN_TX_STD_ID 	0x0a2
#define CAN_RX_STD_ID 	0x012
#define CAN_DATA_LENGTH 0x08

void CAN_Transmit(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *pHeader,
        		const uint8_t aData[], uint32_t *pTxMailbox);

#endif /* __CAN_H__ */
