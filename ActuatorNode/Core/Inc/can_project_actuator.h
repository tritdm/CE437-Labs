#ifndef __CAN_PROJECT_H__
#define __CAN_PROJECT_H__

#include "can_node_1.h"

#define CAN_PROJECT_ACTUATOR_STDID 	0x123
#define CAN_PROJECT_SENSOR_STDID 	0x321

/* format: 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX
 * 		   SEQ		 PRI  SPD  DIR*/
#define CAN_DATA_SEQ_IDX			0x00
#define CAN_SENSOR_DATA_PRI_IDX		0x02
#define CAN_SENSOR_DATA_SPEED_IDX	0x03
#define CAN_SENSOR_DATA_DIRECT_IDX	0x04

#define CAN_SPEED_MIN 				0x00
#define CAN_SPEED_NORMAL			0x28
#define CAN_SPEED_MAX 				0x3c
#define CAN_DIRECTION_FULL_LEFT		0x00
#define CAN_DIRECTION_FORWARD		0x2d
#define CAN_DIRECTION_FULL_RIGHT	0x5a

/* format: 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX 0xXX
 * 		   SEQ		 SPD */
#define CAN_ACTUATOR_DATA_SPEED_IDX	0x02

typedef enum
{
	CONTROL_PRIOR_URGENT = 0,
//	CONTROL_PRIOR
	CONTROL_PRIOR_NORMAL = 2
} CONTROL_PRIORITY;

typedef struct
{
	uint16_t sequence;
	uint8_t speed;
} CANActuatorData;

void CANActuatorResponse(CAN_HandleTypeDef *hcan, CANActuatorData* responseData);

#endif /* __CAN_PROJECT_H__ */
