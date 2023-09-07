#ifndef INC_STEERING_CAN_H_
#define INC_STEERING_CAN_H_

#include "can.h"
#include "stdlib.h"
#include "string.h"

extern uint8_t RxData[];
extern CAN_RxHeaderTypeDef RxHeader;

typedef enum
{
	Disable,
	Enable,
	Control
} CAN_BUS_State;

void Steering_Init(CAN_HandleTypeDef *hcanx, uint8_t Slave_ID);
void Steering_Position_Control(CAN_HandleTypeDef *hcanx, int16_t target_pos, uint8_t enable);
void Steering_Position_Reset(CAN_HandleTypeDef *hcanx);
uint8_t Steering_HeartBeat(CAN_HandleTypeDef *hcanx, uint8_t Slave_ID);


#endif /* INC_STEERING_CAN_H_ */

/* ioc setup */

// Time Quanta in Bit Segment 1 = 2
// Test Mode = Normal

/* nvic setting */
// CAN_RX0_Interrupts = true

/* Put this Code in file "main.c" */

/* USER CODE BEGIN 2 */
//Steering_Init(&hcan, 1);
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	Steering_HeartBeat(hcan, 1);
//}
/* USER CODE END 4 */
