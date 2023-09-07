#include "steering_can.h"

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

CAN_BUS_State SteeringState = Disable;
CAN_BUS_State PrevState = Disable;

uint32_t TxMailbox;

uint8_t TxData[8] = {0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
uint8_t RxData[8];
uint8_t PrevData[8]= {0};

int16_t feedback_angle;

void Steering_Init(CAN_HandleTypeDef *hcanx, uint8_t Slave_ID)
{
	  CAN_FilterTypeDef can_filt_conf;
	  can_filt_conf.FilterActivation = CAN_FILTER_ENABLE;
	  can_filt_conf.FilterBank = 10;
	  can_filt_conf.FilterFIFOAssignment = CAN_RX_FIFO0;
	  can_filt_conf.FilterIdHigh = 0;
	  can_filt_conf.FilterIdLow = 0x0000;
	  can_filt_conf.FilterMaskIdHigh = 0;
	  can_filt_conf.FilterMaskIdLow = 0x0000;
	  can_filt_conf.FilterMode = CAN_FILTERMODE_IDMASK;
	  can_filt_conf.FilterScale = CAN_FILTERSCALE_32BIT;
	  can_filt_conf.SlaveStartFilterBank = 13;
	  HAL_CAN_ConfigFilter(hcanx, &can_filt_conf);

	  HAL_CAN_Start(hcanx);
	  HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING);
	  TxHeader.DLC = 8;
	  TxHeader.ExtId = 0x06000000 + Slave_ID;
	  TxHeader.IDE = CAN_ID_EXT;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.TransmitGlobalTime = DISABLE;
}

void Steering_Position_Control(CAN_HandleTypeDef *hcanx, int16_t target_pos, uint8_t enable)
{
	int16_t command;
	HAL_StatusTypeDef status;
	switch(SteeringState)
	{
	case Disable:
		TxData[1] = 0x0C;
		TxData[4] = 0x0000;
		TxData[5] = 0x0000;
		TxData[6] = 0x0000;
		TxData[7] = 0x0000;
		if (enable) SteeringState = Enable;
		break;
	case Enable:
		TxData[1] = 0x0D;
		if (enable) {
			SteeringState = Control;
			target_pos = feedback_angle;
		} else SteeringState = Disable;
		break;
	case Control:
		TxData[1] = 0x02;
		command = abs(target_pos);
		TxData[4] = (command & 0xFF00) >> 8;
		TxData[5] = command & 0x00FF;
		TxData[6] = 0x0000;
		TxData[7] = 0x0000;
		if (target_pos < 0) {
			TxData[4] = ~TxData[4];
			TxData[5] = ~TxData[5];
			TxData[6] = ~TxData[6];
			TxData[7] = ~TxData[7];
		}
		if (!enable) SteeringState = Enable;
	}
	if (HAL_CAN_AddTxMessage(hcanx, &TxHeader, TxData, &TxMailbox) == HAL_ERROR) {
		SteeringState = PrevState;
	}
	PrevState = SteeringState;
}

uint8_t Steering_HeartBeat(CAN_HandleTypeDef *hcanx, uint8_t Slave_ID)
{
	HAL_CAN_GetRxMessage(hcanx, CAN_RX_FIFO0, &RxHeader, RxData);
	if (RxHeader.ExtId == 0x07000000 + Slave_ID)
	{
		feedback_angle = ((int16_t) (RxData[0] << 8 | RxData[1])) * 10000 / 360;
		return 1;
	}
	return 0;
}

void Steering_Position_Reset(CAN_HandleTypeDef *hcanx)
{
	TxData[1] = 0x0C;
	TxData[3] = 0x09;
	TxData[4] = 0x0000;
	TxData[5] = 0x0000;
	TxData[6] = 0x0000;
	TxData[7] = 0x0000;
	HAL_CAN_AddTxMessage(hcanx, &TxHeader, TxData, &TxMailbox);
	TxData[3] = 0x01;
}
