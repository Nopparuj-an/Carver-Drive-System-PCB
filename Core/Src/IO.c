#include "IO.h"
#include "gpio.h"
#include "adc.h"

void IO_read_write(IOtypedef *var) {
	// read mode switch
	uint8_t AUTO_CMD = HAL_GPIO_ReadPin(AUTO_CMD_GPIO_Port, AUTO_CMD_Pin);
	uint8_t MANUAL_CMD = HAL_GPIO_ReadPin(MANUAL_CMD_GPIO_Port, MANUAL_CMD_Pin);
	if ((AUTO_CMD && MANUAL_CMD) || (!AUTO_CMD && !MANUAL_CMD)) {
		var->DrivingMode = MODE_TRANSITION;
	} else if (AUTO_CMD) {
		var->DrivingMode = MODE_AUTO;
	} else {
		var->DrivingMode = MODE_MANUAL;
	}

	// read gear switch
	uint8_t P = HAL_GPIO_ReadPin(Gear_P_GPIO_Port, Gear_P_Pin);
	uint8_t R = HAL_GPIO_ReadPin(Gear_R_GPIO_Port, Gear_R_Pin);
	uint8_t N = HAL_GPIO_ReadPin(Gear_N_GPIO_Port, Gear_N_Pin);
	uint8_t D = HAL_GPIO_ReadPin(Gear_D_GPIO_Port, Gear_D_Pin);
	if (P + R + N + D == 1) {
		if (P) {
			var->Gear = GEAR_P;
		} else if (R) {
			var->Gear = GEAR_R;
		} else if (N) {
			var->Gear = GEAR_N;
		} else if (D) {
			var->Gear = GEAR_D;
		}
	}

	// read direction switch
	var->DrivingDirection = HAL_GPIO_ReadPin(DIR_SIG_GPIO_Port, DIR_SIG_Pin);

	// read brake input
	var->BrakeStatus = HAL_GPIO_ReadPin(BRAKE_SIG_GPIO_Port, BRAKE_SIG_Pin);
}

uint16_t ADC_buffer[4];
void IO_init_ADC_DMA(){
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_buffer, 4);
}
