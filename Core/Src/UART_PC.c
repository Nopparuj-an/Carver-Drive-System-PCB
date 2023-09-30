#include "UART_PC.h"
#include "IO.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// VARIABLES ======================================================================================

UART_HandleTypeDef *PChuart;
uint8_t PCRxBuffer[1];
char output[70];
extern IOtypedef IOVar;

#define MAX_CMD_SIZE 50
#define MAX_PARAM_SIZE 20
#define MAX_PARAMS_COUNT 5

char rxCommand[MAX_CMD_SIZE];
char rxParameters[MAX_PARAMS_COUNT][MAX_PARAM_SIZE];
int rxIndex = 0;

// FUNCTIONS ======================================================================================

void UART_PC_Set(UART_HandleTypeDef *huart) {
	PChuart = huart;
	HAL_UART_Receive_IT(PChuart, PCRxBuffer, 1);
}

void Send_UART_PC(void) {
	uint8_t tx_data[] = "Hello from STM32!\r\n";
	HAL_UART_Transmit(PChuart, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
}

void UART_PC_Callback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(PChuart, PCRxBuffer, 1);
	if (huart != PChuart) {
		return;
	}

	if (PCRxBuffer[0] == '\r') {
		// nothing
	} else if (PCRxBuffer[0] == '\n') {
		if (rxIndex > 0) {
			rxCommand[rxIndex] = '\0'; // Null-terminate the command string
			ProcessCommand(rxCommand);
			rxIndex = 0;
		}
	} else {
		if (rxIndex < MAX_CMD_SIZE - 1) {
			rxCommand[rxIndex] = PCRxBuffer[0];
			rxIndex++;
		}
	}

	HAL_UART_Receive_IT(PChuart, PCRxBuffer, 1);
}

void ProcessCommand(const char *command) {
	char *token;
	int paramIndex = 0;

	// Split the command into parameters
	token = strtok((char*) command, " ");
	while (token != NULL && paramIndex < MAX_PARAMS_COUNT) {
		strncpy(rxParameters[paramIndex], token, MAX_PARAM_SIZE - 1);
		rxParameters[paramIndex][MAX_PARAM_SIZE - 1] = '\0'; // Ensure null-termination
		paramIndex++;
		token = strtok(NULL, " ");
	}

	// Compare the command and perform actions
	if (strcmp(rxParameters[0], "1") == 0) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	} else if (strcmp(rxParameters[0], "RELAY") == 0) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
	} else if (strcmp(rxParameters[0], "STE") == 0) {
		IOVar.SteeringAngle = atoi(rxParameters[1]);
	}

//	sprintf(output, "PARAMETERS %d\r\n", paramIndex);
//	HAL_UART_Transmit(PChuart, (uint8_t*) output, strlen(output), HAL_MAX_DELAY);
//	sprintf(output, "%s %s %s %s %s\r\n", rxParameters[0], rxParameters[1], rxParameters[2], rxParameters[3], rxParameters[4]);
//	HAL_UART_Transmit(PChuart, (uint8_t*) output, strlen(output), HAL_MAX_DELAY);
}

void UART_PC_Streamer(IOtypedef *var) {
	static uint32_t next_run;
	static uint8_t iterator;
	if (HAL_GetTick() - next_run < 0) {
		return;
	}
	next_run = HAL_GetTick() + 20;
	iterator = (iterator + 1) % 2; // adjust this based on how many commands to be sent.

	switch (iterator) {
	case 0: // Control Mode
		if (var->DrivingMode == MODE_AUTO) {
			HAL_UART_Transmit(PChuart, (uint8_t*) "MOD A\n", 7, HAL_MAX_DELAY);
		} else {
			HAL_UART_Transmit(PChuart, (uint8_t*) "MOD M\n", 7, HAL_MAX_DELAY);
		}
		break;
	case 1: // Torque control signal
		snprintf(output, sizeof(output), "TRQ %.2f\n", var->Throttle);
		HAL_UART_Transmit(PChuart, (uint8_t*) output, strlen(output), HAL_MAX_DELAY);
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}
