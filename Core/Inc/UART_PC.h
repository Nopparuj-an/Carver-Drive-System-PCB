#ifndef INC_UART_PC_H_
#define INC_UART_PC_H_

#include "usart.h"

// This is a function used to setup UART handler.
void UART_PC_Set(UART_HandleTypeDef *huart);

// This is a prototype function used to test transmission of UART data.
void SendUART_PC();

// This function is called on every incoming UART receive to store a string of commands.
void UART_PC_Callback(UART_HandleTypeDef *huart);

// This is an internal function used by UART_PC_Callback to process incoming command.
void ProcessCommand(const char *command);

// This function handle streaming control signal and other information to PC.
void UART_PC_Streamer();

#endif /* INC_UART_PC_H_ */
