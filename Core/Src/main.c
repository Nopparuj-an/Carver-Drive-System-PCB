/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "UART_PC.h"
#include "IO.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// TEMP
#define AMT21_READ_POS 0xD4 + 0x00
#define AMT21_EXT_CMD 0xD4 + 0x02 // Begin Extended Command
#define AMT21_SET_ZERO 0x5E

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

IOtypedef IOVar = { 0 };
extern IOtypedef IOVar;

// TEMP
uint8_t RxBuffer[2];
int32_t Rawpos = 0;
float Kp = 6.0;
float Ki = 2.0;
float Kd = 0.0;
int32_t error_summa;
float control_dt = (1.0 / 500.0);
int32_t duty;
int32_t pwm = 0;
int32_t pos_setpoint = 0;
uint32_t encoder_error = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// TEMP
void amt21_set_zero_pos();
int16_t amt21_get_pos();
int16_t controller(int pos_current);
void setMotor(int PWM);
uint8_t check_parity(uint16_t response);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_I2C2_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	UART_PC_Set(&huart1);
	IO_init_ADC_DMA();

	// TEMP

	HAL_TIM_Base_Start(&htim2); //Motor
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //LPWM
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //RPWM
	setMotor(0);
	IOVar.SteeringSetpoint = 3950;

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)) {
		amt21_set_zero_pos();
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		IO_read_write(&IOVar);
		UART_PC_Streamer(&IOVar);

		//TEMP
		pos_setpoint = IOVar.SteeringSetpoint;

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		static uint32_t timestamp = 0;
		if (HAL_GetTick() >= timestamp) {
			timestamp = HAL_GetTick() + 2;
			Rawpos = amt21_get_pos();
			IOVar.SteeringAngleRaw = Rawpos;
			pwm = controller(Rawpos);
			if (encoder_error < 10) {
				setMotor(pwm);
			} else {
				setMotor(0);
			}
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart1) {
		UART_PC_Callback(huart);
	}
}

// TEMP
int16_t amt21_get_pos() {
	static int16_t L_pos = 4500;
	uint8_t cmd[1] = { AMT21_READ_POS };

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); // send
	HAL_UART_Transmit(&huart2, cmd, 1, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // receive
	HAL_UART_DMAStop(&huart2); // clear UART RX
	HAL_UART_Receive_DMA(&huart2, (uint8_t*) RxBuffer, 2);

	uint16_t pos = ((RxBuffer[1] << 8) | RxBuffer[0]);

	if (check_parity(pos)) {
		// parity correct
		L_pos = pos & 0x3FFFu;

		L_pos = 16384 - L_pos;

		if (L_pos > 12000) {
			L_pos -= 16384;
		}
		RxBuffer[0] = 0;
		RxBuffer[1] = 0;
		encoder_error = 0;
	} else {
		// parity incorrect
		encoder_error++;
	}

	return L_pos;
}

uint8_t check_parity(uint16_t response) {
	// Verify parameters
	if (response == 0)
		return 0;

	// Check odd bits parity (odd parity)
	int k1 = (response >> 15) & 0x01;
	int odd_checkbit = !(((response >> 1) & 0x01) ^ ((response >> 3) & 0x01) ^ ((response >> 5) & 0x01) ^ ((response >> 7) & 0x01)
			^ ((response >> 9) & 0x01) ^ ((response >> 11) & 0x01) ^ ((response >> 13) & 0x01));
	if (odd_checkbit != k1)
		return 0;

	// Check even bits parity (odd parity)
	int k0 = (response >> 14) & 0x01;
	int even_checkbit = !(((response >> 0) & 0x01) ^ ((response >> 2) & 0x01) ^ ((response >> 4) & 0x01) ^ ((response >> 6) & 0x01)
			^ ((response >> 8) & 0x01) ^ ((response >> 10) & 0x01) ^ ((response >> 12) & 0x01));
	if (even_checkbit != k0)
		return 0;

	return 1;
}

void amt21_set_zero_pos() {
	uint8_t cmd[2] = { AMT21_EXT_CMD, AMT21_SET_ZERO };
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2, cmd, 2, 1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_Delay(20);
}

void setMotor(int PWM) {

	if (PWM >= 0) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	}

	else if (PWM < 0) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -1 * PWM);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	}
}

int16_t controller(int pos_current) {
	if (pos_current > 11500 || pos_current < -10) {
		return 0;
	}
	float u = 0;
	if (IOVar.DrivingMode == MODE_AUTO) {
		int error_pos = pos_setpoint - pos_current;
		int error_delta = error_pos / control_dt;
		error_summa += error_pos * control_dt;
		u = Kp * error_pos + Kd * error_delta + Ki * error_summa;
		if (error_pos <= 3 && error_pos >= -3) {
			error_summa = 0;
			return 0;
		}
		if (u >= 3000) {
			u = 3000;
		} else if (u <= -3000) {
			u = -3000;
		}

	} else if (IOVar.DrivingMode == MODE_MANUAL) {
		u = 0;
	}
	return u;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
