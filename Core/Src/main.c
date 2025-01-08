/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "I2C_LCD.h"
#include "I2C_LCD_cfg.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <menu.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union {
	uint32_t counter;
	volatile uint32_t counterPrev;
	volatile int16_t position;
} EncoderParam;

typedef struct {
	float kp;
	float ki;
	float previousError;
	float integral;
} PIDController;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
float action = 0;
float temp = 15;
PIDController pid = { 0.1, 0.01, 0, 0 };

float setTemp = 20;
uint8_t programCountdown = 0;
uint16_t setTime = 0;

// uint32_t counter = 0;
int pulseTim1 = 500;
uint32_t up = 0;
uint32_t down = 0;
int32_t position = 0;
char buffer[17];
uint32_t counter = 0;


// Debounc
volatile uint32_t tickCounter = 0;
volatile uint8_t debounceActive = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* Private Function Prototypes Declaration*/
void pwmRamp(void);
void encoderCallback(int8_t direction);
void updateLCD(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Function Definition*/

void updateLCD(void) {

	MenuItem *pMenuItem = getCurrentMenuItem();
	size_t lengthName = strlen(pMenuItem->name);
	char buffer[17] = "";

	if (pMenuItem) {

		if (strcmp(pMenuItem->action, "time") == 0) {
			sprintf(buffer, "%d:%02d", pMenuItem->value / 60,
					pMenuItem->value % 60);
			setTime = pMenuItem->value;
		} else if (strcmp(pMenuItem->action, "temp") == 0) {
			setTemp = pMenuItem->value;
			sprintf(buffer, "%d %cC", pMenuItem->value, 0xDF);
		} else if (strcmp(pMenuItem->action, "start") == 0) {
			programCountdown = 1;
		}

		I2C_LCD_SetCursor(0, 0, 0);
		I2C_LCD_WriteString(0, "                ");
		I2C_LCD_SetCursor(0, 0, 0);
		I2C_LCD_WriteString(0, pMenuItem->name);
		I2C_LCD_SetCursor(0, lengthName + 1, 0);
		I2C_LCD_WriteString(0, buffer);
	} else {
		I2C_LCD_SetCursor(0, 0, 0);
		I2C_LCD_WriteString(0, "ERROR");
	}
}

// Clasic thermistor measure
float calcThermistorTemp(void) {
	const uint16_t adcResolution = 4095;
	//	uint16_t referenceVoltage = 3238;  // mV
	const uint16_t serialResistance = 10570; // ohm
	uint16_t rawAdc = 0;

	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		rawAdc = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);

	// avoid divide by zero
	if (rawAdc == 0)
		return -999;

	// convert rawAdc to resistance of thermistor with 10k
	float resistance = serialResistance * rawAdc / (adcResolution - rawAdc);

	//	 Steinhart-Hart equation
	float temp = log(resistance);
	temp = 1.0
			/ (0.001129148
					+ (0.000234125 + (0.0000000876741 * temp * temp)) * temp);
	temp = temp - 273.15;

	// calibration
	//	temp = temp * 1; // gain
	temp = temp - 5; // offset

	return temp;
}

// calculate PI regulation action
float PI_Compute(PIDController *pid, float setpoint, float measuredValue) {
	const float dt = 0.5;
	float errorLimit = 60;

	float error = setpoint - measuredValue;

	// P
	float proportional = pid->kp * error;

	// I

	pid->integral += error * dt;

	if (pid->integral > errorLimit) {
		pid->integral = errorLimit;
	} else if (pid->integral < -errorLimit) {
		pid->integral = -errorLimit;
	}

	float integral = pid->ki * pid->integral;

	//	pid->integral += stepError;
	//	integral = pid->ki * pid->integral;

	// D
	// Float derivative = (error - pid->previous_error) / dt;

	/* Write to dispaly
	 I2C_LCD_SetCursor(0, 0, 0);
	 int temp_int = (int) (proportional * 10);
	 sprintf(buffer, "%d.%d ", temp_int / 10, abs(temp_int % 10));
	 I2C_LCD_WriteString(0, buffer);

	 I2C_LCD_SetCursor(0, 7, 0);
	 int integral_int = (int) (integral * 100);
	 sprintf(buffer, "%d.%02d ", integral_int / 100, abs(integral_int % 100));
	 I2C_LCD_WriteString(0, buffer);

	 int error_int = (int) (error * 100);
	 sprintf(buffer, "%d.%02d", error_int / 100, abs(error_int % 100));
	 I2C_LCD_WriteString(0, buffer);
	 */
	float output = proportional + integral;
	// Saturate output to range -1 to 1
	if (output > 1.0f) {
		output = 1.0f;
	} else if (output < -1.0f) {
		output = -1.0f;
	}

	return output;
}

// DH11 temp. and humid. measure
void DELAY_US(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (__HAL_TIM_GET_COUNTER(&htim4) < us)
		;
}

void pwmRamp(void) {
	for (int dutyCycle = 0; dutyCycle <= 100; dutyCycle++) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulseTim1 * dutyCycle);
		HAL_Delay(20);
	}
	for (int dutyCycle = 100; dutyCycle >= 0; dutyCycle--) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulseTim1 * dutyCycle);
		HAL_Delay(20);
	}
}

// TIM Libraries

// Encoder callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static GPIO_PinState A, B, prevA = GPIO_PIN_SET, prevB = GPIO_PIN_SET;

	if (programCountdown == 0) {
		if (GPIO_Pin == GPIO_PIN_0) {
			// CCW decoder
			B = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

			if (B != prevB) {
				if (B == GPIO_PIN_SET) {
					if (A == GPIO_PIN_RESET) {
						moveSibling(1);
						HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
						updateLCD();
//						setTemp++;
					}
				}
				prevB = B;
			}
		}

		if (GPIO_Pin == GPIO_PIN_1) {
			// CCW decoder
			A = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

			if (A != prevA) {
				if (A == GPIO_PIN_SET) {
					if (B == GPIO_PIN_RESET) {
						moveSibling(-1);
						HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
						updateLCD();
//						setTemp--;
					}
				}
				prevA = A;
			}
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		if (!debounceActive && programCountdown == 0) {
			if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				moveParentChild(1);
				updateLCD();
			}
			if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				moveParentChild(-1);
				updateLCD();
			}
			debounceActive = 1;
			tickCounter = 0;
		}
	} else if (htim->Instance == TIM3) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM3) {
		tickCounter++;
		if (tickCounter >= 1000) {
			tickCounter = 0;
		}

		if (debounceActive) {
			if (tickCounter >= 500) {
				debounceActive = 0;
			}
		}

		if (programCountdown == 1 && tickCounter == 0) {
			setTime--;

			if (tickCounter % 500 == 0) {
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

				temp = calcThermistorTemp();
				action = PI_Compute(&pid, setTemp, temp);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, (action > 0) ? SET : RESET);
							HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);

				I2C_LCD_SetCursor(0, 0, 1);
				sprintf(buffer, "               ");
				I2C_LCD_WriteString(0, buffer);

				I2C_LCD_SetCursor(0, 0, 1);
				int temp_int = (int) (temp * 10);
				sprintf(buffer, "T:%d.%dC", temp_int / 10, abs(temp_int % 10));
				I2C_LCD_WriteString(0, buffer);

				I2C_LCD_SetCursor(0, 9, 1);
				int action_int = (int) (action * 10);
				if (action_int > 0){
					sprintf(buffer, "A:.%d", abs(action_int % 10));
				} else {
					sprintf(buffer, "A:-.%d", abs(action_int % 10));
				}
				I2C_LCD_WriteString(0, buffer);

				if (action > 0) {
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
							(1000 * action) - 1);
				} else {
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
				}
			}

			sprintf(buffer, "Count:%d:%02d", setTime / 60, setTime % 60);
			I2C_LCD_SetCursor(0, 0, 0);
			I2C_LCD_WriteString(0, buffer);

			if (setTime <= 0) {
				programCountdown = 0;
				action = 0;
				setTime = 30;

				I2C_LCD_Clear(0);
				I2C_LCD_SetCursor(0, 0, 1);
				I2C_LCD_WriteString(0, "-- DONE --   ");
				moveParentChild(-1);
				moveParentChild(-1);
				updateLCD();
			}
		}
	}
}

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
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM1_Init();
	MX_I2C1_Init();
	MX_TIM4_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);

	initMenu();

	// Inicializace LCD
	I2C_LCD_Init(0);
	I2C_LCD_Clear(0);

	I2C_LCD_SetCursor(0, 0, 0); // Řádek 0, Sloupec 0
	I2C_LCD_WriteString(0, getCurrentMenuItem()->name);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (counter > 40) {
			counter = 0;
		}
		if (counter % 4 == 0) {

			sprintf(buffer, "%ld", counter / 4);
			I2C_LCD_SetCursor(0, 15, 0);
			I2C_LCD_WriteString(0, " ");
			I2C_LCD_SetCursor(0, 15, 0);
			I2C_LCD_WriteString(0, buffer);
		}
		if (counter % 2 == 0) {
//			temp = calcThermistorTemp();
		}
		if (counter % 5 == 0) {
			I2C_LCD_SetCursor(0, 11, 0);
			int tempset_int = (int) (setTemp);
			sprintf(buffer, "T%d", tempset_int);
			I2C_LCD_WriteString(0, buffer);
//
//			I2C_LCD_SetCursor(0, 0, 1);
//			sprintf(buffer, "              ");
//			I2C_LCD_WriteString(0, buffer);
//
//			I2C_LCD_SetCursor(0, 0, 1);
//			int temp_int = (int) (temp * 10);
//			sprintf(buffer, "T:%d.%dC", temp_int / 10, abs(temp_int % 10));
//			I2C_LCD_WriteString(0, buffer);
//
//			I2C_LCD_SetCursor(0, 9, 1);
//			int action_int = (int) (action * 10);
//			sprintf(buffer, "A:.%d", abs(action_int % 10));
//			I2C_LCD_WriteString(0, buffer);
		}
		counter++;
		HAL_Delay(200);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 720 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 36 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 100 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 8;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 72 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 72 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB11 */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef USE_FULL_ASSERT
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
