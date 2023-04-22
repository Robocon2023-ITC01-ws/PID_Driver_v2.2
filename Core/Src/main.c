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
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Right 1
#define Left 2
uint8_t Driver = Right;

PIDController MPID;
#define MKp 7.92
#define MKi 136.48
#define MKd 0.0

// need to configuration
#define CPR 912
#define Sample_time 10 // ms
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define pi 3.1415
#define Motor1 0
#define Motor2 1

uint16_t cnt[2];
uint16_t Enc_count[2];

uint16_t count[2]; // count pulse from encoder
uint16_t new_count[2];
uint8_t count_state[2];
uint16_t diff[2]; // difference between count and new_count in a sample time

float speedM[2];
float rdps[2];

float Motor1_speed = 0;
float Motor2_speed = 0;
float V1 = 0; // target speed of motor1
float V2 = 0; // target speed of motor2
float pwm_M1 = 0;
float pwm_M2 = 0;

float RxData1 = 32768;
float RxData2 = 32768;
float RxData3 = 32768;
float RxData4 = 32768;

uint16_t V1_out = 32768;
uint16_t V2_out = 32768;

uint8_t flag = 0;
uint8_t cntt;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t test_encoder;

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[4];
uint32_t TxMailbox;

float map(float Input, float Min_Input, float Max_Input, float Min_Output, float Max_Output)
{

	return (float)((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	cntt++;
	while (cntt - 100 > 0)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		cntt = 0;
	}
	if(Driver == Left && RxHeader.DLC == 4){
		flag = 1;
	}

	if (RxHeader.DLC == 8)
	{
		RxData1 = (RxData[0] << 8) | RxData[1];
		RxData2 = (RxData[2] << 8) | RxData[3];
		RxData3 = (RxData[4] << 8) | RxData[5];
		RxData4 = (RxData[6] << 8) | RxData[7];
		//
		if (Driver == Right)
		{
			V1 = map(RxData1, 0, 65536, -100, 100);
			V2 = map(RxData2, 0, 65536, -100, 100);
			flag = 1;
		}
		else if (Driver == Left)
		{
			V1 = map(RxData3, 0, 65536, -100, 100);
			V2 = map(RxData4, 0, 65536, -100, 100);
		}
		if(V1 > -0.005 && V1 < 0.005) V1 = 0;
		if(V2 > -0.005 && V2 < 0.005) V2 = 0;

	}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float Motors_RPS(int j, float SampleTime, float N_round)
{
	new_count[Motor1] = TIM2->CNT;
	new_count[Motor2] = TIM1->CNT;

	count_state[Motor1] = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
	count_state[Motor2] = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1);

	if (count_state[j])
	{
		if (new_count[j] <= count[j])
		{ // Check for counter underflow
			diff[j] = count[j] - new_count[j];
		}
		else
		{
			diff[j] = (65536 - new_count[j]) + count[j];
		}
		speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime) * -1.0;
	}
	else
	{
		if (new_count[j] >= count[j])
		{ // Check for counter overflow
			diff[j] = new_count[j] - count[j];
		}
		else
		{
			diff[j] = (65536 - count[j]) + new_count[j];
		}
		speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime);
	}

	rdps[j] = -2.0f * pi * speedM[j];
	count[j] = new_count[j];

	return rdps[j];
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	PID_Init(&MPID, 2);
	MPID.T = 0.01; // T = 20ms
	MPID.limMax = 1000;
	MPID.limMin = -1000;
	MPID.limMaxInt = 1000;
	MPID.limMinInt = -1000;
	MPID.tau = 0; // for Kd
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  // pwm
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  // encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim3);

	HAL_CAN_Start(&hcan);

	// Activate the notification
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

	TxHeader.DLC = 4; // data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	if (Driver == Right)
	{
		TxHeader.StdId = 0x155; // ID
	}
	else if (Driver == Left)
	{
		TxHeader.StdId = 0x140; // ID
	}

	TxData[0] = 128; // ms delay
	TxData[1] = 0;
	TxData[2] = 128;
	TxData[3] = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		V1_out = (uint16_t)map(Motor1_speed, -100, 100, 0, 65536);
		V2_out = (uint16_t)map(Motor2_speed, -100, 100, 0, 65536);

		TxData[0] = ((V1_out & 0xFF00) >> 8);
		TxData[1] = (V1_out & 0x00FF);
		TxData[2] = ((V2_out & 0xFF00) >> 8);
		TxData[3] = (V2_out & 0x00FF);
		//
		if (flag == 1)
		{
			HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
			flag = 0;
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		// PID // need to change for using
		pwm_M1 = PID(&MPID, V1, Motor1_speed, MKp, MKi, MKd, Motor1);
		pwm_M2 = PID(&MPID, V2, Motor2_speed, MKp, MKi, MKd, Motor2);

//		pwm_M1 = map(V1,-100,100,-1000,1000);
//		pwm_M2 = map(V2,-100,100,-1000,1000);
		// feedback speed
		Motor1_speed = Motors_RPS(Motor1, Sample_time, CPR);
		Motor2_speed = Motors_RPS(Motor2, Sample_time, CPR);

		if (pwm_M1 > 20)
		{
			TIM4->CCR2 = pwm_M1;
			TIM4->CCR1 = 0;
		}
		else if (pwm_M1 < -20)
		{
			TIM4->CCR2 = 0;
			TIM4->CCR1 = pwm_M1 * -1;
		}
		else
		{
			TIM4->CCR2 = 0;
			TIM4->CCR1 = 0;
		}
		if (pwm_M2 > 20)
		{
			TIM4->CCR4 = pwm_M2;
			TIM4->CCR3 = 0;
		}
		else if (pwm_M2 < -20)
		{
			TIM4->CCR4 = 0;
			TIM4->CCR3 = pwm_M2 * -1;
		}
		else
		{
			TIM4->CCR3 = 0;
			TIM4->CCR4 = 0;
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
