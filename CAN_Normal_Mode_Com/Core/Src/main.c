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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

uint8_t led_num = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
void CAN1_Tx(void);
void CAN_Filter_Config(void);
/* USER CODE BEGIN PFP */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  CAN_Filter_Config();

  //Enable control bit to receive interrupts on the specified events
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_BUSOFF | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
	  Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
	  Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 4999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, User_LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_B1_Pin */
  GPIO_InitStruct.Pin = Button_B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : User_LD2_Pin PA6 PA7 PA8
                           PA9 */
  GPIO_InitStruct.Pin = User_LD2_Pin|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void CAN1_Tx(void) {
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailBox;
	TxHeader.DLC = 1;
	TxHeader.StdId = 0x111;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	uint8_t message = led_num++;

	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);

	if (led_num == 4) {
		led_num = 0;
	}

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &message, &TxMailBox) != HAL_OK) {
		Error_Handler();
	}

}


void CAN_Filter_Config(void) {
	CAN_FilterTypeDef CAN_Filter;

	CAN_Filter.FilterActivation = ENABLE;
	CAN_Filter.FilterBank = 0;
	CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_Filter.FilterIdHigh = 0x00000000;
	CAN_Filter.FilterIdLow = 0x00000000;
	CAN_Filter.FilterMaskIdHigh = 0x00000000;
	CAN_Filter.FilterMaskIdLow = 0x00000000;
	CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;

	if (HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter) != HAL_OK) {
		Error_Handler();
	}
}
/* USER CODE BEGIN 4 */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	uint8_t uart_msg0[] = "Message Transmitted:M0\r\n";
	HAL_UART_Transmit(&huart2, uart_msg0, sizeof(uart_msg0), HAL_MAX_DELAY);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
	uint8_t uart_msg1[] = "Message Transmitted:M2\r\n";
	HAL_UART_Transmit(&huart2, uart_msg1, sizeof(uart_msg1), HAL_MAX_DELAY);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
	uint8_t uart_msg2[] = "Message Transmitted:M2\r\n";
	HAL_UART_Transmit(&huart2, uart_msg2, sizeof(uart_msg2), HAL_MAX_DELAY);
}
/**
  * @brief this function is used to receive the messages existing in the FIFO0
  * @param None
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	//handle structure to store The header of the received message.
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t Rcv_msg[5];

	uint8_t uart_msg[] = "Message Received : ";

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, Rcv_msg) != HAL_OK) {
		Error_Handler();
	}

	HAL_UART_Transmit(&huart2, uart_msg, sizeof(uart_msg), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, Rcv_msg, sizeof(Rcv_msg), HAL_MAX_DELAY);
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	uint8_t uart_msg3[] = "Error occur while transmission!!\r\n";
	HAL_UART_Transmit(&huart2, uart_msg3, sizeof(uart_msg3), HAL_MAX_DELAY);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	void CAN1_Tx(void);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	HAL_GPIO_TogglePin(GPIOA, User_LD2_Pin);
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
