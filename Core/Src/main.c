/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "stdio.h"
#include "math.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define buffer_size 40
#define N_sense 16
#define TX_FIFO_SIZE 8
#define TX_MAX_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
//-----------------------Variáveis de controle de taxa ----------------------------
volatile uint32_t last_tick[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
volatile float freq[N_sense] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t freq_SD[N_sense * 2] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
volatile uint16_t cont_seed[N_sense] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t cont_seed_SD[N_sense * 2] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
volatile uint32_t cont_send_seed = 0;
volatile uint32_t cont_send_freq = 0;
uint32_t temp_send_seed = 300;
uint32_t temp_send_freq = 300;
volatile uint32_t cont_zero[N_sense] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
// ----------- Variáveis de controle de envio e recebimento de mensagens -------------------
uint8_t bufferRx[buffer_size] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0 };
uint8_t message_RX[buffer_size] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0 };

// ---------------- CONFIGURAÇÕES FIFO UART TX ----------------
uint8_t uart_tx_fifo[TX_FIFO_SIZE][TX_MAX_SIZE];
uint16_t uart_tx_len[TX_FIFO_SIZE];
volatile uint8_t tx_head = 0;
volatile uint8_t tx_tail = 0;
volatile uint8_t tx_busy = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
float FreqCalculate(uint32_t sensor_index, TIM_HandleTypeDef *htim);
static void clean_buffer_rx_BLE(void);
static void handles_BLE_message(uint8_t message[], uint16_t size);
static void send_uart_TX(uint8_t id, uint8_t dlc, uint8_t data[]);
void UART_Send_DMA(uint8_t *data, uint16_t size);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, bufferRx, buffer_size);
	__HAL_DMA_ENABLE_IT(huart1.hdmarx, DMA_IT_HT);
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start(&htim5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		// Verifica se parou a interrupção de sinal dos sensores para zerar
		for (int i = 0; i < 16; i++) {
			if (cont_zero[i] > 500) {
				freq[i] = 0;
			}
		}
		// Envio da frequencia de semente
		if (cont_send_freq > temp_send_freq) {
			for (int i = 0; i < 16; i++) {
				// 1) escala e arredonda
				float frq = freq[i] * 100.0f;
				long frqr = lroundf(frq);
				// 2) satura no range de 16 bits sem sinal
				if (frqr < 0)
					frqr = 0;
				if (frqr > 65535L)
					frqr = 65535L;

				uint16_t send_frq = (uint16_t) frqr;

				freq_SD[2 * i] = (uint8_t) ((send_frq >> 8) & 0xFF);
				freq_SD[2 * i + 1] = (uint8_t) (send_frq & 0xFF);
			}

			cont_send_freq = 0; // reset de contagem para novo envio
			send_uart_TX(2, 32, freq_SD);
		}
		// envio da contagem de semente para média
		if (cont_send_seed > temp_send_seed) {
			for (int i = 0; i < N_sense; i++) {

				cont_seed_SD[2 * i] = (uint8_t) ((cont_seed[i] >> 8) & 0xFF);
				cont_seed_SD[2 * i + 1] = (uint8_t) (cont_seed[i] & 0xFF);

				//zerar após converter para envio
				cont_seed[i] = 0;
			}
			cont_send_seed = 0; // reset de contagem do timer de envio
			send_uart_TX(1, 32, cont_seed_SD);
		}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 99;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 99;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : SENSOR1_Pin SENSOR2_Pin SENSOR3_Pin SENSOR12_Pin
                           SENSOR15_Pin SENSOR16_Pin */
  GPIO_InitStruct.Pin = SENSOR1_Pin|SENSOR2_Pin|SENSOR3_Pin|SENSOR12_Pin
                          |SENSOR15_Pin|SENSOR16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR11_Pin SENSOR13_Pin SENSOR14_Pin SENSOR4_Pin
                           SENSOR5_Pin SENSOR6_Pin SENSOR7_Pin SENSOR8_Pin
                           SENSOR9_Pin SENSOR10_Pin */
  GPIO_InitStruct.Pin = SENSOR11_Pin|SENSOR13_Pin|SENSOR14_Pin|SENSOR4_Pin
                          |SENSOR5_Pin|SENSOR6_Pin|SENSOR7_Pin|SENSOR8_Pin
                          |SENSOR9_Pin|SENSOR10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == SENSOR1_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[0] = 0;
		cont_seed[0]++;
		freq[0] = FreqCalculate(0, &htim5);
	}
	if (GPIO_Pin == SENSOR2_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[1] = 0;
		cont_seed[1]++;
		freq[1] = FreqCalculate(1, &htim5);
	}
	if (GPIO_Pin == SENSOR3_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[2] = 0;
		cont_seed[2]++;
		freq[2] = FreqCalculate(2, &htim5);
	}
	if (GPIO_Pin == SENSOR4_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[3] = 0;
		cont_seed[3]++;
		freq[3] = FreqCalculate(3, &htim5);
	}
	if (GPIO_Pin == SENSOR5_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[4] = 0;
		cont_seed[4]++;
		freq[4] = FreqCalculate(4, &htim5);
	}
	if (GPIO_Pin == SENSOR6_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[5] = 0;
		cont_seed[5]++;
		freq[5] = FreqCalculate(5, &htim5);
	}
	if (GPIO_Pin == SENSOR7_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[6] = 0;
		cont_seed[6]++;
		freq[6] = FreqCalculate(6, &htim5);
	}
	if (GPIO_Pin == SENSOR8_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[7] = 0;
		cont_seed[7]++;
		freq[7] = FreqCalculate(7, &htim5);
	}
	if (GPIO_Pin == SENSOR9_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[8] = 0;
		cont_seed[8]++;
		freq[8] = FreqCalculate(8, &htim5);
	}
	if (GPIO_Pin == SENSOR10_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[9] = 0;
		cont_seed[9]++;
		freq[9] = FreqCalculate(9, &htim5);
	}
	if (GPIO_Pin == SENSOR11_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[10] = 0;
		cont_seed[10]++;
		freq[10] = FreqCalculate(10, &htim5);
	}
	if (GPIO_Pin == SENSOR12_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[11] = 0;
		cont_seed[11]++;
		freq[11] = FreqCalculate(11, &htim5);
	}
	if (GPIO_Pin == SENSOR13_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[12] = 0;
		cont_seed[12]++;
		freq[12] = FreqCalculate(12, &htim5);
	}
	if (GPIO_Pin == SENSOR14_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[13] = 0;
		cont_seed[13]++;
		freq[13] = FreqCalculate(13, &htim5);
	}
	if (GPIO_Pin == SENSOR15_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[14] = 0;
		cont_seed[14]++;
		freq[14] = FreqCalculate(14, &htim5);
	}
	if (GPIO_Pin == SENSOR16_Pin) {
		// ação desejada, por exemplo, incrementar contador
		cont_zero[15] = 0;
		cont_seed[15]++;
		freq[15] = FreqCalculate(15, &htim5);
	}
}

float FreqCalculate(uint32_t sensor_index, TIM_HandleTypeDef *htim) {
	if (sensor_index >= 16) // verificação de estouro de index
		return 0.0f;

	uint32_t current_tick = __HAL_TIM_GET_COUNTER(htim);
	uint32_t delta;

	if (current_tick >= last_tick[sensor_index])
		delta = current_tick - last_tick[sensor_index];
	else
		delta = (0xFFFFFFFF - last_tick[sensor_index]) + current_tick + 1;

	last_tick[sensor_index] = current_tick;

	if (delta == 0) {
		return 0.0f;
	}

	return (HAL_RCC_GetPCLK1Freq() * 2) / ((htim->Init.Prescaler + 1) * delta); // retorna frequencia

}

//-----------------------Tratativas de recebimento de mensagem e envio -------------------------------------------------
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

	if (huart->Instance == USART1) {

		HAL_UART_AbortReceive(&huart1);
		uint8_t message_size = 0;
		uint8_t capture = 0;

		for (uint16_t i = 0; i < Size; i++) {
			uint8_t byte = bufferRx[i];

			if (byte == '@')  // Detecta início da mensagem
					{
				message_size = 0;
				capture = 1;
			}
			if (capture)  // Captura mensagem até o finalizador '\n'
			{
				if (byte == '\n' && message_size == message_RX[2] + 3) {
					capture = 0;  // Para captura
					message_RX[message_size++] = byte; // inclusão do \n na mensagem
					handles_BLE_message(message_RX, message_size); // Processa a mensagem
					clean_buffer_rx_BLE();
				} else if (message_size < buffer_size - 1) // Evita overflow
						{
					message_RX[message_size++] = byte;
				}
			}
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, bufferRx,
		buffer_size);
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // Desativa interrupção de Half Transfer (opcional)
		__HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_TC); // Mantém interrupção de Transfer Complete
	}
}

void handles_BLE_message(uint8_t message[], uint16_t size) {

	if (size == message[2] + 4) {
		uint8_t id = message[1];
		uint8_t dlc = message[2];
		uint8_t message_BLE[dlc];

		for (int i = 3, j = 0; i < dlc + 2; i++, j++) {
			message_BLE[j] = message[i];
		}
		//Tratamento das mensagens recebidas

		if (id == 3) {

			temp_send_seed = ((message_BLE[0] << 8) | message_BLE[1]);
		}
		if (id == 4) {

			temp_send_freq = ((message_BLE[0] << 8) | message_BLE[1]);
		}
	}
}

void clean_buffer_rx_BLE() {
	 memset(bufferRx,  0, sizeof(bufferRx));
	 memset(message_RX, 0, sizeof(message_RX));
}

void send_uart_TX(uint8_t id, uint8_t dlc, uint8_t data[]) {

	uint8_t buffer_BLE_tx[dlc + 4];

	buffer_BLE_tx[0] = '@';
	buffer_BLE_tx[1] = id;
	buffer_BLE_tx[2] = dlc;
	for (int i = 3, j = 0; i < dlc + 3; i++, j++) {
		buffer_BLE_tx[i] = data[j];
	}
	buffer_BLE_tx[dlc + 3] = '\n';

// Envia via DMA com FIFO
	HAL_UART_Transmit(&huart1, buffer_BLE_tx, dlc + 4, 20);
	//UART_Send_DMA(buffer_BLE_tx, dlc + 4);
}
// ---------------- ENVIO UART DMA COM FIFO ----------------
void UART_Send_DMA(uint8_t *data, uint16_t size) {
// Copia mensagem para FIFO
	memcpy(uart_tx_fifo[tx_head], data, size);
	uart_tx_len[tx_head] = size;
	tx_head = (tx_head + 1) % TX_FIFO_SIZE;

// Se DMA estiver livre, inicia transmissão
	if (!tx_busy) {
		tx_busy = 1;
		HAL_UART_Transmit_DMA(&huart1, uart_tx_fifo[tx_tail],
				uart_tx_len[tx_tail]);
	}
}

// ---------------- CALLBACK DE TX COMPLETA ----------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		tx_tail = (tx_tail + 1) % TX_FIFO_SIZE;

		// Verifica se há mais mensagens na fila
		if (tx_tail != tx_head) {
			HAL_UART_Transmit_DMA(&huart1, uart_tx_fifo[tx_tail],
					uart_tx_len[tx_tail]);
		} else {
			tx_busy = 0; // Fila vazia
		}
	}
}

//-------------------- FUNÇÃO DE TIMER--------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM10) {
		// Código a executar no estouro do timer
		if (cont_send_seed < temp_send_seed + 2) {
			cont_send_seed++;
		}
		if (cont_send_freq < temp_send_freq + 2) {
			cont_send_freq++;
		}
		for (int i = 0; i < 16; i++) {
			if (cont_zero[i] < 502) {
				cont_zero[i]++;
			}
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
