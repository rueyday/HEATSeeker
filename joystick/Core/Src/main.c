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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile uint8_t pe9_pressed = 0;
volatile uint8_t pe11_pressed = 0;
volatile uint8_t pf13_pressed = 0;

volatile uint8_t rot_sw = 0;

static uint32_t adc_buffer[2];
static int rot_count = 0;
float left_motor_speed = 0.0f;
float right_motor_speed = 0.0f;
uint8_t uart_buffer[4];
char xbee_buffer[100];

uint8_t power = 1;
uint8_t mode = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();  // for debouncing

    static uint32_t last_pe9  = 0;
    static uint32_t last_pe11 = 0;
    static uint32_t last_pf13 = 0;
    static uint32_t last_sw  = 0;
    static uint32_t last_but = 0;

    switch(GPIO_Pin) {
    case GPIO_PIN_1:
        if (now - last_sw < 200) return;
        last_sw = now;
    	rot_sw = 1;
    	break;
    case GPIO_PIN_9:
        if (now - last_pe9 < 200) return;
        last_pe9 = now;
        pe9_pressed = 1;
    	break;
    case GPIO_PIN_11:
        if (now - last_pe11 < 200) return;
        last_pe11 = now;
        pe11_pressed = 1;
    	break;
    case GPIO_PIN_13:
        if (now - last_pf13 < 200) return;
        last_pf13 = now;
        pf13_pressed = 1;
    	break;
    }
    return;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	return;
}

void xbee_send(const char* cmd) {
	HAL_UART_Transmit(&huart4, (uint8_t*)cmd, strlen(cmd), 100);
	HAL_UART_Transmit(&huart4, (uint8_t*)"\r", 1, 100);
}
int xbee_readline(char *buf, int maxlen)
{
    int i = 0;
    uint8_t ch;
    while (i < maxlen - 1) {
        if (HAL_UART_Receive(&huart4, &ch, 1, 500) != HAL_OK) {
            break;  // timeout
        }
        buf[i++] = ch;
        if (ch == '\r' || ch == '\n') break;
    }
    buf[i] = '\0';
    return i;
}
uint8_t xbee_enter_command(void)
{
    uint8_t resp[8];
    HAL_StatusTypeDef st;

    HAL_Delay(1000);   // guard time before
    uint8_t plus[3] = {'+', '+', '+'};
    HAL_UART_Transmit(&huart4, plus, 3, 100);
    HAL_Delay(1000);   // guard time after

    // XBee should respond with "OK\r"
    st = HAL_UART_Receive(&huart4, resp, 3, 500);  // read 3 bytes

    printf("enter cmd resp: st=%d, bytes=%02X %02X %02X\r\n",
           st, resp[0], resp[1], resp[2]);

    // check for "OK\r"
    if (st == HAL_OK && resp[0] == 'O' && resp[1] == 'K')
        return 1;
    else
        return 0;
}

void xbee_coord_setup() {
	if(xbee_enter_command()) {
		xbee_send("ATID 1111");
		xbee_send("ATCH 10");
		xbee_send("ATMY 1");
		xbee_send("ATDL 2");
		xbee_send("ATWR");
		xbee_send("ATCN");
	}
	else {
		printf("xbee coordinator setup failed! \r\n");
	}
}

void xbee_router_setup() {
	if(xbee_enter_command()) {
		xbee_send("ATID 1111");
		xbee_send("ATCH 10");
		xbee_send("ATMY 2");
		xbee_send("ATDL 1");
		xbee_send("ATWR");
		xbee_send("ATCN");
	}
	else {
		printf("xbee router setup failed! \r\n");
	}
}

// Convert ADC values to normalized range [-1.0, +1.0]
float normalize_adc(uint32_t adc_val) {
    return ((float)adc_val - 2047.5f) / 2047.5f;  // Center at 0, range ±1.0
}

// Convert ADC to float (0.0 to 1.0 range) then to 8-bit
uint8_t adc_to_8bit(uint32_t adc_val) {
    float normalized = (float)adc_val / 4095.0f;  // 0.0 to 1.0
    return (uint8_t)(normalized * 255.0f);        // Scale to 0-255
}

#define X_CENTER_ADC   1890.0f   // approximate X center (adc_buffer[0])
#define Y_CENTER_ADC   1830.0f   // approximate Y center (adc_buffer[1])
//#define R_DEADZONE_ADC 150.0f    // deadzone radius in ADC units, tune

uint8_t encode_direction(uint32_t x_raw, uint32_t y_raw)
{
	float dx = (float)x_raw - X_CENTER_ADC;
	float dy = (float)y_raw - Y_CENTER_ADC;  // flip as above if needed

	float left_val = dx +dy;
	float right_val = dx - dy;

	uint8_t left_command;
	uint8_t right_command;

	if(-350 < left_val && left_val < 350){
		left_command = 0;
	}else if(-800 < left_val && left_val < 800){
		left_command = 1;
	}else if(-1300 < left_val && left_val < 1300){
		left_command = 2;
	}else{
		left_command = 3;
	}

	if(-350 < right_val && right_val < 350){
		right_command = 0;
	}else if(-800 < right_val && right_val < 800){
		right_command = 1;
	}else if(-1300 < right_val && right_val < 1300){
		right_command = 2;
	}else{
		right_command = 3;
	}

	if(left_val > 0){
		left_command += 4;
	}
	if(right_val > 0){
		right_command += 4;
	}
//	printf("left: %d, right: %d \n\r", left_command, right_command);
	return left_command * 8 + right_command;
}

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
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  xbee_coord_setup();
  HAL_Delay(1000);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim3, 32768);
  HAL_ADC_Start_DMA(&hadc1, adc_buffer, 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  int32_t x_raw = adc_buffer[0];  // check which is X / Y on your board
	  int32_t y_raw = adc_buffer[1];

//	  dir_t dir = joystick_get_direction();
	  uart_buffer[0] = encode_direction(x_raw, y_raw);

	  if (pe9_pressed) {
		  pe9_pressed = 0;
		  power = !power;
		  printf("PE9 press!\r\n");

	  }
	  if (pe11_pressed) {
		  pe11_pressed = 0;
		  mode = !mode;
		  printf("PE11 press!\r\n");

	  }
	  if(power){
		  uart_buffer[0] |= 0b10000000;
	  }else{
		  uart_buffer[0] &= 0b01111111;
	  }

	  if(mode){
		  uart_buffer[0] |= 0b01000000;
	  }else{
		  uart_buffer[0] &= 0b10111111;
	  }
//	  printf("command: %d \n", (uart_buffer[0]>>6));
	  if (pf13_pressed) {
		  pf13_pressed = 0;
		  printf("PF13 press!\r\n");
	  }

	  if(rot_sw){
		  rot_sw = 0;
		  __HAL_TIM_SET_COUNTER(&htim3, 32768);
		  printf("rotary switch pressed! \r\n");
	  }

<<<<<<< HEAD
<<<<<<< HEAD
	  rot_count = ((TIM3->CNT) - 32768) >> 2;
//	  printf("rotary count: %d\r\n", rot_count);
=======
	  int32_t cnt = (int32_t)TIM3->CNT;
	  rot_count = (cnt - 32768) >> 2;
	  printf("rotary count: %d\r\n", rot_count);
>>>>>>> e6c57f703057cdbb31548ac93298fc9b9058ac3b
	  uart_buffer[3] = rot_count & 0x00FF;
	  uart_buffer[2] = rot_count >> 8;

//	      printf("-------------------------\r\n");
//	      printf("x_raw: %lu, y_raw: %lu\r\n", x_raw, y_raw);
//	      printf("dir: %d, out L: %d, out R: %d\r\n",
//	             (int)dir, uart_buffer[0], uart_buffer[1]);

	  HAL_UART_Transmit(&huart4, uart_buffer, 4, 100);
	  HAL_Delay(20);
//	  HAL_Delay(100);
  }
=======
        int32_t cnt = (int32_t)TIM3->CNT;
        rot_count = (cnt - 32768) >> 4;
        if(rot_count >= 32){
        	rot_count = 32;
        }
        if(rot_count < -32){
        	rot_count = -32;
        }
        if(rot_count >= 0){
        	uart_buffer[1] = (uint8_t)(rot_count<<2) & 0b01111111;
        }else{
        	uart_buffer[1] = (uint8_t)((-rot_count)<<2) | 0b10000000;
        }
        printf("rotary count: %d\r\n", rot_count);

        if (pf13_pressed) {
			pf13_pressed = 0;
			feature1 = !feature1;
			printf("PF13 press!\r\n");
		}
		if (pe0_pressed) {
			pe0_pressed = 0;
			feature2 = !feature2;
			printf("PE0 press!\r\n");
		}

		if(feature1){
			uart_buffer[1] |= 0b00000001;
		}else{
			uart_buffer[1] &= 0b11111110;
		}

		if(feature2){
			uart_buffer[1] |= 0b00000010;
		}else{
			uart_buffer[1] &= 0b11111101;
		}

        HAL_UART_Transmit(&huart4, uart_buffer, 2, 100);
        HAL_Delay(200);
    }
>>>>>>> 5d139bf (just need to add speaker now)
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin : PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
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
