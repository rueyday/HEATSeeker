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
#include "fonts.h"
#include "oled_driver.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint8_t uart_buffer[32];
uint8_t xbee_buffer[32];
uint8_t xbee_int_buf[2];
volatile uint8_t xbee_int_ready = 0;
uint8_t r = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MX_NVIC_Init(void)
{
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
}

int xbee_readline(char *buf, int maxlen)
{
    int i = 0;
    uint8_t ch;
    while (i < maxlen - 1) {
        if (HAL_UART_Receive(&huart5, &ch, 1, 500) != HAL_OK) {
            break;  // timeout
        }
        buf[i++] = ch;
        if (ch == '\r' || ch == '\n') break;
    }
    buf[i] = '\0';
    return i;
}

void xbee_send(const char* cmd) {
	HAL_UART_Transmit(&huart5, (uint8_t*)cmd, strlen(cmd), 100);
	HAL_UART_Transmit(&huart5, (uint8_t*)"\r", 1, 100);
}

uint8_t xbee_enter_command(void)
{
    uint8_t resp[8];
    HAL_StatusTypeDef st;
//    HAL_UART_AbortReceive(&huart5);
    HAL_Delay(1000);   // guard time before
    HAL_UART_Transmit(&huart5, (uint8_t*)"+++", 3, 100);
    HAL_Delay(1000);   // guard time after
    // XBee should respond with "OK\r"
    st = HAL_UART_Receive(&huart5, resp, 3, 1000);  // read 3 bytes

    printf("enter cmd resp: st=%d, bytes=%02X %02X %02X\r\n",
           st, resp[0], resp[1], resp[2]);
    // check for "OK\r"
    if (st == HAL_OK && resp[0] == 'O' && resp[1] == 'K')
        return 1;
    else
        return 0;
}

void OLED_Circle(int x0, int y0, int r, OLED_COLOR color)
{
    int x = r;
    int y = 0;
    int err = 0;

    while (x >= y)
    {
        // Draw horizontal spans instead of single pixels
        for (int i = x0 - x; i <= x0 + x; i++) {
            OLED_DrawPixel(i, y0 + y, color);
            OLED_DrawPixel(i, y0 - y, color);
        }
        for (int i = x0 - y; i <= x0 + y; i++) {
            OLED_DrawPixel(i, y0 + x, color);
            OLED_DrawPixel(i, y0 - x, color);
        }

        y++;

        if (err <= 0)
        {
            err += 2*y + 1;
        }
        if (err > 0)
        {
            x--;
            err -= 2*x + 1;
        }
    }
}

void OLED_Square(uint8_t x, uint8_t y, uint8_t size, OLED_COLOR color)
{
    // Prevent drawing outside the display
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;

    for (uint8_t i = 0; i < size; i++) {
        for (uint8_t j = 0; j < size; j++) {
            uint8_t px = x + j;
            uint8_t py = y + i;

            if (px < OLED_WIDTH && py < OLED_HEIGHT) {
                OLED_DrawPixel(px, py, color);
            }
        }
    }
}

void xbee_router_setup() {
	if(xbee_enter_command()) {
		xbee_send("ATID 2222");
		xbee_send("ATCH 17");
		xbee_send("ATMY 2");
		xbee_send("ATDL 1");
		xbee_send("ATWR");
		xbee_send("ATCN");
	}
	else {
		printf("xbee router setup failed! \r\n");
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart5) {
        xbee_int_ready = 1;
    }
    // Restart reception IMMEDIATELY
    if (HAL_UART_Receive_IT(&huart5, uart_buffer, 32) != HAL_OK) {
        printf("Failed to restart UART reception!\n");
    }

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5) {
    	HAL_UART_Receive_IT(&huart5, uart_buffer, 32);
    }
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
  MX_NVIC_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);
  xbee_router_setup();
  HAL_Delay(1000);

  // Init lcd using one of the stm32HAL i2c typedefs
  if (OLED_Init(&hi2c1) != 0) {
    Error_Handler();
  }
  HAL_Delay(100);

  OLED_Fill(Black);
  OLED_UpdateScreen(&hi2c1);

  HAL_Delay(1000);

  // Write data to local screenbuffer
  OLED_SetCursor(0, 0);
  OLED_WriteString("Initialization000000000000000000000", Font_7x10, White);
  OLED_SetCursor(0, 10);
  OLED_WriteString("Done >:)000000000000000000000000000", Font_7x10, White);
  OLED_SetCursor(0, 20);
  OLED_WriteString("3rd line000000000000000000000000000", Font_7x10, White);
  OLED_SetCursor(0, 30);
  OLED_WriteString("4rd line000000000000000000000000000", Font_7x10, White);
  OLED_SetCursor(0, 40);
  OLED_WriteString("5th line000000000000000000000000000", Font_7x10, White);
  OLED_SetCursor(0, 50);
  OLED_WriteString("6th line000000000000000000000000000", Font_7x10, White);
  OLED_SetCursor(0, 60);
  OLED_WriteString("7th line000000000000000000000000000", Font_7x10, White);
  OLED_SetCursor(0, 70);
  OLED_WriteString("8th line000000000000000000000000000", Font_7x10, White);
  OLED_SetCursor(0, 80);
  OLED_WriteString("9th line000000000000000000000000000", Font_7x10, White);

//  OLED_UpdateScreen(&hi2c1);
//  OLED_SetCursor(0, 36);
//  OLED_WriteString("Recheck", Font_11x18, White);

//  // Draw rectangle on screen
//  for (uint8_t i=0; i<28; i++) {
//      for (uint8_t j=0; j<64; j++) {
//          OLED_DrawPixel(100+i, 0+j, White);
//      }
//  }

  // Copy all data from local screenbuffer to the screen
  HAL_Delay(100);
  OLED_UpdateScreen(&hi2c1);

  HAL_UART_Receive_IT(&huart5, uart_buffer, 32);
  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	printf("left: %d, right: %d\r\n", uart_buffer[0], uart_buffer[1]);
//	for(uint8_t i = 0; i < 16; i++){
//		OLED_SetCursor(0, 15 * i);
//		fprintf("IR Sensor %d: %d/r/n", i, uart_buffer[i]);
//		snprintf(xbee_buffer, sizeof(xbee_buffer), "IR Sensor %d: %d", i, uart_buffer[i]);
//		OLED_WriteString(xbee_buffer, Font_7x10, White);
//	}
	  OLED_Fill(Black);

	 if(xbee_int_ready){
		 xbee_int_ready = 0;
//	         Print your data
		for (int i = 0; i < 32; i++) {
			uint8_t byte = uart_buffer[i];
			int val1 = (byte >> 4) & 0x0F;  // Upper 4 bits
			int val2 = byte & 0x0F;         // Lower 4 bits

			printf("%d %d  ", (byte >> 4) & 0x0F, byte & 0x0F);
			if (i % 4 == 3) printf("\n\r");

			// Convert to 8x8 grid positions
			int grid_index = i * 2;         // 0-63 (64 total values)
			int row = grid_index / 8;       // 0-7 (8 rows)
			int col = grid_index % 8;       // 0-7 (8 columns)

			// First value
			int radius1 = val1 / 4 + 1;     // Scale 0-15 to radius 1-8
			if(radius1 < 3){
				radius1 = 0;
			}
			OLED_Square(col * 8 + 8, row * 8 + 4, radius1, White);

			// Second value (next column)
			int grid_index2 = grid_index + 1;
			int row2 = grid_index2 / 8;
			int col2 = grid_index2 % 8;
			int radius2 = val2 / 4 + 1;
			if(radius2 < 3){
				radius2 = 0;
			}
			OLED_Square(col2 * 8 + 8, row2 * 8 + 4, radius2, White);


		}
		OLED_UpdateScreen(&hi2c1);
		printf("-----------\n\r");

		//		  OLED_SetCursor(0, 0);

	 }

//	 OLED_Circle(128/2, 64/2, r, White);
//
//	 if(r == 32) {
//		 OLED_Fill(Black);
//		 r = 0;
//		 OLED_SetCursor(50, 20);
//		 OLED_WriteString("POP!", Font_11x18, White);
//	 }
//	 r++;


	//HAL_MAX_DELAY

    HAL_Delay(10);  // Check for new data every second
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
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
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
