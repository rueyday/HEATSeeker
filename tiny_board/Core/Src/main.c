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
//#include "oled_driver.h"
#include "st7735.h"
#include <stdbool.h>
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
UART_HandleTypeDef huart1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

uint8_t indx = 0;
uint8_t xbee_byte;
uint8_t uart_buffer[32];
uint8_t xbee_int_buf[2];
uint8_t batt_dir;
volatile uint8_t xbee_int_ready = 0;
volatile uint8_t got_16_min = 0;
volatile uint8_t new_batt = 0;

uint8_t batt_level = 10;
uint8_t r = 0;

enum {
    FIRST_WAIT,
	SECOND_WAIT,
    READ_PAYLOAD,
	BATT_DIR
} xbee_state = FIRST_WAIT;

// FOR LCD
uint8_t raw_frame[8][8];
//bool draw_queue[8][8];

#define CELL_SIZE  8


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MX_NVIC_Init(void)
{
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}



int xbee_readline(char *buf, int maxlen)
{
    int i = 0;
    uint8_t ch;
    while (i < maxlen - 1) {
        if (HAL_UART_Receive(&huart1, &ch, 1, 500) != HAL_OK) {
            break;  // timeout
        }
        buf[i++] = ch;
        if (ch == '\r' || ch == '\n') break;
    }
    buf[i] = '\0';
    return i;

}



void xbee_send(const char* cmd) {

	uint8_t resp[8];
	HAL_StatusTypeDef st;


	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 100);
	HAL_UART_Transmit(&huart1, (uint8_t*)"\r", 1, 100);
	HAL_Delay(30);

	HAL_UART_Receive(&huart1, resp, 3, 1000);
	printf("resp: %s\r", resp);

}



uint8_t xbee_enter_command(void)

{
    uint8_t resp[8];
    HAL_StatusTypeDef st;

    printf("Entering XBee command mode...\r\n");

    HAL_Delay(1000);   // guard time before
    printf("Sending +++\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)"+++", 3, 100);
    HAL_Delay(1000);   // guard time after

    st = HAL_UART_Receive(&huart1, resp, 3, 1000);  // read 3 bytes

    printf("enter cmd resp: st=%d, bytes=%02X %02X %02X\r\n",
           st, resp[0], resp[1], resp[2]);

    if (st == HAL_OK && resp[0] == 'O' && resp[1] == 'K')
        return 1;
    else
        return 0;

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
		for(int i=0; i<32; i++){
		    printf("%02X ", uart_buffer[i]);
		}

		printf("\n");
	}

}





void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if (huart == &huart1) {
//    	printf("XBEE RECEIVED!! %d\n", indx);
//    	printf("%02X ", xbee_byte);
    	switch (xbee_state) {
    	case FIRST_WAIT:
    		if (xbee_byte == 0xAA) {   // 헤더 발견!
				xbee_state = SECOND_WAIT;
			}
			break;
    	case SECOND_WAIT:
    		if (xbee_byte == 0x55) {
    			indx = 0;
    			xbee_state = READ_PAYLOAD;
    		} else {
    			xbee_state = FIRST_WAIT;
    		}
    		break;
    	case READ_PAYLOAD:
			uart_buffer[indx++] = xbee_byte;
//			printf("%d: %d\n\r", indx, xbee_byte);

			if(indx == 32){
				xbee_state = BATT_DIR;
			}
			break;
    	case BATT_DIR:
    		batt_dir = xbee_byte;

    		xbee_int_ready = 1;
    		xbee_state = FIRST_WAIT;
    	}
    }

    // Restart reception IMMEDIATELY
    if (HAL_UART_Receive_IT(&huart1, &xbee_byte, 1) != HAL_OK) {
        printf("Failed to restart UART reception!\n");
    }

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)

{
    if (huart == &huart1) {
    	HAL_UART_Receive_IT(&huart1, &xbee_byte, 1);
    }
}

// =======================================
// THIS IS FOR MULTI-COLOR DISPLAY
// =======================================

static const uint16_t heatmap_rgb[16] = {
    0x001F, // Deep Blue
    0x101F, // Blue
    0x401F, // Blue-Purple
    0x781F, // Purple
    0xF81F, // Magenta
    0xF801, // Magenta→Red
    0xF800, // Red
    0xFC00, // Orange-Red
    0xFE00, // Orange
    0xFF00, // Yellow-Orange
    0xFFE0, // Yellow
    0xFFF0, // Light Yellow
    0xFFFF, // Near White
    0xFFFF, // White
    0xFFFF, // White
    0xFFFF  // Pure White
};

void interpolate8x8_to_32x32(void)
{
    for (int y = 0; y < 32; y++) {
        float gy = ((float)y) / 4.0f;   // 32/8 = scale factor 4
        int y0 = (int)gy;
        float dy = gy - y0;
        int y1 = (y0 < 7) ? y0 + 1 : y0;

        for (int x = 0; x < 32; x++) {
            float gx = ((float)x) / 4.0f;
            int x0 = (int)gx;

            float dx = gx - x0;
            int x1 = (x0 < 7) ? x0 + 1 : x0;

            float v00 = raw_frame[y0][x0];
            float v01 = raw_frame[y0][x1];
            float v10 = raw_frame[y1][x0];
            float v11 = raw_frame[y1][x1];

            float v0 = v00 + dx * (v01 - v00);
            float v1 = v10 + dx * (v11 - v10);
            uint8_t v = (uint8_t)(v0 + dy * (v1 - v0) + 0.5f);

//            if (v != image_frame[y][x]) {
//                draw_queue[y][x] = true;
//            } else {
//                draw_queue[y][x] = false;
//            }
        }
    }
}

void interpolate8x8_to_40x40(void)
{
    for (int y = 0; y < 40; y++) {
        float gy = ((float)y) / 5.0f;   // scale factor 40/8 = 5
        int y0 = (int)gy;
        float dy = gy - y0;
        int y1 = (y0 < 7) ? y0 + 1 : y0;

        for (int x = 0; x < 40; x++) {
            float gx = ((float)x) / 5.0f;
            int x0 = (int)gx;
            float dx = gx - x0;
            int x1 = (x0 < 7) ? x0 + 1 : x0;

            float v00 = raw_frame[y0][x0];
            float v01 = raw_frame[y0][x1];
            float v10 = raw_frame[y1][x0];
            float v11 = raw_frame[y1][x1];


            float v0 = v00 + dx * (v01 - v00);
            float v1 = v10 + dx * (v11 - v10);
            uint8_t v = (uint8_t)(v0 + dy * (v1 - v0) + 0.5f);
//
//            if (v != image_frame[y][x]) {
//                draw_queue[y][x] = true;
//            } else {
//                draw_queue[y][x] = false;

//            }
        }
    }
}

//void ST7735_DrawBlock(int x, int y, int size, uint16_t color)
//{
//  for (int j = 0; j < size; j++) {
//	  for (int i = 0; i < size; i++) {
//		  ST7735_DrawPixel(x + i, y + j, color);
//	  }
//  }
//}

void ST7735_DrawFullGrid(void)
{
    ST7735_SetAddrWindow(30, 0, 93, 63);

    static uint8_t buf[64 * 64 * 2];
    int ptr = 0;

    for (int i = 0; i < 4096; i++) {   // 64×64 = 4096 pixels
        int y = i >> 6;               // divide by 64
        int x = i & 63;               // modulo 64

        int gy = y >> 3;              // y / 8  (CELL_SIZE=8 → shift 3)
        int gx = x >> 3;              // x / 8

        uint8_t level = raw_frame[gy][gx];
        uint16_t color = heatmap_rgb[level];

        buf[ptr++] = color >> 8;
        buf[ptr++] = color & 0xFF;
    }

    ST7735_DC_HIGH();
    ST7735_CS_LOW();
    HAL_SPI_Transmit(&hspi1, buf, ptr, 10);
    ST7735_CS_HIGH();
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
//  memset(raw_frame, 0, sizeof(raw_frame));
//  memset(draw_queue, 0, sizeof(draw_queue));

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  MX_NVIC_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(1000);
  HAL_UART_Receive_IT(&huart1, &xbee_byte, 1);
  printf("UART1 RX IT started\r\n");

  HAL_Delay(1000);
  ST7735_Init(&hspi1);
  printf("ST7735 init done\r\n");

  HAL_Delay(1000);
	HAL_UART_Receive_IT(&huart1, &xbee_byte, 1);
	HAL_Delay(1000);

   ST7735_Init(&hspi1);

   HAL_Delay(1000);
//   ST7735_FillScreen(RED);

   ST7735_SetCursor(25, 65);
   ST7735_WriteString("BATT:", Font_11x18, WHITE);
   ST7735_SetCursor(25, 115);
   ST7735_WriteString("DIR: ", Font_11x18, WHITE);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)

  {

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */


	  	 if (xbee_int_ready) {
	  		xbee_int_ready = 0;
	  		for (int i = 0; i < 32; i++) {
	  			uint8_t byte = uart_buffer[i];
	  			uint8_t val1 = (byte >> 4) & 0x0F;
	  			uint8_t val2 = byte & 0x0F;

	  			raw_frame[i/4][2*(i%4)] = val1;
	  			raw_frame[i/4][1+2*(i%4)] = val2;
	  		}
	  		for (int i = 0; i < 8; i++) {
				printf("%d %d  %d %d  %d %d  %d %d\n\r", raw_frame[i][0], raw_frame[i][1], raw_frame[i][2], raw_frame[i][3], raw_frame[i][4], raw_frame[i][5], raw_frame[i][6], raw_frame[i][7]);
			}
	  		ST7735_DrawFullGrid();

			ST7735_SetCursor(25, 85);
			uint8_t new_batt_level = (batt_dir & 0b1110000) >> 4;
			if (new_batt_level != batt_level) {
				switch (new_batt_level) {
					case 0:
						ST7735_WriteString("0", Font_11x18, WHITE);
						break;
					case 1:
						ST7735_WriteString("20", Font_11x18, WHITE);
						break;
					case 2:
						ST7735_WriteString("40", Font_11x18, WHITE);
						break;
					case 3:
						ST7735_WriteString("60", Font_11x18, WHITE);
						break;
					case 4:
						ST7735_WriteString("80", Font_11x18, WHITE);
						break;
					case 5:
						ST7735_WriteString("100", Font_11x18, WHITE);
						break;
					default:
						ST7735_WriteString("0", Font_11x18, WHITE);
						break;
				}
			}
			uint8_t right_mov = (batt_dir & 0b1100) >> 2;
			uint8_t left_mov = batt_dir & 0b11;




	  	 }
	  	  // Simple test: cycle colors so you KNOW the TFT works
//	  	  ST7735_FillScreen(RED);
//	  	  printf("Fill RED\r\n");
//	  	  HAL_Delay(500);
//	  	  ST7735_FillScreen(GREEN);
//	  	  printf("Fill GREEN\r\n");
//	  	  HAL_Delay(500);
//	  	  ST7735_FillScreen(BLUE);
//	  	  printf("Fill BLUE\r\n");
//	  	  HAL_Delay(500);
//	  	  ST7735_FillScreen(BLACK);
//	  	  printf("Fill BLACK\r\n");
//
//	  	  HAL_Delay(500);
	     // Fake heatmap data: gradient pattern
//	     for (int i = 0; i < 8; i++) {
//	         for (int j = 0; j < 8; j++) {
//	             raw_frame[i][j] = (i * 2 + j) & 0x0F;  // values 0..15
//	         }
//	     }
//	     ST7735_DrawFullGrid();
//	     HAL_Delay(200);
//	  	 HAL_Delay(100);
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
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */



  /* USER CODE END LPUART1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */



  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */



  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */



  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */



  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
