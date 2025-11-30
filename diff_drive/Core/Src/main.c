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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint8_t uart_buffer[2];
char xbee_buffer[100];
uint8_t xbee_int_buf[2];
volatile uint8_t xbee_int_ready = 0;

// FOR IR CAM 1
uint8_t buf[128];
uint8_t reg = 0x80;
#define SAD_IRCAM_W 0b11010010
#define SAD_IRCAM_R 0b11010011
volatile uint8_t glass_xbee_ready = 0;
uint8_t temp_send[32];
uint8_t xbee_glass_int_buf[2];

//for temp tracking
#define HISTORY_LEN 4
uint8_t brightest_history[HISTORY_LEN];
uint8_t history_pos = 0; // where to write next
uint8_t history_count = 0; // how many valid entries (<= 50)

//for PI controller
float error_integral = 0.0f;
const float Kp_x = 8000.0f;//tune!
const float Kp_rot = 13000.0f;//tune!

const float Ki = 0.0f;//also tune
const float dt = 0.03f;//30ms

//modes
typedef enum {
    MODE_JOYSTICK = 0,
    MODE_IR_ONLY  = 1,
    MODE_MIXED    = 2,
} control_mode_t;

int power = 0;

volatile control_mode_t control_mode = MODE_IR_ONLY;

#define DIR_L1_PORT GPIOE
#define DIR_L1_PIN  GPIO_PIN_2
#define DIR_L2_PORT GPIOE
#define DIR_L2_PIN  GPIO_PIN_4
#define DIR_R1_PORT GPIOA
#define DIR_R1_PIN  GPIO_PIN_2
#define DIR_R2_PORT GPIOA
#define DIR_R2_PIN  GPIO_PIN_3

#define RED_Port	GPIOF
#define RED_Pin		GPIO_PIN_12

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void MX_NVIC_Init(void)
{
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
}

int xbee_readline(char *buf, int maxlen, UART_HandleTypeDef *huartx)
{
    int i = 0;
    uint8_t ch;
    while (i < maxlen - 1) {
        if (HAL_UART_Receive(huartx, &ch, 1, 500) != HAL_OK) {
            break;  // timeout
        }
        buf[i++] = ch;
        if (ch == '\r' || ch == '\n') break;
    }
    buf[i] = '\0';
    return i;
}

void xbee_send(const char* cmd, UART_HandleTypeDef *huartx) {
//	HAL_UART_Transmit(huartx, (uint8_t*)cmd, strlen(cmd), 100);
//	HAL_UART_Transmit(huartx, (uint8_t*)"\r", 1, 100);
	uint8_t resp[8];
	HAL_StatusTypeDef st;

	HAL_UART_Transmit(huartx, (uint8_t)cmd, strlen(cmd), 100);
	HAL_UART_Transmit(huartx, (uint8_t*)"\r", 1, 100);
	HAL_Delay(30);

	HAL_UART_Receive(huartx, resp, 3, 1000);
	printf("resp: %s\r", resp);
}

uint8_t xbee_enter_command(UART_HandleTypeDef *huartx)
{
    uint8_t resp[8];
    HAL_StatusTypeDef st;

//    HAL_Delay(1000);   // guard time before
//    uint8_t plus[3] = {'+', '+', '+'};
//    HAL_UART_Transmit(&huart5, plus, 3, 100);
    HAL_Delay(1000);   // 1.2 sec
    HAL_UART_Transmit(huartx, (uint8_t*)"+++", 3, 100);
    HAL_Delay(1000);

    // XBee should respond with "OK\r"
    st = HAL_UART_Receive(huartx, resp, 3, 1000);

    printf("enter cmd resp: st=%d, bytes=%02X %02X %02X\r\n",
           st, resp[0], resp[1], resp[2]);

    // check for "OK\r"
    if (st == HAL_OK && resp[0] == 'O' && resp[1] == 'K')
        return 1;
    else
        return 0;
}

void xbee_router_setup() {
	if(xbee_enter_command(&huart5)) {
		xbee_send("ATID 1111", &huart5);
		xbee_send("ATCH 10", &huart5);
		xbee_send("ATMY 2", &huart5);
		xbee_send("ATDL 1", &huart5);
		xbee_send("ATWR", &huart5);
		xbee_send("ATCN", &huart5);
	}
	else {
		printf("xbee router setup failed! \r\n");
	}
}

void xbee_ir_setup() {
	if(xbee_enter_command(&huart4)) {
		xbee_send("ATID 2222", &huart4);
		xbee_send("ATCH 17", &huart4);
		xbee_send("ATMY 1", &huart4);
		xbee_send("ATDL 2", &huart4);
		xbee_send("ATWR", &huart4);
		xbee_send("ATCN", &huart4);
	}
	else {
		printf("xbee ir setup failed! \r\n");
	}
}

void motors_gpio_init(void) {
  HAL_GPIO_WritePin(DIR_L1_PORT, DIR_L1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_L2_PORT, DIR_L2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_R1_PORT, DIR_R1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DIR_R2_PORT, DIR_R2_PIN, GPIO_PIN_RESET);
}

void motor_left_set_speed(uint16_t duty) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);
}

void motor_right_set_speed(uint16_t duty) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty);
}

void motor_left_forward(uint16_t speed) {
    HAL_GPIO_WritePin(DIR_L1_PORT, DIR_L1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIR_L2_PORT, DIR_L2_PIN, GPIO_PIN_RESET);
    motor_left_set_speed(speed);
}

void motor_left_reverse(uint16_t speed) {
    HAL_GPIO_WritePin(DIR_L1_PORT, DIR_L1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_L2_PORT, DIR_L2_PIN, GPIO_PIN_SET);
    motor_left_set_speed(speed);
}
void motor_left_stop() {
	HAL_GPIO_WritePin(DIR_L1_PORT, DIR_L1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR_L2_PORT, DIR_L2_PIN, GPIO_PIN_SET);
    motor_left_set_speed(0);
}
void motor_right_forward(uint16_t speed) {
    HAL_GPIO_WritePin(DIR_R1_PORT, DIR_R1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIR_R2_PORT, DIR_R2_PIN, GPIO_PIN_RESET);
    motor_right_set_speed(speed);
}

void motor_right_reverse(uint16_t speed) {
    HAL_GPIO_WritePin(DIR_R1_PORT, DIR_R1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_R2_PORT, DIR_R2_PIN, GPIO_PIN_SET);
    motor_right_set_speed(speed);
}
void motor_right_stop() {
	HAL_GPIO_WritePin(DIR_R1_PORT, DIR_R1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR_R2_PORT, DIR_R2_PIN, GPIO_PIN_SET);
    motor_right_set_speed(0);
}

uint8_t find_brightest_pixel(const uint8_t *frame, uint8_t *max_val_out){
    uint8_t best_idx = 0;
    uint8_t best_val = 0;

    for (int i = 0; i < 32; i++) {
        uint8_t byte = frame[i];
        uint8_t hi = (byte >> 4) & 0x0F;
        uint8_t lo = byte & 0x0F;

        if (hi > best_val) {
            best_val = hi;
            best_idx = i * 2;
        }
        if (lo > best_val) {
            best_val = lo;
            best_idx = i * 2 + 1;
        }
    }

    if (max_val_out) {
        *max_val_out = best_val;
    }
    return best_idx;
}
uint8_t getStable(){
	//make array of counts ranging from 0-64 to see which pixel is brightest and appears most
	uint8_t count[64] = {0};

	for (int i = 0; i < history_count; i++){
		int idx = brightest_history[i];
		if (i < 64){
			count[idx]++;
		}
	}
	//find max and index of the most freq + brightest count
	int best_idx = 0;
	int best_count = 0;
	for (int i = 0; i < 64; i++){
		if (count[i] > best_count){
			best_count = count[i];
			best_idx = i;
		}
	}
	return (uint8_t)best_idx;
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
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  motors_gpio_init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_Delay(1000);
  xbee_router_setup();
  HAL_Delay(1000);
//  xbee_ir_setup();
//  HAL_Delay(1000);

  HAL_UART_Receive_IT(&huart5, xbee_int_buf, 2);
//  HAL_UART_Receive_IT(&huart4, xbee_glass_int_buf, 2);
  uint8_t command;
//  uint8_t left_value;
//  uint8_t right_value;

  uint8_t target_row = 4;
  uint8_t target_col = 4;

  led_Init(&hspi1);
  led_SetIllum(&hspi1, 0x01);
  led_setColor(&hspi1, 0x00, 0x00, 0x10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  HAL_Delay(200);
//
//	  HAL_Delay(200);

//	  HAL_Delay(200);

//	  HAL_Delay(200);
	  uint8_t temp_send[32];
	  HAL_I2C_Master_Transmit(&hi2c1, SAD_IRCAM_W, &reg, 1, 1000);
	  HAL_I2C_Master_Receive(&hi2c1, SAD_IRCAM_R, buf, 128, 1000);


	  int idx = 0;

	  float T_MIN = 18.0f;
	  float T_MAX = 32.0f;

	  for (int i = 0; i < 8; i++) {
		  for (int j = 0; j < 8; j += 2) {

			  uint16_t raw1 = (buf[i*16 + j*2 + 1] << 8) | buf[i*16 + j*2];
			  if (raw1 > 2047) raw1 -= 4096;
			  float t1 = raw1 * 0.25f;

			  if (t1 < T_MIN) t1 = T_MIN;
			  int p1 = (int)(((t1-T_MIN) / (T_MAX-T_MIN)) * 15.0f);

			  uint16_t raw2 = (buf[i*16 + (j+1)*2 + 1] << 8) | buf[i*16 + (j+1)*2];
			  if (raw2 > 2047) raw2 -= 4096;
			  float t2 = raw2 * 0.25f;

			  if (t2 < T_MIN) t2 = T_MIN;
			  int p2 = (int)(((t2-T_MIN) / (T_MAX-T_MIN)) * 15.0f);

			  temp_send[idx++] = (uint8_t)((p1 << 4) | (p2 & 0x0F));
		  }
	  }

//	  for (uint8_t i = 0; i < 32; i++) {
//		  uint8_t byte = temp_send[i];
//		  printf("%d %d  ", (byte >> 4) & 0x0F, byte & 0x0F);
//		  if (i % 4 == 3) printf("\n\r");
//	  }
//	  printf("-----------\n\r");
//		  HAL_UART_Transmit(&huart4, temp_send, 32, 100);
	  // In transmitter, after sending:

//	  HAL_UART_Receive(&huart5, buf, 2, 200);
	  if (xbee_int_ready){
		  command = xbee_int_buf[0];
		  if(command >> 7){
			  power = 1;
		  }else{
			  led_setColor(&hspi1, 0x00, 0x00, 0xFF);
			  power = 0;
			  motor_left_stop();
			  motor_right_stop();
		  }
		  printf("[debug] switch mode: %d \n", (command >> 6));
		  if((command >> 6) & 0b1){
			  control_mode = MODE_JOYSTICK;
		  }else{
			  control_mode = MODE_IR_ONLY;
		  }

		  if(control_mode == MODE_JOYSTICK && power){
			  led_setColor(&hspi1, 0x00, 0xFF, 0x00);
			  uint8_t right_command = command & 0b11;
			  int right_dir = command & 0b100;

			  command = (command >>3);
			  uint8_t left_command = command & 0b11;
			  int left_dir = command & 0b100;
			  printf("left: %d, right: %d\n\r", left_command, right_command);

			  uint32_t right_pwm = (50000*(uint32_t)right_command)/3+70000;
			  if(right_dir){
				  motor_right_forward(right_pwm);
			  }else if(left_command == 0){
				  motor_right_stop();
			  } else {
				  motor_right_reverse(right_pwm);
			  }

			  uint32_t left_pwm = (50000*left_command)/3+70000;
			  if(left_dir){
				  motor_left_forward(left_pwm);
			  }else if(left_command == 0){
				  motor_left_stop();
			  } else {
				  motor_left_reverse(left_pwm);
			  }
		  }
	  }


	  if (control_mode == MODE_IR_ONLY && power){
		  led_setColor(&hspi1, 0xFF, 0x00, 0x00);
		  uint8_t frame_max_val;
		  uint8_t frame_max_idx = find_brightest_pixel(temp_send, &frame_max_val);

		  // circular buffer
		  brightest_history[history_pos] = frame_max_idx;
		  history_pos = (history_pos + 1) % HISTORY_LEN;
		  if (history_count < HISTORY_LEN) {
		      history_count++;
		  }

		  //get stable index
		  uint8_t stable_idx = getStable();
		  uint8_t row = stable_idx / 8;
		  if(row < target_row){
			  target_row--;
		  }else if (row > target_row){
			  target_row++;
		  }
		  uint8_t col = stable_idx % 8;
		  if(col < target_col){
			  target_col--;
		  }else if (col > target_col){
			  target_col++;
		  }


		  printf("[Target]frame max idx=%d val=%d, stable idx=%d (row=%d col=%d)\n\r",
		             frame_max_idx, frame_max_val, stable_idx, target_row, target_col);

		  //PI CONTROLLER
		  float rotate_error = ((float)target_col) - 3.5f; //0+7/2
		  float forward_error = ((float)target_row) - 3.5f; //0+7/2

		  if (frame_max_val < 4){
			  motor_left_stop();
			  motor_right_stop();
		  }else{
			  error_integral += rotate_error * dt;

			  if (error_integral > 50.0f){
				  error_integral = 50.0f;
			  }
			  if (error_integral < -50.0f){
				  error_integral = -50.0f;
			  }

			  if(-1<rotate_error && rotate_error<1){
				  rotate_error = 0;
			  }
			  if(-3<forward_error && forward_error <2){
				  forward_error = 0;
			  }

			  float steering = Kp_rot * rotate_error + Ki * error_integral;
			  float  forward = Kp_x * forward_error;

			  float left_command = forward + steering;
			  float right_command = forward - steering;


			  if(left_command > 0){
				  uint32_t rotate_pwm =	70000 + (uint32_t)(left_command);
				  motor_left_forward(rotate_pwm);
//				  printf("[debug] Left: PI: err=%.2f, integ=%.2f, steer=%.2f, pwm=%ld\r\n",
//				  						  rotate_error, error_integral, steering, rotate_pwm);
			  }else{
				  uint32_t rotate_pwm =	70000 + (uint32_t)(-left_command);
				  motor_left_reverse(rotate_pwm);
//				  printf("[debug] Left: PI: err=%.2f, integ=%.2f, steer=%.2f, pwm=%ld\r\n",
//						  rotate_error, error_integral, steering, rotate_pwm);
			  }
			  if(right_command > 0){
				  uint32_t rotate_pwm =	70000 + (uint32_t)(right_command);
				  motor_right_forward(rotate_pwm);
//				  printf("[debug] Right: PI: err=%.2f, integ=%.2f, steer=%.2f, pwm=%ld\r\n",
//						  rotate_error, error_integral, steering, rotate_pwm);
			  }else{
				  uint32_t rotate_pwm =	70000 + (uint32_t)(-right_command);
				  motor_right_reverse(rotate_pwm);
//				  printf("[debug] Right: PI: err=%.2f, integ=%.2f, steer=%.2f, pwm=%ld\r\n",
//						  rotate_error, error_integral, steering, rotate_pwm);
			  }
		  }
	  }
	  if (HAL_UART_Transmit(&huart4, temp_send, 32, 100) == HAL_OK) {
		  printf("[debug] Sent 32 bytes via UART4\n");
	  //		      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Blink LED on transmit
	  } else {
		  printf("[debug] UART4 transmit failed\n");
	  }
//	  HAL_Delay(30);
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE9 PE10
                           PE11 PE12 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM1_COMP1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5) {
        xbee_int_ready = 1;
        HAL_UART_Receive_IT(&huart5, xbee_int_buf, 2);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5) {
        HAL_UART_Receive_IT(&huart5, xbee_int_buf, 2);
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
