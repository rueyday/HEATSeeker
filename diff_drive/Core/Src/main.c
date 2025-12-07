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
#include "led.h"
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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
//uint8_t uart_buffer[2];
char xbee_buffer[100];
uint8_t xbee_int_buf[4];
volatile uint8_t xbee_int_ready = 0;

// FOR IR CAM 1
uint8_t buf[128];
uint8_t reg = 0x80;
#define SAD_IRCAM_W 0b11010010
#define SAD_IRCAM_R 0b11010011
volatile uint8_t glass_xbee_ready = 0;
uint8_t temp_send[32];
uint8_t xbee_glass_int_buf[2];
uint8_t glass_start_bytes[2] = {0xAA, 0x55};

//for temp tracking
#define HISTORY_LEN 4
uint8_t brightest_history[HISTORY_LEN];
uint8_t history_pos = 0; // where to write next
uint8_t history_count = 0; // how many valid entries (<= 50)

//for PI controller
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
int8_t button1 = 0;
int8_t button2 = 0;

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

//music buzzer stuff
#define TIMER_CLK_HZ 4000000.0f

static const float buzzer_freqs[] = {
			659.26, 493.88, 523.25, 587.33, 523.25, 493.88, 440.00, 220.00, 440.00, 523.25, 659.26, 587.33, 523.25, 493.88, 415.30, 523.25, 587.33, 659.26, 523.25, 440.00, 220.00, 220.00, 587.33, 698.46, 880.00, 783.99, 698.46, 659.26, 523.25, 659.26, 587.33, 523.25, 493.88, 246.94, 261.63, 587.33, 659.26, 523.25, 440.00
	};
	//ms
	static const float buzzer_durations[] = {
			406.35, 191.56, 185.76, 365.71, 203.17, 191.56, 162.54, 162.54, 168.34, 168.34, 371.52, 203.17, 191.56, 388.93, 145.12, 203.17, 377.32, 388.93, 383.13, 191.56, 609.52, 87.07, 406.35, 185.76, 383.13, 191.56, 191.56, 522.45, 214.78, 377.32, 197.37, 185.76, 214.78, 319.27, 168.34, 342.49, 394.74, 383.13, 853.33
	};

static const int buzzer_num_notes = sizeof(buzzer_freqs) / sizeof(buzzer_freqs[0]);


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
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
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

	HAL_UART_Transmit(huartx, (uint8_t*)cmd, strlen(cmd), 100);
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
float batteryValue(){
	float R1 = 10000.0;
	float R2 = 2200.0;
	float VREF = 3.3;
	float MAX = 4095.0;
	float scale = R2/(R1 + R2);

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return -1.0f; //error
    }

//    float sum = 0;
//    for (int i = 0; i < 500; i++){
//    	sum += HAL_ADC_GetValue(&hadc1);
//    }
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    printf("ADC raw = %lu\r\n", raw);

    //vadc = raw / 4096 * vref
    float VADC = (raw / MAX) * VREF;
    float VBAT = VADC / scale;

    return VBAT;
}
uint8_t batteryPercent(float vbat){
//	float sum = 0;
//	if (VBAT > 9.0){
//		for (int i = 0; i < 500; i++){
//			sum += 27.78 * VBAT - 250.0;
//		}
//		return sum / 500.0; //avg 500 samples
//
//	}
//	else{
//		return 0;
	const float V_EMPTY = 9.0f;
	    const float V_FULL  = 12.6f;
	    if (vbat <= V_EMPTY) return 0;
	    if (vbat >= V_FULL)  return 100;

	    float pct_f = (vbat - V_EMPTY) * 100.0f / (V_FULL - V_EMPTY);
	    if (pct_f < 0)   pct_f = 0;
	    if (pct_f > 100) pct_f = 100;

	    uint8_t pct = (uint8_t)(pct_f + 0.5f);

	    static uint8_t level = 0;
	    static uint8_t initialized = 0;

	    // thresholds in % for each step
	    // going UP:   15, 35, 55, 75, 95
	    // going DOWN:  0, 25, 45, 65, 85
	    static const uint8_t up[]   = {15, 35, 55, 75, 95};
	    static const uint8_t down[] = { 0, 25, 45, 65, 85};

	    if (!initialized) {
	        if      (pct >= 90) level = 5;
	        else if (pct >= 70) level = 4;
	        else if (pct >= 50) level = 3;
	        else if (pct >= 30) level = 2;
	        else if (pct >= 10) level = 1;
	        else                level = 0;
	        initialized = 1;
	    } else {
	        if (level < 5 && pct > up[level]) {
	            level++;
	        } else if (level > 0 && pct < down[level - 1]) {
	            level--;
	        }
    }

    switch(level){
		case 0:
			return 0;
		case 1:
			return 20;
		case 2:
			return 40;
		case 3:
			return 60;
		case 4:
			return 80;
		case 5:
			return 100;
		default:
			return 0;
    	}
	}


void buzzer_play_melody(void)
{
    for (int i = 0; i < buzzer_num_notes; i++) {
        float note = buzzer_freqs[i];

        //ARR = (timer_clk / freq) - 1, timer_clk = 4 MHz (MSI, PSC=0)
        uint32_t arr = (uint32_t)(TIMER_CLK_HZ / note) - 1;
        __HAL_TIM_SET_AUTORELOAD(&htim1, arr);

        //50% duty
        uint32_t duty = (arr + 1) / 2;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);

        __HAL_TIM_SET_COUNTER(&htim1, 0);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

        HAL_Delay((uint32_t)buzzer_durations[i]);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

        printf("playing: %f\n", buzzer_freqs[i]);
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
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  motors_gpio_init();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_Delay(1000);
  xbee_router_setup();
  HAL_Delay(1000);
  xbee_ir_setup();
  HAL_Delay(1000);

  HAL_UART_Receive_IT(&huart5, xbee_int_buf, 4);
//  HAL_UART_Receive_IT(&huart4, xbee_glass_int_buf, 2);
  uint8_t command;
  uint8_t info;
  int8_t rot_val = 0;
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
//	  buzzer_play_melody();
	  float batt = batteryValue();
	  float percent = batteryPercent(batt);
	  printf("battery value: %f\r\n", batt);
	  printf("battery percent: %f\r\n", percent);

//	  uint8_t temp_send[32];
//	  float span  = T_MAX - T_MIN;
//	  //normalize
//	  float n1 = (t1 - T_MIN) / span;\
//	  n1 *= gain;

	  uint8_t temp_send[32];
	  HAL_I2C_Master_Transmit(&hi2c1, SAD_IRCAM_W, &reg, 1, 1000);
	  HAL_I2C_Master_Receive(&hi2c1, SAD_IRCAM_R, buf, 128, 1000);

	  if (xbee_int_ready){

		  command = xbee_int_buf[0];
		  info = xbee_int_buf[1];
		  button1 = info%2;
		  button2 = (info%4 - button1)/2;
		  if(button1){
			  buzzer_play_melody;
		  }
		  if(info>>7){
			  rot_val = (int8_t)((info & 0b01111100)>>2);
		  }else{
			  rot_val = -(int8_t)((info & 0b01111100)>>2);
		  }

		  printf("buttons: %d, %d | rotation: %d\r\n", button1, button2, rot_val);
		  if(command >> 7){
			  power = 1;
		  }
		  else {
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
			  if(right_command == 0){
				  motor_right_stop();
			  }else if(right_dir){
				  motor_right_forward(right_pwm);
			  } else {
				  motor_right_reverse(right_pwm);
			  }

			  uint32_t left_pwm = (50000*left_command)/3+70000;
			  if(left_command == 0){
				  motor_left_stop();
			  }else if(left_dir){
				  motor_left_forward(left_pwm);
			  } else {
				  motor_left_reverse(left_pwm);
			  }
		  }
	  }

	  int idx = 0;

	  float T_MIN = 18.0f + (float)rot_val;
	  float T_MAX = 32.0f + (float)rot_val;

	  for (int i = 0; i < 8; i++) {
	      for (int j = 0; j <= 6; j += 2) {   // flipped left-right

	          uint16_t raw1 = (buf[i*16 + j*2 + 1] << 8) | buf[i*16 + j*2];
	          if (raw1 > 2047) raw1 -= 4096;
	          float t1 = raw1 * 0.25f;

	          if (t1 < T_MIN) t1 = T_MIN;
	          int p1 = (int)(((t1 - T_MIN) / (T_MAX - T_MIN)) * 15.0f);

	          uint16_t raw2 = (buf[i*16 + (j+1)*2 + 1] << 8) | buf[i*16 + (j+1)*2];
	          if (raw2 > 2047) raw2 -= 4096;
	          float t2 = raw2 * 0.25f;

	          if (t2 < T_MIN) t2 = T_MIN;
	          int p2 = (int)(((t2 - T_MIN) / (T_MAX - T_MIN)) * 15.0f);

	          temp_send[idx++] = (uint8_t)((p1 << 4) | (p2 & 0x0F));
	      }
	  }

	  for (uint8_t i = 0; i < 32; i++) {
		  uint8_t byte = temp_send[i];
		  printf("%d %d  ", (byte >> 4) & 0x0F, byte & 0x0F);
		  if (i % 4 == 3) printf("\n\r");
	  }
	  printf("-----------\n\r");

	  if ((control_mode == MODE_IR_ONLY) && power){
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
			  if(-1<rotate_error && rotate_error<1){
				  rotate_error = 0;
			  }
			  if(-3<forward_error && forward_error <2){
				  forward_error = 0;
			  }

			  float steering = Kp_rot * rotate_error;
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
	  HAL_UART_Transmit(&huart4, glass_start_bytes, 2, 100);
	  if (!HAL_UART_Transmit(&huart4, temp_send, 32, 100) == HAL_OK) {
		  printf("[debug] UART4 transmit failed\n");
	  }
	  HAL_Delay(400);
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
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
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100D14;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  /* Configure PC0 as analog for ADC */
	GPIO_InitStruct.Pin  = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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
        HAL_UART_Receive_IT(&huart5, xbee_int_buf, 4);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5) {
        HAL_UART_Receive_IT(&huart5, xbee_int_buf, 4);
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
