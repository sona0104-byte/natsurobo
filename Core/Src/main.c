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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "MD03.h"
#include "servoForSTM.h"
#include "AirSet.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX(i,j)(((i)>(j)) ? (i):(j))
#define ABS(X)((X) < 0 ? (-(X)):(X))
#define CHANGE_TO_RAD(X)((X)*pi/180.)
double normalize_angle(double i);
double LIMIT_ACCEL(double v, double old_v);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define UP 0
#define RIGHT 1
#define DOWN 2
#define LEFT 3
#define START 4
#define SELECT 5
#define TRIANGLE 6
#define CIRCLE 7
#define CROSS 8
#define SQUARE 9
#define L2 10
#define R2 11
#define L1 12
#define R1 13

#define Motor_UPPER_RIGHT 0
#define Motor_UPPER_LEFT 1
#define Motor_LOWER_LEFT 2
#define Motor_LOWER_RIGHT 3

#define pi 3.1415926

#define Rotate_Small_Speed CHANGE_TO_RAD(1.) //[rad/s]
#define Rotate_Max_Speed CHANGE_TO_RAD(90.) //[rad/s]

#define Rotate_P_Gain 0.007
#define Rotate_D_Gain 0.005
#define Frame_Length 0.3 //[m]
#define PD_const 2500

#define SERVO_RIGHT 1900
#define SERVO_LEFT  600

#define dt 0.05 //50ms timer interrupt

#define EPS 50  // ゼロ判定�??��閾値

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t DS3_BTNS_DATMAP_BYTE[16] = {
    2, 2, 2, 2, 2, 2,/* 0, 0,*/ 2, 2, 2, 1, 1, 1, 1, 1
};
uint8_t DS3_BTNS_DATMAP_BITMASK[16] = {
    0x03, 0x0C, 0x03, 0x0C, 0x03, 0x0C,/* 0x00, 0x00,*/
    0x10, 0x40, 0x20, 0x01, 0x04, 0x10, 0x02, 0x08,
};
uint8_t DS3_BTNS_DATMAP_FLAGBIT[16] = {
    0x01, 0x04, 0x02, 0x08, 0x03, 0x0C,/* 0x00, 0x00,*/
    0x10, 0x40, 0x20, 0x01, 0x04, 0x10, 0x02, 0x08,
};
uint8_t SBDBTRxData[8];
int buttonsIsPressed[16] = {0};
float LX = 0.0f;
float LY = 0.0f;
float RX = 0.0f;
float RY = 0.0f;

//フラグ?��?覧.
int is_relay_on = 0;
short is_PD_first = 0;
short power_down_mode = 0;
int v_goal = 0;

int is_air_on_shoot = 0;
int is_air_on_arm = 0;
int servo_rotate_arm = 0;
int servo_rotate_emit = 0;

//ジャイロ
float deg;
float deg_rad;
int deg_raw;

//PD制御用の変数
float deg_start_rad;
float target_rad = 0.;
float now_rad;
float error;
float old_error;
float rad;
float PD_v_rot;

//モーターの出?��?
float v_UPPER_RIGHT = 0;
float v_LOWER_RIGHT = 0;
float v_LOWER_LEFT  = 0;
float v_UPPER_LEFT  = 0;
float old_v_UPPER_RIGHT = 0;
float old_v_LOWER_RIGHT = 0;
float old_v_LOWER_LEFT = 0;
float old_v_UPPER_LEFT = 0;

#define VMAX_normal 550.0
#define VMAX_down 150.0
#define AMAX 0.5

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, SBDBTRxData, 8);

  uint8_t init_data[]={0xAA,0x00,0x3D,0x01,0x08};
  uint8_t req_data[]={0xAA,0x01,0x1A,0x02};
  uint8_t rec_data[4];

    HAL_Delay(500);

    HAL_UART_Transmit(&huart4, init_data, 5 , 100);
    HAL_UART_Receive(&huart4, rec_data, 2, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //リレーのONとOFF.
	  if(buttonsIsPressed[START] != 0){
		  if(is_relay_on == 0){
			  is_relay_on = GPIO_PIN_SET;
			  //printf("ON\r\n");
		}else{
			is_relay_on = GPIO_PIN_RESET;
			//printf("OFF\r\n");
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, is_relay_on);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, is_relay_on);
		HAL_Delay(300);
	  }

	  //ジャイロ変数の定義.
	  if(HAL_UART_Transmit(&huart4, req_data, 4 , 100)==HAL_OK){
		  if (HAL_UART_Receive(&huart4, rec_data, 2, 100)==HAL_OK) {
			  if (rec_data[0]==0xBB && rec_data[1]==2){
				  if (HAL_UART_Receive(&huart4, rec_data, 2, 100)==HAL_OK){
					  deg_raw=rec_data[0] | (rec_data[1]<<8);
		  			  deg=(float)deg_raw/16;
		  			  deg_rad = CHANGE_TO_RAD(deg);
		  			  deg_rad = normalize_angle(deg_rad);
		  		  }
		  	  }
		  }
	  }

	  //?��?ープLED
	  if(buttonsIsPressed[TRIANGLE] != 0){
		  if(v_goal == 0){
			  v_goal = 1;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);	//R
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);	//G
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);	//B
			  HAL_Delay(300);
		  }else{
			  v_goal = 0;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
			  HAL_Delay(300);
		  }
	  }

	  //ボタンにおける回転制御.
	  if(is_relay_on == GPIO_PIN_SET){
		  if(buttonsIsPressed[R1]){
			  target_rad += CHANGE_TO_RAD(90.);
			  HAL_Delay(300);
		  }
		  if(buttonsIsPressed[L1]){
			  target_rad -= CHANGE_TO_RAD(90.);
			  HAL_Delay(300);
		  }
	  }

	  //PD制御で車体安定させた?��?.
	  if(is_PD_first == 0){
		  deg_start_rad = deg_rad;
		  old_error = 0.0;
		  is_PD_first = 1;
	  }

	  if(buttonsIsPressed[SQUARE] != 0){
		  deg_start_rad = deg_rad;
		  old_error = 0.0;
	  }

	  //現在の角度.
	  now_rad = deg_rad - deg_start_rad;

	  error = normalize_angle(normalize_angle(target_rad) - now_rad);

	  rad = Rotate_P_Gain * error + Rotate_D_Gain*(error - old_error);
	  PD_v_rot = rad / dt;

	  //微小量を無?????��?��??��?��???��?��??��?��????��?��??��?��???��?��??��?��?.
	  if(ABS(target_rad - now_rad) < Rotate_Small_Speed) PD_v_rot = 0;

	  //?��?大速度制?��?
	  if(ABS(PD_v_rot) > Rotate_Max_Speed) PD_v_rot = Rotate_Max_Speed * (PD_v_rot / ABS(PD_v_rot));

	  //PD_v_rot = 0;//PD制御を無?��?

	  //printf("%f, %f, %f, %f, %f\r\n", deg_rad, now_rad, target_rad, error, PD_v_rot);
	  /*
	  //アー?��?の伸縮
	  if(buttonsIsPressed[UP]){
	  	  if(is_air_on_arm == 0){
		  	  is_air_on_arm = 1;
		  }else{
		  	  is_air_on_arm = 0;
		  }
		  AirSet(USART3, is_air_on_arm, is_air_on_shoot, 0, 0);
		  HAL_Delay(300);
	  }


	  */
	  //�?連のアー�?の動き
	  if(buttonsIsPressed[UP]){
		  if(servo_rotate_arm != SERVO_LEFT){
			  servo22SetPulse(USART3, 13, 0, SERVO_LEFT);
			  HAL_Delay(1000);
		  }
		  AirSet(USART3, 1, is_air_on_shoot, 0, 0);
		  HAL_Delay(500);
		  servo22SetPulse(USART3, 13, 0, SERVO_RIGHT);
		  HAL_Delay(1000);
		  AirSet(USART3, 0, is_air_on_shoot, 0, 0);
		  servo_rotate_arm = SERVO_RIGHT;
	  }
	  if(buttonsIsPressed[DOWN]){
		  if(servo_rotate_arm != SERVO_RIGHT){
			  servo22SetPulse(USART3, 13, 0, SERVO_RIGHT);
			  HAL_Delay(1000);
		  }
		  AirSet(USART3, 1, is_air_on_shoot, 0, 0);
		  HAL_Delay(500);
		  servo22SetPulse(USART3, 13, 0, SERVO_LEFT);
		  HAL_Delay(1000);
		  AirSet(USART3, 0, is_air_on_shoot, 0, 0);
		  servo_rotate_arm = SERVO_LEFT;
	  }
	  //アー??��?��?の回転
	  if(buttonsIsPressed[RIGHT]){
		  if(servo_rotate_arm != SERVO_RIGHT){
			  servo_rotate_arm = SERVO_RIGHT;
			  //printf("RIGHT");
		  }
		  servo22SetPulse(USART3, 13, 0, servo_rotate_arm);
		  HAL_Delay(300);
	  }
	  if(buttonsIsPressed[LEFT]){
		  if(servo_rotate_arm != SERVO_LEFT){
			  servo_rotate_arm = SERVO_LEFT;
			  //printf("LEFT");
		  }
		  servo22SetPulse(USART3, 13, 0, servo_rotate_arm);
		  HAL_Delay(300);
	  }
	  //?��?出機�?
	  if(buttonsIsPressed[CIRCLE]){
		  AirSet(USART3, 0, 1, 0, 0);
		  HAL_Delay(500);
		  AirSet(USART3, 0, 0, 0, 0);
	  }

	  //出力制限モー?��?
	  if(buttonsIsPressed[CROSS] != 0){
		  power_down_mode = 1;
	  }else{
		  power_down_mode = 0;
	  }


	  if(power_down_mode == 1){
		  //出力制限モー?��?
		  //コートたった時に使?��??��?すそ?��?な方
		  //モーターの回転成�?
		  v_UPPER_RIGHT = VMAX_down/128.f*(LY*sinf(    pi/4.f-deg_rad)+LX*cosf(    pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
		  v_UPPER_LEFT  = VMAX_down/128.f*(LY*sinf(3.f*pi/4.f-deg_rad)+LX*cosf(3.f*pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
		  v_LOWER_LEFT  = VMAX_down/128.f*(LY*sinf(5.f*pi/4.f-deg_rad)+LX*cosf(5.f*pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
		  v_LOWER_RIGHT = VMAX_down/128.f*(LY*sinf(7.f*pi/4.f-deg_rad)+LX*cosf(7.f*pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
		  /*
			  v_UPPER_RIGHT = VMAX_down/128.f*(LX*sinf(    pi/4.f-deg_rad)-LY*cosf(    pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
			  v_UPPER_LEFT  = VMAX_down/128.f*(LX*sinf(3.f*pi/4.f-deg_rad)-LY*cosf(3.f*pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
			  v_LOWER_LEFT  = VMAX_down/128.f*(LX*sinf(5.f*pi/4.f-deg_rad)-LY*cosf(5.f*pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
			  v_LOWER_RIGHT = VMAX_down/128.f*(LX*sinf(7.f*pi/4.f-deg_rad)-LY*cosf(7.f*pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
		   */
	  }else{
		  //通常モー?��?
		  //コートたった時に使?��??��?すそ?��?な方
		  //モーターの回転成�?
		  v_UPPER_RIGHT = VMAX_normal/128.f*(LY*sinf(    pi/4.f-deg_rad)+LX*cosf(    pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
		  v_UPPER_LEFT  = VMAX_normal/128.f*(LY*sinf(3.f*pi/4.f-deg_rad)+LX*cosf(3.f*pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
		  v_LOWER_LEFT  = VMAX_normal/128.f*(LY*sinf(5.f*pi/4.f-deg_rad)+LX*cosf(5.f*pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
		  v_LOWER_RIGHT = VMAX_normal/128.f*(LY*sinf(7.f*pi/4.f-deg_rad)+LX*cosf(7.f*pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
		  /*
			  v_UPPER_RIGHT = VMAX_normal/128.f*(LX*sinf(    pi/4.f-deg_rad)-LY*cosf(    pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
			  v_UPPER_LEFT  = VMAX_normal/128.f*(LX*sinf(3.f*pi/4.f-deg_rad)-LY*cosf(3.f*pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
			  v_LOWER_LEFT  = VMAX_normal/128.f*(LX*sinf(5.f*pi/4.f-deg_rad)-LY*cosf(5.f*pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
			  v_LOWER_RIGHT = VMAX_normal/128.f*(LX*sinf(7.f*pi/4.f-deg_rad)-LY*cosf(7.f*pi/4.f-deg_rad)) + PD_v_rot * Frame_Length * PD_const;
		   */
	  }


	  //printf("%d, %d, %d, %d\r\n",(int)v_UPPER_RIGHT, (int)v_UPPER_LEFT, (int)v_LOWER_LEFT, (int)v_LOWER_RIGHT);

	  //大出力を制??��?��?
	  float v_biggest = ABS(MAX(MAX(v_UPPER_RIGHT,v_UPPER_LEFT),MAX(v_LOWER_RIGHT,v_LOWER_LEFT)));

	  if(v_biggest > VMAX_normal){
		  v_UPPER_RIGHT	*= VMAX_normal / v_biggest;
		  v_UPPER_LEFT	*= VMAX_normal / v_biggest;
		  v_LOWER_LEFT	*= VMAX_normal / v_biggest;
		  v_LOWER_RIGHT	*= VMAX_normal / v_biggest;
	  }

	  //モーターを動かす
	  MD03SetMotor(USART3, Motor_UPPER_RIGHT, (int)v_UPPER_RIGHT);
	  MD03SetMotor(USART3, Motor_UPPER_LEFT, (int)v_UPPER_LEFT);
	  MD03SetMotor(USART3, Motor_LOWER_LEFT, (int)v_LOWER_LEFT);
	  MD03SetMotor(USART3, Motor_LOWER_RIGHT, (int)v_LOWER_RIGHT);

	  old_error = error;
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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Analyze_Raw_Data()
{
    for(int i=0; i<16; i++){
    	int f = (SBDBTRxData[DS3_BTNS_DATMAP_BYTE[i]] & DS3_BTNS_DATMAP_BITMASK[i]) == DS3_BTNS_DATMAP_FLAGBIT[i];
        buttonsIsPressed[i] = f;
    }
    LX = (float)(SBDBTRxData[3]) - 64.0f;
    LY = 64.0f - (float)(SBDBTRxData[4]);
    RX = (float)(SBDBTRxData[5]) - 64.0f;
    RY = 64.0f - (float)(SBDBTRxData[6]);
}

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
  return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    HAL_UART_Receive_IT(&huart1, SBDBTRxData, 8);
    Analyze_Raw_Data();
}

double normalize_angle(double rad){
	while(rad > pi) rad -= 2*pi;
	while(rad < -pi) rad += 2*pi;
	return rad;
}

double LIMIT_ACCEL(double v, double old_v) {
	float dv = v - old_v;
	if(ABS(LX) >= 0.1 || ABS(LY) >= 0.1){
		if(ABS(old_v) >= EPS){
			if(dv > AMAX)  v = old_v + AMAX;
			if(dv < -AMAX) v = old_v - AMAX;
		}
	}
	return v;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

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
