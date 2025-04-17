/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "omuni_3.h"
#include "servo.h"
#include "SO1602A.h"
#include "BNO055.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool SW_red = 0;			//takuto switch
bool SW_blue = 0;
bool SW_white = 0;
bool SW_black = 0;
bool status_1;				//toggle 1
bool status_2;				//toggle 2
bool status_3;				//toggle 3
bool servo_sel;

uint16_t speed;				//normal speed
uint16_t test_speed = 0;	//test speed
uint8_t test_servo;
int test_pulse1;
int test_pulse2;

uint16_t servo1_kakudo = servo1_init * 4;
uint16_t servo2_kakudo = servo2_init * 4;
uint16_t servo3_kakudo = servo3_init * 4;
uint16_t servo4_kakudo = servo4_init * 4;
uint16_t servo5_kakudo = servo5_init * 4;
uint16_t servo6_kakudo = servo6_init * 4;
uint16_t servo7_kakudo = servo7_init * 4;
uint16_t servo8_kakudo = servo8_init * 4;


int adcvalue;
uint16_t adc_value[2];
int adcvalue0;
int adcvalue1;
uint8_t cnt;

char str_1[5];				//strings for LCD display
char str_test[4];			//need to use strings for LCD
char str_servo[2];
char str_speed[2];
char str_potentio1[2];
char str_potentio2[2];
char str_bno[2];

char rxbuf[2];

BNO055 bno;				//define structure for BNO055

float defaultYaw;		//define the beggining value of BNO055
float defaultgyrox;
float defaultgyroy;
float defaultgyroz;

int robotYaw;			//define current value of BNO055
int robot_gyro_x;
int robot_gyro_y;
int robot_gyro_z;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0){				//toggle 3
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0) == 0){
			status_3 = 1;
		}else{
			status_3 = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_1){		//toggle 1
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1) == 0){
			status_1 = 1;
		}else{
			status_1 = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_2){		//toggle 2
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2) == 0){
			status_2 = 1;
		}else {
			status_2 = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_3){		//botton black
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3) == 0){
			SW_black = 1;
		}else {
			SW_black = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_4){		//botton white
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4) == 0){
			SW_white = 1;
		}else {
			SW_white = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_5){		//botton red
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5) == 0){
			SW_red = 1;
		}else {
			SW_red = 0;
		}
	}else if(GPIO_Pin == GPIO_PIN_6){		//botton blue
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6) == 0){
			SW_blue = 1;
		}else {
			SW_blue = 0;
		}
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  MX_UART7_Init();
  MX_USART3_UART_Init();
  MX_ADC3_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,SET);			//light up the LED on the board
  HAL_UART_Receive_DMA(&huart3 , (uint8_t *)rxbuf , 2 );		//tpip
  //HAL_UART_Receive_DMA(&huart3 , (uint8_t *)rxbuf , 2 );		//raspi

  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);		//dc motor pwm start
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);		//servo pwm start
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);

  oled_Init(&hi2c1);		//LCD initialize
  OLED_Init();
  set_cursor(1,0);
  Puts("start");
  set_cursor(2,0);
  Puts("3 gouki");

  BNO055_init(&bno , &hi2c1);		//BNO055 initialize

  char bno_check = BNO055_check(&bno);			//for debug
  BNO055_reset(&bno);
  BNO055_set_angle_units(&bno , DEGREES);
  BNO055_setpowermode(&bno , POWER_MODE_NORMAL);
  BNO055_setmode(&bno , OPERATION_MODE_NDOF);

  char bno_state = BNO055_bno_state(&bno);		//for debug
  //xprintf("%d , %d\n" , bno_check , bno_state);		//for debugging BNO055

  BNO055_get_angles(&bno);				//get initial value
  BNO055_get_gyro(&bno);
  defaultYaw = bno.euler.yaw;
  defaultgyrox = bno.gyro.x;
  defaultgyroy = bno.gyro.y;
  defaultgyroz = bno.gyro.z;

  servo_init();

  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,RESET);		//turn off the LED on the board
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,SET);			//turn on the motor driver	1 and 2
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,SET);			//turn on the motor driver  3 and 4


  //HAL_ADC_Start_DMA(&hadc3 , (uint32_t *)adc_value , 2 );

  /* USER CODE END 2 */
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(status_1 && status_2 && status_3){			//uart mode
		  HAL_UART_Receive_DMA(&huart3 , (uint8_t *)rxbuf , 2 );		//tpip

		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,SET);		//turn off the full color LED
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,SET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,SET);

		  speed = rxbuf[1];
		  speed = map(speed , 0 , 100 , 0 , 999);
		  sprintf(str_speed,"%d",speed);		//convert speed value to strings

		  OLED_Init();			//LCD initialize and display
		  set_cursor(1,0);
		  Puts("UART mode");
		  set_cursor(2,0);
		  Puts("Received ");
		  set_cursor(2,10);
		  Puts(rxbuf);
		  set_cursor(2,13);
		  Puts(str_speed);			//display speed value

		  if(rxbuf[0] == 'k'){
		  	  omuni(0,0);
		  	  motor_4(0,0);

	  	  }else if(rxbuf[0] == 'c'){
			  omuni(2,speed);			// forward
			  if(rxbuf[0] == 'k'){
				  for(int cnt = 500 ; cnt > 0 ; cnt--){
					  omuni(2 , cnt);
				  }
				  break;
			  }

		  }else if(rxbuf[0] == 'g'){
			  omuni(1,speed);			// back

		  }else if(rxbuf[0] == 'i'){		//turn right
			  omuni(5,speed);

		  }else if(rxbuf[0] == 'j'){		//turn left
			  omuni(6,speed);

		  }else if(rxbuf[0] == 'a'){		//right
			  omuni(4,speed);

		  }else if(rxbuf[0] == 'e'){		//left
			  omuni(3,speed);

		  }else if(rxbuf[0] == 'l'){		//rescue arm up
			  motor_4(999 , 0);

		  }else if(rxbuf[0] == 'm'){		//rescue arm down
			  motor_4(0 , 999);

		  }else if(rxbuf[0] == 'n'){		//camera arm up
			  for(;servo5_kakudo > 0 ; servo5_kakudo--){
				  servo_5(servo5_kakudo / 4);
				  HAL_Delay(5);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }

		  }else if(rxbuf[0] == 'o'){		//camera arm down
			  for(;servo5_kakudo < 600 ; servo5_kakudo++){
				  servo_5(servo5_kakudo / 4);
				  HAL_Delay(5);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }

		  }else if(rxbuf[0] == 'p'){		//right rescue arm CW
			  for(;servo1_kakudo > 0 ; servo1_kakudo--){
				  servo_1(servo1_kakudo / 4);
				  HAL_Delay(5);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }

		  }else if(rxbuf[0] == 'q'){		//right rescue arm CCW
			  for(;servo1_kakudo < 360  ; servo1_kakudo++){
				  servo_1(servo1_kakudo / 4);
				  HAL_Delay(5);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }

		  }else if(rxbuf[0] == 'r'){		//left rescue arm CW
			  for(;servo2_kakudo > 360 ; servo2_kakudo--){
				  servo_2(servo2_kakudo / 4);
				  HAL_Delay(5);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }

		  }else if(rxbuf[0] == 's'){		//left rescue arm CCW
			  for(;servo2_kakudo < 720 ; servo2_kakudo++){
				  servo_2(servo2_kakudo / 4);
				  HAL_Delay(5);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }

		  }else if(rxbuf[0] == 't'){		//rescue arm up
			  for(;servo3_kakudo < 720 ; servo3_kakudo++){
				  servo_3(servo3_kakudo / 4);
				  servo_4((int)map(servo3_kakudo , 0 , 720 , 720 , 0) / 4);
				  HAL_Delay(8);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }

		  }else if(rxbuf[0] == 'u'){		//rescue arm down
			  for(;servo3_kakudo > 0 ; servo3_kakudo--){
				  servo_3(servo3_kakudo / 4);
				  servo_4((int)map(servo3_kakudo , 0 , 720, 720 , 0) / 4);
				  HAL_Delay(8);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }

		  }else if(rxbuf[0] == 'v'){		//rescue arm initializing position
			  servo_init();
			  servo1_kakudo = servo1_init * 4;
			  servo2_kakudo = servo2_init * 4;
			  servo3_kakudo = servo3_init * 4;
			  servo4_kakudo = servo4_init * 4;

		  }else if(rxbuf[0] == 'w'){		//rescue arm close at the same time
			  for(;servo1_kakudo < 360 ; servo1_kakudo++){
				  servo_1(servo1_kakudo / 4);
				  servo_2((int)map(servo1_kakudo , 0 , 360 , 720 , 360) / 4);
				  HAL_Delay(8);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }

		  }else if(rxbuf[0] == 'x'){		//rescue arm open at the same time
			  for(;servo1_kakudo > 0 ; servo1_kakudo--){
				  servo_1(servo1_kakudo / 4);
				  servo_2((int)map(servo1_kakudo , 0 , 360 , 720 , 360) / 4);
				  HAL_Delay(8);
				  if(rxbuf[0] == 'k'){
					  break;
				  }
			  }
		  }

	  }else if(status_1){			//servo test mode  		under development   don't use

		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,RESET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,SET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,SET);

		  if(SW_white){			//servo no tanntai or fukusuu no sentaku
			  servo_sel = 1;
		  }else if(SW_black){
			  servo_sel = 0;
		  }

		  HAL_ADC_Start(&hadc3);
		  if(HAL_ADC_PollForConversion(&hadc3,1000) == HAL_OK){		//potentiometer no yomitori
			  adcvalue = HAL_ADC_GetValue(&hadc3);
			  test_pulse1 = map(adcvalue , 0 , 1023 , 0 , 180);
			  HAL_ADC_Stop(&hadc3);
		  }

		  //adcvalue0 = adc_value[0];
		  //adcvalue1 = adc_value[1];

		  //test_pulse1 = adcvalue0 * 180 / 1023;
		  //test_pulse2 = adcvalue1 * 180 / 1023;
		  sprintf(str_potentio1,"%d",test_pulse1);
		  //sprintf(str_potentio2,"%d",test_pulse2);

		  if(servo_sel){			//servo tanntai dousa mode
			  OLED_Init();
			  set_cursor(1,0);
			  Puts("singular");

			  if(SW_red){			// servo bangou no sentaku
				  test_servo += 1;
			  }else if(SW_blue){
				  test_servo -= 1;
			  }
			  sprintf(str_servo,"%d",test_servo);		//strings ni henkan


			  set_cursor(2,0);				//settei naiyou no hyouji
			  Puts("servo:");
			  set_cursor(2,7);
			  Puts(str_servo);
			  set_cursor(2,10);
			  Puts(str_potentio1);

			  servo_move(test_servo , test_pulse1);			//servo wo ugokasu

		  }else if(!servo_sel){			// servo fukusuu dousa mode

			  OLED_Init();
			  set_cursor(1,0);
			  Puts("manipulator");
			  set_cursor(2,0);
			  Puts("1:");
			  set_cursor(2,3);
			  Puts(str_potentio1);
			  set_cursor(2,7);
			  Puts("2:");
			  set_cursor(2,10);
			  Puts(str_potentio2);

		  }

		  //HAL_ADC_Stop_DMA(&hadc3);			//hituyou

	  }else if(status_2){		//DC motor speed setting mode

		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,SET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,RESET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,SET);

		  sprintf(str_test,"%d",test_speed);		//convert number to strings

		  OLED_Init();			//initialize LCD and diplay
		  set_cursor(1,0);
		  Puts("speed setting");
		  set_cursor(2,0);
		  Puts("speed =");
		  set_cursor(2,8);
		  Puts(str_test);

		  if(SW_red){
			  test_speed += 50;
		  }else if(SW_blue){
			  test_speed -= 50;
		  }
	  }else if(status_3){		//omuni wheel test mode
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,SET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,SET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,RESET);

		  BNO055_get_angles(&bno);		//get value of BNO055
		  BNO055_get_gyro(&bno);
		  robotYaw = bno.euler.yaw - defaultYaw;		//yaw angle calculation
		  //robot_gyro_x = bno.gyro.x - defaultgyrox;
		  //robot_gyro_y = bno.gyro.y - defaultgyroy;
		  //robot_gyro_z = bno.gyro.z - defaultgyroz;

		  sprintf(str_bno,"%d",robotYaw);		//convert number to strings for displaying sensor value

		  OLED_Init();				//initialize LCD and diplay
		  set_cursor(1,0);
		  Puts("test mode 1");
		  set_cursor(2,0);
		  Puts("Yaw:");
		  set_cursor(2,5);
		  Puts(str_bno);

		  if(SW_red){				//push red sw
			  omuni(2,test_speed);
			  set_cursor(2,10);
			  Puts("FW");		//forward

		  }else if(SW_blue){		//push blue sw
			  omuni(1 , test_speed);
			  set_cursor(2,10);
			  Puts("BK");		//back
		  }else if(SW_black){		//push black sw
			  omuni(5 , test_speed);
			  set_cursor(2,10);
			  Puts("TR");		//turn right
		  }else if(SW_white){		//push white sw
			  omuni(6 , test_speed);
			  set_cursor(2,10);
			  Puts("TL");		//turn left
		  }else{					//not pushing
			  omuni(0,0);
			  set_cursor(2,10);
			  Puts("stop");		//stop
		  }


	  }else{			// dc motor test mode

		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,RESET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,RESET);
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,SET);

		  OLED_Init();
		  set_cursor(1,0);
		  Puts("test mode 2");
		  set_cursor(2,0);
		  Puts("aiueo");

		  if(SW_red){
			  motor_4(test_speed , 0);
		  }else if(SW_blue){
			  motor_4(0 , test_speed);
		  }else{
			  motor_4(0 , 0);
		  }

	  }


		  HAL_Delay(2);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
