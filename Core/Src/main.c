/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "pca9685.h"

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

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
extern char ReceivedData[100];
extern uint8_t Rxcount;
extern uint32_t dataSize;
extern uint8_t check;
// Message animation received from Jetson
char s1[] = "GoLeft";
char s2[] = "GoRight";
char s3[] = "Forward";
char s4[] = "Back";
char s5[] = "Left";
char s6[] = "Right";
char s7[] = "Neckup";
char s8[] = "NeckDown";
char s9[] = "Nod";
char s10[] = "RightHand";
char s11[] = "LeftHand";
char s12[] = "LRHand";
char s13[] = "ShakeHand";
// Value to feedback neck
int countRight = 0;
int countLeft = 0;
// Data test USB CDC
uint8_t data[100] = "STM32 USB CDC\r\n";

uint32_t Time=0;
//void PS2_send(unsigned char commands[], unsigned char data[], int length);
uint8_t RXD[21] = {0};
uint8_t TXD[21] = {0};
uint8_t analog_mode[9] = {0x01,0x44,0x00,0x01,0x03,0,0,0,0};

uint8_t TEST;
uint8_t count=0;
uint8_t one_time_animation=1;
uint8_t servo_angle[9]={75,75,60,60,60,60,60,60,60};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void goRight();
void goLeft();
void forward();
void back();
void leftNeck();
void rightNeck();
void neckUp();
void neckDown();
void right_arm(uint8_t up_down, uint8_t in_out) ;
void left_arm(uint8_t up_down, uint8_t in_out);
void leftHand();
void rightHand();
void LRHand();
void shakeHand();
void hello();
void bye();
void nod();
void stop();
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
  TXD[0] = 0x01;
  TXD[1] = 0x42;
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  PCA9685_Init(&hi2c1);
  for (int i = 0; i < 9; i++) {
  	PCA9685_SetServoAngle(i, servo_angle[i]);
  	HAL_Delay(1000);
  	}

  HAL_GPIO_WritePin(GPIOA, STEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, STEP_DIR_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //CDC_Transmit_FS(data, strlen(data));
	  	  if(check == 1){ //neu co du lieu den thi truyen di du lieu vua nhan duoc
		  		CDC_Transmit_FS((uint8_t *)ReceivedData, strlen(ReceivedData));
		  		if(strcmp(ReceivedData, s1) == 0){
		  				goLeft();
		  		}

		  		else if(strcmp(ReceivedData, s2) == 0){
		  				goRight();
		  		}

		  		else if(strcmp(ReceivedData, s3) == 0){
		  				forward();
		  		}

		  		else if(strcmp(ReceivedData, s4) == 0){
		  				back();
		  		}

		  		else if(strcmp(ReceivedData, s6) == 0){
		  				countRight += 1;
		  				leftNeck();
		  			  	if(countRight > 10){
		  			  		for (int i = 0; i < 10; i++){
								rightNeck();
						  		countRight--;
								for(int j = 0; j < 1; j++){
									goRight();
							  		PCA9685_SetServoAngle(0, servo_angle[0]);
							  		PCA9685_SetServoAngle(1, servo_angle[1]);
								}
		  			  		}
		  			  	}
		  			  	stop();
		  		}

		  		else if(strcmp(ReceivedData, s5) == 0){
						countLeft += 1;
						rightNeck();
						if(countLeft > 10){
							for (int i = 0; i < 10; i++){
								leftNeck();
								countLeft--;
								for(int j = 0; j < 5; j++){
									goLeft();
							  		PCA9685_SetServoAngle(0, servo_angle[0]);
							  		PCA9685_SetServoAngle(1, servo_angle[1]);
								}
							}
						}
						stop();
		  		}

		  		else if(strcmp(ReceivedData, s7) == 0){
		  			neckUp();
		  		}

		  		else if(strcmp(ReceivedData, s8) == 0){
		  			neckDown();
		  		}

		  		else if(strcmp(ReceivedData, s9) == 0){
		  			nod();
		  			neckDown();
		  		}

		  		else if(strcmp(ReceivedData, s10) == 0){
		  			rightHand();
		  		}

		  		else if(strcmp(ReceivedData, s11) == 0){
		  			leftHand();
		  		}

		  		else if(strcmp(ReceivedData, s12) == 0){
		  			LRHand();
		  		}

		  		else if(strcmp(ReceivedData, s13) == 0){
		  			shakeHand();
		  		}

		  		else{
		  			stop();
		  		}

		  		for (int i = 0; i < 9; i++) {
		  		 	PCA9685_SetServoAngle(i, servo_angle[i]);
		  		}

		  		for(int i = 0; i < dataSize; i++)
		  		{
		  			ReceivedData[i] = 0;
		  		}
		  		check = 0;
		  	}


//	 	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
//	 	//HAL_Delay(1);
//	 	for (int i = 0; i < 9; i++) {
//	 		while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX_RX) {
//	 		}
//	 		HAL_SPI_TransmitReceive(&hspi1, &TXD[i], &RXD[i], 1, 10);
//			//HAL_Delay(15/1000);
//	 	}
//	 	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
//	 	HAL_Delay(1);
//	 	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

	 	// select analog mode
//	 	if (RXD[1] == 115) {
//	 		// servo[0] & servo[1] is left right esc to control left and right motor
//	 		// angle = 75 mean stop. higher value is forward while lower is back.
//	 		//left motor back
//	 		if (!(RXD[4] >> 2 & 1)) {
//	 			servo_angle[0]++;
//	 			if (servo_angle[0] > 150)
//					servo_angle[0] = 150;
//	 		} else if (!(RXD[4] >> 0 & 1)) { //left motor forward
//	 			servo_angle[0]--;
//	 			if (servo_angle[0] < 1)
//	 				servo_angle[0] = 1;
//	 		} else {
//	 			if (servo_angle[0] > 75)
//	 				servo_angle[0]--;
//	 			if (servo_angle[0] < 75)
//	 				servo_angle[0]++;
//	 		}
//	 		//right motor forward
//	 		if (!(RXD[4] >> 3 & 1)) {
//	 			servo_angle[1]++;
//	 			if (servo_angle[1] > 150)
//	 				servo_angle[1] = 150;
//	 		} else if (!(RXD[4] >> 1 & 1)) { //right motor back
//	 			servo_angle[1]--;
//	 			if (servo_angle[1] < 1)
//	 				servo_angle[1] = 1;
//	 		} else {
//	 			if (servo_angle[1] > 75)
//	 				servo_angle[1]--;
//	 			if (servo_angle[1] < 75)
//	 				servo_angle[1]++;
//	 		}
//
//	 		switch (RXD[3]) {
//	 		case 239:		//NECK UP
//	 			servo_angle[2]++;
//	 			if (servo_angle[2] > 180)
//	 				servo_angle[2] = 180;
//	 			break;
//	 		case 191:		//NECK DOWN
//	 			servo_angle[2]--;
//	 			if (servo_angle[2] < 1)
//	 				servo_angle[2] = 1;
//	 			break;
//	 		case 127:
//	 			//TURN ON STEPPER
//	 			HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_SET);
//	 			//SET DIRECT = 0
//	 			HAL_GPIO_WritePin(GPIOA, STEP_DIR_Pin, GPIO_PIN_RESET);
//	 			for (count = 0; count < 100; count++) {
//	 				HAL_GPIO_WritePin(GPIOA, STEP_Pin, GPIO_PIN_SET);
//	 				HAL_Delay(0.01);
//	 				HAL_GPIO_WritePin(GPIOA, STEP_Pin, GPIO_PIN_RESET);
//	 				HAL_Delay(0.01);
//	 			}
//	 			HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_RESET);
//	 			break;
//	 		case 223:
//	 			//TURN ON STEPPER
//	 			HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_SET);
//	 			//SET DIRECT = 1
//	 			HAL_GPIO_WritePin(GPIOA, STEP_DIR_Pin, GPIO_PIN_SET);
//	 			for (count = 0; count < 100; count++) {
//	 				HAL_GPIO_WritePin(GPIOA, STEP_Pin, GPIO_PIN_SET);
//	 				HAL_Delay(0.01);
//	 				HAL_GPIO_WritePin(GPIOA, STEP_Pin, GPIO_PIN_RESET);
//	 				HAL_Delay(0.01);
//	 			}
//	 			HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_RESET);
//	 			break;
//	 		case 255:
//	 			//TURN OFF STEPPER
//	 			HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_RESET);
//	 			break;
//	 		}
//	 		//left shoulder up-down
//	 		if (RXD[6] > 200)
//	 			servo_angle[3]++;
//	 		if (RXD[6] < 50)
//	 			servo_angle[3]--;
//	 		if (servo_angle[3] < 1)
//	 			servo_angle[3] = 1;
//	 		if (servo_angle[3] > 180)
//	 			servo_angle[3] = 180;
//	 		//left shoulder in-out
//	 		if (RXD[5] > 200)
//	 			servo_angle[4]++;
//	 		if (RXD[5] < 50)
//	 			servo_angle[4]--;
//	 		if (servo_angle[4] < 1)
//	 			servo_angle[4] = 1;
//	 		if (servo_angle[4] > 180)
//	 			servo_angle[4] = 180;
//	 		//right shoulder up-down
//	 		if (RXD[8] > 200)
//	 			servo_angle[5]++;
//	 		if (RXD[8] < 50)
//	 			servo_angle[5]--;
//	 		if (servo_angle[5] < 1)
//	 			servo_angle[5] = 1;
//	 		if (servo_angle[5] > 180)
//	 			servo_angle[5] = 180;
//	 		//right shoulder up-down
//	 		if (RXD[7] > 200)
//	 			servo_angle[6]++;
//	 		if (RXD[7] < 50)
//	 			servo_angle[6]--;
//	 		if (servo_angle[6] < 1)
//	 			servo_angle[6] = 1;
//	 		if (servo_angle[6] > 180)
//	 			servo_angle[6] = 180;
//	 		//hello
//	 		if (!(RXD[4] >> 6 & 1)) {
//	 			if (one_time_animation)
//	 				hello();
//	 			one_time_animation = 1 - one_time_animation;
//	 		}
//	 		//bye
//	 		if (!(RXD[4] >> 4 & 1)) {
//	 			if (one_time_animation)
//	 				bye();
//	 			one_time_animation = 1 - one_time_animation;
//	 		}
//	 		TEST=!(RXD[3] >> 1 & 1);
//	 		//left hand
//	 		if (!(RXD[3] >> 1 & 1)) {
//	 			if (servo_angle[7] < 60)
//	 				servo_angle[7]++;
//	 		} else if (servo_angle[7] > 2) {
//	 			servo_angle[7]--;
//	 		}
//	 		//right hand
//	 		if (!(RXD[3] >> 2 & 1)) {
//	 			if (servo_angle[8] < 60)
//	 				servo_angle[8]++;
//	 		} else if (servo_angle[8] > 2) {
//	 			servo_angle[8]--;
//	 		}

//  	}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.CRCPolynomial = 10;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|STEP_Pin|STEP_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI1_CS_Pin STEP_Pin STEP_DIR_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|STEP_Pin|STEP_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SLEEP_Pin */
  GPIO_InitStruct.Pin = SLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SLEEP_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void goRight(){
	servo_angle[0] += 5;
	servo_angle[1] -= 5;
	if (servo_angle[0] > 150)
		servo_angle[0] = 150;
	if (servo_angle[1] < 5)
		servo_angle[1] = 5;
}

void goLeft(){
	servo_angle[0] -= 5;
	servo_angle[1] += 5;
	if(servo_angle[0] < 5)
		servo_angle[0] = 5;
	if(servo_angle[1] > 150)
		servo_angle[1] = 150;
}

void forward(){
	servo_angle[0] += 5 ;
	servo_angle[1] += 5;
	if (servo_angle[0] > 150)
		servo_angle[0] = 150;
	if (servo_angle[1] > 150)
		servo_angle[1] = 150;
}

void back(){
	servo_angle[0] -= 5;
	servo_angle[1] -= 5;
	if (servo_angle[0] < 5)
		servo_angle[0] = 5;
	if (servo_angle[1] < 5)
		servo_angle[1] = 5;
}

void rightNeck(){
	//TURN ON STEPPER
	HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_SET);
	//SET DIRECT = 0
	HAL_GPIO_WritePin(GPIOA, STEP_DIR_Pin, GPIO_PIN_RESET);

	for (count = 0; count < 20; count++) {
		HAL_GPIO_WritePin(GPIOA, STEP_Pin, GPIO_PIN_SET);
		HAL_Delay(0.01);
		HAL_GPIO_WritePin(GPIOA, STEP_Pin, GPIO_PIN_RESET);
		HAL_Delay(0.01);
	 	}

	HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_RESET);
}

void neckUp(){
	servo_angle[2] += 5;
	if (servo_angle[2] > 180){
		servo_angle[2] = 180;
	}
	PCA9685_SetServoAngle(2, servo_angle[2]);
}

void neckDown(){
	servo_angle[2] -= 5;
	if (servo_angle[2] < 0){
		servo_angle[2] = 0;
	}
	PCA9685_SetServoAngle(2, servo_angle[2]);
}

void leftNeck(){
	//TURN ON STEPPER
	HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_SET);
	//SET DIRECT = 1
	HAL_GPIO_WritePin(GPIOA, STEP_DIR_Pin, GPIO_PIN_SET);
	for (count = 0; count < 20; count++) {
		HAL_GPIO_WritePin(GPIOA, STEP_Pin, GPIO_PIN_SET);
		HAL_Delay(0.01);
		HAL_GPIO_WritePin(GPIOA, STEP_Pin, GPIO_PIN_RESET);
		HAL_Delay(0.01);
		}

	HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_RESET);
}

void right_arm(uint8_t up_down, uint8_t in_out) {
	servo_angle[3] = up_down;
	servo_angle[4] = in_out;
	PCA9685_SetServoAngle(3, up_down);
	PCA9685_SetServoAngle(4, in_out);
}

void left_arm(uint8_t up_down, uint8_t in_out) {
	servo_angle[5] = up_down;
	servo_angle[6] = in_out;
	PCA9685_SetServoAngle(5, up_down);
	PCA9685_SetServoAngle(6, in_out);
}

void leftHand(){
	left_arm(170, 60);
	HAL_Delay(200);
	left_arm(170, 170);
	HAL_Delay(200);
	left_arm(170, 60);
	HAL_Delay(200);
	left_arm(170, 170);
	HAL_Delay(200);
	left_arm(170, 60);
	HAL_Delay(200);
	left_arm(170, 170);
	HAL_Delay(200);
	left_arm(60, 60);
	HAL_Delay(200);
}

void rightHand(){
	right_arm(170, 60);
	HAL_Delay(200);
	right_arm(170, 5);
	HAL_Delay(200);
	right_arm(170, 60);
	HAL_Delay(200);
	right_arm(170, 5);
	HAL_Delay(200);
	right_arm(170, 60);
	HAL_Delay(200);
	right_arm(170, 5);
	HAL_Delay(200);
	right_arm(60, 60);
	HAL_Delay(200);
}

void LRHand(){
	right_arm(170, 60);
	left_arm(170, 60);
	HAL_Delay(200);
	right_arm(170, 5);
	left_arm(170, 170);
	HAL_Delay(200);
	right_arm(170, 60);
	left_arm(170, 60);
	HAL_Delay(200);
	right_arm(170, 5);
	left_arm(170, 170);
	HAL_Delay(200);
	right_arm(170, 60);
	left_arm(170, 60);
	HAL_Delay(200);
	right_arm(170, 5);
	left_arm(170, 170);
	HAL_Delay(200);
	right_arm(60, 60);
	left_arm(60, 60);
	HAL_Delay(200);
}

void shakeHand(){
	left_arm(60, 60);
	HAL_Delay(300);
	left_arm(170, 60);
	HAL_Delay(300);
	left_arm(60, 60);
	HAL_Delay(300);
	left_arm(170, 60);
	HAL_Delay(300);
	left_arm(60, 60);
}

void hello(){
	left_arm(24, 108);
	HAL_Delay(500);
	right_arm(33, 25);
	HAL_Delay(500);
	right_arm(170, 58);
	HAL_Delay(500);
	right_arm(170, 10);
}

void bye() {
	left_arm(24, 108);
	HAL_Delay(1000);
	right_arm(33, 25);
	HAL_Delay(1000);
	for (int i = 0; i <= 4; i++) {
		right_arm(170, 20);
		HAL_Delay(300);
		right_arm(170, 40);
		HAL_Delay(300);
	}
}

void stop(){
	servo_angle[0] = 75;
	servo_angle[1] = 75;
}

void nod(){
	for (int i = 0; i < 2; i++){
		for (int x = 0; x < 10; x++){
			neckDown();
			HAL_Delay(50);
		}
		for (int y = 0; y < 10; y++){
			neckUp();
			HAL_Delay(50);
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
