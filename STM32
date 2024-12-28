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
#include "stdio.h"
#include "myLCD.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//	Kamu dapat menggunakan fungsi HAL atau menggunakan macro yang disediakan.

/*
 * @brief	Macro untuk Output LED
 * @param	x		Boolean
 * 					0 untuk mematikan LED, 1 untuk mengaktifkan LED
 * @retval	None
 */
#define	LEDR(x)		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, !x)
#define	LEDG(x)		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, !x)
#define	LEDB(x)		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, !x)

/*
 * @brief	Macro untuk Output Shift Register
 * @param	x		Boolean
 *
 * @retval	None
 */
#define	DATA(x)		HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, x)
#define	CLOCK(x)	HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, x)
#define	LOAD(x)		HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, x)

/*
 * @brief	Macro untuk Output Buzzer
 * @param	x		Boolean
 * 					0 untuk mematikan Buzzer, 1 untuk mengaktifkan Buzzer
 * @retval	None
 */
#define	BUZZER(x)	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, x)

/*
 * @brief	Macro untuk input Push Button
 * @param	None
 * @retval	val		Boolean
 * 					0 saat tombol ditekan, 1 saat tombol dilepas
 */
#define SW1			(!HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin))
#define SW2			(!HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin))
#define SW3			(!HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin))
#define SW4			(!HAL_GPIO_ReadPin(SW4_GPIO_Port, SW4_Pin))
#define SW5			(!HAL_GPIO_ReadPin(SW5_GPIO_Port, SW5_Pin))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN PV */
uint16_t adc_vr = 0;
uint16_t adc_ldr = 0;

uint8_t pos_x = 0;
uint8_t pos_y = 0;
char data[16],data1[16];
uint8_t menu=1;
_Bool mSW1, mSW2, mSW3, mSW4, mSW5, hasiljam;
int baca_vr, baca_vr1, baca_vr2;
int vv,vv1,vr,vr1 ;
int x =0, detik, menit, jam;
int next_menu;
char hasil[16],uy[16];ay[16],iy[16];
int long hasily;
int h1;
int nilai1, nilai2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * @brief	fungsi membaca nilai ADC
 * @param	adc_vr untuk pembacaan ADC1
 * 			adc_ldr untuk pembacaan ADC2
 * @retval	val		adc_vr
 * 					adc_ldr
 * 					uint16
 */
void read_adc() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);

	HAL_ADC_PollForConversion(&hadc1, 100);
	HAL_ADC_PollForConversion(&hadc2, 100);

	adc_vr = HAL_ADC_GetValue(&hadc1);
	adc_ldr = HAL_ADC_GetValue(&hadc2);

	HAL_ADC_Stop(&hadc1);
	HAL_ADC_Stop(&hadc2);
}



void short_buzz(uint16_t delay) {
	BUZZER(1);
	HAL_Delay(delay);
	BUZZER(0);
}


/*
 * @brief	fungsi menampilkan data seven segment
 * @param	Data
 * 			isikan nilai data dengan data 8 bit
 * 			contoh angka 9 0B00001001;
 * 							dot,g,f,e,d,c,b,a,
 * 			0 untuk segment on dan 1 untuk segmen off
 *
 * @retval
 *
 *
 */

void send_595(uint8_t data) {
	for(uint8_t i=0; i<8; i++) {
		DATA((data >> i) & 0x01);

		CLOCK(1);
		CLOCK(0);
	}
	LOAD(1);
	LOAD(0);
}
void sendData (uint8_t dataLed){
	for (uint8_t i=0 ; i<8; i++){
		DATA(((dataLed>>i)&1));
		CLOCK(0);
		CLOCK(1);
	}
	LOAD(1);
	LOAD(0);
}
long Mapping(long x, long in_min, long in_max, long out_min, long out_max){
	return (x - in_min)*(out_max - out_min)/(in_max - in_min)+ out_min;
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  LEDR(0);
  LEDG(0);
  LEDB(0);

  LCD_Init();
  LCD_Clear();
  LCD_SetBacklight(1);



  for(uint8_t i=0; i<8; i++) {
	  DATA(1);
	  CLOCK(1);
	  CLOCK(0);
	}
	LOAD(1);
	LOAD(0);

	LCD_SetBacklight(0);
	HAL_Delay(2000);
	LCD_SetBacklight(1);

	LEDR(1);
	send_595(0b10011001);
	LCD_SetCursor(0, 0);
	LCD_Print("Basic Kalkulator");
	LCD_SetCursor(0, 1);
	LCD_Print("Ver. 1.0");
	HAL_Delay(2000);
	LCD_Clear();
	LEDR(0);
    send_595(0b11111111);

//	LCD_SetCursor(0, 0);
//	LCD_Print(" LKS SMK  2024");
//	LCD_SetCursor(0, 1);
//	LCD_Print("Nomor Meja: 4 ");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if ( menu == 1 ){
		  LCD_SetCursor(0,0);
		  LCD_Print(">Kalkukalor V1");
		  LCD_SetCursor(0,1);
		  LCD_Print(" Konversi Detik");

		  if ( mSW4==0 && SW4==1){
			  mSW4=1;
		  }
		  if ( mSW4==1 && SW4==0){

			  menu=2;
			  mSW4=0;
			  LCD_Clear();
		  }


		  if ( mSW3==0 && SW3==1 ){
			  mSW3=1;
			  short_buzz(50);
		  }
		  if ( mSW3==1 && SW3==0 ){
			  menu=0;
			 next_menu=1;
			  mSW3=0;
			  LCD_Clear();
		  }

	  }

	  if ( menu == 2 ){
		  LCD_SetCursor(0,0);
		  LCD_Print(" Kalkukalor V1");
		  LCD_SetCursor(0,1);
		  LCD_Print(">Konversi Detik");

		  if ( mSW2==0 && SW2==1){
			  mSW2=1;
		  }
		  if ( mSW2==1 && SW2==0){

			  menu=1;
			  mSW2=0;
			  LCD_Clear();
		  }

		  if ( mSW3==0 && SW3==1 ){
			  mSW3=1;
			  short_buzz(50);
		  }
		  if ( mSW3==1 && SW3==0 ){
			  menu=0;
			 next_menu=2;
			  mSW3=0;
			  LCD_Clear();
		  }

	  }

	  if ( next_menu==1 ){
		  read_adc();
		  LCD_SetCursor(0,0);
		  LCD_Print("basic kalkulator");
		  LCD_SetCursor(1, 1);
		  LCD_Print("AA");
		  LCD_SetCursor(7, 1);
		  LCD_Print("BB");
		  LCD_SetCursor(10, 1);
		  LCD_Print("=");
		  while(1){
		  read_adc();
		  LCD_SetCursor(0,0);
		  LCD_Print("basic kalkulator");
//		  vr = Mapping(adc_vr, 0, 4028, 0, 99);
//		  sprintf(data, "%2u", vr);
//		  vr1 = Mapping(adc_vr, 0, 4028, 0, 99);
//		  sprintf(data1, "%2u", vr1);

		  if (x==4){
			  x=0;
		  }

		  if (x==-1){
			  x=3;
		  }

		  if (mSW1==0 && SW1==1){
			  mSW1=1;
		  }
		  if (mSW1==1 && SW1==0){
			  vv=1;
			  vv1=0;
			  mSW1=0;
		  }
		  if (mSW3==0 && SW3==1){
			  mSW3=1;
		  }
		  if (mSW3==1 && SW3==0){
			  vv1=1;
			  vv=0;
			  mSW3=0;
		  }
		  if (mSW2==0 && SW2==1 ){
			  mSW2=1;
		  }
		  if (mSW2==1 && SW2==0 ){
			  x++;
			  mSW2=0;
		  }

		  if (mSW4==0 && SW4==1 ){
			  mSW4=1;
		  }
		  if (mSW4==1 && SW4==0 ){
			  x--;
			  mSW4=0;
		  }



		  if (vv == 1){
			  read_adc();
			  int vr = Mapping(adc_vr,0, 4028, 0, 99);
			  nilai1 = vr;
			  LCD_SetCursor(0, 1);
			  LCD_Print(">");
			  LCD_SetCursor(6, 1);
			  LCD_Print(" ");
			  LCD_SetCursor(1, 1);
			  sprintf(data, "%2u", vr);
			  LCD_Print(data);


			  if (mSW3==0 && SW3==1){
				  mSW3=1;
			  }
			  if (mSW3==1 && SW3==0){
				  vv1=1;
				  vv=0;
				  mSW3=0;


			  }



		  }


		  if (vv1 == 1){
			  read_adc();
			  int vr1 = Mapping(adc_vr,0, 4028, 0, 99);
			  nilai2 = vr1;
			  LCD_SetCursor(0, 1);
			  LCD_Print(" ");
			  LCD_SetCursor(6, 1);
			  LCD_Print(">");
			  LCD_SetCursor(7, 1);
			  sprintf(data1, "%2u", vr1);
			  LCD_Print(data1);

			  if (mSW1==0 && SW1==1){
				  mSW1=1;
			  }
			  if (mSW1==1 && SW1==0){
				  vv=1;
				  vv1=0;
				  mSW1=0;

			  }

		  }
		   if (x==0){
//			   hasil = vr1 + vr ;
			   			  	 LCD_SetCursor(4, 1);
			   				 LCD_Print("+");

		   	if (mSW5==0 && SW5==1){
			   		mSW5=1;
			   	}
		   	if (mSW5==1 && SW5==0){
		   		h1=1;
			   		mSW5=0;
				   		short_buzz(50);
			   	}
		   }
		   if (x==1){
//			   hasil = vr - vr1 ;
			   			  	 LCD_SetCursor(4, 1);
			   				 LCD_Print("-");

			   			   	if (mSW5==0 && SW5==1){
			   				   		mSW5=1;
			   				   	}
			   			   	if (mSW5==1 && SW5==0){
			   			   		h1=2;
			   				   		mSW5=0;
			   				   		short_buzz(50);
			   				   	}
		   }
		   if (x==2){
//			   hasil = data * data1 ;
			   			  	 LCD_SetCursor(4, 1);
			   				 LCD_Print("x");

			   			   	if (mSW5==0 && SW5==1){
			   				   		mSW5=1;
			   				   	}
			   			   	if (mSW5==1 && SW5==0){
			   			   		h1=3;
			   				   		mSW5=0;
			   				   		short_buzz(50);
			   				   	}
		   }
		   if (x==3){
//			   hasil = data / data1 ;
			   			  	 LCD_SetCursor(4, 1);
			   				 LCD_Print(":");

			   			   	if (mSW5==0 && SW5==1){
			   				   		mSW5=1;
			   				   	}
			   			   	if (mSW5==1 && SW5==0){
			   			   		h1=4;
			   				   		mSW5=0;
			   				   		short_buzz(50);
			   				   	}
		   }

		   if (h1==1){
			   hasily = nilai1 + nilai2;
				  LCD_SetCursor(12, 1);
				  sprintf(hasil, "%4u", hasily);
				  LCD_Print(hasil);
		   }
		   if (h1==2){
			   hasily = nilai1 - nilai2 ;
				  LCD_SetCursor(12, 1);
				  sprintf(hasil, "%4d", hasily);
				  LCD_Print(hasil);
		   }
		   if (h1==3){
			   hasily = nilai1 * nilai2 ;
				  LCD_SetCursor(12, 1);
				  sprintf(hasil, "%4u", hasily);
				  LCD_Print(hasil);
		   }
		   if (h1==4){
			   hasily = nilai1 / nilai2 ;
				  LCD_SetCursor(12, 1);
				  sprintf(hasil, "%4u", hasily);
				  LCD_Print(hasil);
		   }

//		  switch(x)
//		  {
//		  case 0:
//			  	 LCD_SetCursor(4, 1);
//				 LCD_Print("+");
//
//		  case 1:
//			  	 LCD_SetCursor(4, 1);
//				 LCD_Print("-");
//
//		  case 2:
//			  	 LCD_SetCursor(4, 1);
//				 LCD_Print("x");
//
//		  case 3:
//			  	 LCD_SetCursor(4, 1);
//				 LCD_Print(":");
//
//
//		  }

//		  LCD_SetCursor(4, 1);
//		  LCD_Print(hitung[1]);

//		  if (mSW2==0 && SW2==1 ){
//			  mSW2=1;
//		  }
//		  if (mSW2==1 && SW2==0 ){
//			  x++;
//			  mSW2=0;
//		  }
//
//		  if (mSW4==0 && SW4==1 ){
//			  mSW4=1;
//		  }
//		  if (mSW4==1 && SW4==0 ){
//			  x--;
//			  mSW4=0;
//		  }

//		   if (mSW4==0 && SW4==1){
//			   mSW4=1;
//		   }
//		   if (mSW4==1 && SW4==0){
//			   next_menu=0;
//			   mSW4=0;
//			   break;
//		   }
//



		  }

	  }

	  if ( next_menu==2 ){
//		  read_adc();
		  LCD_SetCursor(0,0);
		  LCD_Print("Data:");
		  LCD_SetCursor(4, 1);
		  LCD_Print("HH");
		  LCD_SetCursor(6, 1);
		  LCD_Print(":");
		  LCD_SetCursor(7, 1);
		  LCD_Print("MM");
		  LCD_SetCursor(9, 1);
		  LCD_Print(":");
		  LCD_SetCursor(10, 1);
		  LCD_Print("SS");
		  while(1){
			  read_adc();
			 int yaya = adc_vr;
		  LCD_SetCursor(6, 0);
		  sprintf(data, "%4u", adc_vr);
		  LCD_Print(data);
		  detik = yaya % 60;
		  menit = yaya  / 60;
		  jam = yaya  / 3600;


			   	if (mSW5==0 && SW5==1){
				   		mSW5=1;
				   	}
			   	if (mSW5==1 && SW5==0){
			   		hasiljam = 1;
				   		mSW5=0;
				   		short_buzz(50);
				   	}

			    if ( hasiljam==1 ){
					  LCD_SetCursor(4, 1);
					  sprintf(ay, "%2u", jam);
					  LCD_Print(ay);
					  LCD_SetCursor(7, 1);
					  sprintf(uy, "%2u", menit);
					  LCD_Print(uy);
					  LCD_SetCursor(10, 1);
					  sprintf(iy, "%2u", detik);
					  LCD_Print(iy);
			    }

				   if (mSW4==0 && SW4==1){
					   mSW4=1;
				   }
				   if (mSW4==1 && SW4==0){
					   next_menu=0;
					   mSW4=0;
					   break;
				   }

				if (yaya== 0){
					 LEDG(1);
					 LEDR(0);
					 LEDB(0);
				}
				if (yaya > 1 && yaya < 2000){
					 LEDR(1);
					 LEDG(1);
					 LEDB(0);
				}
				if (yaya > 2000){
					 LEDG(0);
					 LEDR(1);
					 LEDB(0);
				}
		  }
	  }



  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|LOAD_Pin|CLOCK_Pin|DATA_Pin
                          |LCD_BL_Pin|LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|LCD_E_Pin|LCD_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_B_Pin|LED_G_Pin|LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin SW4_Pin
                           SW5_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin|SW4_Pin
                          |SW5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin LOAD_Pin CLOCK_Pin DATA_Pin
                           LCD_BL_Pin LCD_D7_Pin LCD_D6_Pin LCD_D5_Pin
                           LCD_D4_Pin LCD_E_Pin LCD_RS_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|LOAD_Pin|CLOCK_Pin|DATA_Pin
                          |LCD_BL_Pin|LCD_D7_Pin|LCD_D6_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|LCD_E_Pin|LCD_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_B_Pin LED_G_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin|LED_G_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
