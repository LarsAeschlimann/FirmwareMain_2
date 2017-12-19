/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32f1xx_it.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "shiftregister.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char buffertx[50];
char bufferrx[8];

//Shifreg stuff
volatile char shift_reg_values[HWADDR_CNT];
volatile unsigned int shift_reg_addr_cnt = 0;


char button_values_copy[HWADDR_CNT];//entspricht button_values mit invertierten Encoder
char fifoout[4];

char ringbuffer[BUFFERSIZE][STRINGLENGTH];//FIFO Ringbuffer
struct Buffer{
	char write;
	char read;
} buffer={0,0};

char singlestrings[BUFFERSIZE];

volatile unsigned int timecount;
volatile unsigned int cnt;
unsigned int cnt2;
unsigned int testcount = 0;

volatile char flag; 
volatile char sendflag = 0;

enum select sel = HUE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void ledpwm(void);
void ledtest(void);
void select(void);
void sendfunction(void);
void incremental_invert(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void ledpwm(void){//evt Division entfernen --> weniger Rechenaufwand
	
	if((timecount<=R_GREEN)&&(R_GREEN!=0)){LED_R_GREEN_SET();}else{LED_R_GREEN_RESET();}
	if((timecount<=G_GREEN)&&(G_GREEN!=0)){LED_G_GREEN_SET();}else{LED_G_GREEN_RESET();}
	if((timecount<=B_GREEN)&&(B_GREEN!=0)){LED_B_GREEN_SET();}else{LED_B_GREEN_RESET();}
		
	if((timecount<=R_YELLOW)&&(R_YELLOW!=0)){LED_R_YELLOW_SET();}else{LED_R_YELLOW_RESET();}
	if((timecount<=G_YELLOW)&&(G_YELLOW!=0)){LED_G_YELLOW_SET();}else{LED_G_YELLOW_RESET();}
	if((timecount<=B_YELLOW)&&(B_YELLOW!=0)){LED_B_YELLOW_SET();}else{LED_B_YELLOW_RESET();}
	
	if((timecount<=R_VIOLETT)&&(R_VIOLETT!=0)){LED_R_VIOLETT_SET();}else{LED_R_VIOLETT_RESET();}
	if((timecount<=G_VIOLETT)&&(G_VIOLETT!=0)){LED_G_VIOLETT_SET();}else{LED_G_VIOLETT_RESET();}
	if((timecount<=B_VIOLETT)&&(B_VIOLETT!=0)){LED_B_VIOLETT_SET();}else{LED_B_VIOLETT_RESET();}
	
	if((timecount<=R_BLUE)&&(R_BLUE!=0)){LED_R_BLUE_SET();}else{LED_R_BLUE_RESET();}
	if((timecount<=G_BLUE)&&(G_BLUE!=0)){LED_G_BLUE_SET();}else{LED_G_BLUE_RESET();}
	if((timecount<=B_BLUE)&&(B_BLUE!=0)){LED_B_BLUE_SET();}else{LED_B_BLUE_RESET();}
	
	if((timecount<=R_MAGENTA)&&(R_MAGENTA!=0)){LED_R_MAGENTA_SET();}else{LED_R_MAGENTA_RESET();}
//	if((timecount<=G_MAGENTA)&&(G_MAGENTA!=0)){LED_G_MAGENTA_SET();}else{LED_G_MAGENTA_RESET();}
//	if((timecount<=B_MAGENTA)&&(B_MAGENTA!=0)){LED_B_MAGENTA_SET();}else{LED_B_MAGENTA_RESET();}//Fehler wenn dieser Pin HIGH ist. Interrupt funktioniert nicht mehr!
	
	if((timecount<=R_RED)&&(R_RED!=0)){LED_R_RED_SET();}else{LED_R_RED_RESET();}
	if((timecount<=G_RED)&&(G_RED!=0)){LED_G_RED_SET();}else{LED_G_RED_RESET();}
	if((timecount<=B_RED)&&(B_RED!=0)){LED_B_RED_SET();}else{LED_B_RED_RESET();}
	
	if((timecount<=R_AQUAMARIN)&&(R_AQUAMARIN!=0)){LED_R_AQUAMARIN_SET();}else{LED_R_AQUAMARIN_RESET();}
	if((timecount<=G_AQUAMARIN)&&(G_AQUAMARIN!=0)){LED_G_AQUAMARIN_SET();}else{LED_G_AQUAMARIN_RESET();}
	if((timecount<=B_AQUAMARIN)&&(B_AQUAMARIN!=0)){LED_B_AQUAMARIN_SET();}else{LED_B_AQUAMARIN_RESET();}
	
	if((timecount<=R_ORANGE)&&(R_ORANGE!=0)){LED_R_ORANGE_SET();}else{LED_R_ORANGE_RESET();}
	if((timecount<=G_ORANGE)&&(G_ORANGE!=0)){LED_G_ORANGE_SET();}else{LED_G_ORANGE_RESET();}
	if((timecount<=B_ORANGE)&&(B_ORANGE!=0)){LED_B_ORANGE_SET();}else{LED_B_ORANGE_RESET();}
}

void ledtest(void){
	
	testcount++;
	for(unsigned int i=0; i<400000; i++){
			LED_R_RED_SET(); 				
			LED_R_ORANGE_SET(); 		
			LED_R_YELLOW_SET();		
			LED_R_GREEN_SET();	
			LED_R_BLUE_SET();	  
			LED_R_AQUAMARIN_SET(); 
			LED_R_MAGENTA_SET(); 	
			LED_R_VIOLETT_SET();
			LED_W_DOWN_SET();
	}
	
	for(unsigned int i=0; i<400000; i++){
			LED_G_RED_SET();				
			LED_G_ORANGE_SET(); 		
			LED_G_YELLOW_SET();		
			LED_G_GREEN_SET();	
			LED_G_BLUE_SET();	  
			LED_G_AQUAMARIN_SET(); 
			LED_G_MAGENTA_SET(); 	
			LED_G_VIOLETT_SET(); 	
			LED_W_MIDDLE_SET();
	}
	
	for(unsigned int i=0; i<400000; i++){
			LED_B_RED_SET(); 				
			LED_B_ORANGE_SET(); 		
			LED_B_YELLOW_SET();	
			LED_B_GREEN_SET();	
			LED_B_BLUE_SET();	  
			LED_B_AQUAMARIN_SET(); 
			LED_B_MAGENTA_SET();
			LED_B_VIOLETT_SET();
			LED_W_UP_SET();
	}
	LED_CLEAR_RESET();
}

void select(void){//Funktion fuer die weissen Select LEDs
	
	/*switch(sel){ todo
		case HUE:
			if((button_values[BUTTON_SEL_LUM])&&(!button_values[BUTTON_SEL_HUE])&&(!button_values[BUTTON_SEL_SAT]))sel = LUM;
			if((button_values[BUTTON_SEL_SAT])&&(!button_values[BUTTON_SEL_HUE])&&(!button_values[BUTTON_SEL_LUM]))sel = SAT;
			LED_W_UP_SET();
			LED_W_MIDDLE_RESET();
			LED_W_DOWN_RESET();
		break;
		
		case SAT:
			if((button_values[BUTTON_SEL_HUE])&&(!button_values[BUTTON_SEL_LUM])&&(!button_values[BUTTON_SEL_SAT]))sel = HUE;
			if((button_values[BUTTON_SEL_LUM])&&(!button_values[BUTTON_SEL_HUE])&&(!button_values[BUTTON_SEL_SAT]))sel = LUM;
			LED_W_UP_RESET();
			LED_W_MIDDLE_SET();
			LED_W_DOWN_RESET();
		break;
		
		case LUM:
			if((button_values[BUTTON_SEL_HUE])&&(!button_values[BUTTON_SEL_LUM])&&(!button_values[BUTTON_SEL_SAT]))sel = HUE;
			if((button_values[BUTTON_SEL_SAT])&&(!button_values[BUTTON_SEL_HUE])&&(!button_values[BUTTON_SEL_LUM]))sel = SAT;
			LED_W_UP_RESET();
			LED_W_MIDDLE_RESET();
			LED_W_DOWN_SET();
		break;
	}*/
}

void endcode_button(char cmd[], unsigned char selected_button)
{
	/*if(button_values_copy[BUTTON_FN]==0) todo
		cmd[0] = 'T'; //grossgeschrieben = Normalmodus
	else
		cmd[0] = 't'; //kleingeschrieben = FN Modus
	*/ 
	switch(sel)
	{
		case HUE: cmd[1] = 'A'; break;
		case SAT: cmd[1] = 'B'; break;
		case LUM: cmd[1] = 'C'; break;
		default : cmd[1] = '?'; break;
	}
	switch(selected_button)
	{
		//Encoderbuttons
		case BUTTON_CONTRAST		: cmd[2]='0'; cmd[3]='1'; break;	
    case BUTTON_SHADOW			: cmd[2]='0'; cmd[3]='2'; break;
    case BUTTON_BLACK				: cmd[2]='0'; cmd[3]='3'; break;
    case BUTTON_WHITE				: cmd[2]='0'; cmd[3]='4'; break;
		case BUTTON_LIGHTS			: cmd[2]='0'; cmd[3]='5'; break;
		case BUTTON_EXPOSURE		: cmd[2]='0'; cmd[3]='6'; break;	
		case BUTTON_CLARITY			: cmd[2]='0'; cmd[3]='7'; break;
		case BUTTON_DYNAMIC			: cmd[2]='0'; cmd[3]='8'; break;
	  case BUTTON_ENCSAT			: cmd[2]='0'; cmd[3]='9'; break;	
			
    case BUTTON_RED					: cmd[2]='1'; cmd[3]='0'; break;
    case BUTTON_ORANGE			: cmd[2]='1'; cmd[3]='1'; break;
    case BUTTON_YELLOW			: cmd[2]='1'; cmd[3]='2'; break;
    case BUTTON_GREEN				: cmd[2]='1'; cmd[3]='3'; break;
    case BUTTON_AQUA				: cmd[2]='1'; cmd[3]='4'; break;	
    case BUTTON_BLUE				: cmd[2]='1'; cmd[3]='5'; break;	
    case BUTTON_PURPLE			: cmd[2]='1'; cmd[3]='6'; break;
    case BUTTON_MAGENTA			: cmd[2]='1'; cmd[3]='7'; break;
																			
		case BUTTON_PROG				: cmd[2]='1'; cmd[3]='8'; break;
    case BUTTON_CROP				: cmd[2]='1'; cmd[3]='9'; break;
		
		//Buttons
		case BUTTON_UNDO				: cmd[2]='2'; cmd[3]='0'; break;	
    case BUTTON_REDO				: cmd[2]='2'; cmd[3]='1'; break;	
    case BUTTON_FULL				: cmd[2]='2'; cmd[3]='2'; break;	
    case BUTTON_COLOR_BW		: cmd[2]='2'; cmd[3]='3'; break;	
    case BUTTON_SEL_HUE			: cmd[2]='2'; cmd[3]='4'; break;
    case BUTTON_SEL_SAT			: cmd[2]='2'; cmd[3]='5'; break;
    case BUTTON_SEL_LUM			: cmd[2]='2'; cmd[3]='6'; break;
    case BUTTON_STAR_1			: cmd[2]='2'; cmd[3]='7'; break;
    case BUTTON_STAR_2			: cmd[2]='2'; cmd[3]='8'; break;
    case BUTTON_STAR_3			: cmd[2]='2'; cmd[3]='9'; break;
    case BUTTON_STAR_4			: cmd[2]='3'; cmd[3]='0'; break;
    case BUTTON_STAR_5			: cmd[2]='3'; cmd[3]='1'; break;
    case BUTTON_COPY				: cmd[2]='3'; cmd[3]='2'; break;	
    case BUTTON_PASTE				: cmd[2]='3'; cmd[3]='3'; break;
    case BUTTON_FN					: cmd[2]='3'; cmd[3]='4'; break;
    case BUTTON_PICK				: cmd[2]='3'; cmd[3]='5'; break;	
    case BUTTON_ZOOM				: cmd[2]='3'; cmd[3]='6'; break;
		case BUTTON_RIGHT				: cmd[2]='3'; cmd[3]='7'; break;
		case BUTTON_LEFT      	: cmd[2]='3'; cmd[3]='8'; break;		
		case BUTTON_UP					: cmd[2]='3'; cmd[3]='9'; break;
		case BUTTON_DOWN				: cmd[2]='4'; cmd[3]='0'; break;	
		case BUTTON_BEFOREAFTER	: cmd[2]='4'; cmd[3]='1'; break;
    case BUTTON_DEVELOP			: cmd[2]='4'; cmd[3]='2'; break;
		
		default 								: cmd[2]='?'; cmd[3]='?'; break;
	}
}

char fifoput(char* inputencode){
	
	if ((buffer.write + 1 == buffer.read)||(buffer.read == 0 && buffer.write + 1 == BUFFERSIZE))return 0;
	if(buffer.write >= BUFFERSIZE)buffer.write = 0;
	
	ringbuffer[buffer.write][0] = inputencode[0];
	ringbuffer[buffer.write][1] = inputencode[1];
	ringbuffer[buffer.write][2] = inputencode[2];
	ringbuffer[buffer.write][3] = inputencode[3];
	buffer.write++;

	return 1;
}

char fifoget(char* fifoout){//0 invalid, 1 valid
	
	if (buffer.read == buffer.write)return 0;
	
	fifoout[0] = ringbuffer[buffer.read][0];
	fifoout[1] = ringbuffer[buffer.read][1];
	fifoout[2] = ringbuffer[buffer.read][2];
	fifoout[3] = ringbuffer[buffer.read][3];
	buffer.read++;
	if(buffer.read >= BUFFERSIZE)buffer.read = 0;
	
	return 1;
}
 

void sendfunction(void){
		
		for(unsigned char i =0; i<48; i++)
		{
			if(button_values_copy[i] > 0) //Taste ist gedrückt
			{
				char cmd[4];
				endcode_button(cmd, i); 	//Taste Nr i decodieren
				fifoput(cmd);				//Decodierter Wert in Fifo buffer ablegen
			}
		}
	
	if(fifoget(fifoout)){
		strcpy(buffertx,"");
		strncat(buffertx, fifoout, 4);	
		HAL_UART_Transmit_IT(&huart4, (uint8_t *)buffertx, 8);
	}
}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
	//__HAL_UART_ENABLE_IT(&huart4, UART_IT_TC);
//	__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)//===========================================WHILE========================================================
  {
		HAL_TIM_Base_Stop_IT(&htim3);
		for(int i=0; i<HWADDR_CNT; i++)
		{
			//button_values_copy[i] = button_values[i]; todo
		}
		HAL_TIM_Base_Start_IT(&htim3);
		
		LED_CLEAR_SET();
		ledpwm();
		select();
		sendfunction();
	/*	if(cnt2>500){//sendfunction evt in timerinterupt oder doch UART IR
		sendfunction();
			cnt2 = 0;
		}
		cnt2++;*/
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 23;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 23;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC0 
                           PC1 PC2 PC3 PC4 
                           PC5 PC6 PC7 PC8 
                           PC9 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA7 PA8 
                           PA9 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB13 PB14 
                           PB15 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
