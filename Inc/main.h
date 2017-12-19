/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

//shiftreg
extern volatile char shift_reg_values[];
extern volatile unsigned int shift_reg_addr_cnt;

extern volatile unsigned int timecount;
extern volatile unsigned int cnt;
extern volatile char flag;


extern volatile char sendflag;
char button_get_active_state(char encoder);

#define BUFFERSIZE 48
#define STRINGLENGTH 8



enum select{HUE,SAT,LUM};


#define R_RED				100
#define G_RED				0
#define B_RED				0

#define R_ORANGE		100
#define G_ORANGE		1
#define B_ORANGE		0

#define R_YELLOW		90
#define G_YELLOW		10
#define B_YELLOW		0

#define R_GREEN			0
#define G_GREEN			100
#define B_GREEN			0

#define R_BLUE			0
#define G_BLUE			0
#define B_BLUE			50

#define R_AQUAMARIN 30
#define G_AQUAMARIN	80
#define B_AQUAMARIN	100

#define R_MAGENTA 	100
#define G_MAGENTA 	100
#define B_MAGENTA		100

#define R_VIOLETT		100
#define G_VIOLETT		0
#define B_VIOLETT		10

#define LED_CLEAR_SET()					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define LED_CLEAR_RESET()				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define LED_CLK_SET()						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)
#define LED_CLK_RESET()					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)

#define LED_W_UP_SET()					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define LED_W_MIDDLE_SET()			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define LED_W_DOWN_SET()				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)

#define LED_W_UP_RESET()				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_W_MIDDLE_RESET()		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define LED_W_DOWN_RESET()			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)

#define LED_R_RED_SET() 				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define LED_R_ORANGE_SET() 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define LED_R_YELLOW_SET() 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET)
#define LED_R_GREEN_SET() 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)
#define LED_R_BLUE_SET()		  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET)
#define LED_R_AQUAMARIN_SET() 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET)
#define LED_R_MAGENTA_SET() 		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)
#define LED_R_VIOLETT_SET() 		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET)

#define LED_R_RED_RESET() 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)
#define LED_R_ORANGE_RESET() 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define LED_R_YELLOW_RESET() 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET)
#define LED_R_GREEN_RESET() 		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)
#define LED_R_BLUE_RESET()		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET)
#define LED_R_AQUAMARIN_RESET() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET)
#define LED_R_MAGENTA_RESET() 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET)
#define LED_R_VIOLETT_RESET() 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET)

#define LED_G_RED_SET() 				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)
#define LED_G_ORANGE_SET() 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)
#define LED_G_YELLOW_SET() 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET)
#define LED_G_GREEN_SET() 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET)
#define LED_G_BLUE_SET()		  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET)
#define LED_G_AQUAMARIN_SET() 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET)
#define LED_G_MAGENTA_SET() 		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define LED_G_VIOLETT_SET() 		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)
            
#define LED_G_RED_RESET() 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET)
#define LED_G_ORANGE_RESET() 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)
#define LED_G_YELLOW_RESET() 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET)
#define LED_G_GREEN_RESET() 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET)
#define LED_G_BLUE_RESET()		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET)
#define LED_G_AQUAMARIN_RESET() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET)
#define LED_G_MAGENTA_RESET() 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_G_VIOLETT_RESET() 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)

#define LED_B_RED_SET() 				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define LED_B_ORANGE_SET() 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)
#define LED_B_YELLOW_SET() 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
#define LED_B_GREEN_SET() 			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)
#define LED_B_BLUE_SET()		  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)
#define LED_B_AQUAMARIN_SET() 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)
#define LED_B_MAGENTA_SET() 		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)
#define LED_B_VIOLETT_SET() 		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)
            
#define LED_B_RED_RESET() 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define LED_B_ORANGE_RESET() 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)
#define LED_B_YELLOW_RESET() 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)
#define LED_B_GREEN_RESET() 		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
#define LED_B_BLUE_RESET()		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)
#define LED_B_AQUAMARIN_RESET() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)
#define LED_B_MAGENTA_RESET() 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET)
#define LED_B_VIOLETT_RESET() 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)

#define LED_DEBUG1_SET()				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define LED_DEBUG1_RESET() 			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define LED_DEBUG2_SET()	 			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define LED_DEBUG2_RESET() 			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
	
#define SHIFT_CLK_SET() 				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define SHIFT_CLK_RESET() 			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
	
#define SHIFT_DATA() 						HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)
	
#define SHIFT_ENABLE() 					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define SHIFT_DISABLE() 				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)

#define BUTTON_LEFT       	3
#define BUTTON_RIGHT				0
#define BUTTON_UP						2
#define BUTTON_DOWN					1
#define BUTTON_UNDO					22
#define BUTTON_REDO					21
#define BUTTON_FULL					23
#define BUTTON_COLOR_BW			16
#define BUTTON_SEL_HUE			17
#define BUTTON_SEL_SAT			18
#define BUTTON_SEL_LUM			19
#define BUTTON_STAR_1				44
#define BUTTON_STAR_2				45
#define BUTTON_STAR_3				46
#define BUTTON_STAR_4				47
#define BUTTON_STAR_5				12
#define BUTTON_COPY					43
#define BUTTON_PASTE				42
#define BUTTON_DEVELOP			11
#define BUTTON_PICK					41
#define BUTTON_ZOOM					40
#define BUTTON_BEFOREAFTER	35
#define BUTTON_FN						36

#define BUTTON_RED				15
#define BUTTON_ORANGE			8
#define BUTTON_YELLOW			27
#define BUTTON_GREEN			26
#define BUTTON_AQUA				25
#define BUTTON_BLUE				30
#define BUTTON_PURPLE			29
#define BUTTON_MAGENTA		28
#define BUTTON_CONTRAST		14
#define BUTTON_SHADOW			9
#define BUTTON_BLACK			34
#define BUTTON_WHITE			37
#define BUTTON_CLARITY		24
#define BUTTON_DYNAMIC		31
#define BUTTON_PROG				7
#define BUTTON_ENCSAT			6
#define BUTTON_EXPOSURE		13
#define BUTTON_LIGHTS			10
#define BUTTON_CROP				20



/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
