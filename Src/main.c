/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include <stdlib.h>

//plot size 43*43 pixel

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint32_t Led_a[] = {a1_Pin,a2_Pin,a3_Pin,a4_Pin,a5_Pin,a6_Pin,a7_Pin,a8_Pin,a9_Pin,a10_Pin,a11_Pin,a12_Pin,a13_Pin,a14_Pin,a15_Pin,a16_Pin,a17_Pin};
static uint32_t Led_b[] = {b1_Pin,b2_Pin,b3_Pin,b4_Pin,b5_Pin,b6_Pin,b7_Pin,b8_Pin,b9_Pin,b10_Pin,b11_Pin,b12_Pin,b13_Pin,b14_Pin,b15_Pin,b16_Pin,b17_Pin};
static GPIO_TypeDef * Led_a_port[] = {a1_GPIO_Port,a2_GPIO_Port,a3_GPIO_Port,a4_GPIO_Port,a5_GPIO_Port,a6_GPIO_Port,a7_GPIO_Port,a8_GPIO_Port,a9_GPIO_Port,a10_GPIO_Port,a11_GPIO_Port,a12_GPIO_Port,a13_GPIO_Port,a14_GPIO_Port,a15_GPIO_Port,a16_GPIO_Port,a17_GPIO_Port};
static GPIO_TypeDef * Led_b_port[] = {b1_GPIO_Port,b2_GPIO_Port,b3_GPIO_Port,b4_GPIO_Port,b5_GPIO_Port,b6_GPIO_Port,b7_GPIO_Port,b8_GPIO_Port,b9_GPIO_Port,b10_GPIO_Port,b11_GPIO_Port,b12_GPIO_Port,b13_GPIO_Port,b14_GPIO_Port,b15_GPIO_Port,b16_GPIO_Port,b17_GPIO_Port};

uint16_t frame = 0;
uint16_t angle = 0;
uint16_t period_rpm = 0;
uint16_t period_step = 0;

static int16_t cosin_table[200][17] = {0};


float debug = 0;
float debug1 = 1;
const float xx [] = {
	0,-0.0314,-0.0628,-0.0941,-0.1253,-0.1564,
 -0.1874,-0.2181,-0.2487,-0.2790,-0.3090,-0.3387,
 -0.3681,-0.3971,-0.4258,-0.4540,-0.4818,-0.5090,
 -0.5358,-0.5621,-0.5878,-0.6129,-0.6374,-0.6613,
 -0.6845,-0.7071,-0.7290,-0.7501,-0.7705,-0.7902,
 -0.8090,-0.8271,-0.8443,-0.8607,-0.8763,-0.8910,
 -0.9048,-0.9178,-0.9298,-0.9409,-0.9511,-0.9603,
 -0.9686,-0.9759,-0.9823,-0.9877,-0.9921,-0.9956,
 -0.9980,-0.9995,-1.0000,-0.9995,-0.9980,-0.9956,
 -0.9921,-0.9877,-0.9823,-0.9759,-0.9686,-0.9603,
 -0.9511,-0.9409,-0.9298,-0.9178,-0.9048,-0.8910,
 -0.8763,-0.8607,-0.8443,-0.8271,-0.8090,-0.7902,
 -0.7705,-0.7501,-0.7290,-0.7071,-0.6845,-0.6613,
 -0.6374,-0.6129,-0.5878,-0.5621,-0.5358,-0.5090,
 -0.4818,-0.4540,-0.4258,-0.3971,-0.3681,-0.3387,
 -0.3090,-0.2790,-0.2487,-0.2181,-0.1874,-0.1564,
 -0.1253,-0.0941,-0.0628,-0.0314, 0,      0.0314,
  0.0628, 0.0941, 0.1253, 0.1564, 0.1874, 0.2181,
  0.2487, 0.2790, 0.3090, 0.3387, 0.3681, 0.3971,
  0.4258, 0.4540, 0.4818, 0.5090, 0.5358, 0.5621,
  0.5878, 0.6129, 0.6374, 0.6613, 0.6845, 0.7071,
  0.7290, 0.7501, 0.7705, 0.7902, 0.8090, 0.8271,
  0.8443, 0.8607, 0.8763, 0.8910, 0.9048, 0.9178,
  0.9298, 0.9409, 0.9511, 0.9603, 0.9686, 0.9759,
  0.9823, 0.9877, 0.9921, 0.9956, 0.9980, 0.9995,
  1.0000, 0.9995, 0.9980, 0.9956, 0.9921, 0.9877,
  0.9823, 0.9759, 0.9686, 0.9603, 0.9511, 0.9409,
  0.9298, 0.9178, 0.9048, 0.8910, 0.8763, 0.8607,
  0.8443, 0.8271, 0.8090, 0.7902, 0.7705, 0.7501,
  0.7290, 0.7071, 0.6845, 0.6613, 0.6374, 0.6129,
  0.5878, 0.5621, 0.5358, 0.5090, 0.4818, 0.4540,
  0.4258, 0.3971, 0.3681, 0.3387, 0.3090, 0.2790,
  0.2487, 0.2181, 0.1874, 0.1564, 0.1253, 0.0941,
  0.0628, 0.0314,
};

const float yy [] = {
-1.0000,-0.9995,-0.9980,-0.9956,-0.9921,-0.9877,
-0.9823,-0.9759,-0.9686,-0.9603,-0.9511,-0.9409,
-0.9298,-0.9178,-0.9048,-0.8910,-0.8763,-0.8607,
-0.8443,-0.8271,-0.8090,-0.7902,-0.7705,-0.7501,
-0.7290,-0.7071,-0.6845,-0.6613,-0.6374,-0.6129,
-0.5878,-0.5621,-0.5358,-0.5090,-0.4818,-0.4540,
-0.4258,-0.3971,-0.3681,-0.3387,-0.3090,-0.2790,
-0.2487,-0.2181,-0.1874,-0.1564,-0.1253,-0.0941,
-0.0628,-0.0314,      0, 0.0314, 0.0628, 0.0941,
 0.1253, 0.1564, 0.1874, 0.2181, 0.2487, 0.2790,
 0.3090, 0.3387, 0.3681, 0.3971, 0.4258, 0.4540,
 0.4818, 0.5090, 0.5358, 0.5621, 0.5878, 0.6129,
 0.6374, 0.6613, 0.6845, 0.7071, 0.7290, 0.7501,
 0.7705, 0.7902, 0.8090, 0.8271, 0.8443, 0.8607,
 0.8763, 0.8910, 0.9048, 0.9178, 0.9298, 0.9409,
 0.9511, 0.9603, 0.9686, 0.9759, 0.9823, 0.9877,
 0.9921, 0.9956, 0.9980, 0.9995, 1.0000, 0.9995,
 0.9980, 0.9956, 0.9921, 0.9877, 0.9823, 0.9759,
 0.9686, 0.9603, 0.9511, 0.9409, 0.9298, 0.9178,
 0.9048, 0.8910, 0.8763, 0.8607, 0.8443, 0.8271,
 0.8090, 0.7902, 0.7705, 0.7501, 0.7290, 0.7071,
 0.6845, 0.6613, 0.6374, 0.6129, 0.5878, 0.5621,
 0.5358, 0.5090, 0.4818, 0.4540, 0.4258, 0.3971,
 0.3681, 0.3387, 0.3090, 0.2790, 0.2487, 0.2181,
 0.1874, 0.1564, 0.1253, 0.0941, 0.0628, 0.0314,
      0,-0.0314,-0.0628,-0.0941,-0.1253,-0.1564,
-0.1874,-0.2181,-0.2487,-0.2790,-0.3090,-0.3387,
-0.3681,-0.3971,-0.4258,-0.4540,-0.4818,-0.5090,
-0.5358,-0.5621,-0.5878,-0.6129,-0.6374,-0.6613,
-0.6845,-0.7071,-0.7290,-0.7501,-0.7705,-0.7902,
-0.8090,-0.8271,-0.8443,-0.8607,-0.8763,-0.8910,
-0.9048,-0.9178,-0.9298,-0.9409,-0.9511,-0.9603,
-0.9686,-0.9759,-0.9823,-0.9877,-0.9921,-0.9956,
-0.9980,-0.9995,
};
static const  uint8_t pic [] =  {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFE, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF,
0xC0, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x3F,
0x8F, 0xE1, 0xFE, 0x00, 0x00, 0x7E, 0x0F, 0xE0, 0x7F, 0x00, 0x00, 0xF8, 0x0F, 0xE0, 0x3F, 0x80,
0x01, 0xF0, 0x0F, 0xE0, 0x0F, 0xC0, 0x03, 0xE0, 0x0F, 0xE0, 0x07, 0xE0, 0x07, 0xC0, 0x0F, 0xE0,
0x03, 0xE0, 0x07, 0x80, 0x0F, 0xE0, 0x01, 0xF0, 0x0F, 0x80, 0x0F, 0xE0, 0x01, 0xF8, 0x1F, 0x00,
0x0F, 0xE0, 0x00, 0xF8, 0x1E, 0x00, 0x0F, 0xE0, 0x00, 0x7C, 0x3E, 0x00, 0x0F, 0xE0, 0x00, 0x7C,
0x3C, 0x00, 0x0F, 0xE0, 0x00, 0x3E, 0x3C, 0x00, 0x0F, 0xE0, 0x00, 0x3E, 0x7C, 0x00, 0x0F, 0xE0,
0x00, 0x3E, 0x7C, 0x00, 0x0F, 0xE0, 0x00, 0x3E, 0x78, 0x00, 0x0F, 0xE0, 0x00, 0x1E, 0x78, 0x00,
0x0F, 0xE0, 0x00, 0x1F, 0x78, 0x80, 0x0F, 0xE0, 0x00, 0x1F, 0x78, 0xC0, 0x0F, 0xE0, 0x03, 0x1F,
0x78, 0x70, 0x0F, 0xE0, 0x06, 0x1F, 0x78, 0x78, 0x0F, 0xE0, 0x1E, 0x1F, 0x78, 0x7E, 0x0F, 0xE0,
0x3C, 0x1F, 0x78, 0x3F, 0x0F, 0xE0, 0xFC, 0x1F, 0x7C, 0x3F, 0xCF, 0xE1, 0xFC, 0x3E, 0x7C, 0x1F,
0xCF, 0xE7, 0xF8, 0x3E, 0x3C, 0x0F, 0x8F, 0xE7, 0xF0, 0x3E, 0x3C, 0x00, 0x0F, 0xE0, 0xE0, 0x7E,
0x3E, 0x00, 0x0F, 0xE0, 0x00, 0x7C, 0x1E, 0x00, 0x0F, 0xE0, 0x00, 0x7C, 0x1F, 0x00, 0x0F, 0xE0,
0x00, 0xF8, 0x0F, 0x80, 0x0F, 0xE0, 0x01, 0xF8, 0x0F, 0x80, 0x0F, 0xE0, 0x01, 0xF0, 0x07, 0xC0,
0x0F, 0xE0, 0x03, 0xE0, 0x03, 0xE0, 0x0F, 0xE0, 0x07, 0xE0, 0x01, 0xF0, 0x0F, 0xE0, 0x0F, 0xC0,
0x00, 0xFC, 0x0F, 0xE0, 0x3F, 0x80, 0x00, 0x7E, 0x0F, 0xE0, 0x7F, 0x00, 0x00, 0x3F, 0x8F, 0xE1,
0xFE, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x01,
0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x7F, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00
};


 


static const  uint8_t Untitled [] = {
0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00,
0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0xFF, 0xFF,
0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00,
0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80,
0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0xFF, 0xFF,
0xFF, 0x80, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00,
0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80,
0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0xFF, 0xFF,
0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
0xB6, 0xDB, 0x6D, 0xD5, 0xAD, 0x6B, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF,
0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0x00, 0x00,
0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF,
0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x7F,
0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
0x00, 0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,
0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF,
0xFF, 0xFF, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF
};

//const  uint8_t test [] = {
//0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
//};

	/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

int aa = 1.9;

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void led_a_write(uint32_t tmp);
void led_b_write(uint32_t tmp);
void start_point(void);
void angle_point(void);
void angle_plot(void);
void build_table(void){
	for (int x = 0; x<200 ; x++)// angle
	{
		
		//float xx = -sin((float)x*0.0314159f);
		//float yy = -cos((float)x*0.0314159f);
		
 		for (int y = 0; y < 17; y++) // number
 		{
	
//			int cal_y = (float)(20.0f + (float)(4.0f+(float)y)*yy[x])*0.9302f;
//			int cal_x = (float)(20.0f + (float)(4.0f+(float)y)*xx[x])*0.9302f;	

//			int cal_y = (float)(21.5f + (float)(5.5f+(float)y)*yy[x])*0.9302f;
//			int cal_x = (float)(21.5f + (float)(5.5f+(float)y)*xx[x])*0.9302f;	
			
			int cal_y = (float)(24.0f + (float)(6.0f+(float)y)*yy[x]);
			int cal_x = (float)(24.0f + (float)(6.0f+(float)y)*xx[x]);
			
			cosin_table[x][y] = cal_y*48 + cal_x;

     }
	}
}
/* USER CODE END PFP */
        
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	build_table();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		led_a_write(0xffffffff);
		led_b_write(0xffffffff);
		HAL_Delay(100);
		led_a_write(0);
		led_b_write(0);
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//		led_a_write(0);
//		led_b_write(0);
//		//HAL_Delay(100);
//		if(GPIO_PIN_SET == HAL_GPIO_ReadPin(Pulse_GPIO_Port,Pulse_Pin))
//		{
//		led_a_write(0xffffffff);
//		led_b_write(0xffffffff);
//		}
		
//		angle_plot();
//		HAL_Delay(10);

//		if(angle>=200)angle=0;
		//HAL_Delay(100);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, b9_Pin|b8_Pin|b7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, b5_Pin|b6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, b4_Pin|b3_Pin|b2_Pin|b1_Pin 
                          |a17_Pin|a16_Pin|a14_Pin|a5_Pin 
                          |a4_Pin|a3_Pin|a2_Pin|a1_Pin 
                          |b17_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, a15_Pin|a13_Pin|a12_Pin|a11_Pin 
                          |a10_Pin|a9_Pin|a8_Pin|a6_Pin 
                          |a7_Pin|b16_Pin|b15_Pin|b14_Pin 
                          |b13_Pin|b12_Pin|b11_Pin|b10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : b9_Pin b8_Pin b7_Pin */
  GPIO_InitStruct.Pin = b9_Pin|b8_Pin|b7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : b5_Pin b6_Pin */
  GPIO_InitStruct.Pin = b5_Pin|b6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : b4_Pin b3_Pin b2_Pin b1_Pin 
                           a17_Pin a16_Pin a14_Pin a5_Pin 
                           a4_Pin a3_Pin a2_Pin a1_Pin 
                           b17_Pin */
  GPIO_InitStruct.Pin = b4_Pin|b3_Pin|b2_Pin|b1_Pin 
                          |a17_Pin|a16_Pin|a14_Pin|a5_Pin 
                          |a4_Pin|a3_Pin|a2_Pin|a1_Pin 
                          |b17_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Pulse_Pin */
  GPIO_InitStruct.Pin = Pulse_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Pulse_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : a15_Pin a13_Pin a12_Pin a11_Pin 
                           a10_Pin a9_Pin a8_Pin a6_Pin 
                           a7_Pin b16_Pin b15_Pin b14_Pin 
                           b13_Pin b12_Pin b11_Pin b10_Pin */
  GPIO_InitStruct.Pin = a15_Pin|a13_Pin|a12_Pin|a11_Pin 
                          |a10_Pin|a9_Pin|a8_Pin|a6_Pin 
                          |a7_Pin|b16_Pin|b15_Pin|b14_Pin 
                          |b13_Pin|b12_Pin|b11_Pin|b10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
void angle_point(void){
	uint32_t tmp1 = 0;
	uint32_t tmp2 = 0;
	uint32_t mask = 0;
	
	for(int index = 0; index<17; index++){
//		tmp1 |= mask && 
		
		mask = mask << 1;
	}
	led_a_write(tmp1);
	led_b_write(tmp2);
	
	angle ++;
}

void start_point(void){
	
	period_rpm = TIM4->CNT;
	TIM4->CNT = 0;
	
	period_step = period_rpm / 2 ;
	TIM3->CNT = 0;
	TIM3->ARR = period_step;
	frame++;
	angle = 0;
}

void angle_plot(void){
	
//	if(angle == 50 ||angle == 0||angle == 100||angle == 150)
//	{
//				led_a_write(0xffffffff);
//	}else{
//				led_a_write(0);		
//	}

	for(int led = 0; led < 17; led++)
	{
		int16_t pointer = cosin_table[angle][led];
		uint8_t mask = 0x80;
		uint8_t OnOff = pic[pointer/8]&(mask >>((pointer%8)));
		HAL_GPIO_WritePin(Led_a_port[16-led], Led_a[16-led], OnOff);

		int16_t angle2 = angle+100;
		if(angle2 >= 200) angle2 -=200;
		pointer = cosin_table[angle2][led];
		OnOff = pic[pointer/8]&(mask >>((pointer%8)));
		HAL_GPIO_WritePin(Led_b_port[16-led], Led_b[16-led], OnOff);
	}
	if(angle<200)angle++; 
	
}


void led_a_write(uint32_t tmp){
	uint8_t index = 0;
	uint32_t mask = 1;
	for(index = 0; index<17; index++) 
  {
		HAL_GPIO_WritePin(Led_a_port[index], Led_a[index], (tmp&mask)>>index);
		mask = mask <<1;
  }
}
void led_b_write(uint32_t tmp){
	uint8_t index = 0;
	uint32_t mask = 1;
	for(index = 0; index<17; index++)
  {
		HAL_GPIO_WritePin(Led_b_port[index], Led_b[index], (tmp&mask)>>index);
		mask = mask <<1;
  }
}
/* USER CODE END 4 */

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
