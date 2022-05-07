/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile uint16_t ADC_value = 0;			//global for interrupt access
volatile uint8_t  ADC_flag = 0;			//global for interrupt access
volatile uint16_t ADC_buf[20];
volatile uint16_t ADC_val;

uint16_t min = 0;
uint16_t max = 0;
uint16_t ave = 0;
int count = 0;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
void ADC_Init(void){

	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;		//turn on clock for ADC

	// ADC will run at the same speed as CPU (HCLK / 1) since AHB prescalar is 1
	ADC123_COMMON->CCR = ((ADC123_COMMON->CCR & ~(ADC_CCR_CKMODE)) | ADC_CCR_CKMODE_0);

	//power up ADC and voltage regulator
	ADC1->CR &= ~(ADC_CR_DEEPPWD);
	ADC1->CR |= (ADC_CR_ADVREGEN);
	for(uint16_t i = 0; i < 1000; i++) {	//delay at least 20us for ADC voltage reg to power up
		for(uint16_t j = 0; j < 100; j++) {

		}
	}
	//calibrate ADC
	ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF);	//ensure ADC is not enabled, single ended calibration
	ADC1->CR |= ADC_CR_ADCAL;						//start calibration
	while(ADC1->CR & ADC_CR_ADCAL);					//wait for calibration to finish

	//configure single ended mode before enablung ADC
	ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);			//PA0 is ADC1_IN5, single ended mode

	//enable ADC
	ADC1->ISR |= (ADC_ISR_ADRDY);			//clear ADC ready flag by writing a 1
	ADC1->CR |= ADC_CR_ADEN;				//enable ADC
	while(!(ADC1->ISR & ADC_ISR_ADRDY));	//wait for ADC ready flag
	ADC1->ISR |= (ADC_ISR_ADRDY);			//clear ADC ready fag by writing a 1

	// configure ADC
	ADC1->SQR1 = (ADC1->SQR1 & ~(ADC_SQR1_SQ1_Msk | ADC_SQR1_L_Msk))
				| (5 << ADC_SQR1_SQ1_Pos);	//set sequence to 1 conversion on channel 5

	//enable interrupts for ADC
	ADC1->IER |= (ADC_IER_EOC);				//interrupt after conversion
	ADC1->ISR &= ~(ADC_ISR_EOC);			//clear EOC flag
	NVIC->ISER[0] = (1 << (ADC1_2_IRQn & 0x1F));
	__enable_irq();

	//configure GPIO pin PA0
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);	//turn on clock for GPIO (PA0 = ADC1_IN5)
	GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(GPIO_AFRL_AFSEL0)) | (7 << GPIO_AFRL_AFSEL0_Pos);
	GPIOA->MODER |= (GPIO_MODER_MODE0);		//analog mode for PA0
	GPIOA->ASCR |= GPIO_ASCR_ASC0;			//set PA0 to analog

	ADC1->CR |= ADC_CR_ADSTART;				//start conversion

//	ADC1->ISR |= ADC_ISR_ADRDY;								//Enable ADC conversion
//	ADC1->ISR &= ~(ADC_ISR_EOC);							//End of conversion flag not set
//	ADC1->CR |= (ADC_CR_ADEN | ADC_CR_ADSTART); 			//ADC enable control & start of
//															//  regular conversion
//	ADC1->CFGR &= ~(ADC_CFGR_EXTEN_0 | ADC_CFGR_EXTEN_1);	//Conversion will start immediately
															//  for software trigger config.
}

void ADC1_2_IRQHandler(void) {

	if(ADC1->ISR & ADC_ISR_EOC) {			//check for ADC conversion
		if(count < 20) {
			ADC_value = ADC1->DR;
			ADC_buf[count] = ADC_value;
			ADC_flag = 1;
			count++;
		}

	}
}
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

uint32_t get_max_value(void) {

	uint32_t max_val;
	uint32_t size;
	max_val = ADC_buf[0];
	size = sizeof(ADC_buf) / sizeof(ADC_buf[0]);

	for(uint32_t i = 0; i < size; i++) {
		if(ADC_buf[i] > max_val) {
			max_val = ADC_buf[i];
		}

	}

	return max_val;
}

uint32_t get_min_value(void) {

	uint32_t min_val;
	uint32_t size;
	min_val = ADC_buf[0];
	size = sizeof(ADC_buf) / sizeof(ADC_buf[0]);

	for(uint32_t i = 0; i < size; i++) {
		if(ADC_buf[i] < min_val) {
			min_val = ADC_buf[i];
		}

	}

	return min_val;
}

uint32_t get_ave_value(void) {

	uint32_t ave_val;
	uint32_t size;
	ave_val = 0;
	size = sizeof(ADC_buf) / sizeof(ADC_buf[0]);

	for(uint32_t i = 0; i < size; i++) {
		ave_val += ADC_buf[i];
	}
	ave_val /= 20;

	return ave_val;
}

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  ADC_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

	  if(ADC_flag == 1) {				//check global flag
		  ADC_flag = 0;					//clear flag
		  ADC1->CR |= ADC_CR_ADSTART;	//start a new conversion
	  }
	  if(count == 20) {
		  max = get_max_value();
		  min = get_min_value();
		  ave = get_ave_value();
		  uint32_t done  = 1;
	  }
    /* USER CODE END WHILE */
//	  for(int i = 0; i < 20; i++) {
//		  ADC_val = ADC1->DR;
//		  ADC_buf[i] = ADC_val;
//	  }


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


