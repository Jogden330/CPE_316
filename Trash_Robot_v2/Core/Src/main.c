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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

uint16_t freq;
uint16_t step = 0;

int32_t Right_Arm = 0;
int32_t Left_Arm = 0;


typedef enum{ //set state system
	 	        Wave,
				March_Thing,
				Arms_Opposite_Direction
} state_type;

state_type state;

void mydelay(int count)
{
	int i, j;
	int x = 0;
	for(i=0; i<count; i++)
		for(j=0; j<1000; j++)  // need to tune this?
			x = x + 1;
}

void robot_wave(void) {

	while(Right_Arm < 12000)
		  {
			  TIM3->CCR2 = Right_Arm;

			  TIM3->CCR3 = Left_Arm;
			  //------------------------wave--------------------------------
			  //Right_Arm < 12000
			  Right_Arm += 30; //30

			  mydelay(1);
		  }

	  mydelay(25);

		  while(Right_Arm > 150)
		  {
			  TIM3->CCR2 = Right_Arm;

			  TIM3->CCR3 = Left_Arm;

			  //-----------------------wave-------------------------------------
			  //Right_Arm > 150
			  Right_Arm -= 80; //80
			  //Left Arm NONE

			  mydelay(1);
		  }
}

void robot_march_thing(void){

}
void robot_arms_opposite_directons(void){

}

void TIM3_IRQHandler(void) {

	TIM2->SR &= ~(TIM_SR_CC1IF);




   switch(step){
     	case(0):
		        state = Wave;
     	        TIM2->CCR1 +=  (1 << 4);              //set channel one compare value
     	 	 	break;
     	case(1):
     			state = March_Thing;
     	     	TIM2->CCR1 +=  (1 << 4);              //set channel one compare value
     	     	break;
     	case(2):
     			 state = Arms_Opposite_Direction;
     	     	 TIM2->CCR1 +=  (1 << 4);              //set channel one compare value
     	     	 break;
     	case(3):
     	         state = March_Thing;
     	         TIM2->CCR1 +=  (1 << 4);              //set channel one compare value
     	     	 break;
     	case(4):
     			 state = Arms_Opposite_Direction;
     	     	 TIM2->CCR1 +=  (1 << 4);              //set channel one compare value
     	     	 break;
     	case(5):
     			 state = Wave;
     	     	 TIM2->CCR1 +=  (1 << 4);              //set channel one compare value
     	     	 break;
   }

    step++;

}




int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM2_Init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN);

  // bank A as GPIO output mode
  GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5);  //clears bits 11 & 10
  GPIOA->MODER |= (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0); //sets bit 10
  //GPIOA->MODER &= ~(GPIO_MODER_MODE13);  //clears bits 27 & 26
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5);  //clears bits 27 & 26

  // bank C as GPIO input mode, no pull-up or pull-down
  GPIOC->MODER &= ~(GPIO_MODER_MODE13);  //clears bits 27 & 26
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD13);  //clears bits 27 & 26

  __enable_irq();

  NVIC->ISER[0] = (1 << (TIM2_IRQn & 0x1F));

  RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN); //Turn on timer 2
  TIM2->CCR1 = (0x90);                    //set channel one compare value
  TIM2->DIER |= (TIM_DIER_CC1IE);		   //inable interupt on chanle one
  TIM2->SR &= ~(TIM_SR_CC1IF);            //Clear the interupt flag
  TIM2->ARR = (0xFFFFFFFF);               //reset the counter
  TIM2->CR1 |= TIM_CR1_CEN;			   //start timer


  //state = Wave;

  //Left_Arm = 10;



  while (1)
  {

	  GPIOA->ODR &= ~GPIO_PIN_5;	// set pin 5 of PORT A to 0
	  GPIOA->ODR |= GPIO_PIN_4;	// set pin 4 of PORT A to 1


	  mydelay(100);
	  //HAL_Delay(200);
	  GPIOA->ODR |= GPIO_PIN_5;		// set pin 5 of PORT A to 1
	  GPIOA->ODR &= ~GPIO_PIN_4;	// set pin 4 of PORT A to 0

	  mydelay(100);
	  //HAL_Delay(200);

  	 switch(state){
  	 	 case(Wave):
  	 	    robot_wave();
  	 	 	break;

  	 	 case(March_Thing):
  	 		robot_march_thing();
  	 	    break;

  	 	 case(Arms_Opposite_Direction):
			robot_arms_opposite_directons();
  	 	    break;



  	 }


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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}




/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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




