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


TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);

//Globale variables for head movement
uint64_t count = 0;
uint64_t head_pose = 1500;
uint8_t head_increase = 1;

//sets the position of the left are
void drive_left_arm(uint16_t pose){
	TIM2->CCR1 = pose;
}

//sets the position of the Right are
void drive_right_arm(uint16_t pose){
	TIM2->CCR2 = pose;
}

//sets the position of the head
void drive_head(uint16_t pose){
	TIM4->CCR1 = pose;
}

//softwere delay used to not interfear with interupts
void mydelay(int count)
{
	int i, j;
	int x = 0;
	for(i=0; i<count; i++)
		for(j=0; j<1000; j++)  // need to tune this?
			x = x + 1;
}

//make the robot swing his arms up and down in oposite directios
void The_Monkey(void){

	drive_left_arm(2500);
	drive_right_arm(2500);

	mydelay(60);
	drive_left_arm(500);
	drive_right_arm(500);

    mydelay(60);

}

//make the robot wave with its left hand
void The_Wave(void){
	uint16_t i = 0;

	drive_left_arm(2500);
	drive_right_arm(500);


    mydelay(60);
    for(i = 0; i < 4; i++){
    	drive_left_arm(1000);

    	mydelay(30);
    	drive_left_arm(500);

    	mydelay(30);
    }
    drive_left_arm(2500);

    mydelay(200);
}

//make the robot raises its right are and moves it back and forth
void The_Right_Hiker(void){
	uint16_t i = 0;

	drive_left_arm(2500);
	drive_right_arm(2500);


    mydelay(30);
    for(i = 0; i < 4; i++){
    	drive_right_arm(2500);

    	mydelay(25);
    	drive_right_arm(2000);

    	mydelay(25);


    }
    drive_right_arm(500);

    mydelay(30);

}

//make the robot raises its left are and moves it back and forth
void The_left_Hiker(void){
	uint16_t i = 0;

	drive_left_arm(500);
	drive_right_arm(500);


    mydelay(30);
    for(i = 0; i < 4; i++){
    	drive_left_arm(500);

    	mydelay(25);
    	drive_left_arm(1000);

    	mydelay(25);


    }
    drive_left_arm(2500);

    mydelay(30);

}

//robot raised then lowers its arms
void The_Hands_up(void){

	drive_left_arm(500);
	drive_right_arm(2500);

	mydelay(60);
	drive_left_arm(2500);
	drive_right_arm(500);

    mydelay(60);

}

//robot swings its arms in a small ark
void The_Shimmy(void){
	drive_left_arm(1800);
	drive_right_arm(1800);

	mydelay(30);
	drive_left_arm(900);
	drive_right_arm(900);

    mydelay(30);

}

//robot lifts its arms to shoulder hight and the take turns rising its arms
void The_pop(void){
	drive_left_arm(1500);
	drive_right_arm(1500);

	mydelay(15);
	drive_left_arm(900);
	mydelay(20);
	drive_left_arm(1500);
	mydelay(20);
	drive_right_arm(1800);
	mydelay(20);
	drive_right_arm(1500);

    mydelay(15);

}



//timer handles eyes blinking and head rotation
void TIM3_IRQHandler(void) {

	TIM3->SR &= ~(TIM_SR_CC3IF);
	TIM3->CCR3 += (0xFFFFFFFFF);

	//switch with eye is blinking
	if((count%2) == 0){
		GPIOA->ODR &= ~GPIO_PIN_5;	// set pin 5 of PORT A to 0
		GPIOA->ODR |= GPIO_PIN_4;	// set pin 4 of PORT A to 1
	} else {
		GPIOA->ODR |= GPIO_PIN_5;		// set pin 5 of PORT A to 1
		GPIOA->ODR &= ~GPIO_PIN_4;	// set pin 4 of PORT A to 0
	}
	//set the heads possition based on the the set direction
	if(head_increase){
		head_pose += 20;
	} else {
		head_pose -= 20;
	}
	//if the head has reached its max rotation
	if(head_pose >= 2500){
		head_increase = 0;
	}
	if(head_pose <= 500){
		head_increase = 1;
	}
	//set the heads possition and increases count
	TIM4->CCR1 = head_pose;
	count++;
}


int main(void)
{

   HAL_Init();


   SystemClock_Config();

   //Initalises the and starts the PWM signals
   MX_GPIO_Init();
   MX_TIM2_Init();
   MX_TIM4_Init();
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

   RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN);

   // bank A as GPIO output mode
   GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5);  //clears bits 11 & 10
   GPIOA->MODER |= (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0); //sets bit 10
   GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5);  //clears bits 27 & 26

   //starts timer for eyes and head controlls
   __enable_irq();

   NVIC->ISER[0] = (1 << (TIM3_IRQn & 0x1F));

   RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM3EN); //Turn on timer 3
   TIM3->CCR3 = (0x90);                    //set channel one compare value
   TIM3->DIER |= (TIM_DIER_CC3IE);		   //inable interupt on chanle one
   TIM3->SR &= ~(TIM_SR_CC3IF);            //Clear the interupt flag
   TIM3->ARR = (0xFFFFFFFF);               //reset the counter
   TIM3->CR1 |= TIM_CR1_CEN;			   //start timer


   //Move arms to starting possitions
   drive_left_arm(1500);
   drive_right_arm(1500);

   while (1)
   {

	   //sets order for the dance routain
 	  The_Wave();
 	  The_Monkey();
 	  The_Monkey();
 	  The_Shimmy();
 	  The_Shimmy();
 	  The_Monkey();
 	  The_Shimmy();
 	  The_pop();
 	  The_Hands_up();
 	  The_Right_Hiker();
 	  The_left_Hiker();
 	  The_Shimmy();
 	  The_Monkey();
 	  The_pop();
 	  The_pop();
 	  The_Shimmy();
 	  The_Monkey();
 	  The_Hands_up();


   }

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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
