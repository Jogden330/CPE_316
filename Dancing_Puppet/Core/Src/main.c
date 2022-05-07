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


//Initalises the PWN for timer 2
void PWM_int(void){
//	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;    // GPIOE clock enable
//	GPIOC->MODER &= ~GPIO_MODER_MODER0;  // Configure PE14 as Alternative Function.
//    GPIOC->MODER |= GPIO_MODER_MODER0_1;
//    GPIOC->AFR[1] &= ~GPIO_AFRH_AFRH1; // PE14 configured as AF1: TIM1_CH4
//    GPIOC->AFR[1] |= 0x01000000;

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;    // GPIOE clock enable
    GPIOA->MODER &= ~GPIO_MODER_MODER0;  // Configure PA0 as Alternative Function.
    GPIOA->MODER |= GPIO_MODER_MODER10_1;
    GPIOA->AFR[0] = 0x1; // PA0 configured as AF1: TIM2_CH1

	// Start by making sure the timer's 'counter' is off
//	TIM2->CR1 &= ~(TIM_CR1_CEN);
//		TIM2->SR &= ~(TIM_SR_UIF);

		//RESET THE TIMER2 BUS
//		RCC->APB1RSTR |=  (RCC_APB1RSTR_TIM2RST);
//		RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM2RST);
//
//		//enable Timer2channel 1 gpio periperial
//		T2C1_pin_conf();
//		//Enable Timer2 peripheral clock
//	    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

		RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN); //Turn on timer 2

	    //enable the preload for CCR1 register -OC1PE bits
	    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;
	    //enable the preload for ARR register
	    TIM2->CR1 |= TIM_CR1_ARPE;
	    //Set channel1 as output for PWM mode
	    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
	    //enable the channel 1 of TIM2
	 	TIM2->CCER |= TIM_CCER_CC1E;
        /******************* assigning values to ARR,PSC,CCR1***************/
	    TIM2->CR1 |= TIM_CR1_UDIS; // enable this bit so that no register value is updated

	    //Timer prescaler value
	    TIM2->PSC = 32;
	     //Timer Auto reload register value - this decides the frequency of my signal
	    TIM2->ARR = 20000;
	     //timer output compare register for channel 1 upto which my counter will count: this decides the duty cycle of my signal
	    TIM2->CCR1 = 15000;

	    TIM2-> CR1 &= ~TIM_CR1_UDIS; // disable this bit so that values assigned from Timer register can move to preload registers
	     //Enable the update generation for updating the shadow register with preload register contents with new values
	    TIM2->EGR |= TIM_EGR_UG;
	      //Clear the UIF flag as that is set when UG bit is set in EGR to update the content of my register
	    TIM2->SR &= ~(TIM_SR_UIF);

          /*** Setting the output mode of the output signal,its polarity and enabling the corresponding channel ********/

	      //This bits selects which output mode is selected :set the pwm mode 1 on channel1- OC1M bits
	      TIM2->CCMR1 |= (1<<6)|(1<<5);
	      // output signal polarity of channel 1 - active high
	      TIM2->CCER &= ~(TIM_CCER_CC1P);
	      //set the p
	      //enable the counter
	      TIM2->CR1 |= TIM_CR1_CEN;


}

//void intialize_gpioe(void){
//    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;    // GPIOE clock enable
//    GPIOA->MODER &= ~GPIO_MODER_MODER0;  // Configure PA0 as Alternative Function.
//    GPIOA->MODER |= GPIO_MODER_MODER10_1;
//    GPIOA->AFR[0] = 0x1; // PA0 configured as AF1: TIM1_CH1
//
//}

//void initalize_tim1(void){
//
//    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // TIM1 timer clock enable
//    TIM1->PSC = 79; // Set Prescaler to 1MHz
//    TIM1->ARR = 100;
//    TIM1-> CCR1 = 50;
//    //TIM1->CCR1 = 0x50;
//    TIM1->CCMR1 |= TIM_CCMR2_OC1M; // CC4 channel is configured as output
//    TIM1->CCER &= ~TIM_CCER_CC4P;   // Output Polarity set to Active High
//    TIM1->CCMR2 &= ~TIM_CCMR2_OC4M; // Output Compare 4 Mode set as PWM Mode 1.
//    TIM1->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // Output Compare 4 Mode set as PWM Mode 1.
//    TIM1->CCMR2 |= TIM_CCMR2_OC4PE; // Enable the corresponding preload register
//    TIM1->CCER |= TIM_CCER_CC4E; // Capture/Compare 4 Output Enable
//    TIM1->EGR |= TIM_EGR_UG;    // Before starting the counter, you have to initialize all the registers
//    TIM1->BDTR |= TIM_BDTR_MOE;
//    TIM1->CR1 |= TIM_CR1_CEN; // Start Timer
//}

void SystemClock_Config(void);


int main(void)
{
  HAL_Init();
  SystemClock_Config();

  PWM_int();



  while (1)
  {
  }
}

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
