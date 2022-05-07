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
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /*
   * Configure SPI Pins		PA4 - SPI_1_NSS		PA5 - SPI_1_SCK
   * 						PA6 - SPI_1_MISO	PA7 - SPI_1_MOSI
   * follow order of configuring registers AFR, OTYPER, PUPDR, OSPEEDR, MODDER
   * to avoid a glitch is created on the output pin
   */
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 |		// mask AF selection
		  	  	  	  GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
  GPIOA->AFR[0] |= ((5 << GPIO_AFRL_AFSEL4_Pos) |				// select SPI_1 (AF5)
  		  	  	    (5 << GPIO_AFRL_AFSEL5_Pos) |
				    (5 << GPIO_AFRL_AFSEL6_Pos) |
				    (5 << GPIO_AFRL_AFSEL7_Pos));
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 |		// push-pull output
		  	  	  	  GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 |		// no pull ups or pull downs
		  	  	  	GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED4 | 					// low speed
		  	  	  	  GPIO_OSPEEDR_OSPEED5 |
					  GPIO_OSPEEDR_OSPEED6 |
					  GPIO_OSPEEDR_OSPEED7);
  GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 |		// mask function
		  	  	  	GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
  GPIOA->MODER |= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 |	// enable alternate function
		  	  	   GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
  // configure SPI 1
  RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);		// enable SPI1 clock
  SPI1->CR1 = (SPI_CR1_MSTR);				// enable master mode, fck/2, hardware CS, MSB first, full duplex
  SPI1->CR2 = (SPI_CR2_SSOE |				// enable CS output
		  	   SPI_CR2_NSSP |				// create CS pulse
			   (0xF << SPI_CR2_DS_Pos));	// 16-bit data frames
  SPI1->CR1 |= (SPI_CR1_SPE);				// enable SPI​
  uint16_t spi_data0 = 0x3555;
  uint16_t spi_data1 = 0x30F0;
  uint16_t temp;
  while (1)
  {
	  while(!(SPI1->SR & SPI_SR_TXE));		// ensure room in TXFIFO before writing
	  SPI1->DR = spi_data0;
	  while(!(SPI1->SR & SPI_SR_RXNE)); 	// wait to receive 16-bits
	  temp = SPI1->DR;						// clear RX FIFO
	  while(!(SPI1->SR & SPI_SR_TXE));		// ensure room in TXFIFO before writing
	  SPI1->DR = spi_data1;
	  while(!(SPI1->SR & SPI_SR_RXNE));
	  //while(((SPI1->SR & SPI_SR_FRLVL_Msk) >> SPI_SR_FRLVL_Pos) != 0x02 ); // wait to receive 16-bits
	  temp = SPI1->DR;						// clear RX FIFO
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
