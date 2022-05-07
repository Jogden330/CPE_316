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
#include "DAC.h"


void DAC_Init(void){

	  RCC->AHB2ENR |=  (RCC_AHB2ENR_GPIOAEN);
	  RCC->APB2ENR |=  (RCC_APB2ENR_SPI1EN);
	  GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 |		// mask function
	 			  	  	GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
	  GPIOA->MODER |= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 |	// enable alternate function
	 		 	  	   GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
	  GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7 );
	  GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
	  GPIOA->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED4_Pos) |
	 					  (3 << GPIO_OSPEEDR_OSPEED5_Pos) |
	 				      (3 << GPIO_OSPEEDR_OSPEED6_Pos) |
 					  (3 << GPIO_OSPEEDR_OSPEED7_Pos));
	  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 |		// mask AF selection
			  	  	  	 GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
	  GPIOA->AFR[0] |= ((5 << GPIO_AFRL_AFSEL4_Pos) |				// select SPI_1 (AF5)
	  		  	  	    (5 << GPIO_AFRL_AFSEL5_Pos) |
					    (5 << GPIO_AFRL_AFSEL6_Pos) |
					    (5 << GPIO_AFRL_AFSEL7_Pos));

	 SPI1->CR1 |= (SPI_CR1_MSTR);				// enable master mode, fck/2, hardware CS, MSB first, full duplex
	 SPI1->CR2 |= (SPI_CR2_SSOE |				// enable CS output
			  	   SPI_CR2_NSSP |				// create CS pulse
				  (0xF << SPI_CR2_DS_Pos));	    // 16-bit data frames
	 SPI1->CR1 |= (SPI_CR1_SPE);				// enable SPI

}


void DAC_writ(uint16_t data){

    uint16_t temp;
    while(!(SPI1->SR & SPI_SR_TXE));       // ensure room in TXFIFO before writing
    		SPI1->DR = ((0x0FFF &  data) | 0x3000);
    while(!(SPI1->SR & SPI_SR_RXNE));  // wait to receive 16-bits
    		temp = SPI1->DR;
}



