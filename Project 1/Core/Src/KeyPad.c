/*
 * KeyPad.c
 *
 *  Created on: Oct 9, 2021
 *      Author: jogde
 */

#include "main.h"
#include <math.h>
#include <stdio.h>
#include "KeyPad.h"

void LED_Light_on(void){

	 GPIOB->BSRR |= (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11);

}

void LED_Light_off(void){

	 GPIOB->BRR |= (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11);
}


int read_Pad(void){


	volatile uint8_t  row;

	for(uint8_t colum = 0; colum < 3; colum++){

		GPIOB->BRR |= 0X70;
		GPIOB->BSRR |= (1 << (4 + colum));
		row |= (GPIOB->IDR & 0x0F); //mask

		switch(colum){
		case 0:
			switch(row){
				case 1:
					return 1;
				case 2:
					return 4;
				case 4:
					return 7;
				case 8:
					return 10;
				}
		case 1:
			switch(row){
				case 1:
					return 2;
				case 2:
					return 5;
				case 4:
					return 8;
				case 8:
					return 0;
			}
		case 2:
			switch(row){
				case 1:
					return 3;
				case 2:
					return 6;
				case 4:
					return 9;
				case 8:
					return 11;
				}


		}

	}
	return 0xFF;

}



void Keypad_init(void){


  RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOBEN);
  GPIOB->MODER   &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3 |
		              GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 |
					  GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10 | GPIO_MODER_MODE11);
  GPIOB->MODER   |=  (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 |
		              GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0 | GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0);
  GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3 |
		  	  	  	  GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7 |
					  GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11);
  GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3 |
		  	  	  	  GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD8 |
					  GPIO_PUPDR_PUPD9 | GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11);
  GPIOB->PUPDR   |=  (GPIO_PUPDR_PUPD0_1 | GPIO_PUPDR_PUPD1_1 | GPIO_PUPDR_PUPD2_1 | GPIO_PUPDR_PUPD3_1 |
  		  	  	  	  GPIO_PUPDR_PUPD4_1 | GPIO_PUPDR_PUPD5_1 | GPIO_PUPDR_PUPD6_1 );
  GPIOB->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED0_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEED1_Pos) |
					  (3 << GPIO_OSPEEDR_OSPEED2_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEED3_Pos) |
					  (3 << GPIO_OSPEEDR_OSPEED4_Pos) |
					  (3 << GPIO_OSPEEDR_OSPEED5_Pos) |
		              (3 << GPIO_OSPEEDR_OSPEED6_Pos) |
					  (3 << GPIO_OSPEEDR_OSPEED7_Pos) |
					  (3 << GPIO_OSPEEDR_OSPEED8_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEED9_Pos) |
  	          		  (3 << GPIO_OSPEEDR_OSPEED10_Pos)|
                      (3 << GPIO_OSPEEDR_OSPEED11_Pos));

  GPIOB->BRR = ( 0X3F );   // preset PC0, - PC 10 to zero


    GPIOB->BRR |= (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11);



}
