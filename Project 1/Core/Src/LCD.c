/*
 * LCD.c
 *
 *  Created on: Oct 9, 2021
 *      Author: jogde
 */

#include "main.h"
#include <math.h>
#include "LCD.h"



void LCD_command(unsigned char command){

	GPIOC->ODR = (command);

	HAL_Delay(1);
    GPIOC->BSRR |= (command | E);
    HAL_Delay(1);
    GPIOC->BRR |= (E);
    HAL_Delay(1);
    GPIOC->BRR |= (command);




}

void LCD_write_char(unsigned char letter){

	GPIOC->ODR = (letter | RS);

	HAL_Delay(1);
    GPIOC->BSRR |= (letter | RS | E);
    HAL_Delay(1);
	GPIOC->BRR |= (E);
	HAL_Delay(1);
	GPIOC->BRR |= (letter | RS);

}

void LCD_clear(void){
	LCD_command(0X01);
}

void LCD_newline(void){

	LCD_command(0XC0);
}

void LCD_init(void){

	  // configure PC0 - PC10 for GPIO output, push-pull
	  // no pull up / pull down, high speed
	  RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
	  GPIOC->MODER   &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3 |
			              GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7 |
						  GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
	  GPIOC->MODER   |=  (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0 |
			  	  	  	  GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0 |
						  GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0 | GPIO_MODER_MODE10_0);
	  GPIOC->OTYPER  &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3 |
			  	  	  	  GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7 |
						  GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10);
	  GPIOC->PUPDR   &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3 |
			  	  	  	  GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7 |
						  GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9 | GPIO_PUPDR_PUPD10);
      GPIOC->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED0_Pos) |
	                      (3 << GPIO_OSPEEDR_OSPEED1_Pos) |
						  (3 << GPIO_OSPEEDR_OSPEED2_Pos) |
	                      (3 << GPIO_OSPEEDR_OSPEED3_Pos) |
						  (3 << GPIO_OSPEEDR_OSPEED4_Pos) |
						  (3 << GPIO_OSPEEDR_OSPEED5_Pos) |
			              (3 << GPIO_OSPEEDR_OSPEED6_Pos) |
						  (3 << GPIO_OSPEEDR_OSPEED7_Pos) |
						  (3 << GPIO_OSPEEDR_OSPEED8_Pos) |
						  (3 << GPIO_OSPEEDR_OSPEED9_Pos) |
						  (3 << GPIO_OSPEEDR_OSPEED10_Pos));

	   GPIOC->BRR = ( 0x7FF );   // preset PC0, - PC 10 to zero


	LCD_command(0x30);
	HAL_Delay(100);

	LCD_command(0x30);
	HAL_Delay(10);

    LCD_command(0x30);
    HAL_Delay(10);

    LCD_command(0x38);

    LCD_command(0x10);

    LCD_command(0x0F);

    LCD_command(0x06);


}



