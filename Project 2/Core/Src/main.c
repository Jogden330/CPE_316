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
#include "KeyPad.h"
#include "LCD.h"
#include <math.h>

#define FREQ_100Hz 2413
#define FREQ_200Hz 1206
#define FREQ_300Hz 803
#define FREQ_400Hz 602
#define FREQ_500Hz 480

int SineData[100];
int SawData[100];
int TriangleData[100];

uint16_t Duty = 50;
uint16_t count = 0;
uint16_t data = 0;
uint16_t freq;


void SystemClock_Config(void);

typedef enum{ //set state system
	 	        SineWave,
	 	        Triangle,
	 	        SawTooth,
	 	        SquareWave

	 } state_type;

state_type state;

void sineinitate(void){
    int i = 0;
    for(i = 0; i<100;i++){
        SineData[i] =(sin(2*3.14*i/100)+1)*2047;
    }
}

void sawinitate(void){


    for(int i = 0; i<100;i++){
    	SawData[i] = i * 400;
    }

}

void tiangleinitate(void){

	for(int i = 0; i < 51; i++){
		 TriangleData[i] = i * 80;
	}

	for(int i = 50; i<100;i++){
		TriangleData[i] = 4000 - (i * 80);
	}


}

void LCD_write_freq(void){

	int i = 0;

	char line2[5] = {'0','0',' ','H','z'};

	LCD_newline();

		switch(freq){
		case(FREQ_100Hz):
			LCD_write_char(0x30 + 1);
		    break;
		case(FREQ_200Hz):
			LCD_write_char(0x30 + 2);
		    break;
		case(FREQ_300Hz):
			LCD_write_char(0x30 + 3);
		    break;
		case(FREQ_400Hz):
			LCD_write_char(0x30 + 4);
			break;
		case(FREQ_500Hz):
			LCD_write_char(0x30 + 5);
		    break;

		}
		for(i = 0;i < (sizeof(line2)/sizeof(line2[0]));i++){   //cycle throw second line array printing LCD
					  LCD_write_char(line2[i]);
	    }
}

void LCD_write_string(char *sentence) {
	LCD_clear();


	int i = 0;
	while (sentence[i] != 0) {
		LCD_write_char(sentence[i++]);
	}

	LCD_write_freq();

}


void TIM2_IRQHandler(void) {

	TIM2->SR &= ~(TIM_SR_CC1IF);
	TIM2->CCR1 +=  (freq);              //set channel one compare value

    if(count < 99){

     	 switch(state){
     	 	 case(SineWave):
		        data = SineData[count];
     	 	 	break;

     	 	 case(Triangle):
     	 		data = TriangleData[count];
     	 	    break;

     	 	 case(SawTooth):
				data = SawData[count];
     	 	    break;

     	 	 case(SquareWave):
				if (count < Duty){
		            data = 0x0FFF;
				}else{
				    data = 0x0000;
				}

     	 	    break;

     	 }
    }else{
    	count = 0;
    }
    count++;

}

int delay(void){
	int x;
	for(uint8_t i = 0; i < 50; i++){
		for(uint8_t j = 0; i < 50; i++){
			  x += i + j;
	    }

	}
	return x;
}

int main(void){

  HAL_Init();

  SystemClock_Config();

  Keypad_init();
  DAC_Init();
  LCD_init();

  freq = FREQ_100Hz;

  __enable_irq();

  RCC->CFGR = ((RCC->CFGR & ~(RCC_CFGR_MCOSEL)) | (RCC_CFGR_MCOSEL_0));

   __enable_irq();

   NVIC->ISER[0] = (1 << (TIM2_IRQn & 0x1F));

   RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN); //Turn on timer 2
   TIM2->CCR1 = (freq);                    //set channel one compare value
   TIM2->DIER |= (TIM_DIER_CC1IE);		   //inable interupt on chanle one
   TIM2->SR &= ~(TIM_SR_CC1IF);            //Clear the interupt flag
   TIM2->ARR = (0xFFFFFFFF);               //reset the counter
   TIM2->CR1 |= TIM_CR1_CEN;			   //start timer

  uint8_t Key = 0xFF;
  uint8_t LastKey = 0xFF;


  sineinitate();
  sawinitate();
  tiangleinitate();

  LCD_write_string("SQUARE");
  state =  SquareWave;



  while (1){

  	  DAC_writ(data);

      Key = read_Pad();

      delay();


  	  if((Key != 0xFF) && (Key != LastKey) && (Key < 13)){
  		  switch(Key){
  		      case(1):
  		        freq = FREQ_100Hz;
  		        LCD_write_freq();
  		        TIM2->CCR1 +=  (freq);              //set channel one compare value
  		    	break;

  		      case(2):
  				freq = FREQ_200Hz;
  		        LCD_write_freq();
  		        TIM2->CCR1 +=  (freq);              //set channel one compare value
  		      	break;

  		      case(3):
  				freq = FREQ_300Hz;
  		        LCD_write_freq();
  		        TIM2->CCR1 +=  (freq);              //set channel one compare value
  		    	break;

  		      case(4):
  				freq = FREQ_400Hz;
  		        LCD_write_freq();
  		        TIM2->CCR1 +=  (freq);              //set channel one compare value
  		      	break;

  		      case(5):
  		    	freq = FREQ_500Hz;
  		        LCD_write_freq();
  		        TIM2->CCR1 +=  (freq);              //set channel one compare value
  		      	break;

  		  	  case(6):
  		       	  state =  SineWave;
  		  	      LCD_write_string("SINEWAVE");
  		  	      break;

  		  	  case(7):
  		          state =  Triangle;
  		  	      LCD_write_string("TRIANGLE");
  		  	      break;

  		  	  case(8):
  		          state =  SawTooth;
  		  	      LCD_write_string("SAWTOOTH");
  				  break;

  		  	  case(9):
  		  		  state =  SquareWave;
  		  	      LCD_write_string("SQUARE");
  		  		  break;

  		  	  case(10):
  				  if(Duty > 10)
  					  Duty = Duty - 10;
  		  	      break;

  		  	  case(0):
  				  Duty = 50;
  		  	      break;

  		  	  case(11):
  				  if(Duty < 90)
  					  Duty = Duty + 10;
  		  	      break;
  		  	  }

                LastKey = Key;

  	  	  }


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

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  //RCC_OscInitStruct.MSIState = RCC_MSI_ON;  //datasheet says NOT to turn on the MSI then change the frequency.
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
	/* from stm32l4xx_hal_rcc.h
	#define RCC_MSIRANGE_0                 MSI = 100 KHz
	#define RCC_MSIRANGE_1                 MSI = 200 KHz
	#define RCC_MSIRANGE_2                 MSI = 400 KHz
	#define RCC_MSIRANGE_3                 MSI = 800 KHz
	#define RCC_MSIRANGE_4                 MSI = 1 MHz
	#define RCC_MSIRANGE_5                 MSI = 2 MHz
	#define RCC_MSIRANGE_6                 MSI = 4 MHz
	#define RCC_MSIRANGE_7                 MSI = 8 MHz
	#define RCC_MSIRANGE_8                 MSI = 16 MHz
	#define RCC_MSIRANGE_9                 MSI = 24 MHz
	#define RCC_MSIRANGE_10                MSI = 32 MHz
	#define RCC_MSIRANGE_11                MSI = 48 MHz   dont use this one*/
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;  //datasheet says NOT to turn on the MSI then change the frequency.
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
