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
#include "KeyPad.h"
#include "LCD.h"

#define Delay 500

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

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

void set_NEW_code_text(void){
           char line1[8] = {'N','E','W',' ','C','O','D','E'};                      //set first text line of array
           char line2[7] = {'E','N','T','E','R',':',' '};                           //set second text line of array

           LCD_clear();                                                             //clear led screen

           uint8_t i = 0;
           for(i = 0;i < (sizeof(line1)/sizeof(line1[0]));i++){                     //cycal throw line array printing LCD
              LCD_write_char(line1[i]);
           }
           LCD_newline();                                                          //go to next line
           for(i = 0;i < (sizeof(line2)/sizeof(line2[0]));i++){                     //cycal throw second line array printing LCD
                  LCD_write_char(line2[i]);
               }

}

void set_unlock_text(void){
        char line1[8] = {'U','N','L','O','C','K','E','D'};                          //set first text line of array
        char line2[13] = {'P','R','E','S','S',' ','A','N','Y',' ','K','E','Y'};     //set second text line of array

        LCD_clear();                                                                 //clear led screen

        uint8_t i = 0;                                                                   //cycal throw line array printing LCD
        for(i = 0;i < (sizeof(line1)/sizeof(line1[0]));i++){
           LCD_write_char(line1[i]);
        }
        LCD_newline();                                                              //go to next line
        for(i = 0;i < (sizeof(line2)/sizeof(line2[0]));i++){                         //cycal throw second line array printing LCD
               LCD_write_char(line2[i]);
            }
}

void set_lock_text(void){
	    char line1[6] = {'L','O','C','K','E','D'};                                   //set first text line of array
	    char line2[11] = {'E','N','T','E','R',' ','K','E','Y',':',' '};              //set second text line of array

	    LCD_clear();                                                                 //clear led screen

	    uint8_t i = 0;
	    for(i = 0;i < (sizeof(line1)/sizeof(line1[0]));i++){                         //cycal throw first line array printing LCD
	        LCD_write_char(line1[i]);
	    }
	    LCD_newline();                                                              //go to next line
	    for(i = 0;i < (sizeof(line2)/sizeof(line2[0]));i++){                         //cycal throw second line array printing LCD
	        LCD_write_char(line2[i]);
	    }
}



int main(void)
{
	 HAL_Init();
	 SystemClock_Config();

     uint8_t Key = 0xFF;  //set variables
     uint8_t Code[4] = {1,2,3,4};
	 uint8_t count = 0;
	 uint8_t Entered[4];

	 LCD_init(); //initalis LCD

	 Keypad_init();



	 typedef enum{ //set state system
	 	        Lock,
	 	        EnterCode,
	 	        Unlock,
	 	        EnterKey,
	 	        Set,
	 	        SetCode
	 } state_type;

	 state_type state = Lock; //initialize to lock state

	  while(1){ //Forever loop

	        Key = read_Pad(); //gets the input from the keypad
                  if(Key == 10){
                      state = Lock;

                      continue;
                  }


	        switch(state){ //check states and select case

	        case(Lock): //Lock state

	             set_lock_text(); //right to LCD
	             count = 0;  //start at the beginning of array
	             LED_Light_off();
	             state = EnterCode;     //go to the enter state
	             break;

	        case(EnterCode): //Were code is entered and checked

	             //Key = read_Pad(); //gets the input from the keypad

	             if( Key != 0xFF && count < (sizeof(Code)/sizeof(Code[0])) ){
                        //checks if the keypad button is
	                 // and if the make number of presses have been reached


	            	LCD_write_char(0x30+Key); //Right key presses on screen
	            	Entered[count++] = Key; //save key into an array for entered numbers

	            	HAL_Delay(Delay);            //delay to mitigate multiple button presses


	                 } else if(count == (sizeof(Code)/sizeof(Code[0]))) {
                                         //check if last digit was entered
	                	 uint8_t i = 0;
	                	 uint8_t Is_equal = 1; //set is equal to true
	                     for(i = 0;i < (sizeof(Code)/sizeof(Code[0]));i++){
                                    //check if number entered matches code number
	                         if(Code[i] != Entered[i]){
                   // if there is a number that does not match set is equal to false
	                             Is_equal = 0;

	                         }
	                     }
	                     if(Is_equal){

	                        	 //if the entered numbers are the same as code unlock, otherwise leave locket
	                         state = Unlock;
	                      } else {
	                         state = Lock;
	                      }

	                    }
	             break;

	     case(Unlock): //state that sets unlock protocol
                  set_unlock_text(); //sets screen to read unlock text
	     	 	  LED_Light_on();
	     	 	  HAL_Delay(Delay);
	              state = EnterKey; //sets state the enterKey
	              break;
	     case(EnterKey): //state waits for the user to press a key lock the system
	                     //if # is entered user can set a new code

	                  if( Key != 0xFF){  //checks if a key is pressed
	                      if(Key == 11){ //if # key is pressed set state to set
	                          state = Set;
	                          HAL_Delay(Delay);
	                           break;
	                      }
                          // if any other key is pressed go back to lock
	                      state = Lock;
	                      HAL_Delay(Delay);
	                      break;
	                  }break;


	     case(Set): //sets the set state

	             set_NEW_code_text(); //sets the LCD to tell the user to enter a new code
	             count = 0;          //sets to the front or the array
	             HAL_Delay(Delay);
	             state = SetCode;    //gose to the setcodestate
	             break;

	     case(SetCode): //allows the user to enter a new code

	            if( Key != 0xFF && count < (sizeof(Code)/sizeof(Code[0])) && Key != 11 ){           //checks if a valet key is pressed
	                                                                                                            //checks if 4 keys have been pressed
	                  LCD_write_char(0x30+Key);   //Wright key to LCD
	                  Code[count++] = Key;        //Set key to code
	                  HAL_Delay(Delay);

	             } else if(count == (sizeof(Code)/sizeof(Code[0]))) {
                            //When the digits are filled set the lock state
	                 state = Lock;
	                 break;
	             }


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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
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
