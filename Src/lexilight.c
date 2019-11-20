/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "lexilight.h"
/* Present of signal LexiLight
 * Signal must be:
 *                      ______________
 * |                    |            |
 * |                    |            |
 * |____________________|            |
 * <------ 10 ms -------><---- T ---->
 * 
 * So that (10 ms + T) = 1 / f
 * Where f ? [ 65 - 120 ] Hz
 * uC = 48Mhz
 *
 */
 
LEXILIGHT_DATA lexilight;

void Lexi_Init_Data() {
	lexilight.system_clock 	= SYSTEM_CLOCK_PER_SECOND;
	lexilight.duty_min 			= LEXILIGHT_DUTY_MIN;
	lexilight.duty_max 			= LEXILIGHT_DUTY_MAX;
	lexilight.duty_cmd 			= LEXILIGHT_DUTY_DEFAULT;
	lexilight.duty					= 30;
	lexilight.freq_max 			= LEXILIGHT_FREQUENCY_MAX;
	lexilight.freq_min 			= LEXILIGHT_FREQUENCY_MIN;
	lexilight.freq_cmd 			= LEXILIGHT_FREQUENCY_DEFAULT;
	lexilight.freq					= 90;
	lexilight.adc_variant	  = LEXILIGHT_ADC_VARIANT;
	lexilight.lum_level 		= LIGHT_LUM_LEVEL_0;
	lexilight.state = LIGHT_STATE_WAIT_500_MS;
	//lexilight.state = LIGHT_STATE_ON;
	//Update ADC number step
	//lexilight.adc_number_step = 1;
	//for(int i = 0; i < LEXILIGHT_ADC_BIT ; i ++){
	//	lexilight.adc_number_step *=2;
	//}
	lexilight.adc_number_step = 1023;
}


void Lexi_Task (TIM_HandleTypeDef htim_pwm_led, TIM_HandleTypeDef htim_pwm_lum_driver, uint32_t * adc_raw){
	
	switch (lexilight.state)
	{
		case LIGHT_STATE_ERROR:
		case LIGHT_STATE_INIT:
			Lexi_Init_Data();
			lexilight.state = LIGHT_STATE_WAIT_500_MS;
			break;
		
		case LIGHT_STATE_WAIT_500_MS:
			HAL_Delay(500);
			lexilight.state = LIGHT_STATE_SERVICE;
			break;
		
		case LIGHT_STATE_SERVICE:
			Lexi_DO_ADC_PWM (htim_pwm_led, adc_raw);
			Lexi_DO_PWM_LUM_DRIVER(htim_pwm_lum_driver);
			break;
		
		case LIGHT_STATE_ON:			
			Lexi_DO_ADC_PWM (htim_pwm_led, adc_raw);
			Lexi_DO_PWM_LUM_DRIVER(htim_pwm_lum_driver);
			//Turn On EN_DRIVER
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET);
			break;
		
		case LIGHT_STATE_STANDARD:
			Lexi_DO_Standard_PWM(htim_pwm_led);
			Lexi_DO_PWM_LUM_DRIVER(htim_pwm_lum_driver);
			//Turn On EN_DRIVER
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET);
			break;
		
		case LIGHT_STATE_OFF:
			Lexi_DO_ADC_PWM (htim_pwm_led, adc_raw);
			Lexi_DO_PWM_LUM_DRIVER(htim_pwm_lum_driver);
			//Turn Off EN_DRIVER
			break;
		
		
		
		default:
			lexilight.state = LIGHT_STATE_ERROR;
			break;
		}
}

void Lexi_DO_ADC_PWM (TIM_HandleTypeDef htim, uint32_t * adc_raw){
	lexilight.adc_duty_new = adc_raw[0];	//Get new value adc duty from pot1;
	lexilight.adc_freq_new = adc_raw[1];	//Get new value adc freq from pot2;
	//Task update Duty
	if (((lexilight.adc_duty_raw + lexilight.adc_variant) < lexilight.adc_duty_new ) 
		|| (lexilight.adc_duty_raw > (lexilight.adc_duty_new + lexilight.adc_variant))){
		lexilight.adc_duty_raw = lexilight.adc_duty_new;	//Update New value of ADC Duty;
		Lexi_Update_Duty(htim);														//Update value Duty
		
				
	}			
	//Task update Freq
	if (((lexilight.adc_freq_raw + lexilight.adc_variant) < lexilight.adc_freq_new ) 
		|| (lexilight.adc_freq_raw > (lexilight.adc_freq_new + lexilight.adc_variant))){
		lexilight.adc_freq_raw = lexilight.adc_freq_new;	//Update New value of ADC Freq
		Lexi_Update_Frequency(htim); 											//Update value Freq
	}
}

void Lexi_DO_Standard_PWM (TIM_HandleTypeDef htim){
		int prescaler = 59;		//Set frequency 119 = 4KHZ
		__HAL_TIM_SET_PRESCALER(&htim, prescaler);
		//__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_2, lexilight.duty);
		__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_2, 100);
		
}

void Lexi_DO_PWM_LEXIMODE (TIM_HandleTypeDef htim){
		int prescaler = (uint16_t)(lexilight.system_clock/(lexilight.freq*100) - 1);
		__HAL_TIM_SET_PRESCALER(&htim, prescaler);
		__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_2, lexilight.duty);
}

/*
void Lexi_DO_PWM_LUM_DRIVER (TIM_HandleTypeDef htim){
	//Todo :  do the  signal PWM_LUM_DRIVER 
	unsigned int value = 0;
	unsigned int value_2 = 0;
	switch (lexilight.lum_level){
		case LIGHT_LUM_LEVEL_0:
			//Calcule value I en fonction de dutycycle
			value_2 = (unsigned int)(10 * 40 / lexilight.duty) ;
			value  = (unsigned int)((140 - value_2)*0.77);
			lexilight.pwm_led_driver = value;
			__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, value);
			break;
		case LIGHT_LUM_LEVEL_1:
			value_2 = (unsigned int)(20 * 40 / lexilight.duty) ;
			value  = (unsigned int)((140 - value_2)*0.77);
			lexilight.pwm_led_driver = value;
			__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, value);
			break;
		case LIGHT_LUM_LEVEL_2:
			value_2 = (unsigned int)(35 * 40 / lexilight.duty) ;
			value  = (unsigned int)((140 - value_2)*0.77);
			lexilight.pwm_led_driver = value;
			__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, value);
			break;
		default:
			break;
	}
}
*/

void Lexi_DO_PWM_LUM_DRIVER (TIM_HandleTypeDef htim){
	//Todo :  do the  signal PWM_LUM_DRIVER
	unsigned int lux_cmd = 0;
	unsigned int value = 0;
	unsigned int value_2 = 0;
	switch (lexilight.lum_level){
		/*
		case LIGHT_LUM_LEVEL_0:
			//Calcule value I en fonction de dutycycle
			if (lexilight.state  == LIGHT_STATE_STANDARD){
				value_2 = (unsigned int)(1000 / 100) ;
				value  = (unsigned int)((140 - value_2)*0.77);
				lexilight.pwm_led_driver = value;
			}
			else{
				value_2 = (unsigned int)(1000 / lexilight.duty) ;
				value  = (unsigned int)((140 - value_2)*0.77);
				lexilight.pwm_led_driver = value;
			}
			__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, value);
			break;
		*/
		case LIGHT_LUM_LEVEL_0:
			//Calcule value I en fonction de dutycycle
			//if (lexilight.state  == LIGHT_STATE_STANDARD){
			//	value_2 = (unsigned int)(1400 / 100) ;
			//	value  = (unsigned int)((140 - value_2)*0.77);
			//	lexilight.pwm_led_driver = value;
			//}
			//else{
			//	value_2 = (unsigned int)(1400 / lexilight.duty) ;
			//	value  = (unsigned int)((140 - value_2)*0.77);
			//	lexilight.pwm_led_driver = value;
			//}
			//__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, value);
			if (lexilight.duty <= 20){
				lux_cmd =  900 + (1400 - 900)*(lexilight.duty - 10)/10;
			}
			else {
				lux_cmd =  1400;
			}
			//Calcule value I en fonction de dutycycle
			if (lexilight.state  == LIGHT_STATE_STANDARD){
				value_2 = (unsigned int)(lux_cmd / 100) ;
				value  = (unsigned int)((140 - value_2)*0.77);
				lexilight.pwm_led_driver = value;
			}
			else{
				value_2 = (unsigned int)(lux_cmd / lexilight.duty) ;
				value  = (unsigned int)((140 - value_2)*0.77);
				lexilight.pwm_led_driver = value;
			}
			__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, value);
			break;
			
		case LIGHT_LUM_LEVEL_1:
			//Calculate lux max
			if (lexilight.duty <= 20){
				lux_cmd =  1000 + (2200 - 1000)*(lexilight.duty - 10)/10;
			}
			else {
				lux_cmd =  2200;
			}
			//Calcule value I en fonction de dutycycle
			if (lexilight.state  == LIGHT_STATE_STANDARD){
				value_2 = (unsigned int)(lux_cmd / 100) ;
				value  = (unsigned int)((140 - value_2)*0.77);
				lexilight.pwm_led_driver = value;
			}
			else{
				value_2 = (unsigned int)(lux_cmd / lexilight.duty) ;
				value  = (unsigned int)((140 - value_2)*0.77);
				lexilight.pwm_led_driver = value;
			}
			__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, value);
			break;
			
		case LIGHT_LUM_LEVEL_2:
			//Calculate lux max
			if (lexilight.duty <= 20){
				lux_cmd =  1200 + (2800 - 1200)*(lexilight.duty - 10)/10;
			}
			else {
				lux_cmd =  2800;
			}
			//Calcule value I en fonction de dutycycle
			if (lexilight.state  == LIGHT_STATE_STANDARD){
				value_2 = (unsigned int)(lux_cmd / 100) ;
				value  = (unsigned int)((140 - value_2)*0.77);
				lexilight.pwm_led_driver = value;
			}
			else{
				value_2 = (unsigned int)(lux_cmd / lexilight.duty) ;
				value  = (unsigned int)((140 - value_2)*0.77);
				lexilight.pwm_led_driver = value;
			}
			__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_1, value);
			break;
			
		default:
			break;
	}
}


void Lexi_Switch_Lum_Level(){
	switch (lexilight.lum_level){
		
		case LIGHT_LUM_LEVEL_0:
			lexilight.lum_level = LIGHT_LUM_LEVEL_1;
			break;
		
		case LIGHT_LUM_LEVEL_1:
			lexilight.lum_level = LIGHT_LUM_LEVEL_2;
			break;
		
		case LIGHT_LUM_LEVEL_2:
		default:
			lexilight.lum_level = LIGHT_LUM_LEVEL_0;
			break;
	}
}
void Lexi_Switch_State_On_Off(TIM_HandleTypeDef htim , TIM_HandleTypeDef htim_lexi){
	if ( Lexi_Detect_Long_Push_ON_OFF())	//Long Appuie => Turn OFF Lampe
	{
		lexilight.state = LIGHT_STATE_OFF;	//Set State
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET);	//Turn OFF LED DRIVER
		//Turn off all led
		__HAL_TIM_SET_COMPARE(&htim , TIM_CHANNEL_1,0);	//Set Led Red Off
		__HAL_TIM_SET_COMPARE(&htim , TIM_CHANNEL_2,0);	//Set Led Green Off
		__HAL_TIM_SET_COMPARE(&htim , TIM_CHANNEL_3,0);	//Set Led Blue Off
	}
	else	//If not
	{
		if (lexilight.state == LIGHT_STATE_ON)	//Mode LexiLight => Standard
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET);
			lexilight.state = LIGHT_STATE_STANDARD;
			//Turn ON led green
			__HAL_TIM_SET_COMPARE(&htim , TIM_CHANNEL_1,0);		//Set Led Red Off
			__HAL_TIM_SET_COMPARE(&htim , TIM_CHANNEL_2,0);		//Set Led Green Off
			__HAL_TIM_SET_COMPARE(&htim , TIM_CHANNEL_3,50);	//Set Led Blue On
		}
		else	//Other Mode to LexiLight
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET);
			lexilight.state = LIGHT_STATE_ON;
			Lexi_DO_PWM_LEXIMODE (htim_lexi);
			//Turn ON led green
			__HAL_TIM_SET_COMPARE(&htim , TIM_CHANNEL_1,0);		//Set Led Red Off
			__HAL_TIM_SET_COMPARE(&htim , TIM_CHANNEL_2,50);	//Set Led Green On
			__HAL_TIM_SET_COMPARE(&htim , TIM_CHANNEL_3,0);		//Set Led Blue Off
	  }
	}
}


static void Lexi_Update_Frequency(TIM_HandleTypeDef htim){
	//Freq = (FreqMax-FreqMin)*ADC_Value/1024 + FreqMin
	//uint16_t interval  =  lexilight.freq_max - lexilight.freq_min;
	//lexilight.freq_cmd = (uint32_t)((lexilight.adc_freq_new*interval/lexilight.adc_number_step) + lexilight.freq_min);
	//Custom courbe 30% 25% = 255, 35% =  358
	if (lexilight.adc_freq_new <= 225 ){
		uint16_t interval  =  80 - lexilight.freq_min;
		lexilight.freq_cmd = (uint32_t)((lexilight.adc_freq_new*interval/(225 - 0)) + lexilight.freq_min);
	}
	else if (lexilight.adc_freq_new >= 378 ){
		uint16_t interval  =  120 - 80;
		lexilight.freq_cmd = (uint32_t)(((lexilight.adc_freq_new - 378 )*interval/(1023 - 378)) + 80);
	}
	else {
		lexilight.freq_cmd = 80;
	}
	
	
	if (lexilight.freq_cmd >= lexilight.freq_max){
		lexilight.freq_cmd = lexilight.freq_max;
	}
	//Detect if turn the potentiometer
	if ((lexilight.freq_cmd >= lexilight.freq_min) 
		&& (lexilight.freq_cmd <= lexilight.freq_max) 
		&& (lexilight.freq != lexilight.freq_cmd))
	{
		//Update prescaler => Update Frequency
		int prescaler = (uint16_t)(lexilight.system_clock/(lexilight.freq*100) - 1);
		__HAL_TIM_SET_PRESCALER(&htim, prescaler);
		lexilight.freq = lexilight.freq_cmd;
	}
}



static void Lexi_Update_Duty(TIM_HandleTypeDef htim){
	//Duty = (DutyMax-DutyMin)*ADC_Value/1024 + DutyMin
	//uint16_t interval  =  lexilight.duty_max - lexilight.duty_min;
	//lexilight.duty_cmd = (uint32_t)((lexilight.adc_duty_new*interval/lexilight.adc_number_step) + lexilight.duty_min);
	//Custom courbe 30% 25% = 255, 35% =  358
	if (lexilight.adc_duty_new <= 225 ){
		uint16_t interval  =  20 - lexilight.duty_min;
		lexilight.duty_cmd = (uint32_t)((lexilight.adc_duty_new*interval/(225 - 0)) + lexilight.duty_min);
	}
	else if (lexilight.adc_duty_new >= 378 ){
		uint16_t interval  =  40 - 20;
		lexilight.duty_cmd = (uint32_t)(((lexilight.adc_duty_new - 378 )*interval/(1023 - 378)) + 20);
	}
	else {
		lexilight.duty_cmd = 20;
	}
	
	if (lexilight.duty_cmd >= lexilight.duty_max){
		lexilight.duty_cmd = lexilight.duty_max;
	}
	//Detect if turn the potentiometer
	if ((lexilight.duty_cmd >= lexilight.duty_min) 
		&& (lexilight.duty_cmd <= lexilight.duty_max) 
		&& (lexilight.duty != lexilight.duty_cmd))
	{
		//Update duty
		__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_2,lexilight.duty);
		lexilight.duty = lexilight.duty_cmd;
	}
}

void Lexi_RecordTime_ON_OFF_Start(){
	lexilight.timer_on_off_start = HAL_GetTick ();
}
void Lexi_RecordTime_ON_OFF_End(){
	lexilight.timer_on_off_end = HAL_GetTick ();
}
bool Lexi_Detect_Long_Push_ON_OFF (){
	if ((lexilight.timer_on_off_end - lexilight.timer_on_off_start) > 1500 ) //Time in milisecond
	{
		return true;
	}
	else
	{
		return false;
	}
}
