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
	lexilight.adc_number_step = 1;
	lexilight.lum_level 		= LIGHT_LUM_LEVEL_0;
	lexilight.state = LIGHT_STATE_WAIT_500_MS;
	for(int i = 0; i < LEXILIGHT_ADC_BIT ; i ++){
		lexilight.adc_number_step *=2;
	}
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
		
		case LIGHT_STATE_OFF:
			Lexi_DO_ADC_PWM (htim_pwm_led, adc_raw);
			Lexi_DO_PWM_LUM_DRIVER(htim_pwm_lum_driver);
			//Turn Off EN_DRIVER
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET);
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
void Lexi_DO_PWM_LUM_DRIVER (TIM_HandleTypeDef htim){
	switch (lexilight.lum_level){
		case LIGHT_LUM_LEVEL_0:
			__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_3,lexilight.duty);
			break;
		case LIGHT_LUM_LEVEL_1:
			__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_3,lexilight.duty);
			break;
		case LIGHT_LUM_LEVEL_2:
			__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_3,lexilight.duty);
			break;
		case LIGHT_LUM_LEVEL_3:
			__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_3,lexilight.duty);
			break;
		case LIGHT_LUM_LEVEL_4:
			__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_3,lexilight.duty);
			break;
		default:
			break;
	}
}

void Lexi_Switch_Lum_Level(){
	switch (lexilight.lum_level){
		
		case LIGHT_LUM_LEVEL_0:
			lexilight.lum_level = LIGHT_LUM_LEVEL_1;
			//Todo : Update couleur LED_RGB
			break;
		
		case LIGHT_LUM_LEVEL_1:
			lexilight.lum_level = LIGHT_LUM_LEVEL_2;
		//Todo : Update couleur LED_RGB
			break;
		
		case LIGHT_LUM_LEVEL_2:
			lexilight.lum_level = LIGHT_LUM_LEVEL_3;
		//Todo : Update couleur LED_RGB
			break;
		
		case LIGHT_LUM_LEVEL_3:
			lexilight.lum_level = LIGHT_LUM_LEVEL_4;
			//Todo : Update couleur LED_RGB
			break;
		
		case LIGHT_LUM_LEVEL_4:
		default:
			lexilight.lum_level = LIGHT_LUM_LEVEL_0;
			//Todo : Update couleur LED_RGB
			break;
	}
}
void Lexi_Switch_State_On_Off(){
	//Turn OFF if light is on
	if (lexilight.state == LIGHT_STATE_ON){
		lexilight.state = LIGHT_STATE_OFF;
	}
	//Turn ON if light is off
	else if ((lexilight.state == LIGHT_STATE_OFF)
		||(lexilight.state == LIGHT_STATE_SERVICE))
	{
		lexilight.state = LIGHT_STATE_ON;
	}
}

static void Lexi_Update_Frequency(TIM_HandleTypeDef htim){
	uint16_t interval  =  lexilight.freq_max - lexilight.freq_min;
	lexilight.freq_cmd = (uint32_t)((lexilight.adc_freq_new*interval/lexilight.adc_number_step) + lexilight.freq_min);
	if ((lexilight.freq_cmd >= lexilight.freq_min) 
		&& (lexilight.freq_cmd <= lexilight.freq_max) 
		&& (lexilight.freq != lexilight.freq_cmd))
	{
		int prescaler = (uint16_t)(lexilight.system_clock/(lexilight.freq*100) - 1);
		__HAL_TIM_SET_PRESCALER(&htim, prescaler);
		lexilight.freq = lexilight.freq_cmd;
	}
}

static void Lexi_Update_Duty(TIM_HandleTypeDef htim){
	uint16_t interval  =  lexilight.duty_max - lexilight.duty_min;
	lexilight.duty_cmd = (uint32_t)((lexilight.adc_duty_new*interval/lexilight.adc_number_step) + lexilight.duty_min);
	if ((lexilight.duty_cmd >= lexilight.duty_min) 
		&& (lexilight.duty_cmd <= lexilight.duty_max) 
		&& (lexilight.duty != lexilight.duty_cmd))
	{
		__HAL_TIM_SET_COMPARE(&htim, TIM_CHANNEL_2,lexilight.duty);
		//Todo : changer le signal PWM_LUM_DRIVER adapter avec Duty de LED
		lexilight.duty = lexilight.duty_cmd;
	}
}
