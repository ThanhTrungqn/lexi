#ifndef __LEXILIGHT_H
#define __LEXILIGHT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

#define SYSTEM_CLOCK_PER_SECOND 		48000000
#define LEXILIGHT_DUTY_INTERVAL 		100		//Level of Luminosity from 0 to 100
#define LEXILIGHT_FREQUENCY_MAX 		120		//Need Value (max +1)
#define LEXILIGHT_FREQUENCY_MIN 		65
#define LEXILIGHT_FREQUENCY_DEFAULT 80
#define LEXILIGHT_DUTY_DEFAULT 			20
#define LEXILIGHT_DUTY_MAX 					40		//Need value (max + 1)
#define LEXILIGHT_DUTY_MIN 					10
#define LEXILIGHT_ADC_VARIANT				10 	//If variant >  10 value; will update
#define LEXILIGHT_ADC_BIT						10	//Change if Varier

typedef enum
{
	/* Application's state machine's initial state. */
	LIGHT_STATE_INIT=0,				//Initial State
	LIGHT_STATE_WAIT_500_MS,	//Wait 500ms after Initial
	LIGHT_STATE_SERVICE,			//Ready for Service
	LIGHT_STATE_ON,						//Turn On Light LexiLight
	LIGHT_STATE_STANDARD,			//Turn Light Normal
	LIGHT_STATE_OFF,					//Turn Off Light
	LIGHT_STATE_ERROR					//Error

} LIGHT_STATES;

typedef enum
{
	/* Light Luminosity Level */
	LIGHT_LUM_LEVEL_0 = 0,
	LIGHT_LUM_LEVEL_1,	
	LIGHT_LUM_LEVEL_2
} LIGHT_LUM_LEVEL;

typedef struct {
		unsigned long system_clock;
    unsigned int duty;
    unsigned int freq;
    unsigned int duty_cmd;
    unsigned int freq_cmd;
    unsigned int duty_max;
    unsigned int duty_min;
    unsigned int freq_max;
    unsigned int freq_min;
		unsigned int adc_duty_raw;
		unsigned int adc_freq_raw;
		unsigned int adc_duty_new;
		unsigned int adc_freq_new;
		unsigned int adc_variant;
		unsigned int adc_number_step;
		unsigned int timer_on_off_start;
		unsigned int timer_on_off_end;
		unsigned int pwm_led_driver;
		LIGHT_STATES state;
		LIGHT_LUM_LEVEL lum_level;
} LEXILIGHT_DATA;

void Lexi_RecordTime_ON_OFF_Start(void);
void Lexi_RecordTime_ON_OFF_End(void);
bool Lexi_Detect_Long_Push_ON_OFF (void);
//Function main task of lexilight
void Lexi_Task (TIM_HandleTypeDef htim_pwm_led, TIM_HandleTypeDef htim_pwm_lum_driver, uint32_t * adc_raw);
//Function init data of lexilight
void Lexi_Init_Data (void);
//Function read ADC value and do PWM_LED signal PB3
void Lexi_DO_ADC_PWM (TIM_HandleTypeDef htim, uint32_t * adc_raw);
//Function do standard PWM
void Lexi_DO_Standard_PWM (TIM_HandleTypeDef htim);
//Function do PWM_LUM_DRIVER en fonction de Boutton Capacitive et duty
void Lexi_DO_PWM_LUM_DRIVER (TIM_HandleTypeDef htim);
//Function do State Light en fonction de signal CDE_ON_OFF
void Lexi_Switch_State_On_Off(TIM_HandleTypeDef htim , TIM_HandleTypeDef htim_lexi);
//Function do Lum_Level en fonction de signal BOUTON_LUM
void Lexi_Switch_Lum_Level(void);
void Lexi_DO_PWM_LEXIMODE (TIM_HandleTypeDef htim);
//Function update Frequency and Duty of PWM_LED (PB3)
static void Lexi_Update_Frequency(TIM_HandleTypeDef htim);
static void Lexi_Update_Duty(TIM_HandleTypeDef htim);

#endif
