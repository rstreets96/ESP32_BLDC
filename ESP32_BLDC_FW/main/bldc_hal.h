/*
 * HAL.h
 *
 *  Created on: May 31, 2023
 *      Author: rstre
 */

#ifndef MAIN_HAL_H_
#define MAIN_HAL_H_

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"

#include "bldc_math.h"
/*
 * ----------------------------------------------------------------------------------------------------------
 * Pin Definitions
 * ----------------------------------------------------------------------------------------------------------
 */
/*
 * MCPWMs' Pins
 */
#define GPIO_A_HI 				42
#define GPIO_A_LO 				41
#define GPIO_B_HI 				40
#define GPIO_B_LO 				39
#define GPIO_C_HI 				38
#define GPIO_C_LO 				37

/*
 * MCPWM Sense Pins (used to trigger ADC measurements in sync with PWMs)
 */
#define GPIO_PWM_SENSE_A			(1ULL << GPIO_NUM_14)
#define GPIO_PWM_SENSE_B			(1ULL << GPIO_NUM_17)
#define GPIO_PWM_SENSE_C			(1ULL << GPIO_NUM_18)

/*
 * ADC Channels
 */
#define ADC_A_V					ADC_CHANNEL_3
#define ADC_B_V					ADC_CHANNEL_4
#define ADC_C_V					ADC_CHANNEL_5
#define ADC_DC_V				ADC_CHANNEL_6
#define ADC_DC_I				ADC_CHANNEL_7
#define ADC_CT_V				ADC_CHANNEL_8

/*
 * HALL GPIOs
 */
#define GPIO_HALL_A				48
#define GPIO_HALL_B				47
#define GPIO_HALL_C				21

/*
 * DRV Pins
 */
#define GPIO_NFAULT				GPIO_NUM_11
#define GPIO_NSLEEP				(1ULL << GPIO_NUM_12)
#define GPIO_DRVOFF				(1ULL << GPIO_NUM_13)
#define GPIO_DRVLED				(1ULL << GPIO_NUM_35)

/*
 * ----------------------------------------------------------------------------------------------------------
 * ADC Conversion Factors
 * ----------------------------------------------------------------------------------------------------------
 */
//Shared Values
#define ADC_ATTEN_FACTOR		(2.0f)						//TODO:Learn more about this

//Current Measurements
#define CURR_RES_OHMS			(0.001f)					//Current shunt resistor
#define CSA_GAIN				(5.0f)						//Can be 5, 10, 20, or 40
#define ONE_OVER_RES_GAIN		(200.0f)					//1 / (CURR_RES_OHMS * CSA_GAIN)
#define CURR_OFFSET				(1.65f)						//Set with CSA gain of the DRV8329

//Voltage Measurements
#define VOLT_DIV_SCALAR			(23.044f)					//114.99k / 4.99k, same for all voltage measurements


/*
 * ----------------------------------------------------------------------------------------------------------
 * HW-Related Constants
 * ----------------------------------------------------------------------------------------------------------
 */
//Defines used to set the MCPWM clock prescaler
#define MCPWM_FREQUENCY_HZ		20000												//20kHz, period = 50us
#define MCPWM_RESOLUTION_HZ		10000000 											//10MHz, 1 tick = 0.1us
#define MCPWM_PERIOD_TICKS		500     		//MCPWM_RESOLUTION_HZ / MCPWM_FREQUENCY_HZ

#define CMP_INDEX_A				0
#define CMP_INDEX_B				1
#define GEN_INDEX_HI			0
#define GEN_INDEX_LO			1

#define ADC_ATTEN				ADC_ATTEN_DB_11

/*
 * ----------------------------------------------------------------------------------------------------------
 * Object to hold the measured values from the ADC channels
 * ----------------------------------------------------------------------------------------------------------
 */
typedef struct adc_data
{
	abc_t phaseV_V;
	abc_t phaseV_raw;
	abc_t phaseI_A;
	abc_t phaseI_raw;
	float dcV_V;
	float dcI_A;
	float ctV_V;
}adc_data_t;

/*
 * ----------------------------------------------------------------------------------------------------------
 * Object to hold HAL data and configurations
 * ----------------------------------------------------------------------------------------------------------
 */
typedef struct hal_obj
{
	//MCPWM objects
	mcpwm_timer_handle_t pwm_timer;
	mcpwm_oper_handle_t pwm_operators[3];
	mcpwm_gen_handle_t pwm_generators[3][2];
	mcpwm_cmpr_handle_t pwm_comparators[3][2];
	mcpwm_fault_handle_t pwm_fault;

	//ADC objects
	adc_oneshot_unit_handle_t adc1_handle;
	adc_cali_handle_t adc_chnl_cali;

	//Object to hold ADC measurements
	adc_data_t adc_data;
}hal_obj_t;

/*
 * ----------------------------------------------------------------------------------------------------------
 * MCPWM Related Functions
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to configure the MCPWMs
void configure_mcpwms(hal_obj_t *hal_obj);

//Function to set the duty cycle of each of the three MCPWMs (the low side signals are always the inverse + dead time)
void mcpwm_set_duty(hal_obj_t *hal_obj, abc_t pwmData);


/*
 * ----------------------------------------------------------------------------------------------------------
 * ADC Related Functions
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to configure the ADCs
void configure_adcs(hal_obj_t *hal_obj);

//Function to read the ADC channels and save the results to the adc_data struct
void read_adcs(hal_obj_t *hal_obj);

//Function to convert current measurements from raw value to amperage
float current_raw_to_amps(float curr_raw);

//Function to convert current measurements from raw value to volts
float voltage_raw_to_volts(float volt_raw, float vbatt_over_two);


/*
 * ----------------------------------------------------------------------------------------------------------
 * Other GPIO Functions
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to Configure the remaining GPIOs
void configure_gpios(hal_obj_t *hal_obj);

#endif /* MAIN_HAL_H_ */
