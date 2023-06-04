/*
 * HAL.h
 *
 *  Created on: May 31, 2023
 *      Author: rstre
 */

#ifndef MAIN_HAL_H_
#define MAIN_HAL_H_

#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"

/*
 * ----------------------------------------------------------------------------------------------------------
 * Pin Definitions
 * ----------------------------------------------------------------------------------------------------------
 */
/*
 * Define the GPIOs for the MCPWMs
 */
#define GPIO_A_HI 				42
#define GPIO_A_LO 				41
#define GPIO_B_HI 				40
#define GPIO_B_LO 				39
#define GPIO_C_HI 				38
#define GPIO_C_LO 				37

/*
 * Define the ADC Channels
 */
#define ADC_A_V					ADC_CHANNEL_3
#define ADC_B_V					ADC_CHANNEL_4
#define ADC_C_V					ADC_CHANNEL_5
#define ADC_DC_V				ADC_CHANNEL_6
#define ADC_DC_I				ADC_CHANNEL_7
#define ADC_CT_V				ADC_CHANNEL_8

/*
 * Define the HALL GPIOs
 */
#define GPIO_HALL_A				48
#define GPIO_HALL_B				47
#define GPIO_HALL_C				21

/*
 * DRV Pins
 */
#define GPIO_NFAULT				11
#define GPIO_NSLEEP				12
#define GPIO_DRVOFF				13

/*
 * ----------------------------------------------------------------------------------------------------------
 * HW-Related Constants
 * ----------------------------------------------------------------------------------------------------------
 */
//Defines used to set the MCPWM clock prescaler
#define MCPWM_FREQUENCY_HZ		20000												//20kHz, period = 50us
#define MCPWM_RESOLUTION_HZ		10000000 											//10MHz, 1 tick = 0.1us

#define CMP_INDEX_A				0
#define CMP_INDEX_B				1
#define GEN_INDEX_HI			0
#define GEN_INDEX_LO			1

#define ADC_ATTEN				ADC_ATTEN_DB_11

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
}hal_obj_t;

/*
 * ----------------------------------------------------------------------------------------------------------
 * Object to hold the measured values from the ADC channels
 * ----------------------------------------------------------------------------------------------------------
 */
typedef struct adc_data
{
	float phaseV_V[3];
	float dcV_V;
	float dcI_A;
	float ctV_V;
}adc_data_t;


/*
 * ----------------------------------------------------------------------------------------------------------
 * Function to configure the MCPWMs
 * ----------------------------------------------------------------------------------------------------------
 */
void configure_mcpwms(hal_obj_t *hal_obj);


/*
 * ----------------------------------------------------------------------------------------------------------
 * Function to configure the ADCs
 * ----------------------------------------------------------------------------------------------------------
 */
void configure_adcs(hal_obj_t *hal_obj);


#endif /* MAIN_HAL_H_ */
