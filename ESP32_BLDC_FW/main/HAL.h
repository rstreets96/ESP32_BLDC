/*
 * HAL.h
 *
 *  Created on: May 31, 2023
 *      Author: rstre
 */

#ifndef MAIN_HAL_H_
#define MAIN_HAL_H_

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
#define MCPWM_CLK_PRESCALER		4
#define MCPWM_FREQUENCY_HZ		20000												//20kHz, period = 50us
#define MCPWM_RESOLUTION_HZ		10000000 											//10MHz, 1 tick = 0.1us
#define MCPWM_PERIOD_TICKS		MCPWM_RESOLUTION_HZ / MCPWM_FREQUENCY				//PWM period in us

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
	mcpwm_gen_handle_t pwm_generators[3];
	mcpwm_comparator_handle_t pwm_comparators[3];
	mcpwm_fault_handle_t pwm_fault;

}hal_obj_t;


/*
 * ----------------------------------------------------------------------------------------------------------
 * Function to configure the MCPWMs
 * ----------------------------------------------------------------------------------------------------------
 */
void configure_mcpwms(mcpwm_timer_handle_t timer, mcpwm_oper_handle_t operators[], mcpwm_gen_handle_t generators[], mcpwm_comparator_handle_t comparators[], mcpwm_fault_handle_t fault);


#endif /* MAIN_HAL_H_ */
