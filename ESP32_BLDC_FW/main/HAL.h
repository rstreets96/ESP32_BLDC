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
#define MCPWM_FREQUENCY			15000


#endif /* MAIN_HAL_H_ */
