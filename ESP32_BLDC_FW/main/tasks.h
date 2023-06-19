/*
 * tasks_common.h
 *
 *  Created on: Jun 13, 2023
 *      Author: rstre
 */

#ifndef MAIN_TASKS_H_
#define MAIN_TASKS_H_


#define FOC_MAIN_TASK_STACK_SIZE							8192
#define FOC_MAIN_TASK_PRIORITY								7				//TODO: Adjust as needed
#define FOC_MAIN_TASK_CORE_ID								1

#define FAULT_MONITOR_TASK_STACK_SIZE						4096
#define FAULT_MONITOR_TASK_PRIORITY							5			//TODO: Adjust as needed
#define FAULT_MONITOR_TASK_CORE_ID							1

#define ADC_I_CONV_TASK_STACK_SIZE							4096
#define ADC_I_CONV_TASK_PRIORITY							6			//TODO: Adjust as needed. Keep lower than main task to prevent changes mid calc
#define ADC_I_CONV_TASK_CORE_ID								1



/*
 * ----------------------------------------------------------------------------------------------------------
 * Enums to be used for Queue messages
 * ----------------------------------------------------------------------------------------------------------
 */
typedef enum isr_adc_message
{
	PHASE_A_CONVERSION = 0,
	PHASE_B_CONVERSION,
	PHASE_C_CONVERSION
}isr_adc_message_e;

/*
 * ----------------------------------------------------------------------------------------------------------
 * Structs to hold Queue enums
 * ----------------------------------------------------------------------------------------------------------
 */
typedef struct isr_adc_queue_message
{
	isr_adc_message_e msgID;
}isr_adc_queue_message_t;

/*
 * ----------------------------------------------------------------------------------------------------------
 * Task-Related functions to be called from Main
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to initialize the motor members and start the control loop task
void start_motor_control(void);


#endif /* MAIN_TASKS_H_ */
