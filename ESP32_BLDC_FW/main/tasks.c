/*
 * tasks.c
 *
 *  Created on: Jun 16, 2023
 *      Author: rstre
 *
 *  This file is used to build the primary tasks for motor control, using other file functions as building blocks
 */
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "tasks.h"
#include "motor_model.h"

/*
 * ----------------------------------------------------------------------------------------------------------
 * Functions to run infinitely as FreeRTOS tasks
 * ----------------------------------------------------------------------------------------------------------
 */
//This function handles the main FOC control loop
static void foc_main_task(void *motor_obj)
{
	motor_obj_t *motor = (motor_obj_t *)motor_obj;

	for(;;)
	{
		if(!motor->locked_flag)
		{
			motor->locked_flag = true;
			run_motor_control(motor);
			motor->locked_flag = false;
			vTaskDelay(5 / portTICK_PERIOD_MS);			//Maybe time this with the PWMs?
		}
	}
}

//This function monitors the measurements for errors
static void fault_monitor_task(void * motor_obj)
{

	motor_obj_t *motor = (motor_obj_t *)motor_obj;
	for(;;)
	{
		if(!motor->locked_flag)
		{
			motor->locked_flag = true;
			motor_fault_check(motor);
			motor->locked_flag = false;
			vTaskDelay(200 / portTICK_PERIOD_MS);
		}
	}
}



/*
 * ----------------------------------------------------------------------------------------------------------
 * Task-Related Functions to be Callled From Main
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to initialize the motor members and start the control loop task
void start_motor_control(motor_obj_t *motor)
{
	//Configure the IOs in the HAL
	configure_mcpwms(&motor->hal_obj);
	configure_adcs(&motor->hal_obj);
	configure_gpios(&motor->hal_obj);
	read_adcs(&motor->hal_obj);

	//Create the main tasks to handle motor control
	xTaskCreatePinnedToCore(&foc_main_task, "foc_main_task", FOC_MAIN_TASK_STACK_SIZE, NULL, FOC_MAIN_TASK_PRIORITY, NULL, FOC_MAIN_TASK_CORE_ID);
	xTaskCreatePinnedToCore(&fault_monitor_task, "fault_monitor_task", FAULT_MONITOR_TASK_STACK_SIZE, NULL, FAULT_MONITOR_TASK_PRIORITY, NULL, FAULT_MONITOR_TASK_CORE_ID);
}





