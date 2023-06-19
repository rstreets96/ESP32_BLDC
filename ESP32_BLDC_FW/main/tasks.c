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
#include "freertos/queue.h"

#include "tasks.h"
#include "motor_model.h"

/*
 * ----------------------------------------------------------------------------------------------------------
 * Global Variables
 * ----------------------------------------------------------------------------------------------------------
 */
extern motor_obj_t global_motor;

QueueHandle_t pwm_isr_adc_queue_handle;

/*
 * ----------------------------------------------------------------------------------------------------------
 * Functions to run infinitely as FreeRTOS tasks
 * ----------------------------------------------------------------------------------------------------------
 */
//This function handles the main FOC control loop
static void foc_main_task()
{

	for(;;)
	{
		if(!global_motor.locked_flag)
		{
			global_motor.locked_flag = true;
			run_motor_control(&global_motor);
			global_motor.locked_flag = false;
			vTaskDelay(5 / portTICK_PERIOD_MS);			//Maybe time this with the PWMs?
		}
	}
}

//This function monitors the measurements for errors
static void fault_monitor_task()
{

	for(;;)
	{
		if(!global_motor.locked_flag)
		{
			global_motor.locked_flag = true;
			motor_fault_check(&global_motor);
			global_motor.locked_flag = false;
			vTaskDelay(200 / portTICK_PERIOD_MS);
		}
	}
}

//This function watches for pwm interrupt to trigger an adc reading, and converts reading to float value
static void adc_currents_raw_to_amps()
{
	isr_adc_queue_message_t msg;

	for(;;)
	{
		if(xQueueReceive(pwm_isr_adc_queue_handle, &msg, portMAX_DELAY))
		{
			switch(msg.msgID)
			{
				case PHASE_A_CONVERSION:
					break;
				case PHASE_B_CONVERSION:
					break;
				case PHASE_C_CONVERSION:
					break;
			}
		}
	}
}


/*
 * ----------------------------------------------------------------------------------------------------------
 * Task-Related functions to be called from Main
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to initialize the motor members and start the control loop task
void start_motor_control(void)
{
	//Configure the IOs in the HAL
	configure_mcpwms(&global_motor.hal_obj);
	configure_adcs(&global_motor.hal_obj);
	configure_gpios(&global_motor.hal_obj);
	read_adcs(&global_motor.hal_obj);

	//Create the Queue to pass info from ISR to ADC conversion task
	pwm_isr_adc_queue_handle = xQueueCreate(5, sizeof(isr_adc_queue_message_t));

	//Create the main tasks to handle motor control
	xTaskCreatePinnedToCore(&foc_main_task, "foc_main_task", FOC_MAIN_TASK_STACK_SIZE, NULL, FOC_MAIN_TASK_PRIORITY, NULL, FOC_MAIN_TASK_CORE_ID);
	xTaskCreatePinnedToCore(&fault_monitor_task, "fault_monitor_task", FAULT_MONITOR_TASK_STACK_SIZE, NULL, FAULT_MONITOR_TASK_PRIORITY, NULL, FAULT_MONITOR_TASK_CORE_ID);
	xTaskCreatePinnedToCore(&adc_currents_raw_to_amps, "adc_I_conv_task", ADC_I_CONV_TASK_STACK_SIZE, NULL, ADC_I_CONV_TASK_PRIORITY, NULL, ADC_I_CONV_TASK_CORE_ID);
}





