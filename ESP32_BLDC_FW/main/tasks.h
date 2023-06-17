/*
 * tasks_common.h
 *
 *  Created on: Jun 13, 2023
 *      Author: rstre
 */

#ifndef MAIN_TASKS_H_
#define MAIN_TASKS_H_

#include "motor_model.h"


#define FOC_MAIN_TASK_STACK_SIZE							8192
#define FOC_MAIN_TASK_PRIORITY								7				//TODO: Adjust as needed
#define FOC_MAIN_TASK_CORE_ID								1

#define FAULT_MONITOR_TASK_STACK_SIZE						4096
#define FAULT_MONITOR_TASK_PRIORITY							5			//TODO: Adjust as needed
#define FAULT_MONITOR_TASK_CORE_ID							1



/*
 * ----------------------------------------------------------------------------------------------------------
 * Task-Related Functions to be Callled From Main
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to initialize the motor members and start the control loop task
void start_motor_control(motor_obj_t *motor);


#endif /* MAIN_TASKS_H_ */
