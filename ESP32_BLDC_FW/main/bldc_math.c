/*
 * bldc_math.c
 *
 *  Created on: Jun 4, 2023
 *      Author: rstre
 *
 *  This file will house the math behind Clarke and Park transforms, as well as PID controllers
 */
#include "bldc_math.h"


/*
 * ----------------------------------------------------------------------------------------------------------
 * Functions for PID Control operations
 * ----------------------------------------------------------------------------------------------------------
 */

//Function to create a new PID Object
pid_obj_t new_pid(float kp, float ki, float kd, float dt, float min, float max)
{
	pid_obj_t pid_obj;
	pid_obj.kp = kp;
	pid_obj.ki = ki;
	pid_obj.kd = kd;
	pid_obj.dt = dt;
	pid_obj.min_out = min;
	pid_obj.max_out = max;

	return pid_obj;
}


//Function to get the next output from the PID object
float run_pid(pid_obj_t *pid_obj, float current_value)
{
	float output;
	pid_obj->current_value = current_value;
	pid_obj->prev_err = pid_obj->err;
	pid_obj->err = pid_obj->setpoint - pid_obj->current_value;
	pid_obj->err_sum += pid_obj->err;

	output = (pid_obj->kp * pid_obj->err) + (pid_obj->ki * pid_obj->err_sum * pid_obj->dt) + (pid_obj->kd * (pid_obj->err - pid_obj->prev_err) / pid_obj->dt);

	output = output > pid_obj->max_out ? pid_obj->max_out : output;
	output = output < pid_obj->min_out ? pid_obj->min_out : output;

	return output;
}


//Function to reset the operational parameters of the PID object
void reset_pid(pid_obj_t *pid_obj)
{
	pid_obj->err_sum = 0;
	pid_obj->prev_err = 0;
	pid_obj->err = 0;
	pid_obj->setpoint = 0;
	pid_obj->current_value = 0;
}


/*
 * ----------------------------------------------------------------------------------------------------------
 * Functions for Clark and Parke Transforms
 * ----------------------------------------------------------------------------------------------------------
 */

