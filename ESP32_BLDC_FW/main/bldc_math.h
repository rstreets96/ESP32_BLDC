/*
 * bldc_math.h
 *
 *  Created on: Jun 4, 2023
 *      Author: rstre
 */

#ifndef MAIN_BLDC_MATH_H_
#define MAIN_BLDC_MATH_H_

typedef struct pid_obj
{
	float setpoint;
	float current_value;
	float kp;
	float ki;
	float kd;
	float err;
	float prev_err;
	float err_sum;
	float max_out;
	float min_out;
	float dt;
}pid_obj_t;

pid_obj_t new_pid(float kp, float ki, float kd, float dt, float min, float max);

//Function to return the next PID output given the current value
float run_pid(pid_obj_t *pid_obj, float current_value);

//Function to reset the PID
void reset_pid(pid_obj_t *pid_obj);

#endif /* MAIN_BLDC_MATH_H_ */
