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
 * Functions for Clarke Transforms
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to transform 3 phase values into the Beta vs Alpha domain
void run_clarke(float phases[3], alpha_beta_t *alpha_beta)
{
	alpha_beta->alpha = ((2.0f * phases[0]) - (phases[1] + phases[2])) * MATH_ONE_OVER_THREE;
	alpha_beta->beta = (phases[1] - phases[2]) * MATH_ONE_OVER_ROOT_THREE;
}

//Function to transform Beta vs Alpha domain into the 3 phase values
void run_inv_clarke(alpha_beta_t alpha_beta, float *phases[3])
{
	phases[0] = alpha_beta.alpha;
	phases[1] = (alpha_beta.alpha * (-MATH_ONE_OVER_TWO)) + (alpha_beta.beta * ROOT_THREE_OVER_TWO);
	phases[2] = (alpha_beta.alpha * (-MATH_ONE_OVER_TWO)) - (alpha_beta.beta * ROOT_THREE_OVER_TWO);
}

/*
 * ----------------------------------------------------------------------------------------------------------
 * Functions for Park Transforms
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to transform the stationary Beta vs Alpha domain to the rotating D-Q domain
void run_park(float theta_rad, alpha_beta_t alpha_beta, d_q_t *d_q)							//Theta is calculated with motor equations and Valpha, Vbeta, Ialpha, and Ibeta
{
	d_q->direct = alpha_beta.alpha * __cos(theta_rad) + alpha_beta.beta * __sin(theta_rad);
	d_q->quadrature = alpha_beta.beta * __cos(theta_rad) - alpha_beta.alpha * __sin(theta_rad);
}

//Function to transform the rotating D-Q domain to the stationary Beta vs Alpha domain
void run_inv_park(float theta_rad, d_q_t d_q, alpha_beta_t *alpha_beta)
{
	alpha_beta->alpha = d_q.direct * __cos(theta_rad) - d_q.quadrature * __sin(theta_rad);
	alpha_beta->beta = d_q.direct * __sin(theta_rad) + d_q.quadrature * __cos(theta_rad);
}


