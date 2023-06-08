/*
 * bldc_math.c
 *
 *  Created on: Jun 4, 2023
 *      Author: rstre
 *
 *  This file will house the main math behind FOC. This includes Clarke and Park transforms, PID controllers, and
 *  observers
 */
#include <math.h>

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
void run_clarke(abc_t phases, alpha_beta_t *alpha_beta)
{
	alpha_beta->alpha = ((2.0f * phases.a) - (phases.b + phases.c)) * MATH_ONE_OVER_THREE;
	alpha_beta->beta = (phases.b - phases.c) * MATH_ONE_OVER_ROOT_THREE;
}

//Function to transform Beta vs Alpha domain into the 3 phase values
void run_inv_clarke(alpha_beta_t alpha_beta, abc_t *phases)
{
	phases->a = alpha_beta.alpha;
	phases->b = (alpha_beta.alpha * (-MATH_ONE_OVER_TWO)) + (alpha_beta.beta * MATH_ROOT_THREE_OVER_TWO);
	phases->c = (alpha_beta.alpha * (-MATH_ONE_OVER_TWO)) - (alpha_beta.beta * MATH_ROOT_THREE_OVER_TWO);
}


/*
 * ----------------------------------------------------------------------------------------------------------
 * Functions for Park Transforms
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to transform the stationary Beta vs Alpha domain to the rotating D-Q domain
void run_park(float theta_rad, alpha_beta_t alpha_beta, d_q_t *d_q)
{
	d_q->direct = alpha_beta.alpha * cosf(theta_rad) + alpha_beta.beta * sinf(theta_rad);
	d_q->quadrature = alpha_beta.beta * cosf(theta_rad) - alpha_beta.alpha * sinf(theta_rad);
}

//Function to transform the rotating D-Q domain to the stationary Beta vs Alpha domain
void run_inv_park(float theta_rad, d_q_t d_q, alpha_beta_t *alpha_beta)
{
	alpha_beta->alpha = d_q.direct * cosf(theta_rad) - d_q.quadrature * sinf(theta_rad);
	alpha_beta->beta = d_q.direct * sinf(theta_rad) + d_q.quadrature * cosf(theta_rad);
}


/*
 * ----------------------------------------------------------------------------------------------------------
 * Observer Functions
 * ----------------------------------------------------------------------------------------------------------
 */
//Function used to create and initialize a new Luenberger Observer object
luenberger_observer_t new_luenberger(float dt, float r_s, float l_s, float pole_tune)
{
	luenberger_observer_t obs_obj;
	obs_obj.dt = dt;
	obs_obj.r_s = r_s;
	obs_obj.l_s = l_s;

	obs_obj.coeff_f = 1 - (r_s * dt / l_s);
	obs_obj.coeff_g = dt / l_s;

	//Tune both k1 and k2 with one parameter
	float eo1 = obs_obj.coeff_f / pole_tune;
	float eo2 = 1 / pole_tune;
	float a1 = eo1 + eo2;
	float a2 = eo1 * eo2;
	obs_obj.k1 = (1 + obs_obj.coeff_f - a1) / dt;
	obs_obj.k2 = (a1 - a2 - 1) / (obs_obj.coeff_g * dt);

	return obs_obj;
}

//Function to take in feedback and calculate the next estimations
//Reference: https://pcimasia-expo.cn.messefrankfurt.com/content/dam/messefrankfurt-redaktion/pcim_asia/download/ppt_pac2021/Qianbao%20Mi.pdf
void run_luenberger(alpha_beta_t v_ab, alpha_beta_t i_ab, luenberger_observer_t *obs_obj)
{
	//Middle step computations
	float ia_bar = obs_obj->coeff_f * obs_obj->ia_est + obs_obj->coeff_g * (v_ab.alpha - obs_obj->ea_est);
	float ib_bar = obs_obj->coeff_f * obs_obj->ib_est + obs_obj->coeff_g * (v_ab.beta - obs_obj->eb_est);
	float ea_bar = obs_obj->ea_est - (obs_obj->mot_speed * obs_obj->dt * obs_obj->eb_est);
	float eb_bar = obs_obj->ea_est - (obs_obj->mot_speed * obs_obj->dt * obs_obj->eb_est);

	//Calculate error
	float ia_err = i_ab.alpha - ia_bar;
	float ib_err = i_ab.beta - ib_bar;

	//Use the calculations to find the next iteration of the current and back-emf variables
	obs_obj->ia_est = ia_bar + obs_obj->k1 * obs_obj->dt * ia_err;
	obs_obj->ib_est = ib_bar + obs_obj->k1 * obs_obj->dt * ib_err;
	obs_obj->ea_est = ea_bar + obs_obj->k2 * obs_obj->dt * ia_err;
	obs_obj->eb_est = eb_bar + obs_obj->k2 * obs_obj->dt * ib_err;

	//Calculate angle from back emf components for PLL using arctan approximation
	float temp = -(obs_obj->ea_est / obs_obj->eb_est);
	float squared = temp * temp;
	float angle_out = ((A * squared - B) * squared + C) * temp;				//TODO: Verify that quadrant is preserved

	//Run PLL for speed and angle estimation
	float delta_theta = angle_out - obs_obj->mot_angle;						//TODO: Research more PLLs, add angle normalization (-pi to pi
	obs_obj->mot_angle += (obs_obj->mot_speed + obs_obj->pll_kp * delta_theta) * obs_obj->dt;
	obs_obj->mot_speed += obs_obj->pll_ki * delta_theta * obs_obj->dt;


}
