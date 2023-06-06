/*
 * bldc_math.h
 *
 *  Created on: Jun 4, 2023
 *      Author: rstre
 */

#ifndef MAIN_BLDC_MATH_H_
#define MAIN_BLDC_MATH_H_

//Define math constants for improved float performance (avoid dividing)
#define MATH_ONE_OVER_THREE				0.333f
#define MATH_ONE_OVER_TWO				0.5f
#define MATH_ONE_OVER_ROOT_THREE		0.577f
#define ROOT_THREE_OVER_TWO				0.866f

/*
 * ----------------------------------------------------------------------------------------------------------
 * Object for PID Control operations
 * ----------------------------------------------------------------------------------------------------------
 */
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

/*
 * ----------------------------------------------------------------------------------------------------------
 * Object for Beta vs Alpha domain values
 * ----------------------------------------------------------------------------------------------------------
 */
typedef struct alpha_beta
{
	float alpha;
	float beta;
}alpha_beta_t;

/*
 * ----------------------------------------------------------------------------------------------------------
 * Object for D-Q domain values
 * ----------------------------------------------------------------------------------------------------------
 */
typedef struct d_q
{
	float direct;
	float quadrature;
}d_q_t;

/*
 * ----------------------------------------------------------------------------------------------------------
 * Functions for PID Control operations
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to create a new PID Object
pid_obj_t new_pid(float kp, float ki, float kd, float dt, float min, float max);

//Function to return the next PID output given the current value
float run_pid(pid_obj_t *pid_obj, float current_value);

//Function to reset the PID
void reset_pid(pid_obj_t *pid_obj);

/*
 * ----------------------------------------------------------------------------------------------------------
 * Functions for Clarke Transforms
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to transform 3 phase values into the Beta vs Alpha domain
void run_clarke(float phases[3], alpha_beta_t *alpha_beta);

//Function to transform Beta vs Alpha domain into the 3 phase values
void run_inv_clarke(alpha_beta_t alpha_beta, float *phases[3]);

/*
 * ----------------------------------------------------------------------------------------------------------
 * Functions for Park Transforms
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to transform the stationary Beta vs Alpha domain to the rotating D-Q domain
void run_park(float theta, alpha_beta_t alpha_beta, d_q_t *d_q);

//Function to transform the rotating D-Q domain to the stationary Beta vs Alpha domain
void run_inv_park(float theta, d_q_t d_q, alpha_beta_t alpha_beta);

#endif /* MAIN_BLDC_MATH_H_ */
