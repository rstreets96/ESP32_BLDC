/*
 * bldc_math.h
 *
 *  Created on: Jun 4, 2023
 *      Author: rstre
 */

#ifndef MAIN_BLDC_MATH_H_
#define MAIN_BLDC_MATH_H_

/*
 * ----------------------------------------------------------------------------------------------------------
 * Math Constants
 * ----------------------------------------------------------------------------------------------------------
 */
//Define math constants for improved float performance (avoid dividing)
#define MATH_ONE_OVER_THREE				(0.333f)
#define MATH_ONE_OVER_TWO				(0.5f)
#define MATH_ONE_OVER_ROOT_THREE		(0.577f)
#define MATH_ROOT_THREE_OVER_TWO		(0.866f)
#define RAD_SEC_TO_RPM					(9.5493f)  //60 / 2pi

//Used for Arctan approximation
#define A 								(0.07765f)
#define B								(0.28743f)
#define C								(0.99518f)
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
 * Object for Three Phase values
 * ----------------------------------------------------------------------------------------------------------
 */
typedef struct abc
{
	float a;
	float b;
	float c;
}abc_t;

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
 * Struct for estimating rotor angle and speed
 * ----------------------------------------------------------------------------------------------------------
 */
//Luenberger observer object to track back-emf in the stationary Alpha-Beta domain
typedef struct luenberger_observer
{
	float k1;
	float k2;
	float coeff_f;
	float coeff_g;
	float dt;
	float r_s;
	float l_s;
	float ia_est;
	float ib_est;
	float va_est;
	float vb_est;
	float ea_est;
	float eb_est;
	float mot_angle;
	float mot_speed;
	float pll_kp;
	float pll_ki;
}luenberger_observer_t;

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
void run_clarke(abc_t phases, alpha_beta_t *alpha_beta);

//Function to transform Beta vs Alpha domain into the 3 phase values
void run_inv_clarke(alpha_beta_t alpha_beta, abc_t *phases);

/*
 * ----------------------------------------------------------------------------------------------------------
 * Functions for Park Transforms
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to transform the stationary Beta vs Alpha domain to the rotating D-Q domain
void run_park(float theta, alpha_beta_t alpha_beta, d_q_t *d_q);

//Function to transform the rotating D-Q domain to the stationary Beta vs Alpha domain
void run_inv_park(float theta, d_q_t d_q, alpha_beta_t *alpha_beta);

/*
 * ----------------------------------------------------------------------------------------------------------
 * Observer Functions
 * ----------------------------------------------------------------------------------------------------------
 */
//Function used to create and initialize a new Luenberger Observer object
luenberger_observer_t new_luenberger(float dt, float r_s, float l_s, float pole_tune);

//Function to take in feedback and calculate the next estimations
void run_luenberger(alpha_beta_t v_ab, alpha_beta_t i_ab, luenberger_observer_t *obs_obj);


#endif /* MAIN_BLDC_MATH_H_ */
