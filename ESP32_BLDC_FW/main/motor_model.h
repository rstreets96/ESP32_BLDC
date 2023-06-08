/*
 * motor_model.h
 *
 *  Created on: Jun 5, 2023
 *      Author: rstre
 */

#ifndef MAIN_MOTOR_MODEL_H_
#define MAIN_MOTOR_MODEL_H_

#include "bldc_hal.h"
#include "bldc_math.h"
#include "project_config.h"
/*
 * ----------------------------------------------------------------------------------------------------------
 * Constants of the Motor
 * ----------------------------------------------------------------------------------------------------------
 */
#define R_S_OHMS						(0.100f)
#define L_S_H							(0.125f)
#define K_V								(4.5f)		//calculate later with given motor
#define POLE_PAIRS						(10)
#define FLUX_LINKAGE					(0.9f)

#define SPEED_MAX_RPM					(200.0f)
#define SPEED_MIN_RPM					(20.0f)
#define CURRENT_MAX_A					(15.0f)

#define CONTROL_FREQ_HZ					(5000)
#define CONTROL_PERIOD_S				(0.0002f) //1 / CONTROL_FREQ_HZ

#define OBSERVER_GAIN					(0.5f)

/*
 * ----------------------------------------------------------------------------------------------------------
 * Main Motor Object
 * ----------------------------------------------------------------------------------------------------------
 */
typedef struct motor_obj
{
	//Motor Constants
	int pole_pairs;
	float r_s;
	float l_s;
	float flux_linkage;

	//HAL objects to interface with the motor and peripherals
	hal_obj_t hal_obj;
	adc_data_t adc_data;

	//Math objects for FOC
	pid_obj_t speed_pid;
	pid_obj_t iq_pid;
	pid_obj_t id_pid;
	alpha_beta_t i_ab;
	alpha_beta_t v_ab;
	d_q_t i_dq_measured;
	d_q_t i_dq_setpoint;
	d_q_t v_dq;
	float i_stator_setpoint;
	abc_t v_phase_out_v;
	abc_t v_phase_out_pu;  //Normalized to DC voltage
#if defined(LUENBERGER)
	luenberger_observer_t observer;
#endif
	float angle_foc_rad;
	float speed_RPM;


}motor_obj_t;

/*
 * ----------------------------------------------------------------------------------------------------------
 * Motor Functions
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to create and configure a new motor object
motor_obj_t new_mot_obj(void);

//Function to run one control loop
void run_motor_control(motor_obj_t *motor);

#endif /* MAIN_MOTOR_MODEL_H_ */
