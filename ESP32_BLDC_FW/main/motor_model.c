/*
 * motor_model.c
 *
 *  Created on: Jun 5, 2023
 *      Author: rstre
 */
#include "motor_model.h"
#include "bldc_math.h"

/*
 * ----------------------------------------------------------------------------------------------------------
 * Motor Functions
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to create and configure a new motor object
motor_obj_t new_mot_obj(void)
{
	motor_obj_t motor;
	motor.r_s = R_S_OHMS;
	motor.l_s = L_S_H;
	motor.pole_pairs = POLE_PAIRS;
	motor.flux_linkage = FLUX_LINKAGE;

	motor.speed_pid = new_pid(1.0, 1.0, 1.0, CONTROL_PERIOD_S, SPEED_MIN_RPM, SPEED_MAX_RPM); 		//TODO: Tune these PIDs (based off motor parameters?)
#if defined(FIELD_WEAKENING)
														//TODO: Add Id PID configuration
#else
	motor.iq_pid = new_pid(1.0, 1.0, 1.0, CONTROL_PERIOD_S, 0, SPEED_MAX_RPM);
	motor.id_pid = new_pid(1.0, 1.0, 1.0, CONTROL_PERIOD_S, 0, 0);
#endif
#if defined(LUENBERGER)
	motor.observer = new_luenberger(CONTROL_PERIOD_S, R_S_OHMS, L_S_H, OBSERVER_GAIN);
#else
#endif
	return motor;
}

//Function to run one control loop
void run_motor_control(motor_obj_t *motor)
{
	//Convert A,B,C currents to Alpa-Beta
	run_clarke(motor->adc_data.phaseI_A, &motor->i_ab);

#if defined(LUENBERGER)
	//Run Observer using measured Alpha-Beta current output and voltage input
	run_luenberger(motor->v_ab, motor->i_ab, &motor->observer);
#else
#endif
	//Save angle and speed from chosen observer
	motor->angle_foc_rad = motor->observer.mot_angle;
	motor->speed_RPM = motor->observer.mot_speed * RAD_SEC_TO_RPM;			//Does this speed value need filtering?

	//Run Speed PID Loop
	motor->i_stator_setpoint = run_pid(&motor->speed_pid, motor->speed_RPM);
#if defined(FIELD_WEAKENING)
														//TODO: Add Id PID run and total current calculations
#else
	//With no Field Weakening, the direct axis current should always be zero
	motor->i_dq_setpoint.direct = 0;
	motor->i_dq_setpoint.quadrature = motor->i_stator_setpoint;
#endif
	//Set the setpoints within their PIDs
	motor->id_pid.setpoint = motor->i_dq_setpoint.direct;
	motor->iq_pid.setpoint = motor->i_dq_setpoint.quadrature;

	//Run Park transform on the Alpha-Beta current
	run_park(motor->angle_foc_rad, motor->i_ab, &motor->i_dq_measured);

	//Run Iq and Id PID Loops
	motor->v_dq.direct = run_pid(&motor->id_pid, motor->i_dq_measured.direct);
	motor->v_dq.quadrature = run_pid(&motor->iq_pid, motor->i_dq_measured.quadrature);

	//Convert Vd and Vq to the Alpha-Beta domain
	run_inv_park(motor->angle_foc_rad, motor->v_dq, &motor->v_ab);

	//Convert Alpha-Beta voltages to 3-phase for PWM outputs
	run_inv_clarke(motor->v_ab, &motor->v_phase_out_v);													//TODO:SVM verification. Maybe just subtract Vct?

	//Convert to per unit values
	motor->v_phase_out_pu.a = motor->v_phase_out_v.a / motor->adc_data.dcV_V;							//TODO: Avoid float division? Do this in a slower loop?
	motor->v_phase_out_pu.b = motor->v_phase_out_v.b / motor->adc_data.dcV_V;
	motor->v_phase_out_pu.c = motor->v_phase_out_v.c / motor->adc_data.dcV_V;

	//Write PWM data with SVM
	mcpwm_set_duty(&motor->hal_obj, motor->v_phase_out_pu);

}






