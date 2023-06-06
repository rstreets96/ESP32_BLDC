/*
 * HAL.c
 *
 *  Created on: May 31, 2023
 *      Author: rstre
 *
 *  This file is used to configure all of the hardware as appropriate at the start of operation
 */
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "bldc_hal.h"
#include "project_config.h"


//Tag used for ESP serial console messages
static const char TAG[] = "HAL";


/*
 * ----------------------------------------------------------------------------------------------------------
 * MCPWM Related Functions
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to initialize and configure the MCPWMs
void configure_mcpwms(hal_obj_t *hal_obj)
{
	ESP_LOGI(TAG, "Configuring the MCPWMs");

	//Timer initialization
	//----------------------------------------------------------------------------------------------------------
	//Set timer configuration to desired resolution, period ticks, and count up-down mode for symmetry
	mcpwm_timer_config_t timer_config = {
		.group_id = 0,
		.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
		.resolution_hz = MCPWM_RESOLUTION_HZ,
		.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
		.period_ticks = MCPWM_PERIOD_TICKS
	};

	ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &hal_obj->pwm_timer));

	//Operator initialization
	//----------------------------------------------------------------------------------------------------------
	//Define the operator configuration. You can decide when gen_actions and dead_times are updated.
	mcpwm_operator_config_t operator_config = {
		.group_id = 0,
	};

	//Create the 3 operators with the configuration, and connect them all to the one timer
	for (int i = 0; i < 3; i++)
	{
		ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &hal_obj->pwm_operators[i]));
		ESP_ERROR_CHECK(mcpwm_operator_connect_timer(hal_obj->pwm_operators[i], hal_obj->pwm_timer));
	}

	//Comparator initialization
	//----------------------------------------------------------------------------------------------------------
	mcpwm_comparator_config_t compare_config = {
		.flags.update_cmp_on_tez = true,
	};

	//Configure two comparators for each pwm. These will be used for start and stop times within the pwm period.
	for (int i = 0; i < 3; i++)
	{
		ESP_ERROR_CHECK(mcpwm_new_comparator(hal_obj->pwm_operators[i], &compare_config, &hal_obj->pwm_comparators[i][CMP_INDEX_A]));
		ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(hal_obj->pwm_comparators[i][CMP_INDEX_A], 0));

		ESP_ERROR_CHECK(mcpwm_new_comparator(hal_obj->pwm_operators[i], &compare_config, &hal_obj->pwm_comparators[i][CMP_INDEX_B]));
		ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(hal_obj->pwm_comparators[i][CMP_INDEX_B], 0));
	}

	//Fault input initialization
	//----------------------------------------------------------------------------------------------------------
	mcpwm_gpio_fault_config_t gpio_fault_config = {
		.gpio_num = GPIO_NFAULT,
		.group_id = 0,
		.flags.active_level = 0, // low level means fault
		.flags.pull_up = true,   // internally pull up
	};
	ESP_ERROR_CHECK(mcpwm_new_gpio_fault(&gpio_fault_config, &hal_obj->pwm_fault));

	//Brake Mode configuration
	//----------------------------------------------------------------------------------------------------------
	//Configure the fault to One-Shot mode when triggered by the DRV8329
	mcpwm_brake_config_t brake_config = {
		.brake_mode = MCPWM_OPER_BRAKE_MODE_OST,
		.fault = hal_obj->pwm_fault,
	};

	for (int i = 0; i < 3; i++) {
	        ESP_ERROR_CHECK(mcpwm_operator_set_brake_on_fault(hal_obj->pwm_operators[i], &brake_config));
	    }

	//Generator Initialization and Configuration
	//----------------------------------------------------------------------------------------------------------
	mcpwm_generator_config_t gen_config = {};
	const int gen_gpios[3][2] = {
		{GPIO_A_HI, GPIO_A_LO},
		{GPIO_B_HI, GPIO_B_LO},
		{GPIO_C_HI, GPIO_C_LO},
	};

	//Create 2 generators for each operator
	for (int i = 0; i < 3; i++)
	{
		gen_config.gen_gpio_num = gen_gpios[i][GEN_INDEX_HI];
		ESP_ERROR_CHECK(mcpwm_new_generator(hal_obj->pwm_operators[i], &gen_config, &hal_obj->pwm_generators[i][GEN_INDEX_HI]));

		gen_config.gen_gpio_num = gen_gpios[i][GEN_INDEX_LO];
		ESP_ERROR_CHECK(mcpwm_new_generator(hal_obj->pwm_operators[i], &gen_config, &hal_obj->pwm_generators[i][GEN_INDEX_LO]));
	}

	//Set high and low pwms to go up on cmpA event and go down on cmpB event. They will be the same waveform from this
    // we will use the dead time module to add edge delay, also make gen_high and gen_low complementary
    for (int i = 0; i < 3; i++)
    {
    	ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(hal_obj->pwm_generators[i][GEN_INDEX_HI],
						MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, hal_obj->pwm_comparators[i][CMP_INDEX_A], MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(hal_obj->pwm_generators[i][GEN_INDEX_HI],
                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, hal_obj->pwm_comparators[i][CMP_INDEX_B], MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_brake_event(hal_obj->pwm_generators[i][GEN_INDEX_HI],
                        MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_brake_event(hal_obj->pwm_generators[i][GEN_INDEX_HI],											//Why is this doubled in example?
                        MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));    //Can I set brake action for both timer directions?

    	ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(hal_obj->pwm_generators[i][GEN_INDEX_LO],
						MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, hal_obj->pwm_comparators[i][CMP_INDEX_A], MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(hal_obj->pwm_generators[i][GEN_INDEX_LO],
                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, hal_obj->pwm_comparators[i][CMP_INDEX_B], MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_brake_event(hal_obj->pwm_generators[i][GEN_INDEX_LO],
                        MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_brake_event(hal_obj->pwm_generators[i][GEN_INDEX_LO],
                        MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));
    }

    //Deadtime Initialization and Configuration
	//----------------------------------------------------------------------------------------------------------
    //Use the dead time submodule to configure the generators into active-high complementary mode
    //Leading edge is delayed on high side to allow low side to full turn off
    mcpwm_dead_time_config_t dt_config = {
    		.posedge_delay_ticks = 50,
    };
    for (int i = 0; i < 3; i++)
    {
            ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(hal_obj->pwm_generators[i][GEN_INDEX_HI], hal_obj->pwm_generators[i][GEN_INDEX_HI], &dt_config));
    }

    //Trailing edge is delayed on low side to allow high side to fully turn off
    //Output is inverted so that low side is on when high side is off
    dt_config = (mcpwm_dead_time_config_t) {
		.negedge_delay_ticks = 50,
		.flags.invert_output = true,
	};
    for (int i = 0; i < 3; i++)
    {
            ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(hal_obj->pwm_generators[i][GEN_INDEX_LO], hal_obj->pwm_generators[i][GEN_INDEX_LO], &dt_config));
    }

    //Turn off all generators
    //----------------------------------------------------------------------------------------------------------
    for (int i = 0; i < 3; i++)
    {
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(hal_obj->pwm_generators[i][GEN_INDEX_HI], 0, true));
        // because gen_low is inverted by dead time module, so we need to set force level to 1
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(hal_obj->pwm_generators[i][GEN_INDEX_LO], 1, true));
    }
}

//Function to set the duty cycle of each of the three MCPWMs (the low side signals are always the inverse + dead time)
void mcpwm_set_duty(hal_obj_t *hal_obj, float dutyA, float dutyB, float dutyC)
{
	//Change range from 0 - 1 to 0 - period_ticks
	int cmpVal_A, cmpVal_B, cmpVal_C;
	cmpVal_A = (1 - dutyA) * MCPWM_PERIOD_TICKS;
	cmpVal_B = (1 - dutyB) * MCPWM_PERIOD_TICKS;
	cmpVal_C = (1 - dutyC) * MCPWM_PERIOD_TICKS;


	//CMP A and B are equal when pwms are symmetrical
	ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(hal_obj->pwm_comparators[0][CMP_INDEX_A], cmpVal_A));
	ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(hal_obj->pwm_comparators[0][CMP_INDEX_B], cmpVal_A));
	ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(hal_obj->pwm_comparators[1][CMP_INDEX_A], cmpVal_B));
	ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(hal_obj->pwm_comparators[1][CMP_INDEX_B], cmpVal_B));
	ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(hal_obj->pwm_comparators[2][CMP_INDEX_A], cmpVal_C));
	ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(hal_obj->pwm_comparators[2][CMP_INDEX_B], cmpVal_C));

	//TODO: Add ability to phase shift pwms when there's not enough time for current sampling (single-shunt)

}

/*
 * ----------------------------------------------------------------------------------------------------------
 * ADC Related Functions
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to initialize and configure the ADCs
void configure_adcs(hal_obj_t *hal_obj)
{
	//Initialize ADC1 Object
	//----------------------------------------------------------------------------------------------------------
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &hal_obj->adc1_handle));

    //Configure bitwidth and attenuation on each channel
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(hal_obj->adc1_handle, ADC_A_V, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(hal_obj->adc1_handle, ADC_B_V, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(hal_obj->adc1_handle, ADC_C_V, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(hal_obj->adc1_handle, ADC_DC_V, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(hal_obj->adc1_handle, ADC_DC_I, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(hal_obj->adc1_handle, ADC_CT_V, &config));

    //Initialize and configure the calibration of the ADC
    //----------------------------------------------------------------------------------------------------------
    adc_cali_curve_fitting_config_t cali_config = {
		.unit_id = ADC_UNIT_1,
		.atten = ADC_ATTEN,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &hal_obj->adc_chnl_cali));
}

//Function to read the ADC channels and save the results to the adc_data struct
void read_adcs(hal_obj_t *hal_obj, adc_data_t *adc_data)
{
	int adc_raw[6];
	ESP_ERROR_CHECK(adc_oneshot_read(hal_obj->adc1_handle, ADC_A_V, &adc_raw[0]));
	ESP_ERROR_CHECK(adc_oneshot_read(hal_obj->adc1_handle, ADC_B_V, &adc_raw[1]));
	ESP_ERROR_CHECK(adc_oneshot_read(hal_obj->adc1_handle, ADC_C_V, &adc_raw[2]));
	ESP_ERROR_CHECK(adc_oneshot_read(hal_obj->adc1_handle, ADC_DC_V, &adc_raw[3]));
	ESP_ERROR_CHECK(adc_oneshot_read(hal_obj->adc1_handle, ADC_DC_I, &adc_raw[4]));
	ESP_ERROR_CHECK(adc_oneshot_read(hal_obj->adc1_handle, ADC_CT_V, &adc_raw[5]));

	adc_data->phaseV_V[0] = adc_raw[0] * 3.3 / 4096;		//TODO: Verify Attenuation workings
	adc_data->phaseV_V[1] = adc_raw[1] * 3.3 / 4096;
	adc_data->phaseV_V[2] = adc_raw[2] * 3.3 / 4096;
	adc_data->dcV_V = adc_raw[3] * 3.3 / 4096;				//TODO: Change these scalers based on hardware
	adc_data->phaseI_A[0] = adc_raw[4] * 3.3 / 4096;		//Current index will change with pwm timing
	adc_data->ctV_V = adc_raw[5] * 3.3 / 4096;				//Find when to measure actual DC current
}

/*
 * ----------------------------------------------------------------------------------------------------------
 * Other GPIO Functions
 * ----------------------------------------------------------------------------------------------------------
 */
//Function to handle pwm interrupts
static void IRAM_ATTR pwm_isr_handler(void* arg)
{
	vTaskDelay(1 / portTICK_PERIOD_MS);					//TODO: Define pwm_isr_handler
														//Variables used in the isr need to be defined inDRAM?
}

//Function to Configure the remaining GPIOs
void configure_gpios()
{
	//Configure nSleep and DRVLED to output, pullup enabled, no interrupts.
	gpio_config_t io_conf = {
			.intr_type = GPIO_INTR_DISABLE,
			.mode = GPIO_MODE_OUTPUT,
			.pin_bit_mask = (GPIO_NSLEEP | GPIO_DRVLED),
			.pull_down_en = 0,
			.pull_up_en = 1
	};
	gpio_config(&io_conf);

	//Configure DRVOFF to output, pull-down enabled, no interrupts.
	io_conf.pin_bit_mask = GPIO_DRVOFF;
	io_conf.pull_down_en = 1;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	//Configure PWM sense pins to input, no pulls, interrupt enabled.
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = (GPIO_PWM_SENSE_A | GPIO_PWM_SENSE_B | GPIO_PWM_SENSE_C);
	io_conf.pull_down_en = 0;
	gpio_config(&io_conf);

	//Set up the ISR and handler
	gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
	gpio_isr_handler_add(GPIO_PWM_SENSE_A, pwm_isr_handler, (void*) GPIO_PWM_SENSE_A);
	gpio_isr_handler_add(GPIO_PWM_SENSE_B, pwm_isr_handler, (void*) GPIO_PWM_SENSE_B);
	gpio_isr_handler_add(GPIO_PWM_SENSE_C, pwm_isr_handler, (void*) GPIO_PWM_SENSE_C);
}






