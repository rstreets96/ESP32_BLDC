/*
 * HAL.c
 *
 *  Created on: May 31, 2023
 *      Author: rstre
 */
#include "driver/mcpwm_prelude.h"

#include "HAL.h"
#include "project_config.h"


//Tag used for ESP serial console messages
static const char TAG[] = "HAL";

/*
 * ----------------------------------------------------------------------------------------------------------
 * Set Up MCPWMs
 * ----------------------------------------------------------------------------------------------------------
 */
void configure_mcpwms(mcpwm_timer_handle_t timer, mcpwm_oper_handle_t operators[], mcpwm_gen_handle_t generators[][], mcpwm_comparator_handle_t comparators[][], mcpwm_fault_handle_t fault)
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
		.period_ticks = MCPWM_RESOLUTION_HZ / MCPWM_FREQUENCY_HZ
	};

	ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

	//Operator initialization
	//----------------------------------------------------------------------------------------------------------

	//Define the operator configuration. You can decide when gen_actions and dead_times are updated.
	mcpwm_operator_config_t operator_config = {
		.group_id = 0,
	};

	//Create the 3 operators with the configuration, and connect them all to the one timer
	for (int i = 0; i < 3; i++)
	{
		ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
		ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));
	}

	//Comparator initialization
	//----------------------------------------------------------------------------------------------------------
	mcpwm_comparator_config_t compare_config = {
		.flags.update_cmp_on_tez = true,
	};

	//Configure two comparators for each pwm. These will be used for start and stop times within the pwm period.
	for (int i = 0; i < 3; i++)
	{
		ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &compare_config, &comparators[i][CMP_INDEX_A]));
		ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i][CMP_INDEX_A], 0));

		ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &compare_config, &comparators[i][CMP_INDEX_B]));
		ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i][CMP_INDEX_B], 0));
	}

	//Fault input initialization
	//----------------------------------------------------------------------------------------------------------
	mcpwm_gpio_fault_config_t gpio_fault_config = {
		.gpio_num = GPIO_NFAULT,
		.group_id = 0,
		.flags.active_level = 0, // low level means fault
		.flags.pull_up = true,   // internally pull up
	};
	ESP_ERROR_CHECK(mcpwm_new_gpio_fault(&gpio_fault_config, &fault));

	//Brake Mode configuration
	//----------------------------------------------------------------------------------------------------------
	//Configure the fault to One-Shot mode when triggered by the DRV8329
	mcpwm_brake_config_t brake_config = {
		.brake_mode = MCPWM_OPER_BRAKE_MODE_OST,
		.fault = fault,
	};

	for (int i = 0; i < 3; i++) {
	        ESP_ERROR_CHECK(mcpwm_operator_set_brake_on_fault(operators[i], &brake_config));
	    }

	//Generator Configuration
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
		ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &gen_config, &generators[i][GEN_INDEX_HI]));

		gen_config.gen_gpio_num = gen_gpios[i][GEN_INDEX_LO];
		ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &gen_config, &generators[i][GEN_INDEX_LO]));
	}

	//Set high and low pwms to go up on cmpA event and go down on cmpB event. They will be the same waveform from this
    // we will use the dead time module to add edge delay, also make gen_high and gen_low complementary
    for (int i = 0; i < 3; i++)
    {
    	ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generators[i][GEN_INDEX_HI],
						MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i][CMP_INDEX_A], MCPWM_GEN_ACTION_UP)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generators[i][GEN_INDEX_HI],
                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_LOW, comparators[i][CMP_INDEX_B], MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(generators[i][GEN_INDEX_HI],
                        MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(generators[i][GEN_INDEX_HI],											//Why is this doubled in example?
                        MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));    //Can I set brake action for both timer directions?

    	ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generators[i][GEN_INDEX_LO],
						MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i][CMP_INDEX_A], MCPWM_GEN_ACTION_UP)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generators[i][GEN_INDEX_LO],
                        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_LOW, comparators[i][CMP_INDEX_B], MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(generators[i][GEN_INDEX_LO],
                        MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(generators[i][GEN_INDEX_LO],
                        MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GEN_ACTION_LOW)));
    }

    //Use the dead time submodule to configure the generators into active-high complementary mode
    //Leading edge is delayed on high side to allow low side to full turn off
    mcpwm_dead_time_config_t dt_config = {
    		.posedge_delay_ticks = 50,
    };
    for (int i = 0; i < 3; i++)
    {
            ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators[i][GEN_INDEX_HI], generators[i][GEN_INDEX_HI], &dt_config));
    }

    //Trailing edge is delayed on low side to allow high side to fully turn off
    //Output is inverted so that low side is on when high side is off
    dt_config = (mcpwm_dead_time_config_t) {
		.negedge_delay_ticks = 50,
		.flags.invert_output = true,
	};
    for (int i = 0; i < 3; i++)
    {
            ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators[i][GEN_INDEX_LO], generators[i][GEN_INDEX_LO], &dt_config));
    }

    //Turn off all generators
    for (int i = 0; i < 3; i++)
    {
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generators[i][GEN_INDEX_HI], 0, true));
        // because gen_low is inverted by dead time module, so we need to set force level to 1
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generators[i][GEN_INDEX_LO], 1, true));
    }
}
