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
void configure_mcpwms(mcpwm_timer_handle_t timer, mcpwm_oper_handle_t operators[], mcpwm_gen_handle_t generators[], mcpwm_comparator_handle_t comparators[], mcpwm_fault_handle_t fault)
{
	ESP_LOGI(TAG, "Configuring the MCPWMs");

	//Timer initialization
	//----------------------------------------------------------------------------------------------------------
#if defined(SYMMETRIC_PWMS)
	//Set timer configuration to desired resolution, period ticks, and count up-down mode for symmetry
	mcpwm_timer_config_t timer_config = {
			.group_id = 0,
			.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
			.resolution_hz = MCPWM_RESOLUTION_HZ,
			.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
			.period_ticks = MCPWM_RESOLUTION_HZ / MCPWM_FREQUENCY_HZ
	};
#else
	//Set timer configuration to desired resolution, period ticks, and count up mode for asymmetry
	mcpwm_timer_config_t timer_config = {
			.group_id = 0,
			.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
			.resolution_hz = MCPWM_RESOLUTION_HZ,
			.count_mode = MCPWM_TIMER_COUNT_MODE_UP,
			.period_ticks = MCPWM_RESOLUTION_HZ / MCPWM_FREQUENCY_HZ
	};
#endif
	ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

	//Operator initialization
	//----------------------------------------------------------------------------------------------------------

	//Define the operator configuration. You can decide when gen_actions and dead_times are updated.
	mcpwm_operator_config_t operator_config = {
			.group_id = 0,
	};

	//Create the 3 operators with the configuration
	for (int i = 0; i < 3; i++)
	{
		ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
	}

	//Comparator initialization
	//----------------------------------------------------------------------------------------------------------


	//Brake Mode configuration
	//----------------------------------------------------------------------------------------------------------


	//Generator initialization
	//----------------------------------------------------------------------------------------------------------
}

static void gen_action_config(mcpwm_gen_handle_t gena, mcpwm_gen_handle_t genb, mcpwm_cmpr_handle_t cmpa, mcpwm_cmpr_handle_t cmpb)
{
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gena,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gena,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmpa, MCPWM_GEN_ACTION_LOW)));
}

static void dead_time_config(mcpwm_gen_handle_t gena, mcpwm_gen_handle_t genb)
{
    mcpwm_dead_time_config_t dead_time_config = {
        .posedge_delay_ticks = 50,
        .negedge_delay_ticks = 0
    };
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gena, gena, &dead_time_config));
    dead_time_config.posedge_delay_ticks = 0;
    dead_time_config.negedge_delay_ticks = 100;
    dead_time_config.flags.invert_output = true;
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gena, genb, &dead_time_config));
}
