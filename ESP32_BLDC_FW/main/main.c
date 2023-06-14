#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "bldc_hal.h"
#include "bldc_math.h"
#include "motor_model.h"

void app_main(void)
{
	//Creates motor object, initializes motor constants and math objects
	motor_obj_t motor = new_mot_obj();

	//Configure the IOs in the HAL
	configure_mcpwms(&motor.hal_obj);
	configure_adcs(&motor.hal_obj);
	configure_gpios(&motor.hal_obj);
	read_adcs(&motor.hal_obj);

	run_motor_control(&motor);


    while (true) {
        printf("Hello from app_main!\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
