#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "bldc_hal.h"
#include "bldc_math.h"


void app_main(void)
{
	hal_obj_t hal_obj;

	configure_mcpwms(&hal_obj);
	configure_adcs(&hal_obj);
	configure_gpios();
	pid_obj_t speed_pid = new_pid(5, 5, 5, 0.01, 0, 300);
	speed_pid.setpoint = 100;
	run_pid(&speed_pid, 70);

    while (true) {
        printf("Hello from app_main!\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
