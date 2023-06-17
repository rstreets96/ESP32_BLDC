#include "esp_timer.h"

#include "bldc_hal.h"
#include "bldc_math.h"
#include "motor_model.h"
#include "tasks.h"

void app_main(void)
{
	//Creates motor object, initializes motor constants and math objects
	motor_obj_t motor = new_mot_obj();

	start_motor_control(&motor);

}
