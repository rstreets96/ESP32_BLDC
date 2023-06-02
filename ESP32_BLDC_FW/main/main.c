#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "HAL.h"

void app_main(void)
{
    while (true) {
        printf("Hello from app_main!\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
