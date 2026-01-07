#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ultrasonic.h"

extern "C" void app_main(void) {
    ultrasonic_init();
    printf("ultrasonic initialized\n");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}