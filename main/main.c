#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "APA106.h"

#define LED_GPIO    GPIO_NUM_18

#ifndef SPI_DIRECT
apa106_handle_t apa = NULL;
#endif

void app_main() {
#ifndef SPI_DIRECT
    ESP_ERROR_CHECK(apa106_init(LED_GPIO, &apa));
#else
    apa106_init(LED_GPIO);
#endif

    while (1) {
        for (uint8_t i = 0; i <= 6; ++i) {
            for (uint16_t c = 1; c < 256; c = (c << 1) | 1) {
#ifndef SPI_DIRECT
                apa106_rgb(apa, (i == 0) || (i == 3) || (i == 4) || (i == 6) ? c : 0, (i == 1) || (i == 3) || (i >= 5) ? c : 0, (i == 2) || (i >= 4) ? c : 0);
#else
                apa106_rgb((i == 0) || (i == 3) || (i == 4) || (i == 6) ? c : 0, (i == 1) || (i == 3) || (i >= 5) ? c : 0, (i == 2) || (i >= 4) ? c : 0);
#endif
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            vTaskDelay(pdMS_TO_TICKS(400));
        }
#ifndef SPI_DIRECT
        apa106_rgb(apa, 0, 0, 0);
#else
        apa106_rgb(0, 0, 0);
#endif
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
