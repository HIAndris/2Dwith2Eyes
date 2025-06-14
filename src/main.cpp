#include "motor.hpp"
#include "status.hpp"
#include "touch.hpp"
#include "controller.hpp"
#include "config.hpp"
#include "config.hpp"

Touch touch;
Controller controller;

extern "C" void app_main() {
    esp_log_level_set(TAG, ESP_LOG_INFO);

    touch.init(&controller);

    // controller loop
    controller.main();

    return;
}


    /*
    TaskHandle_t core1_task;
    xTaskCreatePinnedToCore(
        collector,
        "Collect data from periferias",
        10000,
        nullptr,
        0,
        &core1_task,
        1);
    */

        //ESP_LOGI("Main", "Core: %d", xPortGetCoreID());
        /*
        // Write to log
        constexpr size_t BUF_SIZE = 321 * 5;
        const auto line = static_cast<char*>(malloc(BUF_SIZE));
        size_t pos = 0;
        for (const uint16_t pixel : camera_1_row) {
            pos += snprintf(line + pos, BUF_SIZE - pos, "%04X ", pixel);
        }
        ESP_LOGI("OV7670-1", "%s", line);
        pos = 0;
        for (const uint16_t pixel : camera_2_row) {
            pos += snprintf(line + pos, BUF_SIZE - pos, "%04X ", pixel);
        }
        ESP_LOGI("OV7670-2", "%s", line);
        free(line);
        */