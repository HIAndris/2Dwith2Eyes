#include "motor.hpp"
//#include "status.hpp"  A controller.hpp és controller.cpp-be is ki van kommentezve!
#include "touch.hpp"
#include "controller.hpp"
#include "collector.hpp"
#include "config.hpp"
#include "config.hpp"

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t distance_mutex;

//Status status;  Ez is!
Touch touch;
Controller controller;
Collector collector;

void setup() {
    distance_mutex = xSemaphoreCreateMutex();

    Serial.begin(115200);
    while (! Serial) {
        delay(1);
    }
    Serial.printf("Hello!\n");

    Wire.begin(SDA_PIN, SCL_PIN);

    // inits
    touch.init(&controller);
    collector.init();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    Serial.printf("Initialisation successful!\n");

    // collector loops on core 1
    collector.main();

    // controller loop on core 0
    controller.main();
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}