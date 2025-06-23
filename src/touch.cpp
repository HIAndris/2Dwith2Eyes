#include "touch.hpp"

Controller* Touch::controller = nullptr;
TickType_t Touch::press_tick = 0;

esp_err_t Touch::init(Controller *controller) {
    Touch::controller = controller;

    static gpio_config_t touch_config = {
        .pin_bit_mask = 1ULL << TOUCH_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };

    gpio_config(&touch_config);
    gpio_install_isr_service(0);
    return gpio_isr_handler_add(GPIO_TOUCH, Touch::handler, NULL);
};

void IRAM_ATTR Touch::handler(void* arg) {
    if (gpio_get_level(GPIO_TOUCH) == 0) {
        uint16_t delta = xTaskGetTickCountFromISR() - press_tick;

        if (delta < TOUCH_LENGTH_MIN || delta > TOUCH_LENGTH_LONG) {
            // invalid touch
        } else if (delta < TOUCH_LENGTH_SHORT) {
            controller->touch(TCH_SHORT);
        } else {
            controller->touch(TCH_LONG);
        }
    } else {
        press_tick = xTaskGetTickCountFromISR();
    }
}