#include "controller.hpp"

Controller::Controller() {
    touch_state = OFF;
}


void Controller::touch(touch_type_t touch_type){
    if (touch_state == OFF) {
        touch_state = touch_type;
    }
};

touch_type_t Controller::get_touch_type() {
    return touch_state;
};

void Controller::main() {
    status.init();

    TickType_t action = 0;
    while (true) {
        ESP_LOGI(TAG, "Start loop...");
        switch (touch_state)
        {
        case SHORT:
            ESP_LOGI(TAG, "Set color to GREEN");
            status.set_color(GREEN);
            action = xTaskGetTickCount();
            touch_state = BUSY;
            break;
        
        case LONG:
            ESP_LOGI(TAG, "Set color to RED");
            status.set_color(RED);
            action = xTaskGetTickCount();
            touch_state = BUSY;
            break;
        
        case BUSY:
            break;

        default:
            ESP_LOGI(TAG, "Set color to WHITE");
            status.set_color(WHITE);
        }

        if ((xTaskGetTickCount() - action) > TICKS_S*3) {
            touch_state = OFF;
        }

        ESP_LOGI(TAG, "Wait for 100ms");
        vTaskDelay(TICKS_100MS);
    }
}