#include "controller.hpp"

volatile int8_t Controller::speed = 0;
int64_t Controller::motor_state = 0;

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
    drive_motor.init(
        drive_pins,
        DRIVE_MIN_START,
        DRIVE_MIN,
        DRIVE_ACCEL,
        200
    );

    speed = 50;
    bool speed_change_direction = true;

    xTaskCreate(motor_task, "StepperTask", 16384, this, 5, nullptr);

    TickType_t action = 0;
    TickType_t last_speed_change = xTaskGetTickCount();
    ESP_LOGI(TAG, "Start loop...");
    while (true) {
        switch (touch_state)
        {
        case SHORT:
            ESP_LOGD(TAG, "Set color to GREEN");
            status.set_color(GREEN);
            action = xTaskGetTickCount();
            touch_state = BUSY;
            break;
        
        case LONG:
            ESP_LOGD(TAG, "Set color to RED");
            status.set_color(RED);
            action = xTaskGetTickCount();
            touch_state = BUSY;
            break;
        
        case BUSY:
            break;

        default:
            ESP_LOGD(TAG, "Set color to WHITE");
            status.set_color(WHITE);
        }

        if ((xTaskGetTickCount() - action) > TICKS_S*3) {
            touch_state = OFF;
        }

        if (speed_change_direction) {
            speed++;
            if (speed == 100) {
                speed_change_direction = false;
            }
        } else {
            speed--;
            if (speed == -100) {
                speed_change_direction = true;
            }
        }
        ESP_LOGI(TAG, "Speed changed to %d", speed);

        ESP_LOGD(TAG, "Wait for 100ms");
        vTaskDelay(TICKS_100MS);
    }
}

void Controller::motor_task(void *parameters) {
    const uint8_t full_step_seq[4][4] = {
        {1, 0, 1, 0},
        {0, 1, 1, 0},
        {0, 1, 0, 1},
        {1, 0, 0, 1}
    };

    while (true) {
        if (speed > 0) {
            motor_state++;
        }
        if (speed < 0) {
            motor_state--;
        }
        if (speed == 0) {
            // Stop, release motor
            for (int pid = 0; pid < 0; pid++) {
                gpio_set_level(drive_pins[pid], 0);
            }
            vTaskDelay(200); // Wait to start
        } else {
            // Write next step
            for (int pid = 0; pid < 4; pid++) {
                gpio_set_level(drive_pins[pid], full_step_seq[motor_state & 0x03][pid]);
            }
            uint32_t delay = abs(1000 / speed) - 5;
            vTaskDelay(delay);
        }
    }
}
