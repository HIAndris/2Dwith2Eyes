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
    drive_motor.init(
        drive_pins,
        DRIVE_MIN_START,
        DRIVE_MIN
    );
    steer_motor.init(
        steer_pins,
        STEER_MIN,
        STEER_CALIBRATE
    );
    

    xTaskCreate(DriveMotor::task_entry, "DriveTask", 16384, &drive_motor, 5, nullptr);
    xTaskCreate(SteerMotor::task_entry, "SteerTask", 16384, &steer_motor, 5, nullptr);

    TickType_t action = 0;
    TickType_t last_speed_change = xTaskGetTickCount();
    status.sse_log("Start loop");
    while (true) {
        switch (touch_state)
        {
        case SHORT:
            ESP_LOGD(TAG, "Set color to GREEN");
            status.sse_log("Short press");
            status.set_color(GREEN);
            action = xTaskGetTickCount();
            touch_state = BUSY;
            break;
        
        case LONG:
            ESP_LOGD(TAG, "Set color to RED");
            status.sse_log("Long press");
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

        
        steer_motor.steer = 68;


        vTaskDelay(TICKS_100MS);
    }
}
