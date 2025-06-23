#include "controller.hpp"

Controller::Controller() {
    touch_state = TCH_READY;
}

void Controller::touch(touch_type_t touch_type){
    if (touch_state == TCH_READY) {
        touch_state = touch_type;
    }
};

touch_type_t Controller::get_touch_type() {
    return touch_state;
};

uint16_t Controller::get_distance(uint8_t sensor_num) {
    uint16_t dist;
    portENTER_CRITICAL(&mux);
    dist = collector.distance_data[sensor_num];
    portEXIT_CRITICAL(&mux);

    return dist;
};

void Controller::main() {
    //status.init();

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
    while (true) {
        switch (touch_state)
        {
        case TCH_SHORT:
            ESP_LOGD(TAG, "Set color to GREEN");
            //status.sse_log("Short press");
            //status.set_color(GREEN);
            action = xTaskGetTickCount();
            touch_state = TCH_BUSY;
            break;
        
        case TCH_LONG:
            ESP_LOGD(TAG, "Set color to RED");
            //status.sse_log("Long press");
            //status.set_color(RED);
            action = xTaskGetTickCount();
            touch_state = TCH_BUSY;
            break;
        
        case TCH_BUSY:
            break;

        default:
            ESP_LOGD(TAG, "Set color to WHITE");
            //status.set_color(WHITE);
        }

        if ((xTaskGetTickCount() - action) > TICKS_S*3) {
            touch_state = TCH_READY;
        }

        
        steer_motor.steer = 20;
        ESP_LOGI(TAG, "distance: 1=%u, 2=%u, 3=%u", get_distance(0), get_distance(1), get_distance(2));

        vTaskDelay(TICKS_100MS);
    }
}
