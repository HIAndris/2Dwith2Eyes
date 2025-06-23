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
    if (xSemaphoreTake(distance_mutex, portMAX_DELAY)) {
        dist = collector.distances[sensor_num];
        xSemaphoreGive(distance_mutex);
    }
    Serial.printf("Distance #%u: %u\n", sensor_num, dist);

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
        switch (touch_state) {
        case TCH_SHORT:
            Serial.printf("Set color to GREEN\n");
            //status.sse_log("Short press\n");
            //status.set_color(GREEN);
            action = xTaskGetTickCount();
            touch_state = TCH_BUSY;
            break;
        
        case TCH_LONG:
            Serial.printf("Set color to RED\n");
            //status.sse_log("Long press\n");
            //status.set_color(RED);
            action = xTaskGetTickCount();
            touch_state = TCH_BUSY;
            break;
        
        case TCH_BUSY:
            break;

        default:
            //status.set_color(WHITE);
            break;
        }

        if ((xTaskGetTickCount() - action) > TICKS_S*3) {
            touch_state = TCH_READY;
        }
        
        steer_motor.steer = 0;
        drive_motor.speed = 60;
        while (get_distance(2) > 250 || get_distance(1) > 250) {
            vTaskDelay(TICKS_50MS);
        }
        steer_motor.steer = 100;
        drive_motor.speed = 30;
        vTaskDelay(TICKS_S);
    }
}
