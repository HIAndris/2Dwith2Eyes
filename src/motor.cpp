#include "motor.hpp"

// DriveMotor:
void DriveMotor::init(const pins_t& pins, const motor_delay_t min_start_delay, const motor_delay_t min_delay) {
    if (pins.size() != 4) {
        ESP_LOGE("MOTOR", "Wrong amount of GPIO pins provided (%zu)! Must be exactly 4.", pins.size());
        std::abort();
    } else if (min_start_delay < min_delay) {
        ESP_LOGE("MOTOR", "Minimum start delay (%lu) is shorter than minimum delay (%lu)!", min_start_delay, min_delay);
        std::abort();
    }

     this->pins = pins;
    start_speed = floor(1000/(min_start_delay-min_delay+10));
            top = min_delay;
          speed = 0;
    motor_state = 0;

    for (gpio_num_t pin : pins) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type    = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
    }
}

void DriveMotor::task() {
    motor_speed_t real_speed = 0;
    motor_delay_t delay = 0;
    while (true) {
        if (speed > 0) {
            if (motor_state != 3) {
                motor_state++;
            } else {
                motor_state = 0;
            }
        }
        if (speed < 0) {
            if (motor_state != 0) {
                motor_state--;
            } else {
                motor_state = 3;
            }
        }

        if (speed == 0) {
            // Stop, release motor
            for (int pid = 0; pid < 0; pid++) {
                gpio_set_level(pins[pid], 0);
            }
            vTaskDelay(200*TICKS_MS); // Wait to start
        } else {
            // Write next step
            for (int pid = 0; pid < 4; pid++) {
                gpio_set_level(pins[pid], step_seq[motor_state][pid]);
            }

            if (abs(speed) > start_speed) {
                if (real_speed < start_speed) {
                    real_speed = start_speed;
                } else if (real_speed < 100) {
                    real_speed++;
                }
                delay = (1000/real_speed)-10+top;
            } else {
                delay = abs(1000/speed)-10+top;
            }
            vTaskDelay(delay*TICKS_MS);
        }
    }
}

// SteerMotor:    
void SteerMotor::init(const pins_t& pins, const motor_delay_t min_delay, const gpio_num_t calibrate_pin) {
    if (pins.size() != 4) {
        ESP_LOGE("MOTOR", "Wrong amount of GPIO pins provided (%zu)! Must be exactly 4.", pins.size());
        std::abort();
    }
     this->pins = pins;
            top = min_delay;
          steer = 0;
    motor_state = 0;
    
    for (gpio_num_t pin : pins) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type    = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
    }

    static gpio_config_t steer_border_config = {
        .pin_bit_mask = 1ULL << calibrate_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&steer_border_config);
    
    steer_border = calibrate(calibrate_pin);
    ESP_LOGI("STEER-MOTOR", "Calibration successful: Steer border: %li", steer_border);
};

uint32_t SteerMotor::calibrate(gpio_num_t port) {
    while (gpio_get_level(port) == 0) {
        if (motor_state != 3) {
            motor_state++;
        } else {
            motor_state = 0;
        }
        for (int pid = 0; pid < 4; pid++) {
            gpio_set_level(pins[pid], step_seq[motor_state][pid]);
        }
        vTaskDelay(top*TICKS_MS);
    }

    uint32_t tick_count = 0;
    while (gpio_get_level(port) == 1) {
        if (motor_state != 0) {
            motor_state--;
        } else {
            motor_state = 3;
        }
        for (int pid = 0; pid < 4; pid++) {
            gpio_set_level(pins[pid], step_seq[motor_state][pid]);
        }
        tick_count++;
        vTaskDelay(top*TICKS_MS);
    }
    while (gpio_get_level(port) == 0) {
        if (motor_state != 0) {
            motor_state--;
        } else {
            motor_state = 3;
        }
        for (int pid = 0; pid < 4; pid++) {
            gpio_set_level(pins[pid], step_seq[motor_state][pid]);
        }
        tick_count++;
        vTaskDelay(top*TICKS_MS);
    }

    tick_count /= 2;
    for (uint32_t i = 0; i < tick_count; i++) {
        if (motor_state != 3) {
            motor_state++;
        } else {
            motor_state = 0;
        }
        for (int pid = 0; pid < 4; pid++) {
            gpio_set_level(pins[pid], step_seq[motor_state][pid]);
        }
        vTaskDelay(top*TICKS_MS);
    }

    for (int pid = 0; pid < 0; pid++) {
        gpio_set_level(pins[pid], 0);
    }
    vTaskDelay(top*TICKS_MS);

    return tick_count;
};

void SteerMotor::task() {
    int64_t real_position = 0;
    while (true) {
        int8_t steer_now = steer;
        if (steer_now > 100) {
            steer_now = 100;
        } else if (steer_now < -100) {
            steer_now = -100;
        }

        int64_t position = (int64_t)(std::abs((double)steer_now / 100.0) * steer_border);
        if (real_position < position) {
            if (motor_state != 3) {
                motor_state++;
            } else {
                motor_state = 0;
            }
            real_position++;

        } else if (real_position > position) {
            if (motor_state != 0) {
                motor_state--;
            } else {
                motor_state = 3;
            }
            real_position--;
        }
        if (real_position == position) {
            // Stop, release motor
            for (int pid = 0; pid < 4; pid++) {
                gpio_set_level(pins[pid], 0);
            }
            vTaskDelay(200*TICKS_MS); // Wait to start
        } else {
            // Write next step
            for (int pid = 0; pid < 4; pid++) {
                gpio_set_level(pins[pid], step_seq[motor_state][pid]);
            }

            vTaskDelay(top*TICKS_MS);
        }
    }
};