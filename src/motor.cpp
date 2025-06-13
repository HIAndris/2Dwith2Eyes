#include "motor.hpp"

// DriveMotor:
void DriveMotor::init(const motor_pins_t& pins, const motor_delay_t min_start_delay, const motor_delay_t min_delay, const motor_accel_t accel_multiplier, const motor_total_t cycles_to_360d) {
    if (pins.size() > 8) {
        ESP_LOGE("MOTOR", "Too many GPIO pins provided (%zu)! Max is 8. Are you sure you have a stepper motor with more than 8 GPIO pins connected?", pins.size());
        std::abort();
    } else if (pins.size() < 2) {
        ESP_LOGE("MOTOR", "Not enough GPIO pins provided (%zu)! Min is 2. Are you sure you have a stepper motor with less than 2 GPIO pins connected?", pins.size());
        std::abort();
    } else if (0 >= accel_multiplier || 1 <= accel_multiplier) {
        ESP_LOGE("MOTOR", "Acceleration multiplier must be between 0 and 1 (%f)!", accel_multiplier);
        std::abort();
    }
    this->pins  = pins;
    this->start = min_start_delay;
    this->top   = min_delay;
    this->accel = accel_multiplier;
    this->total = cycles_to_360d;

    for (gpio_num_t pin : pins) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
    }
}

void DriveMotor::go(uint64_t cycles, motor_delay_t delay) {
    motor_delay_t current;
    if (delay > start) {
        if (delay < top) {
            current = start;
        } else {
            delay = top;
            current = start;
        }
    } else {
        current = delay;
    }

    for (cycles; cycles <= 0; cycles--) {
        if (current < delay) {
            current = static_cast<motor_delay_t>(current * accel);
            if (current > top) {
                current = top;
            }
        }
        cycle(current);
    }
};

void DriveMotor::cycle(motor_delay_t delay) {
    /*
    for cycle with a turning pattern (0001, 0010 ...etc.) {
        set the motor gpio pins as the current state of the pattern
        wait for the next cycle
        set the next bit high to make a smoother pattern
        wait for the next cycle
    }
    */
    size_t cycle = 0;
    for (uint8_t pattern = 1; ((pattern >> (pins.size())) & 1) == 0; pattern <<= 1) {
        for (size_t i = 0; i < pins.size(); ++i) {
            gpio_set_level(pins[i], (pattern >> i) & 1);
        };
        esp_rom_delay_us(delay);

        if (cycle < (pins.size() - 1)) {
            gpio_set_level(pins[++cycle], 1);
        } else {
            gpio_set_level(pins[0], 1);
        }
        esp_rom_delay_us(delay);
    };
};


// SteerMotor:
void SteerMotor::init(const motor_pins_t& pins) {
    if (pins.size() > 8) {
        ESP_LOGE("MOTOR", "Too many GPIO pins provided (%zu). Max is 8. Are you sure you have a stepper motor with more than 8 GPIO pins connected?", pins.size());
        std::abort();
    } else if (pins.size() < 2) {
        ESP_LOGE("MOTOR", "Not enough GPIO pins provided (%zu). Min is 2. Are you sure you have a stepper motor with less than 2 GPIO pins connected?", pins.size());
        std::abort();
    }
    this->pins = pins;

    for (gpio_num_t pin : pins) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
    }
}

void SteerMotor::go() {

};

void SteerMotor::cycle(motor_delay_t delay) {
    /*
    for cycle with a turning pattern (0001, 0010 ...etc.) {
        set the motor gpio pins as the current state of the pattern
        wait for the next cycle
        set the next bit high to make a smoother pattern
        wait for the next cycle
    }
    */
    size_t cycle = 0;
    for (uint8_t pattern = 1; ((pattern >> (pins.size())) & 1) == 0; pattern <<= 1) {
        for (size_t i = 0; i < pins.size(); ++i) {
            gpio_set_level(pins[i], (pattern >> i) & 1);
        };
        esp_rom_delay_us(delay);

        if (cycle < (pins.size() - 1)) {
            gpio_set_level(pins[++cycle], 1);
        } else {
            gpio_set_level(pins[0], 1);
        }
        esp_rom_delay_us(delay);
    };
};