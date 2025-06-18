#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "config.hpp"

typedef std::vector<gpio_num_t> motor_pins_t;  // motor GPIO pins in order
typedef uint32_t                motor_delay_t; // delay between motor steps in milliseconds
typedef int8_t                  motor_speed_t; // motor speed between -100 and 100, -100 = backward in full speed, 0 = stop, 100 = forward in full speed


class DriveMotor {
  public:
    volatile int8_t speed = 0;
    
    void init(
        const motor_pins_t& pins,
        const motor_delay_t min_start_delay,
        const motor_delay_t min_delay
    );
    void task();

    static void task_entry(void* pvParameters) {
        DriveMotor* self = static_cast<DriveMotor*>(pvParameters);
        self->task();
    }

  private:
    const uint8_t step_seq[4][4] = {
        {1, 0, 1, 0},
        {0, 1, 1, 0},
        {0, 1, 0, 1},
        {1, 0, 0, 1}
    };
    uint8_t       motor_state;
    motor_pins_t  pins;
    motor_delay_t start_speed;
    motor_delay_t top;
};



class SteerMotor {
  public:
    volatile int8_t steer = 0;
    
    void init(
        const motor_pins_t& pins,
        const motor_delay_t min_delay,
        const gpio_num_t    calibrate_pin
    );
    void task();

    static void task_entry(void* pvParameters) {
        SteerMotor* self = static_cast<SteerMotor*>(pvParameters);
        self->task();
    }

  private:
    uint32_t calibrate(gpio_num_t port);

    const uint8_t step_seq[4][4] = {
        {1, 1, 0, 0},
        {0, 1, 1, 0},
        {0, 0, 1, 1},
        {1, 0, 0, 1}
    };
    uint8_t       motor_state;
    uint32_t      steer_border;
    motor_pins_t  pins;
    motor_delay_t top;
};

#endif
