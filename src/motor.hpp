#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "config.hpp"

typedef std::vector<gpio_num_t> motor_pins_t;  // motor GPIO pins in order
typedef uint32_t                motor_delay_t; // delay between motor steps in microseconds
typedef uint32_t                motor_total_t; // cycles needed to complete a 360 degree turn (a cycle is a rotation of bits on the motor pins)
typedef float                   motor_accel_t; // acceleration rate of the motor (0 < a < 1) (0.1 = rapid acceleration, 0.9 = slow acceleration)


class DriveMotor {
  public:
    void init(
        const motor_pins_t& pins,
        const motor_delay_t min_start_delay,
        const motor_delay_t min_delay,
        const motor_accel_t accel_multiplier,
        const motor_total_t cycles_to_360d
    );
    void go(uint16_t cycles, motor_delay_t delay);

  private:
    motor_pins_t  pins;
    motor_delay_t start;
    motor_delay_t top;
    motor_accel_t accel;
    motor_total_t total;

    void cycle(motor_delay_t delay);
};

class SteerMotor {
  public:
    void init(
        const motor_pins_t& pins
    );
    void go();

  private:
    motor_pins_t pins;

    void cycle(motor_delay_t delay);
};

#endif
