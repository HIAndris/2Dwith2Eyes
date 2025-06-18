#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "config.hpp"
#include "status.hpp"
#include "motor.hpp"

typedef enum {
    OFF,
    SHORT,
    LONG,
    BUSY
} touch_type_t;

extern Status status;

class Controller {
  public:
    Controller();

    void touch(touch_type_t touch_type);
    touch_type_t get_touch_type();
    void main();
  
  private:
    touch_type_t touch_state;

    DriveMotor drive_motor;
    SteerMotor steer_motor;
};

#endif
