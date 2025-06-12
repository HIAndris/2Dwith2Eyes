#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "config.hpp"
#include "status.hpp"

typedef enum {
    OFF,
    SHORT,
    LONG,
    BUSY
} touch_type_t;

extern int global_counter;

class Controller {
  private:
    Status status;
    touch_type_t touch_state;

  public:
    Controller();

    void touch(touch_type_t touch_type);
    touch_type_t get_touch_type();
    void main();
};

#endif
