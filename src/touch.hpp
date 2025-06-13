#ifndef TOUCH_HPP
#define TOUCH_HPP

#include "config.hpp"
#include "controller.hpp"

class Touch {
  public:
    static esp_err_t init(Controller *controller);
    static void IRAM_ATTR handler(void* arg);

  private:
    static TickType_t press_tick;
    static Controller *controller;

};

#endif