#ifndef STATUS_HPP
#define STATUS_HPP

#include "config.hpp"

class Status {
  public:
    Status();

    void set_color(const led_color_t color);

    void init();

  private:
    void write(uint8_t* data, int length = 3);

    rmt_item32_t bit0;
    rmt_item32_t bit1;
};

#endif
