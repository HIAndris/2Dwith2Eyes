#include "config.hpp"

#ifndef COLLECTOR_HPP
#define COLLECTOR_HPP

typedef enum {
    DISTANCE1 = DIST1_SHUT_PIN,
    DISTANCE2 = DIST2_SHUT_PIN,
    DISTANCE3 = DIST3_SHUT_PIN
} dist_sensors_t;

class Collector {
  public:
    void init();
    static void main(void* pvParameters);
  private:


};

/*
typedef enum {
    CAMERA1 = GPIO_CAM1_PWDN,
    CAMERA2 = GPIO_CAM2_PWDN,
    DISTANCE1 = DIST1_SHUT_PIN,
    DISTANCE2 = DIST2_SHUT_PIN,
    DISTANCE3 = DIST3_SHUT_PIN
} serial_io_devices_t;

constexpr serial_io_devices_t DEVICES[] = {
    CAMERA1,
    CAMERA2,
    DISTANCE1,
    DISTANCE2,
    DISTANCE3
};

class Collector {
  private:
    void disable_others(const serial_io_devices_t selector);
    void activate_device(const serial_io_devices_t selector);
    esp_err_t init_camera(const serial_io_devices_t selector);
    esp_err_t disable_camera();
    void update_camera_row(const camera_fb_t* pic, uint16_t row_buffer[]);
    void loop(void * parameter);
};
*/

#endif
