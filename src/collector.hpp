#ifndef COLLECTOR_HPP
#define COLLECTOR_HPP

#include "config.hpp"

const pins_t distance_shut_pins = {
    DIST1_SHUT_PIN,
    DIST2_SHUT_PIN,
    DIST3_SHUT_PIN
};

class Collector {
  public:
    static volatile uint16_t distance_data[3];
    static volatile float gyro_data[3];

    void init();
    static void main(void* pvParameters);

  private:
    static VL53L0X vldist;

    static uint16_t read_distance_sensor(gpio_num_t sensor_shut);
    static void read_gyro_sensor(volatile float* data_out);
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
