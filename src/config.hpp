#ifndef CONFIG_H
#define CONFIG_H

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <thread>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_camera.h>
#include <esp_timer.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <driver/gpio.h>
#include <driver/rmt.h>
#include <driver/i2c_master.h>
#include <math.h>

extern "C" {
    #include "VL53L0X.h"
}


typedef std::vector<gpio_num_t> pins_t;  // GPIO pins in order
typedef uint32_t                motor_delay_t; // delay between motor steps in milliseconds

extern portMUX_TYPE mux;

// title
#define TAG "2Dwith2Eyes"

// times in ticks
constexpr TickType_t TICKS_MS    = pdMS_TO_TICKS(   1);
constexpr TickType_t TICKS_5MS   = pdMS_TO_TICKS(   5);
constexpr TickType_t TICKS_10MS  = pdMS_TO_TICKS(  10);
constexpr TickType_t TICKS_100MS = pdMS_TO_TICKS( 100);
constexpr TickType_t TICKS_S     = pdMS_TO_TICKS(1000);

// I2C pins
#define I2C_PORT I2C_NUM_0
#define SDA_PIN GPIO_NUM_10
#define SCL_PIN GPIO_NUM_9

// camera-1 pins
#define CAM1_PIN_PWDN 3
#define GPIO_CAM1_PWDN GPIO_NUM_3

// camera-2 pins
#define CAM2_PIN_PWDN 8
#define GPIO_CAM2_PWDN GPIO_NUM_8

// camera common pins
#define CAM_PIN_RESET (-1)
#define CAM_PIN_XCLK 14
#define COMMON_PIN_SIOD 10
#define COMMON_PIN_SIOC 9
#define CAM_PIN_D0 18
#define CAM_PIN_D1 17
#define CAM_PIN_D2 16
#define CAM_PIN_D3 15
#define CAM_PIN_D4 7
#define CAM_PIN_D5 6
#define CAM_PIN_D6 5
#define CAM_PIN_D7 4
#define CAM_PIN_VSYNC 11
#define CAM_PIN_HREF 12
#define CAM_PIN_PCLK 13

// gyro sensor pins
#define GYRO_CHANNEL 0x68

// distance sensor pins
#define DIST1_SHUT_PIN GPIO_NUM_19
#define DIST2_SHUT_PIN GPIO_NUM_20
#define DIST3_SHUT_PIN GPIO_NUM_21

// drive motor pins
const pins_t drive_pins = {
    GPIO_NUM_1,
    GPIO_NUM_2,
    GPIO_NUM_42,
    GPIO_NUM_41
};
#define DRIVE_MIN_START 20
#define DRIVE_MIN        2

// steering motor pins
const pins_t steer_pins = {
    GPIO_NUM_38,
    GPIO_NUM_40,
    GPIO_NUM_47,
    GPIO_NUM_46
};
#define STEER_MIN 3
#define STEER_CALIBRATE GPIO_NUM_45

// RGB LED pin
#define RGB_LED_PIN 48

// RGB LED configs
#define GPIO_LED GPIO_NUM_48
#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define LED_NUM 1
#define RMT_CLK_DIV 2
#define CLK_TICK_NS 25
#define T0H_NS  350 // 0 bit HIGH time (ns)
#define T0L_NS  900 // 0 bit LOW time  (ns)
#define T1H_NS  900 // 1 bit HIGH time (ns)
#define T1L_NS  350 // 1 bit LOW time  (ns)

// touch sensor configs
#define TOUCH_PIN 39
#define GPIO_TOUCH GPIO_NUM_39
constexpr TickType_t TOUCH_LENGTH_MIN   =  TICKS_100MS ; // minimum ticks for touch to be valid
constexpr TickType_t TOUCH_LENGTH_SHORT =  TICKS_S     ; // maximum ticks for touch to be short
constexpr TickType_t TOUCH_LENGTH_LONG  = (TICKS_S * 3); // maximum ticks for touch to be long


// RGB LED color map
typedef enum {
    WHITE = 0x404040,
    BLACK = 0x000000,
    RED = 0x400000,
    GREEN = 0x004000,
    BLUE = 0x000040,
    YELLOW = 0x404000,
    CYAN = 0x004040,
    MAGENTA = 0x400040
} led_color_t;

// camera-1
static camera_config_t camera1_config = {
    .pin_pwdn = CAM1_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = COMMON_PIN_SIOD,
    .pin_sccb_scl = COMMON_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_RGB565,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_DRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = -1
};

// camera-2
static camera_config_t camera2_config = {
    .pin_pwdn = CAM2_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = COMMON_PIN_SIOD,
    .pin_sccb_scl = COMMON_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_RGB565,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_DRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = -1
};

#endif
