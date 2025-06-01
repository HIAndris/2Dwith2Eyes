#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <sys/param.h>
#include <cstring>
#include "driver/gpio.h"

#include "esp_camera.h"

#include "driver/i2c_master.h"

//#include "vl53l0x/vl53l0x.h"

#define TAG "2Dwith2Eyes"

#define CAM1_PIN_PWDN 3
#define GPIO_CAM1_PWDN GPIO_NUM_3
#define CAM2_PIN_PWDN 8
#define GPIO_CAM2_PWDN GPIO_NUM_8
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

#define GYRO_CHANNEL 0x68

#define DIST1_SHUT_PIN 19
#define DIST2_SHUT_PIN 20
#define DIST3_SHUT_PIN 21

#define MOTOR1_D0_PIN 1
#define MOTOR1_D1_PIN 2
#define MOTOR1_D2_PIN 42
#define MOTOR1_D3_PIN 41

#define MOTOR2_D0_PIN 40
#define MOTOR2_D1_PIN 39
#define MOTOR2_D2_PIN 47
#define MOTOR2_D3_PIN 46

#define RGB_LED 48

#define TOUCH_PIN 38

// Global variables that is written by CPU Core #0 and read by CPU Core #1
uint16_t camera_1_row[320] = {};
uint16_t camera_2_row[320] = {};

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
    //DISTANCE1,
    //DISTANCE2,
    //DISTANCE3
};

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

static void disable_others(const serial_io_devices_t selector) {
    for (const serial_io_devices_t device : DEVICES) {
        if (device != selector) {
            ESP_LOGE(TAG, "Disable device %d", device);
            gpio_set_direction(static_cast<gpio_num_t>(selector), GPIO_MODE_OUTPUT);
            gpio_set_pull_mode(static_cast<gpio_num_t>(selector), GPIO_FLOATING);
            gpio_set_level(static_cast<gpio_num_t>(selector), 1);
        }
    }
}

static void activate_device(const serial_io_devices_t selector) {
    disable_others(selector);
    ESP_LOGE(TAG, "Enable device %d", selector);
    gpio_set_direction(static_cast<gpio_num_t>(selector), GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(static_cast<gpio_num_t>(selector), GPIO_FLOATING);
    gpio_set_level(static_cast<gpio_num_t>(selector), 0);
}

static esp_err_t init_camera(const serial_io_devices_t selector) {
    disable_others(selector);
    ESP_LOGE(TAG, "Initialize camera device %d", selector);
    gpio_reset_pin(static_cast<gpio_num_t>(selector));
    if (const esp_err_t err = esp_camera_init(selector == CAMERA1 ? &camera1_config : &camera2_config); err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
    return ESP_OK;
}

static esp_err_t disable_camera() {
    if (const esp_err_t err = esp_camera_deinit(); err != ESP_OK) {
        ESP_LOGE(TAG, "Disable Camera Failed");
        return err;
    }

    return ESP_OK;
}

void update_camera_row(const camera_fb_t* pic, uint16_t row_buffer[]) {
    for (size_t col = 0; col < 320; col++) {
        row_buffer[col] = 0;
        // Use dominant bit as an average
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint8_t low = 0;
            uint8_t hi = 0;
            for (size_t row = 110; row <= 130; row += 5) {
                if ((pic->buf[(row * 320 + col) * 2] >> bit) & 1) {
                    low++;
                }
                if ((pic->buf[(row * 320 + col) * 2 + 1] >> bit) & 1) {
                    hi++;
                }
            }
            if (low >= 3) { // At least 3 of 5
                row_buffer[col] |= (1 << bit);
            }
            if (hi >= 3) { // At least 3 of 5
                row_buffer[col] |= (1 << (bit + 8));
            }
        }
    }
}

void core1_main(void * parameter) {
    ESP_LOGI(TAG, "Starting Core #%d worker...", xPortGetCoreID());

    while (true) {
        ESP_LOGI(TAG, "Init CAMERA #1 and take picture...");

        gpio_set_direction(GPIO_CAM2_PWDN, GPIO_MODE_OUTPUT);
        //gpio_set_pull_mode(GPIO_CAM2_PWDN, GPIO_FLOATING);
        gpio_set_level(GPIO_CAM2_PWDN, 1);

        if (ESP_OK != init_camera(CAMERA1)) {
            return;
        }
        // Get picture from CAMERA1
        camera_fb_t* pic = esp_camera_fb_get();

        ESP_LOGI(TAG, "Picture taken from CAMERA #1");

        // 320*240*2 bytes; we average pixels horizontally
        update_camera_row(pic, camera_1_row);

        // Release picture to free up memory
        esp_camera_fb_return(pic);
        // Disable CAMERA1 to release IOT channel
        if (ESP_OK != disable_camera()) {
            return;
        }

        ESP_LOGI(TAG, "Init CAMERA #2 and take picture...");

        gpio_set_direction(GPIO_CAM1_PWDN, GPIO_MODE_OUTPUT);
        //gpio_set_pull_mode(GPIO_CAM1_PWDN, GPIO_FLOATING);
        gpio_set_level(GPIO_CAM1_PWDN, 1);

        if (ESP_OK != init_camera(CAMERA2)) {
            return;
        }
        // Get picture from CAMERA2
        pic = esp_camera_fb_get();

        // 320*240*2 bytes; we average pixels horizontally
        update_camera_row(pic, camera_2_row);

        ESP_LOGI(TAG, "Picture taken from CAMERA #2");

        // Release picture to free up memory
        esp_camera_fb_return(pic);
        // Disable CAMERA2 to release IOT channel
        if (ESP_OK != disable_camera()) {
            return;
        }

        //activate_device(DISTANCE1);

        ESP_LOGI(TAG, "Worker Core #%d tick", xPortGetCoreID());

        //vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

extern "C" [[noreturn]] void app_main() {
    esp_log_level_set(TAG, ESP_LOG_INFO);

    TaskHandle_t core0_task;
    xTaskCreatePinnedToCore(
        core1_main,
        "Collect data from periferias",
        10000,
        nullptr,
        0,
        &core0_task,
        1);

    while (true) {
        //ESP_LOGI("Main", "Core: %d", xPortGetCoreID());

        // Write to log
        constexpr size_t BUF_SIZE = 321 * 5;
        const auto line = static_cast<char*>(malloc(BUF_SIZE));
        size_t pos = 0;
        for (const uint16_t pixel : camera_1_row) {
            pos += snprintf(line + pos, BUF_SIZE - pos, "%04X ", pixel);
        }
        ESP_LOGI("OV7670-1", "%s", line);
        pos = 0;
        for (const uint16_t pixel : camera_2_row) {
            pos += snprintf(line + pos, BUF_SIZE - pos, "%04X ", pixel);
        }
        ESP_LOGI("OV7670-2", "%s", line);
        free(line);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
