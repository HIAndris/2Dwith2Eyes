#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <sys/param.h>
#include <cstring>
#include "driver/gpio.h"

#include "esp_camera.h"

#include "driver/i2c_master.h"

#define TAG "2Dwith2Eyes"

#define CAM1_PIN_PWDN 3
#define GPIO_CAM1_PWDN GPIO_NUM_3
#define CAM2_PIN_PWDN 8
#define GPIO_CAM2_PWDN GPIO_NUM_8
#define CAM_PIN_RESET (-1)
#define CAM_PIN_XCLK 14
#define CAM_PIN_SIOD 10
#define CAM_PIN_SIOC 9

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

#define DIST1_SHUT 19
#define DIST2_SHUT 20
#define DIST3_SHUT 21
#define DIST_SCL 47
#define DIST_SDA 48

#define GYRO_SCL 1
#define GYRO_SDA 2

#define MOTOR1_D0 39
#define MOTOR1_D1 40
#define MOTOR2_D0 41
#define MOTOR2_D1 42

#define RGB_LED 48

#define START_TOUCH 38
#define MODE_SELECT 47

// Global variables that is written by CPU Core #0 and read by CPU Core #1
uint16_t camera_1_row[320] = {};
uint16_t camera_2_row[320] = {};


/*
#include "conversions/include/img_converters.h"

static constexpr char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static constexpr int mod_table[] = {0, 2, 1};

char* base64_encode(const uint8_t *data, size_t input_length, size_t *output_length) {
    *output_length = 4 * ((input_length + 2) / 3);
    const auto encoded_data = static_cast<char*>(malloc(*output_length + 1));
    if (encoded_data == nullptr) {
        return nullptr;
    }
    for (size_t i = 0, j = 0; i < input_length;) {
        const uint32_t octet_a = i < input_length ? data[i++] : 0;
        const uint32_t octet_b = i < input_length ? data[i++] : 0;
        const uint32_t octet_c = i < input_length ? data[i++] : 0;
        const uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;
        encoded_data[j++] = base64_chars[(triple >> 3 * 6) & 0x3F];
        encoded_data[j++] = base64_chars[(triple >> 2 * 6) & 0x3F];
        encoded_data[j++] = base64_chars[(triple >> 1 * 6) & 0x3F];
        encoded_data[j++] = base64_chars[(triple >> 0 * 6) & 0x3F];
    }
    for (size_t i = 0; i < mod_table[input_length % 3]; i++)
        encoded_data[*output_length - 1 - i] = '=';
    encoded_data[*output_length] = '\0';
    return encoded_data;
}

void convertImageAndWriteToLogInBase64(camera_fb_t* pic) {
    uint8_t* jpg_buffer;
    size_t jpg_size;
    // Convert to JPEG
    if (!frame2jpg(pic, 80, &jpg_buffer, &jpg_size)) {
        ESP_LOGI(TAG, "The convert failed");
    }
    // Baee64 conversion and log writing
    size_t b64_len;
    char* base64_img = base64_encode(jpg_buffer, jpg_size, &b64_len);
    ESP_LOGI("OV7670-1", "%s", base64_img);
    free(base64_img);
    // Free up JPEG memory buffer
    heap_caps_free(jpg_buffer);
}*/

typedef enum {
    CAMERA1,
    CAMERA2
} camera_t;

static camera_config_t camera1_config = {
    .pin_pwdn = CAM1_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
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
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = -1
};

static camera_config_t camera2_config = {
    .pin_pwdn = CAM2_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
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
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = -1
};


static esp_err_t init_camera(const camera_t selector) {
    gpio_reset_pin(selector == CAMERA1 ? GPIO_CAM1_PWDN : GPIO_CAM2_PWDN);
    gpio_set_direction(selector == CAMERA1 ? GPIO_CAM2_PWDN : GPIO_CAM1_PWDN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(selector == CAMERA1 ? GPIO_CAM2_PWDN : GPIO_CAM1_PWDN, GPIO_FLOATING);
    gpio_set_level(selector == CAMERA1 ? GPIO_CAM2_PWDN : GPIO_CAM1_PWDN, 1);
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

void core0_main(void * parameter) {
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Starting");

    while (true) {
        ESP_LOGI(TAG, "Init CAMERA #1 and take picture...");
        if (ESP_OK != init_camera(CAMERA1)) {
            return;
        }
        // Get picture from CAMERA1
        camera_fb_t* pic = esp_camera_fb_get();

        // 320*240*2 bytes; we average pixels horizontally
        update_camera_row(pic, camera_1_row);

        // Release picture to free up memory
        esp_camera_fb_return(pic);
        // Disable CAMERA1 to release IOT channel
        if (ESP_OK != disable_camera()) {
            return;
        }

        ESP_LOGI(TAG, "Init CAMERA #2 and take picture...");
        if (ESP_OK != init_camera(CAMERA2)) {
            return;
        }
        // Get picture from CAMERA2
        pic = esp_camera_fb_get();

        // 320*240*2 bytes; we average pixels horizontally
        update_camera_row(pic, camera_2_row);

        // Release picture to free up memory
        esp_camera_fb_return(pic);
        // Disable CAMERA2 to release IOT channel
        if (ESP_OK != disable_camera()) {
            return;
        }

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

        ESP_LOGI("Worker", "Core: %d", xPortGetCoreID());

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

extern "C" [[noreturn]] void app_main() {
    TaskHandle_t core0_task;
    xTaskCreatePinnedToCore(
        core0_main,
        "Collect data from periferias",
        10000,
        nullptr,
        0,
        &core0_task,
        1);

    while (true) {
        ESP_LOGI("Main", "Core: %d", xPortGetCoreID());
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
