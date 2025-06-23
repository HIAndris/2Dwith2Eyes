#include "collector.hpp"

#define COLLECTOR_TAG "COLLECTOR"

volatile uint16_t Collector::distance_data[3] = {966, 696, 669};
volatile float Collector::gyro_data[3] = {0, 0, 0};

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

std::array<SensorInfo, 3> Collector::dist_sensors = {{
    { &lox1, 0x2a, DIST1_SHUT_PIN, 'L' },
    { &lox2, 0x2b, DIST2_SHUT_PIN, 'R' },
    { &lox3, 0x2c, DIST3_SHUT_PIN, 'F' }
}};

void Collector::init() {
    Wire.begin(10, 9);

    for (auto& dist_sensor : dist_sensors) {
        pinMode(dist_sensor.shut_pin, OUTPUT);
        digitalWrite(dist_sensor.shut_pin, LOW);
    }
    vTaskDelay(TICKS_100MS);

    for (auto& dist_sensor : dist_sensors) {
        digitalWrite(dist_sensor.shut_pin, HIGH);
        vTaskDelay(TICKS_100MS);

        ESP_LOGI(COLLECTOR_TAG, "Adafruit VL53L0X test with address: %d", dist_sensor.i2c_addr);
        if (!dist_sensor.sensor->begin(dist_sensor.i2c_addr)) {
            ESP_LOGE(COLLECTOR_TAG, "Failed to start sensor with address: %d", dist_sensor.i2c_addr);
            while (1);
        }
    }

    for (auto& dist_sensor : dist_sensors) {
        dist_sensor.sensor->startRangeContinuous();
    }

    ESP_LOGI(COLLECTOR_TAG, "All sensors initialized!");
}

void Collector::main() {
    for (uint8_t sensor_index = 0; sensor_index < dist_sensors.size(); sensor_index++) {
        uint8_t* i = new uint8_t(sensor_index);
        xTaskCreatePinnedToCore(
            Collector::distance_task,
            "DistanceSensor",
            1024,
            i,
            2,
            NULL,
            1
        );
    }
    ESP_LOGI(COLLECTOR_TAG, "All sensors started measuring!");
}

void Collector::distance_task(void* pvParameters) {
    uint8_t sensor_index = *(uint8_t*)pvParameters;
    delete (uint8_t*)pvParameters;
    read_distance_sensor(sensor_index);
    vTaskDelete(NULL);
}

void Collector::read_distance_sensor(uint8_t sensor_index) {
    while (true) {
        portENTER_CRITICAL(&mux);
        distance_data[sensor_index] = dist_sensors[sensor_index].sensor->readRange();
        portEXIT_CRITICAL(&mux);
        vTaskDelay(DIST_DELAY_TICKS);
    }
}

void Collector::read_gyro_sensor(volatile float* data_out) {
    data_out[0] = 0.0f;
    data_out[1] = 0.0f;
    data_out[2] = 0.0f;
}


/*
void Collector::disable_others(const serial_io_devices_t selector) {
    for (const serial_io_devices_t device : DEVICES) {
        if (device != selector) {
            ESP_LOGI(TAG, "Disable device %d", device);
            gpio_set_direction(static_cast<gpio_num_t>(selector), GPIO_MODE_OUTPUT);
            gpio_set_pull_mode(static_cast<gpio_num_t>(selector), GPIO_FLOATING);
            gpio_set_level(static_cast<gpio_num_t>(selector), 1);
        }
    }
}

void Collector::activate_device(const serial_io_devices_t selector) {
    disable_others(selector);
    ESP_LOGI(TAG, "Enable device %d", selector);
    gpio_set_direction(static_cast<gpio_num_t>(selector), GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(static_cast<gpio_num_t>(selector), GPIO_FLOATING);
    gpio_set_level(static_cast<gpio_num_t>(selector), 0);
}

esp_err_t Collector::init_camera(const serial_io_devices_t selector) {
    disable_others(selector);
    ESP_LOGI(TAG, "Initialize camera device %d", selector);
    gpio_reset_pin(static_cast<gpio_num_t>(selector));
    if (const esp_err_t err = esp_camera_init(selector == CAMERA1 ? &camera1_config : &camera2_config); err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
    return ESP_OK;
}

esp_err_t Collector::disable_camera() {
    ESP_LOGI(TAG, "Deinitialize camera device");
    if (const esp_err_t err = esp_camera_deinit(); err != ESP_OK) {
        ESP_LOGE(TAG, "Disable Camera Failed");
        return err;
    }
    return ESP_OK;
}

void Collector::update_camera_row(const camera_fb_t* pic, uint16_t row_buffer[]) {
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

void Collector::loop(void * parameter) {
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
        //update_camera_row(pic, camera_1_row);

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
        //update_camera_row(pic, camera_2_row);

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
*/