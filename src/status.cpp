#include "status.hpp"

Status::Status() {
    bit0.level0 = 1;
    bit0.duration0 = T0H_NS / CLK_TICK_NS;
    bit0.level1 = 0;
    bit0.duration1 = T0L_NS / CLK_TICK_NS;

    bit1.level0 = 1;
    bit1.duration0 = T1H_NS / CLK_TICK_NS;
    bit1.level1 = 0;
    bit1.duration1 = T1L_NS / CLK_TICK_NS;
}

void Status::init() {
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_TX_CHANNEL;
    config.gpio_num = GPIO_LED;
    config.clk_div = RMT_CLK_DIV;
    config.mem_block_num = 1;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.carrier_en = false;
    config.tx_config.loop_en = false;
    config.tx_config.idle_output_en = true;

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

void Status::write(uint8_t* data, int length) {

    ESP_LOGD("STATUS", "New color: %x:%x:%x", data[0], data[1], data[2]);

    rmt_item32_t items[24 * LED_NUM];
    int idx = 0;

    for (int i = 0; i < length; ++i) {
        uint8_t byte = data[i];
        for (int bit = 7; bit >= 0; --bit) {
            items[idx++] = (byte & (1 << bit)) ? bit1 : bit0;
        }
    }

    rmt_write_items(RMT_TX_CHANNEL, items, idx, true);
    rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
}

void Status::set_color(led_color_t color) {
    uint32_t temp = (uint32_t)color;
    uint8_t grb[3] = {
        static_cast<uint8_t>((temp >> 8) & 0xff),
        static_cast<uint8_t>((temp >> 16) & 0xff),
        static_cast<uint8_t>(temp & 0xff)
    };

    write(grb, 3);
}