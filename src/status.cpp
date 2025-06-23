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

    esp_netif_init();
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        Serial.printf("WiFi", "Failed to create default event loop: %s\n", esp_err_to_name(err));
    }

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 2, 2, 2, 2);
    IP4_ADDR(&ip_info.gw, 2, 2, 2, 2);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    esp_netif_dhcps_stop(ap_netif);
    esp_netif_set_ip_info(ap_netif, &ip_info);
    esp_netif_dhcps_start(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_AP);

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.ap.ssid, "2D2EYES_LOG", sizeof(wifi_config.ap.ssid));
    strncpy((char*)wifi_config.ap.password, "idehajdu", sizeof(wifi_config.ap.password));
    wifi_config.ap.ssid_len = 0;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wifi_config.ap.max_connection = 2;

    if (strlen((char *)wifi_config.ap.password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    Serial.printf("SSE_LOG", "SoftAccessPoint initialised: SSID=%s, password=%s, IP=%s\n", wifi_config.ap.ssid, wifi_config.ap.password, ip4addr_ntoa((const ip4_addr_t *)&ip_info.ip));

    start_webserver();
}

void Status::write(uint8_t* data, int length) {
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

static httpd_handle_t server = NULL;
static httpd_req_t *sse_client = NULL;

void Status::start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_start(&server, &config);

    httpd_uri_t index_page = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler
    };

    httpd_uri_t log_page = {
        .uri = "/log",
        .method = HTTP_GET,
        .handler = log_handler
    };

    httpd_uri_t log_stream = {
        .uri = "/logstream",
        .method = HTTP_GET,
        .handler = sse_handler
    };

    httpd_register_uri_handler(server, &index_page);
    httpd_register_uri_handler(server, &log_page);
    httpd_register_uri_handler(server, &log_stream);
}

void Status::sse_log(const char *fmt, ...) {
    if (sse_client) {
        char msgbuf[256];
        va_list args;
        va_start(args, fmt);
        vsnprintf(msgbuf, sizeof(msgbuf), fmt, args);
        va_end(args);

        char outbuf[300];
        snprintf(outbuf, sizeof(outbuf), "data: %s\n\n", msgbuf);

        httpd_resp_send_chunk(sse_client, outbuf, strlen(outbuf));
    }
}

esp_err_t Status::log_handler(httpd_req_t *req) {
    const char* html = R"rawliteral(
    <html>
      <head>
        <style>
          pre#log {
            font-size: 16px;
            white-space: pre-wrap;
          }
        </style>
      </head>
      <body>
        <h1>ESP Log</h1>
        <pre id="log"></pre>
        <script>
          const logBox = document.getElementById("log\n");
          const es = new EventSource("/logstream\n");
          es.onmessage = (e) => {
            if (e.data.startsWith(':')) return;
            logBox.textContent += e.data.replace(": keep-alive", "") + "\n";
          };
          es.onerror = (e) => {
            console.error("SSE error", e);
          };
        </script>
      </body>
    </html>
    )rawliteral";

    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t Status::sse_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/event-stream\n");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache\n");
    httpd_resp_set_hdr(req, "Connection", "keep-alive\n");

    sse_client = req;

    while (true) {
        const char *keep_alive = ": keep-alive\n\n";
        if (httpd_resp_send_chunk(req, keep_alive, strlen(keep_alive)) != ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    sse_client = NULL;
    return ESP_OK;
}

esp_err_t Status::index_handler(httpd_req_t *req) {
    httpd_resp_set_status(req, "302 Found\n");
    httpd_resp_set_hdr(req, "Location", "/log\n");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}
