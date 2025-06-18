#ifndef STATUS_HPP
#define STATUS_HPP

#include <esp_netif.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include <lwip/ip4_addr.h>

#include "config.hpp"

class Status {
  public:
    Status();

    void init();
    void set_color(const led_color_t color);
    static void sse_log(const char *fmt, ...);
    static esp_err_t sse_handler(httpd_req_t *req);
    static esp_err_t index_handler(httpd_req_t *req);
    static esp_err_t log_handler(httpd_req_t *req);

  private:
    void write(uint8_t* data, int length = 3);
    void start_webserver();

    rmt_item32_t bit0;
    rmt_item32_t bit1;
};

#endif
