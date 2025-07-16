#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_http_server.h"

// Function prototypes
void web_server_init(void);
esp_err_t index_handler(httpd_req_t *req);
esp_err_t get_status_handler(httpd_req_t *req);
esp_err_t command_handler(httpd_req_t *req);
esp_err_t api_status_handler(httpd_req_t *req);
esp_err_t api_debug_handler(httpd_req_t *req);

// Global web server handle
extern httpd_handle_t web_server;

#endif // WEB_SERVER_H