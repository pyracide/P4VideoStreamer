/**
 * @file app_control.h
 * @brief Application control interface header file
 */

#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize application control module
 * 
 * This function initializes buttons and knob controls, and registers corresponding callbacks
 * 
 * @return
 *      - ESP_OK: Success
 *      - Others: Fail
 */
esp_err_t app_control_init(void);

/**
 * @brief Start the UDP server for haptic commands
 * 
 * Must be called AFTER lwIP is initialized
 */
void app_control_start_udp_server(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_H */