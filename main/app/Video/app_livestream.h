/**
 * @file app_livestream.h
 * @brief Wi-Fi H.264 WebSocket livestream module
 *
 * Provides Wi-Fi connectivity via ESP32-C6 coprocessor (esp_hosted/esp_wifi_remote),
 * hardware H.264 encoding using the ESP32-P4's built-in encoder, and a WebSocket
 * server for streaming compressed video to network clients.
 */

#pragma once

#include <esp_err.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Livestream connection state
 */
typedef enum {
    LIVESTREAM_STATE_IDLE = 0,       // Not started
    LIVESTREAM_STATE_WIFI_CONNECTING, // Wi-Fi connecting
    LIVESTREAM_STATE_WIFI_CONNECTED,  // Wi-Fi connected, server starting
    LIVESTREAM_STATE_READY,           // WebSocket server running, waiting for clients
    LIVESTREAM_STATE_STREAMING,       // Actively streaming to client(s)
    LIVESTREAM_STATE_ERROR,           // Error state
} livestream_state_t;

/**
 * @brief Initialize the livestream module
 *
 * Sets up Wi-Fi STA connection (via esp_hosted C6 coprocessor),
 * initializes the H.264 hardware encoder, and prepares the WebSocket server.
 * Wi-Fi connection starts immediately.
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t app_livestream_init(void);

/**
 * @brief Start the WebSocket server and begin accepting connections
 *
 * Should be called after Wi-Fi is connected.
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t app_livestream_start_server(void);

/**
 * @brief Stop the WebSocket server
 *
 * @return ESP_OK on success
 */
esp_err_t app_livestream_stop_server(void);

/**
 * @brief Feed a camera frame for H.264 encoding and WebSocket transmission
 *
 * This function should be called from the video frame processing callback
 * when the livestream page is active. It converts the RGB565 frame to YUV420,
 * encodes it with the hardware H.264 encoder, and sends the resulting NAL
 * units to all connected WebSocket clients.
 *
 * @param rgb565_buf Pointer to the RGB565 camera frame buffer
 * @param width Frame width in pixels
 * @param height Frame height in pixels
 * @return ESP_OK on success, error code on failure
 */
esp_err_t app_livestream_feed_frame(uint8_t *rgb565_buf, uint32_t width, uint32_t height);

/**
 * @brief Get the current livestream state
 *
 * @return Current livestream state enum
 */
livestream_state_t app_livestream_get_state(void);

/**
 * @brief Get a human-readable status string for display
 *
 * @return Static string describing current state (e.g., "Connecting...", "Streaming")
 */
const char* app_livestream_get_status_str(void);

/**
 * @brief Get the WebSocket server URL
 *
 * @return Static string with the URL (e.g., "ws://192.168.1.100:8080/stream")
 *         or "No IP" if not connected
 */
const char* app_livestream_get_ws_url(void);

/**
 * @brief Check if there are any connected WebSocket clients
 *
 * @return true if at least one client is connected
 */
bool app_livestream_has_clients(void);

#ifdef __cplusplus
}
#endif
