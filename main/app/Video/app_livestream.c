/**
 * @file app_livestream.c
 * @brief Wi-Fi H.264 WebSocket livestream implementation
 */

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_private/esp_cache_private.h"
#include "esp_h264_enc_single.h"
#include "esp_h264_enc_param_hw.h"
#include "esp_h264_enc_single_hw.h"

#include "app_livestream.h"

static const char *TAG = "app_livestream";

/* Configuration */
#define STREAM_WIDTH        640
#define STREAM_HEIGHT       480
#define STREAM_FPS          15
#define STREAM_BITRATE      1500000  /* 1.5 Mbps */
#define STREAM_GOP          30
#define WS_SERVER_PORT      8080
#define MAX_WS_CLIENTS      2
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define MAX_RETRY           10

/* State */
static livestream_state_t s_state = LIVESTREAM_STATE_IDLE;
static char s_ws_url[64] = "No IP";
static char s_ip_str[16] = "";
static EventGroupHandle_t s_wifi_event_group = NULL;
static httpd_handle_t s_server = NULL;
static int s_retry_count = 0;
static bool s_wifi_connected = false;

/* H.264 encoder */
static esp_h264_enc_handle_t s_h264_enc = NULL;
static uint8_t *s_yuv_buf = NULL;
static uint8_t *s_out_buf = NULL;
static size_t s_yuv_buf_size = 0;
static size_t s_data_cache_line = 0;
static bool s_encoder_ready = false;

/* WebSocket clients tracking */
static int s_ws_fds[MAX_WS_CLIENTS];
static int s_ws_client_count = 0;
static SemaphoreHandle_t s_ws_mutex = NULL;

/* ── RGB565 to YUV420 conversion ────────────────────────── */

static inline void rgb565_to_yuv(uint16_t rgb565, uint8_t *y, uint8_t *u, uint8_t *v)
{
    uint8_t r = (rgb565 >> 11) & 0x1F;
    uint8_t g = (rgb565 >> 5) & 0x3F;
    uint8_t b = rgb565 & 0x1F;
    r = (r << 3) | (r >> 2);
    g = (g << 2) | (g >> 4);
    b = (b << 3) | (b >> 2);
    int yi = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
    int ui = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
    int vi = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
    *y = (uint8_t)(yi < 0 ? 0 : (yi > 255 ? 255 : yi));
    *u = (uint8_t)(ui < 0 ? 0 : (ui > 255 ? 255 : ui));
    *v = (uint8_t)(vi < 0 ? 0 : (vi > 255 ? 255 : vi));
}

static void rgb565_to_ouyy_evyy(const uint8_t *rgb565, uint8_t *ouyy_evyy,
                                uint32_t width, uint32_t height)
{
    /* 
     * O_UYY_E_VYY format:
     * Odd lines (1, 3, 5...): u y y u y y u y y...
     * Even lines (2, 4, 6...): v y y v y y v y y...
     * 
     * For a 2x2 block:
     * (row, col)     | Y | U | V
     * (0, 0)         | Y0| U0| V0
     * (0, 1)         | Y1|   | 
     * (1, 0)         | Y2|   |
     * (1, 1)         | Y3|   |
     * 
     * Line 0 (even in 0-indexed, but "odd" in documentation 1-indexed): U0 Y0 Y1
     * Line 1 (odd in 0-indexed, "even" in doc): V0 Y2 Y3
     */
    const uint16_t *src = (const uint16_t *)rgb565;
    uint8_t *dst = ouyy_evyy;

    for (uint32_t row = 0; row < height; row += 2) {
        /* Process two lines at a time */
        uint8_t *line_u = dst + row * width * 3 / 2;
        uint8_t *line_v = dst + (row + 1) * width * 3 / 2;

        for (uint32_t col = 0; col < width; col += 2) {
            uint8_t y00, u00, v00;
            uint8_t y01, u01, v01;
            uint8_t y10, u10, v10;
            uint8_t y11, u11, v11;

            rgb565_to_yuv(src[row * width + col], &y00, &u00, &v00);
            rgb565_to_yuv(src[row * width + col + 1], &y01, &u01, &v01);
            rgb565_to_yuv(src[(row + 1) * width + col], &y10, &u10, &v10);
            rgb565_to_yuv(src[(row + 1) * width + col + 1], &y11, &u11, &v11);

            /* Average U and V for the 2x2 block */
            uint8_t u_avg = (u00 + u01 + u10 + u11) / 4;
            uint8_t v_avg = (v00 + v01 + v10 + v11) / 4;

            /* Line 0: U Y Y */
            *line_u++ = u_avg;
            *line_u++ = y00;
            *line_u++ = y01;

            /* Line 1: V Y Y */
            *line_v++ = v_avg;
            *line_v++ = y10;
            *line_v++ = y11;
        }
    }
}

/* ── Downscale RGB565 (nearest-neighbor) ────────────────── */

static void downscale_rgb565(const uint8_t *src, uint32_t src_w, uint32_t src_h,
                             uint8_t *dst, uint32_t dst_w, uint32_t dst_h)
{
    const uint16_t *s = (const uint16_t *)src;
    uint16_t *d = (uint16_t *)dst;
    for (uint32_t y = 0; y < dst_h; y++) {
        uint32_t sy = y * src_h / dst_h;
        for (uint32_t x = 0; x < dst_w; x++) {
            uint32_t sx = x * src_w / dst_w;
            d[y * dst_w + x] = s[sy * src_w + sx];
        }
    }
}

/* ── Wi-Fi event handler ────────────────────────────────── */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        s_state = LIVESTREAM_STATE_WIFI_CONNECTING;
        if (s_retry_count < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGI(TAG, "Retry Wi-Fi connect (%d/%d)", s_retry_count, MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            s_state = LIVESTREAM_STATE_ERROR;
            ESP_LOGE(TAG, "Wi-Fi connect failed after %d retries", MAX_RETRY);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        snprintf(s_ws_url, sizeof(s_ws_url), "ws://%s:%d/stream", s_ip_str, WS_SERVER_PORT);
        ESP_LOGI(TAG, "Wi-Fi connected! IP: %s", s_ip_str);
        ESP_LOGI(TAG, "WebSocket URL: %s", s_ws_url);
        s_retry_count = 0;
        s_wifi_connected = true;
        s_state = LIVESTREAM_STATE_WIFI_CONNECTED;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ── WebSocket handler ──────────────────────────────────── */

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WS handshake from client");
        /* Track client fd */
        if (s_ws_mutex) xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
        if (s_ws_client_count < MAX_WS_CLIENTS) {
            s_ws_fds[s_ws_client_count++] = httpd_req_to_sockfd(req);
            s_state = LIVESTREAM_STATE_STREAMING;
            ESP_LOGI(TAG, "Client connected (total: %d)", s_ws_client_count);
        } else {
            ESP_LOGW(TAG, "Max clients reached, rejecting");
        }
        if (s_ws_mutex) xSemaphoreGive(s_ws_mutex);
        return ESP_OK;
    }

    /* Handle incoming WS frames (ping/pong/close) */
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WS recv failed: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

static const httpd_uri_t ws_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = ws_handler,
    .is_websocket = true,
};

/* Send data to all connected WS clients */
static void ws_broadcast(const uint8_t *data, size_t len)
{
    if (!s_server || !s_ws_mutex) return;

    httpd_ws_frame_t ws_pkt = {
        .payload = (uint8_t *)data,
        .len = len,
        .type = HTTPD_WS_TYPE_BINARY,
        .final = true,
    };

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < s_ws_client_count; ) {
        esp_err_t ret = httpd_ws_send_frame_async(s_server, s_ws_fds[i], &ws_pkt);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Client %d send failed, removing", s_ws_fds[i]);
            /* Remove dead client by shifting array */
            for (int j = i; j < s_ws_client_count - 1; j++) {
                s_ws_fds[j] = s_ws_fds[j + 1];
            }
            s_ws_client_count--;
            if (s_ws_client_count == 0) {
                s_state = LIVESTREAM_STATE_READY;
            }
        } else {
            i++;
        }
    }
    xSemaphoreGive(s_ws_mutex);
}

/* ── H.264 Encoder ──────────────────────────────────────── */

static esp_err_t init_h264_encoder(void)
{
    esp_h264_enc_cfg_hw_t cfg = {
        .gop = STREAM_GOP,
        .fps = STREAM_FPS,
        .res = { .width = STREAM_WIDTH, .height = STREAM_HEIGHT },
        .rc = {
            .bitrate = STREAM_BITRATE,
            .qp_min = 22,
            .qp_max = 40,
        },
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,
    };

    esp_h264_err_t err = esp_h264_enc_hw_new(&cfg, &s_h264_enc);
    if (err != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "H.264 encoder create failed: %d", err);
        return ESP_FAIL;
    }

    err = esp_h264_enc_open(s_h264_enc);
    if (err != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "H.264 encoder open failed: %d", err);
        esp_h264_enc_del(s_h264_enc);
        s_h264_enc = NULL;
        return ESP_FAIL;
    }

    /* Allocate YUV420 buffer: W * H * 1.5 */
    s_yuv_buf_size = STREAM_WIDTH * STREAM_HEIGHT * 3 / 2;
    s_yuv_buf = heap_caps_aligned_calloc(s_data_cache_line, 1, s_yuv_buf_size,
                                         MALLOC_CAP_SPIRAM);
    s_out_buf = heap_caps_aligned_calloc(s_data_cache_line, 1, s_yuv_buf_size,
                                         MALLOC_CAP_SPIRAM);
    if (!s_yuv_buf || !s_out_buf) {
        ESP_LOGE(TAG, "Failed to allocate encoder buffers");
        if (s_yuv_buf) heap_caps_free(s_yuv_buf);
        if (s_out_buf) heap_caps_free(s_out_buf);
        esp_h264_enc_close(s_h264_enc);
        esp_h264_enc_del(s_h264_enc);
        s_h264_enc = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_encoder_ready = true;
    ESP_LOGI(TAG, "H.264 encoder initialized: %dx%d @ %dfps, %d bps",
             STREAM_WIDTH, STREAM_HEIGHT, STREAM_FPS, STREAM_BITRATE);
    return ESP_OK;
}

/* ── Public API ─────────────────────────────────────────── */

esp_err_t app_livestream_init(void)
{
    esp_err_t ret;

    s_ws_mutex = xSemaphoreCreateMutex();
    if (!s_ws_mutex) return ESP_ERR_NO_MEM;

    memset(s_ws_fds, -1, sizeof(s_ws_fds));

    ret = esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &s_data_cache_line);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Cache alignment query failed");
        return ret;
    }

    /* Initialize H.264 encoder */
    ret = init_h264_encoder();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "H.264 init failed");
        return ret;
    }

    /* Initialize networking stack */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    s_wifi_event_group = xEventGroupCreate();

    esp_event_handler_instance_t inst_any_id, inst_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &inst_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &inst_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_state = LIVESTREAM_STATE_WIFI_CONNECTING;
    ESP_LOGI(TAG, "Wi-Fi STA started, connecting to %s...", CONFIG_ESP_WIFI_SSID);

    /* Wait for connection (non-blocking via event group, but we do a brief wait) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connected to %s", CONFIG_ESP_WIFI_SSID);
        /* Start WS server immediately */
        app_livestream_start_server();
    } else {
        ESP_LOGW(TAG, "Wi-Fi connect timeout, will retry in background");
        /* Will keep retrying via event handler */
    }

    return ESP_OK;
}

esp_err_t app_livestream_start_server(void)
{
    if (s_server) {
        ESP_LOGW(TAG, "Server already running");
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WS_SERVER_PORT;
    config.max_open_sockets = MAX_WS_CLIENTS + 1;
    config.stack_size = 8192;

    esp_err_t ret = httpd_start(&s_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return ret;
    }

    httpd_register_uri_handler(s_server, &ws_uri);
    s_state = LIVESTREAM_STATE_READY;
    ESP_LOGI(TAG, "WebSocket server started on port %d", WS_SERVER_PORT);
    ESP_LOGI(TAG, "Stream URL: %s", s_ws_url);
    return ESP_OK;
}

esp_err_t app_livestream_stop_server(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
        s_ws_client_count = 0;
        s_state = s_wifi_connected ? LIVESTREAM_STATE_WIFI_CONNECTED : LIVESTREAM_STATE_IDLE;
        ESP_LOGI(TAG, "WebSocket server stopped");
    }
    return ESP_OK;
}

esp_err_t app_livestream_feed_frame(uint8_t *rgb565_buf, uint32_t width, uint32_t height)
{
    if (!s_encoder_ready || !s_h264_enc) return ESP_ERR_INVALID_STATE;
    if (s_ws_client_count == 0) return ESP_OK;  /* No clients, skip encoding */

    /* Downscale if needed */
    static uint8_t *s_downscale_buf = NULL;
    const uint8_t *src_buf = rgb565_buf;

    if (width != STREAM_WIDTH || height != STREAM_HEIGHT) {
        if (!s_downscale_buf) {
            s_downscale_buf = heap_caps_aligned_calloc(s_data_cache_line, 1,
                STREAM_WIDTH * STREAM_HEIGHT * 2, MALLOC_CAP_SPIRAM);
            if (!s_downscale_buf) {
                ESP_LOGE(TAG, "Downscale buf alloc failed");
                return ESP_ERR_NO_MEM;
            }
        }
        downscale_rgb565(rgb565_buf, width, height,
                         s_downscale_buf, STREAM_WIDTH, STREAM_HEIGHT);
        src_buf = s_downscale_buf;
    }

    /* Convert RGB565 to O_UYY_E_VYY */
    rgb565_to_ouyy_evyy(src_buf, s_yuv_buf, STREAM_WIDTH, STREAM_HEIGHT);

    /* Encode with H.264 */
    esp_h264_enc_in_frame_t in_frame = {
        .raw_data = { .buffer = s_yuv_buf, .len = s_yuv_buf_size },
        .pts = (uint64_t)(esp_timer_get_time() / 1000),
    };
    esp_h264_enc_out_frame_t out_frame = {
        .raw_data = { .buffer = s_out_buf, .len = s_yuv_buf_size }
    };

    esp_h264_err_t err = esp_h264_enc_process(s_h264_enc, &in_frame, &out_frame);
    if (err != ESP_H264_ERR_OK) {
        ESP_LOGW(TAG, "H.264 encode failed: %d", err);
        return ESP_FAIL;
    }

    /* Broadcast encoded data to WebSocket clients */
    if (out_frame.raw_data.buffer && out_frame.length > 0) {
        ws_broadcast(out_frame.raw_data.buffer, out_frame.length);
    }

    return ESP_OK;
}

livestream_state_t app_livestream_get_state(void)
{
    return s_state;
}

const char* app_livestream_get_status_str(void)
{
    switch (s_state) {
        case LIVESTREAM_STATE_IDLE:            return "Idle";
        case LIVESTREAM_STATE_WIFI_CONNECTING: return "Wi-Fi Connecting...";
        case LIVESTREAM_STATE_WIFI_CONNECTED:  return "Wi-Fi Connected";
        case LIVESTREAM_STATE_READY:           return "Ready (No Clients)";
        case LIVESTREAM_STATE_STREAMING:       return "Streaming";
        case LIVESTREAM_STATE_ERROR:           return "Error";
        default:                              return "Unknown";
    }
}

const char* app_livestream_get_ws_url(void)
{
    return s_ws_url;
}

bool app_livestream_has_clients(void)
{
    return s_ws_client_count > 0;
}
