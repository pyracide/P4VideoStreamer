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
#include "app_video_utils.h"
#include "driver/ppa.h"

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
static uint8_t *s_task_rgb_buf = NULL;
static size_t s_yuv_buf_size = 0;
static size_t s_data_cache_line = 0;
static bool s_encoder_ready = false;

/* FreeRTOS synchronization for decoupled processing */
static TaskHandle_t s_stream_task_handle = NULL;
static SemaphoreHandle_t s_frame_ready_sem = NULL;
static volatile bool s_is_processing = false;

/* WebSocket clients tracking */
static int s_ws_fds[MAX_WS_CLIENTS];
static int s_ws_client_count = 0;
static SemaphoreHandle_t s_ws_mutex = NULL;
static float s_actual_fps = 0;

/* ── Hardware Conversion (YUV422 -> O_UYY_E_VYY) ────────── */



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

    /* Consume any incoming data (pings/messages) to prevent protocol desync.
       Clients often send pings which must be read to keep the session alive. */
    uint8_t buf[64];
    httpd_ws_frame_t ws_pkt = {
        .payload = buf,
    };
    httpd_ws_recv_frame(req, &ws_pkt, sizeof(buf));
    return ESP_OK;
}

static const httpd_uri_t ws_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = ws_handler,
    .is_websocket = true,
};

/* Helper to find H.264 NAL start code */
static const uint8_t* find_nal_start_code(const uint8_t *data, size_t len, size_t *start_code_len)
{
    for (size_t i = 0; i + 2 < len; i++) {
        if (data[i] == 0x00 && data[i+1] == 0x00) {
            if (data[i+2] == 0x01) {
                if (start_code_len) *start_code_len = 3;
                return &data[i];
            }
            if (i + 3 < len && data[i+2] == 0x00 && data[i+3] == 0x01) {
                if (start_code_len) *start_code_len = 4;
                return &data[i];
            }
        }
    }
    return NULL;
}

/* Send data to all connected WS clients via NAL Unit Chunking */
static void ws_broadcast(const uint8_t *data, size_t len)
{
    if (!s_server || !s_ws_mutex || len == 0) return;

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    
    for (int i = 0; i < s_ws_client_count; i++) {
        int client_fd = s_ws_fds[i];
        const uint8_t *ptr = data;
        size_t remaining = len;

        size_t sc_len = 0;
        const uint8_t *nal_start = find_nal_start_code(ptr, remaining, &sc_len);

        while (nal_start != NULL) {
            /* Find the next start code to determine the end of this NAL unit */
            const uint8_t *next_nal_start = NULL;
            size_t next_sc_len = 0;
            
            size_t search_offset = (nal_start - ptr) + sc_len;
            if (search_offset < remaining) {
                next_nal_start = find_nal_start_code(ptr + search_offset, remaining - search_offset, &next_sc_len);
            }

            size_t nal_len = 0;
            if (next_nal_start != NULL) {
                nal_len = next_nal_start - nal_start;
            } else {
                nal_len = remaining - (nal_start - ptr);
            }

            if (nal_len > 0) {
                /* Fragmentation logic: slice large NAL units into smaller WS frames */
                const size_t max_ws_chunk = 2048;
                size_t nal_offset = 0;
                bool client_error = false;

                while (nal_offset < nal_len) {
                    size_t chunk_size = MIN(max_ws_chunk, nal_len - nal_offset);
                    bool is_first = (nal_offset == 0);
                    bool is_last = (nal_offset + chunk_size >= nal_len);

                    httpd_ws_frame_t ws_pkt = {
                        .payload = (uint8_t *)(nal_start + nal_offset),
                        .len = chunk_size,
                        .type = is_first ? HTTPD_WS_TYPE_BINARY : HTTPD_WS_TYPE_CONTINUE,
                        .fragmented = !is_last,
                        .final = is_last,
                    };

                    esp_err_t ret = httpd_ws_send_frame_async(s_server, client_fd, &ws_pkt);
                    if (ret != ESP_OK) {
                        if (ret == ESP_ERR_NO_MEM || ret == ESP_ERR_INVALID_STATE || ret == ESP_FAIL) {
                            ESP_LOGW(TAG, "Network choked, dropping NAL for client %d", client_fd);
                        } else {
                            ESP_LOGW(TAG, "Client %d send failed: 0x%x", client_fd, ret);
                        }
                        client_error = true;
                        break;
                    }

                    nal_offset += chunk_size;
                    /* Pacing within a NAL unit if it's large */
                    if (!is_last) {
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                }
                
                if (client_error) break; /* Skip rest of the frame for this client */
                
                /* Brief pacing between NAL units */
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            if (next_nal_start != NULL) {
                remaining -= (next_nal_start - ptr);
                ptr = next_nal_start;
                nal_start = ptr;
                sc_len = next_sc_len;
            } else {
                break;
            }
        }
    }
    
    /* Clean up dead clients using a PING check */
    for (int i = 0; i < s_ws_client_count; ) {
        httpd_ws_frame_t ping = { .type = HTTPD_WS_TYPE_PING };
        if (httpd_ws_send_frame_async(s_server, s_ws_fds[i], &ping) != ESP_OK) {
            for (int j = i; j < s_ws_client_count - 1; j++) s_ws_fds[j] = s_ws_fds[j + 1];
            s_ws_client_count--;
        } else {
            i++;
        }
    }
    if (s_ws_client_count == 0) s_state = LIVESTREAM_STATE_READY;
    
    xSemaphoreGive(s_ws_mutex);
}

/* ── H.264 Encoder Task ─────────────────────────────────── */

static void livestream_process_task(void *arg)
{
    static int64_t last_time = 0;
    static int frame_count = 0;

    while (1) {
        if (xSemaphoreTake(s_frame_ready_sem, portMAX_DELAY) == pdTRUE) {
            if (s_ws_client_count == 0 || !s_h264_enc) {
                s_is_processing = false;
                continue;
            }

            /* Zero-Copy: Input is already YUV420 (O_UYY_E_VYY) from camera */
            /* We just need to ensure the buffer is the correct size */
            uint8_t *encoding_buf = s_task_rgb_buf;

            /* Encode with H.264 */
            esp_h264_enc_in_frame_t in_frame = {
                .raw_data = { .buffer = encoding_buf, .len = s_yuv_buf_size },
                .pts = (uint64_t)(esp_timer_get_time() / 1000),
            };
            esp_h264_enc_out_frame_t out_frame = {
                .raw_data = { .buffer = s_out_buf, .len = s_yuv_buf_size }
            };

            esp_h264_err_t err = esp_h264_enc_process(s_h264_enc, &in_frame, &out_frame);
            if (err == ESP_H264_ERR_OK) {
                /* Broadcast encoded data to WebSocket clients */
                if (out_frame.raw_data.buffer && out_frame.length > 0) {
                    ws_broadcast(out_frame.raw_data.buffer, out_frame.length);
                    
                    /* Calculate actual FPS */
                    frame_count++;
                    int64_t now = esp_timer_get_time();
                    if (now - last_time >= 1000000) {
                        s_actual_fps = (float)frame_count * 1000000.0f / (float)(now - last_time);
                        last_time = now;
                        frame_count = 0;
                    }
                }
            } else {
                ESP_LOGW(TAG, "H.264 encode failed: %d", err);
            }

            /* Release the buffer lock so the camera thread can push the next frame */
            s_is_processing = false;
        }
    }
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

    /* Allocate buffers with 128-byte alignment for hardware efficiency */
    s_yuv_buf_size = STREAM_WIDTH * STREAM_HEIGHT * 3 / 2;
    s_yuv_buf = heap_caps_aligned_calloc(128, 1, s_yuv_buf_size, MALLOC_CAP_SPIRAM);
    s_out_buf = heap_caps_aligned_calloc(128, 1, s_yuv_buf_size, MALLOC_CAP_SPIRAM);
    if (!s_yuv_buf || !s_out_buf) {
        ESP_LOGE(TAG, "Failed to allocate encoder buffers");
        if (s_yuv_buf) heap_caps_free(s_yuv_buf);
        if (s_out_buf) heap_caps_free(s_out_buf);
        esp_h264_enc_close(s_h264_enc);
        esp_h264_enc_del(s_h264_enc);
        s_h264_enc = NULL;
        return ESP_ERR_NO_MEM;
    }

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

    s_task_rgb_buf = heap_caps_aligned_calloc(128, 1, 
        STREAM_WIDTH * STREAM_HEIGHT * 2, MALLOC_CAP_SPIRAM);
    if (!s_task_rgb_buf) {
        ESP_LOGE(TAG, "Task RGB buffer allocation failed");
        return ESP_ERR_NO_MEM;
    }

    s_frame_ready_sem = xSemaphoreCreateBinary();
    if (!s_frame_ready_sem) {
        return ESP_ERR_NO_MEM;
    }

    /* Create the decoupled processing task */
    xTaskCreatePinnedToCore(livestream_process_task, "livestream_task", 8192, NULL, 5, &s_stream_task_handle, 1);

    /* Initialize H.264 encoder */
    ret = init_h264_encoder();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "H.264 init failed");
        return ret;
    }

    s_encoder_ready = true;

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
    config.stack_size = 10240;
    config.task_priority = 6;      // Higher priority than processing task
    config.core_id = 0;            // Pinned to Core 0 (Processing is on Core 1)
    config.lru_purge_enable = true;

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

esp_err_t app_livestream_feed_frame(uint8_t *yuv420_buf, uint32_t width, uint32_t height)
{
    if (!s_encoder_ready || !s_h264_enc) return ESP_ERR_INVALID_STATE;
    if (s_ws_client_count == 0) return ESP_OK;  /* No clients, skip encoding */

    if (s_is_processing) {
        /* Task is still busy encoding/sending the previous frame.
           Drop this frame to avoid blocking the camera capture thread (High FPS preservation). */
        return ESP_OK;
    }
    
    s_is_processing = true;

    /* Input is now YUV420 (O_UYY_E_VYY) */
    if (width != STREAM_WIDTH || height != STREAM_HEIGHT) {
        /* Downscale if needed via PPA (YUV420 -> YUV420) */
        app_image_process_scale_crop(yuv420_buf, width, height, PPA_SRM_COLOR_MODE_YUV420,
                                     STREAM_WIDTH, STREAM_HEIGHT,
                                     s_task_rgb_buf, STREAM_WIDTH, STREAM_HEIGHT, 
                                     s_yuv_buf_size, PPA_SRM_COLOR_MODE_YUV420, PPA_SRM_ROTATION_ANGLE_0);
    } else {
        memcpy(s_task_rgb_buf, yuv420_buf, s_yuv_buf_size);
    }

    /* Signal the processing task that a new frame is ready */
    xSemaphoreGive(s_frame_ready_sem);

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
float app_livestream_get_actual_fps(void)
{
    return s_actual_fps;
}
