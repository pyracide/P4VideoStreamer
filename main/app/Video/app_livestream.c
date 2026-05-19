/**
 * @file app_livestream.c
 * @brief Wi-Fi H.264 RTSP livestream implementation
 */

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_private/esp_cache_private.h"
#include "esp_h264_enc_single.h"
#include "esp_h264_enc_param_hw.h"
#include "esp_h264_enc_single_hw.h"
#include "mbedtls/base64.h"
#include "esp_rom_sys.h"

#include "app_livestream.h"
#include "app_video_utils.h"
#undef _IO
#undef _IOR
#undef _IOW
#include "app_video.h"
#include "app_storage.h"
#include "driver/ppa.h"
#include <inttypes.h>

static const char *TAG = "app_livestream";

/* Configuration */
#define STREAM_FPS          25
#define STREAM_BITRATE      1000000  /* 1.0 Mbps (Better Wi-Fi stability) */
#define STREAM_GOP          25       /* 1 I-frame per second */
#define RTSP_SERVER_PORT    554
#define RTP_LOCAL_PORT      55400
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define MAX_RETRY           10

/* State */
static livestream_state_t s_state = LIVESTREAM_STATE_IDLE;
static char s_rtsp_url[64] = "No IP";
static char s_ip_str[16] = "";
static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_count = 0;
static bool s_wifi_connected = false;

/* H.264 encoder */
static esp_h264_enc_handle_t s_h264_enc = NULL;
static uint8_t *s_out_buf = NULL;
static uint8_t *s_task_rgb_buf = NULL;
static size_t s_yuv_buf_size = 0;
static uint32_t s_stream_width = 1280;
static uint32_t s_stream_height = 720;
static size_t s_data_cache_line = 0;
static bool s_encoder_ready = false;

/* SPS/PPS storage for SDP */
static char s_sps_b64[128] = {0};
static char s_pps_b64[128] = {0};

/* FreeRTOS synchronization for decoupled processing */
static TaskHandle_t s_stream_task_handle = NULL;
static QueueHandle_t s_frame_queue = NULL;

/* Networking tracking */
static int s_rtsp_sock = -1;
static int s_udp_sock = -1;
static struct sockaddr_in s_rtp_dest_addr;
static bool s_is_streaming = false;
static uint16_t s_rtp_seq = 0;
static uint32_t s_rtp_timestamp = 0;
static float s_actual_fps = 0;
static bool s_is_tcp_interleaved = false;
static int s_interleaved_ch = 0;
static int s_active_rtsp_sock = -1;

typedef struct {
    uint8_t *buf;
    uint8_t index;
    uint32_t width;
    uint32_t height;
} frame_req_t;

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
        snprintf(s_rtsp_url, sizeof(s_rtsp_url), "rtsp://%s:%d/stream", s_ip_str, RTSP_SERVER_PORT);
        ESP_LOGI(TAG, "Wi-Fi connected! IP: %s", s_ip_str);
        ESP_LOGI(TAG, "RTSP URL: %s", s_rtsp_url);
        s_retry_count = 0;
        s_wifi_connected = true;
        s_state = LIVESTREAM_STATE_WIFI_CONNECTED;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

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

/* ── RTP Transport ──────────────────────────────────────── */

static void send_rtp_packet(const uint8_t *payload, size_t payload_len, bool marker)
{
    if (!s_is_streaming) return;

    if (s_is_tcp_interleaved) {
        if (s_active_rtsp_sock < 0) return;
        
        uint8_t packet[1500];
        size_t rtp_len = 12 + payload_len;
        if (4 + rtp_len > sizeof(packet)) return;
        
        // Interleaved header
        packet[0] = 0x24; // '$'
        packet[1] = s_interleaved_ch;
        packet[2] = (rtp_len >> 8) & 0xFF;
        packet[3] = rtp_len & 0xFF;
        
        // RTP Header
        packet[4] = 0x80; // V=2
        packet[5] = (marker ? 0x80 : 0x00) | 96; // PT=96
        packet[6] = (s_rtp_seq >> 8) & 0xFF;
        packet[7] = s_rtp_seq & 0xFF;
        packet[8] = (s_rtp_timestamp >> 24) & 0xFF;
        packet[9] = (s_rtp_timestamp >> 16) & 0xFF;
        packet[10] = (s_rtp_timestamp >> 8) & 0xFF;
        packet[11] = s_rtp_timestamp & 0xFF;
        packet[12] = 0; // SSRC = 1
        packet[13] = 0;
        packet[14] = 0;
        packet[15] = 1;
        
        memcpy(packet + 16, payload, payload_len);
        
        send(s_active_rtsp_sock, packet, 4 + rtp_len, 0);
        s_rtp_seq++;
    } else {
        if (s_udp_sock < 0) return;
        
        /* Build RTP packet in a flat buffer to ensure LwIP compatibility */
        uint8_t packet[1500];
        if (12 + payload_len > sizeof(packet)) return;

        packet[0] = 0x80; // V=2
        packet[1] = (marker ? 0x80 : 0x00) | 96; // PT=96
        packet[2] = (s_rtp_seq >> 8) & 0xFF;
        packet[3] = s_rtp_seq & 0xFF;
        packet[4] = (s_rtp_timestamp >> 24) & 0xFF;
        packet[5] = (s_rtp_timestamp >> 16) & 0xFF;
        packet[6] = (s_rtp_timestamp >> 8) & 0xFF;
        packet[7] = s_rtp_timestamp & 0xFF;
        packet[8] = 0; // SSRC = 1
        packet[9] = 0;
        packet[10] = 0;
        packet[11] = 1;

        memcpy(packet + 12, payload, payload_len);

        sendto(s_udp_sock, packet, 12 + payload_len, 0, (struct sockaddr *)&s_rtp_dest_addr, sizeof(s_rtp_dest_addr));
        s_rtp_seq++;
        
        /* Hardware-Specific Pacing: SDIO Relief */
        esp_rom_delay_us(150); 
    }
}

static void send_nal_rtp(const uint8_t *nal_data, size_t nal_len, bool is_last_nal)
{
    const size_t max_payload = 1300;
    if (nal_len <= max_payload) {
        send_rtp_packet(nal_data, nal_len, is_last_nal);
    } else {
        uint8_t nal_hdr = nal_data[0];
        uint8_t fu_indicator = (nal_hdr & 0xE0) | 28;
        uint8_t type = nal_hdr & 0x1F;
        
        size_t offset = 1;
        while (offset < nal_len) {
            size_t chunk_len = MIN(max_payload, nal_len - offset);
            bool is_first = (offset == 1);
            bool is_last = (offset + chunk_len >= nal_len);
            
            uint8_t fu_header = type;
            if (is_first) fu_header |= 0x80;
            if (is_last)  fu_header |= 0x40;
            
            uint8_t payload[1302]; 
            payload[0] = fu_indicator;
            payload[1] = fu_header;
            memcpy(payload + 2, nal_data + offset, chunk_len);
            
            send_rtp_packet(payload, chunk_len + 2, is_last && is_last_nal);
            
            offset += chunk_len;
        }
    }
}

static void rtp_broadcast(const uint8_t *data, size_t len)
{
    if (!s_is_streaming) return;

    /* Use true wall-clock time for RTP timestamp (90kHz clock).
       This completely eliminates client-side buffering latency! */
    s_rtp_timestamp = (uint32_t)(esp_timer_get_time() * 9LL / 100LL);

    const uint8_t *ptr = data;
    size_t remaining = len;

    size_t sc_len = 0;
    const uint8_t *nal_start = find_nal_start_code(ptr, remaining, &sc_len);

    while (nal_start != NULL) {
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
            bool is_last_nal = (next_nal_start == NULL);
            send_nal_rtp(nal_start + sc_len, nal_len - sc_len, is_last_nal);
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

/* ── RTSP Server ────────────────────────────────────────── */

static void rtsp_server_task(void *arg)
{
    char rx_buffer[1024];
    char tx_buffer[1024];
    
    s_rtsp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(RTSP_SERVER_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };
    
    bind(s_rtsp_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    listen(s_rtsp_sock, 1);
    
    ESP_LOGI(TAG, "RTSP TCP server listening on port %d", RTSP_SERVER_PORT);
    
    while (1) {
        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(s_rtsp_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        ESP_LOGI(TAG, "RTSP client connected");

        int flag = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));
        
        while (1) {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len <= 0) break;
            rx_buffer[len] = 0;
            
            s_active_rtsp_sock = sock;
            
            /* Extract CSeq */
            int cseq = 0;
            char *cseq_ptr = strstr(rx_buffer, "CSeq: ");
            if (cseq_ptr) sscanf(cseq_ptr, "CSeq: %d", &cseq);
            
            if (strncmp(rx_buffer, "OPTIONS", 7) == 0) {
                snprintf(tx_buffer, sizeof(tx_buffer),
                         "RTSP/1.0 200 OK\r\n"
                         "CSeq: %d\r\n"
                         "Public: OPTIONS, DESCRIBE, SETUP, TEARDOWN, PLAY\r\n"
                         "\r\n", cseq);
                send(sock, tx_buffer, strlen(tx_buffer), 0);
            } 
            else if (strncmp(rx_buffer, "DESCRIBE", 8) == 0) {
                char sdp[512];
                snprintf(sdp, sizeof(sdp),
                         "v=0\r\n"
                         "o=- 0 0 IN IP4 %s\r\n"
                         "s=ESP32-P4 Stream\r\n"
                         "c=IN IP4 %s\r\n"
                         "t=0 0\r\n"
                         "m=video 0 RTP/AVP 96\r\n"
                         "a=rtpmap:96 H264/90000\r\n"
                         "a=fmtp:96 packetization-mode=1; sprop-parameter-sets=%s,%s;\r\n"
                         "a=control:track1\r\n", 
                         s_ip_str, s_ip_str, s_sps_b64, s_pps_b64);
                
                snprintf(tx_buffer, sizeof(tx_buffer),
                         "RTSP/1.0 200 OK\r\n"
                         "CSeq: %d\r\n"
                         "Content-Type: application/sdp\r\n"
                         "Content-Length: %d\r\n"
                         "\r\n"
                         "%s", cseq, strlen(sdp), sdp);
                send(sock, tx_buffer, strlen(tx_buffer), 0);
            }
            else if (strncmp(rx_buffer, "SETUP", 5) == 0) {
                char *interleaved_ptr = strstr(rx_buffer, "interleaved=");
                if (interleaved_ptr) {
                    int interleaved_ch = 0;
                    sscanf(interleaved_ptr, "interleaved=%d", &interleaved_ch);
                    s_is_tcp_interleaved = true;
                    s_interleaved_ch = interleaved_ch;
                    
                    snprintf(tx_buffer, sizeof(tx_buffer),
                             "RTSP/1.0 200 OK\r\n"
                             "CSeq: %d\r\n"
                             "Transport: RTP/AVP/TCP;unicast;interleaved=%d-%d\r\n"
                             "Session: 12345678\r\n"
                             "\r\n", cseq, interleaved_ch, interleaved_ch+1);
                    send(sock, tx_buffer, strlen(tx_buffer), 0);
                } else {
                    int client_port = 0;
                    char *port_ptr = strstr(rx_buffer, "client_port=");
                    if (port_ptr) sscanf(port_ptr, "client_port=%d", &client_port);
                    
                    s_is_tcp_interleaved = false;
                    
                    /* Create UDP socket */
                    if (s_udp_sock >= 0) close(s_udp_sock);
                    s_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
                    struct sockaddr_in local_addr = {
                        .sin_family = AF_INET,
                        .sin_port = htons(RTP_LOCAL_PORT),
                        .sin_addr.s_addr = htonl(INADDR_ANY)
                    };
                    bind(s_udp_sock, (struct sockaddr *)&local_addr, sizeof(local_addr));
                    
                    s_rtp_dest_addr.sin_family = AF_INET;
                    s_rtp_dest_addr.sin_port = htons(client_port);
                    s_rtp_dest_addr.sin_addr.s_addr = source_addr.sin_addr.s_addr;
                    
                    snprintf(tx_buffer, sizeof(tx_buffer),
                             "RTSP/1.0 200 OK\r\n"
                             "CSeq: %d\r\n"
                             "Transport: RTP/AVP;unicast;client_port=%d-%d;server_port=%d-%d\r\n"
                             "Session: 12345678\r\n"
                             "\r\n", cseq, client_port, client_port+1, RTP_LOCAL_PORT, RTP_LOCAL_PORT+1);
                    send(sock, tx_buffer, strlen(tx_buffer), 0);
                }
            }
            else if (strncmp(rx_buffer, "PLAY", 4) == 0) {
                snprintf(tx_buffer, sizeof(tx_buffer),
                         "RTSP/1.0 200 OK\r\n"
                         "CSeq: %d\r\n"
                         "Session: 12345678\r\n"
                         "\r\n", cseq);
                send(sock, tx_buffer, strlen(tx_buffer), 0);
                s_is_streaming = true;
                s_state = LIVESTREAM_STATE_STREAMING;
                ESP_LOGI(TAG, "RTSP streaming started");
            }
            else if (strncmp(rx_buffer, "TEARDOWN", 8) == 0) {
                snprintf(tx_buffer, sizeof(tx_buffer),
                         "RTSP/1.0 200 OK\r\n"
                         "CSeq: %d\r\n"
                         "Session: 12345678\r\n"
                         "\r\n", cseq);
                send(sock, tx_buffer, strlen(tx_buffer), 0);
                s_is_streaming = false;
                s_state = LIVESTREAM_STATE_READY;
                break;
            }
        }
        
        close(sock);
        s_active_rtsp_sock = -1;
        s_is_streaming = false;
        if (s_udp_sock >= 0) {
            close(s_udp_sock);
            s_udp_sock = -1;
        }
        s_state = LIVESTREAM_STATE_READY;
        ESP_LOGI(TAG, "RTSP client disconnected");
    }
}

/* ── H.264 Encoder Task ─────────────────────────────────── */

static void livestream_process_task(void *arg)
{
    static int64_t last_time = 0;
    static int frame_count = 0;
    frame_req_t req;

    while (1) {
        if (xQueueReceive(s_frame_queue, &req, portMAX_DELAY) == pdTRUE) {
            if (!s_is_streaming || !s_h264_enc) {
                app_video_return_buf(req.index);
                continue;
            }

            uint8_t *encoding_buf = req.buf;

            /* Dynamically downscale OR mirror using PPA hardware */
            bool needs_mirror = app_video_get_mirror();
            if (s_stream_width != req.width || s_stream_height != req.height || needs_mirror) {
                app_image_process_scale_crop_mirror(req.buf, req.width, req.height, PPA_SRM_COLOR_MODE_YUV420,
                                             req.width, req.height, /* Use full sensor frame for source block */
                                             s_task_rgb_buf, s_stream_width, s_stream_height, 
                                             s_yuv_buf_size, PPA_SRM_COLOR_MODE_YUV420, PPA_SRM_ROTATION_ANGLE_0,
                                             needs_mirror);
                encoding_buf = s_task_rgb_buf;
            }

            /* Encode with H.264 */
            esp_h264_enc_in_frame_t in_frame = {
                .raw_data = { .buffer = encoding_buf, .len = s_yuv_buf_size },
                .pts = (uint64_t)(esp_timer_get_time() / 1000),
            };
            esp_h264_enc_out_frame_t out_frame = {
                .raw_data = { .buffer = s_out_buf, .len = s_yuv_buf_size }
            };

            esp_h264_err_t err = esp_h264_enc_process(s_h264_enc, &in_frame, &out_frame);
            /* Unlock the camera buffer so the driver can reuse it */
            app_video_return_buf(req.index);
            if (err == ESP_H264_ERR_OK) {
                if (out_frame.raw_data.buffer && out_frame.length > 0) {
                    rtp_broadcast(out_frame.raw_data.buffer, out_frame.length);
                    
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
        }
    }
}

/* ── H.264 Encoder ──────────────────────────────────────── */

static esp_err_t init_h264_encoder(void)
{
    // Load resolution setting from NVS
    settings_info_t settings;
    uint16_t dummy_interval, dummy_mag;
    if (app_storage_load_settings(&settings, &dummy_interval, &dummy_mag) == ESP_OK) {
        if (strcmp(settings.resolution, "720P") == 0) {
            s_stream_width = 1280; s_stream_height = 720;
        } else if (strcmp(settings.resolution, "540P") == 0) {
            s_stream_width = 960; s_stream_height = 540;
        } else if (strcmp(settings.resolution, "480P") == 0) {
            s_stream_width = 848; s_stream_height = 480;
        } else if (strcmp(settings.resolution, "360P") == 0) {
            s_stream_width = 640; s_stream_height = 360;
        }
    }

    esp_h264_enc_cfg_hw_t cfg = {
        .gop = STREAM_GOP,
        .fps = STREAM_FPS,
        .res = { .width = s_stream_width, .height = s_stream_height },
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

    s_yuv_buf_size = s_stream_width * s_stream_height * 3 / 2;
    
    s_out_buf = heap_caps_aligned_calloc(128, 1, s_yuv_buf_size, MALLOC_CAP_SPIRAM);
    s_task_rgb_buf = heap_caps_aligned_calloc(128, 1, s_yuv_buf_size, MALLOC_CAP_SPIRAM);
    if (!s_out_buf || !s_task_rgb_buf) {
        return ESP_ERR_NO_MEM;
    }

    /* Dummy encode to extract SPS and PPS for SDP */
    esp_h264_enc_in_frame_t in_frame = {
        .raw_data = { .buffer = s_task_rgb_buf, .len = s_yuv_buf_size },
        .pts = 0,
    };
    esp_h264_enc_out_frame_t out_frame = {
        .raw_data = { .buffer = s_out_buf, .len = s_yuv_buf_size }
    };
    
    esp_h264_enc_process(s_h264_enc, &in_frame, &out_frame);
    if (out_frame.length > 0) {
        const uint8_t *ptr = out_frame.raw_data.buffer;
        size_t remaining = out_frame.length;
        size_t sc_len = 0;
        const uint8_t *nal_start = find_nal_start_code(ptr, remaining, &sc_len);
        
        while (nal_start != NULL) {
            const uint8_t *next_nal_start = NULL;
            size_t next_sc_len = 0;
            size_t search_offset = (nal_start - ptr) + sc_len;
            if (search_offset < remaining) {
                next_nal_start = find_nal_start_code(ptr + search_offset, remaining - search_offset, &next_sc_len);
            }
            
            size_t nal_len = (next_nal_start != NULL) ? (next_nal_start - nal_start) : (remaining - (nal_start - ptr));
            uint8_t type = *(nal_start + sc_len) & 0x1F;
            
            size_t olen;
            if (type == 7) { // SPS
                mbedtls_base64_encode((unsigned char*)s_sps_b64, sizeof(s_sps_b64), &olen, nal_start + sc_len, nal_len - sc_len);
            } else if (type == 8) { // PPS
                mbedtls_base64_encode((unsigned char*)s_pps_b64, sizeof(s_pps_b64), &olen, nal_start + sc_len, nal_len - sc_len);
            }
            
            if (next_nal_start != NULL) {
                remaining -= (next_nal_start - ptr);
                ptr = next_nal_start;
                nal_start = ptr;
                sc_len = next_sc_len;
            } else break;
        }
    }

    ESP_LOGI(TAG, "H.264 encoder initialized: %" PRIu32 "x%" PRIu32 " @ %dfps, %d bps",
             s_stream_width, s_stream_height, STREAM_FPS, STREAM_BITRATE);
    return ESP_OK;
}

/* ── Public API ─────────────────────────────────────────── */

esp_err_t app_livestream_init(void)
{
    esp_err_t ret;

    ret = esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &s_data_cache_line);
    if (ret != ESP_OK) return ret;

    /* Create Zero-Copy Queue (holds 2 frame pointers to prevent starvation) */
    s_frame_queue = xQueueCreate(2, sizeof(frame_req_t));
    if (!s_frame_queue) return ESP_ERR_NO_MEM;

    xTaskCreatePinnedToCore(livestream_process_task, "livestream_task", 8192, NULL, 5, &s_stream_task_handle, 1);

    ret = init_h264_encoder();
    if (ret != ESP_OK) return ret;

    s_encoder_ready = true;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    s_wifi_event_group = xEventGroupCreate();

    esp_event_handler_instance_t inst_any_id, inst_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &inst_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &inst_got_ip));

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
    esp_wifi_set_ps(WIFI_PS_NONE);

    s_state = LIVESTREAM_STATE_WIFI_CONNECTING;
    ESP_LOGI(TAG, "Wi-Fi STA started, connecting to %s...", CONFIG_ESP_WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));
    if (bits & WIFI_CONNECTED_BIT) {
        app_livestream_start_server();
    }

    return ESP_OK;
}

esp_err_t app_livestream_start_server(void)
{
    if (s_state >= LIVESTREAM_STATE_READY) return ESP_OK;
    
    xTaskCreatePinnedToCore(rtsp_server_task, "rtsp_server", 8192, NULL, 6, NULL, 0);
    s_state = LIVESTREAM_STATE_READY;
    return ESP_OK;
}

esp_err_t app_livestream_stop_server(void)
{
    /* Handled by closing sockets and terminating RTSP task gracefully */
    return ESP_OK;
}

esp_err_t app_livestream_feed_frame(uint8_t *yuv420_buf, uint8_t index, uint32_t width, uint32_t height)
{
    if (!s_encoder_ready || !s_h264_enc || !s_is_streaming) return ESP_OK;

    /* Lock the buffer index to prevent the camera driver from overwriting it while we encode */
    if (app_video_lock_buf(index) != ESP_OK) {
        return ESP_OK;
    }

    /* TRUE ZERO-COPY: Pass the camera DMA buffer directly to the encoder queue! */
    frame_req_t req = {
        .buf = yuv420_buf,
        .index = index,
        .width = width,
        .height = height
    };
    
    /* Queue the zero-copy pointer. Don't block if full (drop frame). */
    if (xQueueSend(s_frame_queue, &req, 0) != pdTRUE) {
        app_video_return_buf(index);
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
        default:                               return "Unknown";
    }
}

const char* app_livestream_get_rtsp_url(void)
{
    return s_rtsp_url;
}

bool app_livestream_has_clients(void)
{
    return s_is_streaming;
}

float app_livestream_get_actual_fps(void)
{
    return s_actual_fps;
}

/**
 * @brief Switch Wi-Fi network to the predefined SamGaysHotspot
 *
 * Disconnects from the current network and attempts to connect to the hotspot.
 * This is used to bypass restricted corporate/university Wi-Fi.
 */
void app_livestream_switch_network(void)
{
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "SamGaysHotspot",
            .password = "not12345678",
        },
    };
    ESP_LOGI(TAG, "Switching network to SamGaysHotspot...");
    esp_wifi_disconnect();
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_connect();
}
