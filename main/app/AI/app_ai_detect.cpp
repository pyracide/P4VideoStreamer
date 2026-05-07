#include "app_ai_detect.h"
#include "esp_log.h"

static const char *TAG = "app_ai_detect";

esp_err_t app_ai_detect_init(void)
{
    ESP_LOGI(TAG, "AI detect disabled to save memory");
    return ESP_OK;
}

esp_err_t app_ai_detection_init_buffers(size_t cache_line_size)
{
    return ESP_OK;
}

esp_err_t app_ai_detection_process_frame(uint8_t *detect_buf, uint32_t width, uint32_t height, int ai_detect_mode)
{
    return ESP_OK;
}

esp_err_t app_ai_detection_deinit(void)
{
    return ESP_OK;
}

// Stub out other functions just in case they are referenced
esp_err_t app_coco_od_detect(uint16_t *data, int width, int height) { return ESP_OK; }
esp_err_t app_humanface_ai_detect(uint16_t *detect_buf, uint16_t *draw_buf, int width, int height) { return ESP_OK; }
esp_err_t app_pedestrian_ai_detect(uint16_t *detect_buf, uint16_t *draw_buf, int width, int height) { return ESP_OK; }