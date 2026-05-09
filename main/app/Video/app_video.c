/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "app_video.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_video_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "linux/videodev2.h"
#include <fcntl.h>
#include <string.h>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>


static const char *TAG = "app_video";

#define MAX_BUFFER_COUNT (6)
#define MIN_BUFFER_COUNT (2)
#define VIDEO_TASK_STACK_SIZE (4 * 1024)
#define VIDEO_TASK_PRIORITY (6)

typedef struct {
  uint8_t *camera_buffer[MAX_BUFFER_COUNT];
  size_t camera_buf_size;
  uint32_t camera_buf_hes;
  uint32_t camera_buf_ves;
  struct v4l2_buffer v4l2_buf;
  uint8_t camera_mem_mode;
  app_video_frame_operation_cb_t user_camera_video_frame_operation_cb;
  TaskHandle_t video_stream_task_handle;
  bool video_task_delete;
  SemaphoreHandle_t video_stop_sem;
} app_video_t;

static app_video_t app_camera_video;
static int s_video_fd = -1;
static int s_current_index_in_task = -1;
static bool s_buffer_locked[EXAMPLE_CAM_BUF_NUM] = {false};
static bool s_sensor_mirrored = false;

esp_err_t app_video_main(i2c_master_bus_handle_t i2c_bus_handle) {
  const esp_video_init_csi_config_t base_csi_config = {
      .sccb_config =
          {
              .init_sccb = true,
              .i2c_config =
                  {
                      .port = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_PORT,
                      .scl_pin = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_SCL_PIN,
                      .sda_pin = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_SDA_PIN,
                  },
              .freq = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_FREQ,
          },
      .reset_pin = CONFIG_EXAMPLE_MIPI_CSI_CAM_SENSOR_RESET_PIN,
      .pwdn_pin = CONFIG_EXAMPLE_MIPI_CSI_CAM_SENSOR_PWDN_PIN,
  };

  esp_video_init_csi_config_t csi_config = base_csi_config;
  if (i2c_bus_handle != NULL) {
    csi_config.sccb_config.init_sccb = false;
    csi_config.sccb_config.i2c_handle = i2c_bus_handle;
  }

  esp_video_init_config_t cam_config = {
#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR > 0
      .csi = &csi_config,
#endif
  };

  return esp_video_init(&cam_config);
}

int app_video_open(char *dev, video_fmt_t init_fmt) {
  struct v4l2_format default_format;
  struct v4l2_capability capability;
  const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  int fd = open(dev, O_RDONLY);
  if (fd < 0) {
    ESP_LOGE(TAG, "Open video failed");
    return -1;
  }

  if (ioctl(fd, VIDIOC_QUERYCAP, &capability)) {
    ESP_LOGE(TAG, "failed to get capability");
    goto exit_0;
  }

  ESP_LOGI(TAG, "version: %d.%d.%d", (uint16_t)(capability.version >> 16),
           (uint8_t)(capability.version >> 8), (uint8_t)capability.version);
  ESP_LOGI(TAG, "driver:  %s", capability.driver);
  ESP_LOGI(TAG, "card:    %s", capability.card);
  ESP_LOGI(TAG, "bus:     %s", capability.bus_info);

  memset(&default_format, 0, sizeof(struct v4l2_format));
  default_format.type = type;
  if (ioctl(fd, VIDIOC_G_FMT, &default_format) != 0) {
    ESP_LOGE(TAG, "failed to get format");
    goto exit_0;
  }

  ESP_LOGI(TAG, "width=%" PRIu32 " height=%" PRIu32,
           default_format.fmt.pix.width, default_format.fmt.pix.height);

  app_camera_video.camera_buf_hes = 1280;
  app_camera_video.camera_buf_ves = 720;

  struct v4l2_format format = {
      .type = type,
      .fmt.pix.width = 1280, // Force Native 720p
      .fmt.pix.height = 720,
      .fmt.pix.pixelformat = init_fmt,
  };

  if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) {
    ESP_LOGE(TAG, "failed to set format");
    goto exit_0;
  }

  app_camera_video.video_stop_sem = xSemaphoreCreateBinary();
  s_video_fd = fd;

  return fd;
exit_0:
  close(fd);
  return -1;
}

esp_err_t app_video_set_bufs(int video_fd, uint32_t fb_num, const void **fb) {
  if (fb_num > MAX_BUFFER_COUNT) {
    ESP_LOGE(TAG, "buffer num is too large");
    return ESP_FAIL;
  } else if (fb_num < MIN_BUFFER_COUNT) {
    ESP_LOGE(TAG, "At least two buffers are required");
    return ESP_FAIL;
  }

  struct v4l2_requestbuffers req;
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  memset(&req, 0, sizeof(req));
  req.count = fb_num;
  req.type = type;

  app_camera_video.camera_mem_mode = req.memory =
      fb ? V4L2_MEMORY_USERPTR : V4L2_MEMORY_MMAP;

  if (ioctl(video_fd, VIDIOC_REQBUFS, &req) != 0) {
    ESP_LOGE(TAG, "req bufs failed");
    goto errout_req_bufs;
  }
  for (int i = 0; i < fb_num; i++) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = type;
    buf.memory = req.memory;
    buf.index = i;

    if (ioctl(video_fd, VIDIOC_QUERYBUF, &buf) != 0) {
      ESP_LOGE(TAG, "query buf failed");
      goto errout_req_bufs;
    }

    if (req.memory == V4L2_MEMORY_MMAP) {
      app_camera_video.camera_buffer[i] =
          mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, video_fd,
               buf.m.offset);
      if (app_camera_video.camera_buffer[i] == NULL) {
        ESP_LOGE(TAG, "mmap failed");
        goto errout_req_bufs;
      }
    } else {
      if (!fb[i]) {
        ESP_LOGE(TAG, "frame buffer is NULL");
        goto errout_req_bufs;
      }
      buf.m.userptr = (unsigned long)fb[i];
      app_camera_video.camera_buffer[i] = (uint8_t *)fb[i];
    }

    app_camera_video.camera_buf_size = buf.length;

    if (ioctl(video_fd, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "queue frame buffer failed");
      goto errout_req_bufs;
    }
  }

  return ESP_OK;

errout_req_bufs:
  close(video_fd);
  return ESP_FAIL;
}

esp_err_t app_video_get_bufs(int fb_num, void **fb) {
  if (fb_num > MAX_BUFFER_COUNT) {
    ESP_LOGE(TAG, "buffer num is too large");
    return ESP_FAIL;
  } else if (fb_num < MIN_BUFFER_COUNT) {
    ESP_LOGE(TAG, "At least two buffers are required");
    return ESP_FAIL;
  }

  for (int i = 0; i < fb_num; i++) {
    if (app_camera_video.camera_buffer[i] != NULL) {
      fb[i] = app_camera_video.camera_buffer[i];
    } else {
      ESP_LOGE(TAG, "frame buffer is NULL");
      return ESP_FAIL;
    }
  }

  return ESP_OK;
}

uint32_t app_video_get_buf_size(void) {
  uint32_t buf_size = app_camera_video.camera_buf_hes *
                      app_camera_video.camera_buf_ves *
                      (APP_VIDEO_FMT == APP_VIDEO_FMT_RGB565 ? 2 : 3);

  return buf_size;
}

static inline esp_err_t video_receive_video_frame(int video_fd) {
  memset(&app_camera_video.v4l2_buf, 0, sizeof(app_camera_video.v4l2_buf));
  app_camera_video.v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  app_camera_video.v4l2_buf.memory = app_camera_video.camera_mem_mode;

  int res = ioctl(video_fd, VIDIOC_DQBUF, &(app_camera_video.v4l2_buf));
  if (res != 0) {
    ESP_LOGE(TAG, "failed to receive video frame");
    goto errout;
  }

  return ESP_OK;

errout:
  return ESP_FAIL;
}

static inline void video_operation_video_frame(int video_fd) {
  app_camera_video.v4l2_buf.m.userptr =
      (unsigned long)
          app_camera_video.camera_buffer[app_camera_video.v4l2_buf.index];
  app_camera_video.v4l2_buf.length = app_camera_video.camera_buf_size;

  uint8_t buf_index = app_camera_video.v4l2_buf.index;

  app_camera_video.user_camera_video_frame_operation_cb(
      app_camera_video.camera_buffer[buf_index], buf_index,
      app_camera_video.camera_buf_hes, app_camera_video.camera_buf_ves,
      app_camera_video.camera_buf_size);
}

static inline esp_err_t video_free_video_frame(int video_fd) {
  if (ioctl(video_fd, VIDIOC_QBUF, &(app_camera_video.v4l2_buf)) != 0) {
    ESP_LOGE(TAG, "failed to free video frame");
    goto errout;
  }

  return ESP_OK;

errout:
  return ESP_FAIL;
}

static inline esp_err_t video_stream_start(int video_fd) {
  ESP_LOGI(TAG, "Video Stream Start");

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(video_fd, VIDIOC_STREAMON, &type)) {
    ESP_LOGE(TAG, "failed to start stream");
    goto errout;
  }

  struct v4l2_format format = {0};
  format.type = type;
  if (ioctl(video_fd, VIDIOC_G_FMT, &format) != 0) {
    ESP_LOGE(TAG, "get fmt failed");
    goto errout;
  }

  return ESP_OK;

errout:
  return ESP_FAIL;
}

static inline esp_err_t video_stream_stop(int video_fd) {
  ESP_LOGI(TAG, "Video Stream Stop");

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(video_fd, VIDIOC_STREAMOFF, &type)) {
    ESP_LOGE(TAG, "failed to stop stream");
    goto errout;
  }

  return ESP_OK;

errout:
  return ESP_FAIL;
}

static void video_stream_task(void *arg) {
  int video_fd = *((int *)arg);

  while (1) {
    ESP_ERROR_CHECK(video_receive_video_frame(video_fd));
    uint8_t index = app_camera_video.v4l2_buf.index;

    /* Mark this buffer as active in the task to prevent 'Smart Return' races */
    s_current_index_in_task = index;

    video_operation_video_frame(video_fd);

    /* Hand-over point: Task is done with callbacks.
       If the buffer is still locked, the livestream task now owns it. */
    s_current_index_in_task = -1;

    if (!s_buffer_locked[index]) {
      ESP_ERROR_CHECK(video_free_video_frame(video_fd));
    }

    if (app_camera_video.video_task_delete) {
      app_camera_video.video_task_delete = false;
      ESP_ERROR_CHECK(video_stream_stop(video_fd));
      xSemaphoreGive(app_camera_video.video_stop_sem);
      vTaskDelete(NULL);
    }
  }
  vTaskDelete(NULL);
}

esp_err_t app_video_stream_task_start(int video_fd, int core_id) {
  video_stream_start(video_fd);

  BaseType_t result = xTaskCreatePinnedToCore(
      video_stream_task, "video stream task", VIDEO_TASK_STACK_SIZE, &video_fd,
      VIDEO_TASK_PRIORITY, &app_camera_video.video_stream_task_handle, core_id);

  if (result != pdPASS) {
    ESP_LOGE(TAG, "failed to create video stream task");
    goto errout;
  }

  return ESP_OK;

errout:
  video_stream_stop(video_fd);
  return ESP_FAIL;
}

esp_err_t app_video_stream_task_stop(int video_fd) {
  app_camera_video.video_task_delete = true;

  return ESP_OK;
}

esp_err_t app_video_wait_video_stop(void) {
  return xSemaphoreTake(app_camera_video.video_stop_sem, portMAX_DELAY);
}

esp_err_t app_video_register_frame_operation_cb(
    app_video_frame_operation_cb_t operation_cb) {
  app_camera_video.user_camera_video_frame_operation_cb = operation_cb;

  return ESP_OK;
}

esp_err_t app_video_close(int video_fd) {
  s_video_fd = -1;
  close(video_fd);
  return ESP_OK;
}

esp_err_t app_video_lock_buf(uint8_t index) {
  if (index >= EXAMPLE_CAM_BUF_NUM)
    return ESP_ERR_INVALID_ARG;
  s_buffer_locked[index] = true;
  return ESP_OK;
}

esp_err_t app_video_return_buf(uint8_t index) {
  if (index >= EXAMPLE_CAM_BUF_NUM)
    return ESP_ERR_INVALID_ARG;

  /* Clear the lock flag */
  s_buffer_locked[index] = false;

  /* SMART RETURN: If the main video task is currently processing this exact
     index, it will handle the return (QBUF) itself once it finishes. We only
     call ioctl if the task has already moved past this frame. */
  if (index == s_current_index_in_task) {
    return ESP_OK;
  }

  if (s_video_fd < 0)
    return ESP_OK;

  struct v4l2_buffer v4l2_buf;
  memset(&v4l2_buf, 0, sizeof(v4l2_buf));
  v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2_buf.memory = app_camera_video.camera_mem_mode;
  v4l2_buf.index = index;

  if (ioctl(s_video_fd, VIDIOC_QBUF, &v4l2_buf) != 0) {
    /* This can happen if the task returned it just as we checked
       s_current_index_in_task. In that case, it's already queued, so we can
       ignore the error. */
    return ESP_OK;
  }
  return ESP_OK;
}

esp_err_t app_video_set_mirror(bool enable) {
  s_sensor_mirrored = enable;
  ESP_LOGI(TAG, "Hardware PPA Mirror %s", enable ? "Enabled" : "Disabled");
  return ESP_OK;
}

bool app_video_get_mirror(void) { return s_sensor_mirrored; }