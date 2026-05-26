#include "bsp/esp-bsp.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>

#include "app_album.h"
#include "app_control.h"
#include "app_livestream.h"
#include "app_video.h"
#include "app_video_stream.h"
#include "ui_extra.h"

#include "driver/i2c_master.h"

/* Private definitions */
static const char *TAG = "app_control";

/* DRV2605L Haptic Variables */
static i2c_master_dev_handle_t s_drv2605_dev = NULL;
static esp_timer_handle_t s_haptic_timer = NULL;

typedef struct {
  uint8_t effect;
  const char *name;
  uint32_t period_ms;
} haptic_test_state_t;

static const haptic_test_state_t s_haptic_states[] = {
    {1, "Proximity Phase 1 (Very Slow Clicks 800ms)", 800},
    {1, "Proximity Phase 2 (Slow Clicks 600ms)", 600},
    {1, "Proximity Phase 3 (Medium Clicks 400ms)", 400},
    {1, "Proximity Phase 4 (Fast Clicks 200ms)", 200},
    {1, "Proximity Phase 5 (Ultra Fast Clicks 100ms)", 100},
    {55, "Effect 55 (Transition Hum)", 0},
    {12, "Effect 12 (Triple Click 100%)", 0},
    {15, "Effect 15 (Strong Buzz 100%)", 0},
    {64, "Effect 64 (Transient Click 100%)", 0}};
static const int NUM_HAPTIC_STATES =
    sizeof(s_haptic_states) / sizeof(s_haptic_states[0]);
static int s_current_haptic_state = -1; // starts at -1 so first press goes to 0

static esp_timer_handle_t s_double_press_timer = NULL;
static esp_timer_handle_t s_proxim_watchdog_timer = NULL;
static esp_timer_handle_t s_hand_lost_timer = NULL;
static int s_current_proxim = -1;
static bool s_hand_tracked = false;

static void play_haptic_effect(uint8_t effect, uint32_t period_ms);

static esp_err_t drv_write_reg(uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  esp_err_t err = i2c_master_transmit(s_drv2605_dev, buf, 2, 1000);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "DRV2605L I2C write error reg 0x%02X: %s", reg,
             esp_err_to_name(err));
  }
  return err;
}

static uint8_t drv_read_reg(uint8_t reg) {
  uint8_t read_buf[1] = {0};
  if (i2c_master_transmit_receive(s_drv2605_dev, &reg, 1, read_buf, 1, 1000) !=
      ESP_OK) {
    ESP_LOGE(TAG, "DRV2605L I2C read error reg 0x%02X", reg);
  }
  return read_buf[0];
}

static void haptic_timer_cb(void *arg) {
  if (s_drv2605_dev) {
    drv_write_reg(0x0C, 0x01); // GO
  }
}

static void stop_haptic_timer(void) {
  if (s_haptic_timer != NULL) {
    esp_timer_stop(s_haptic_timer);
    esp_timer_delete(s_haptic_timer);
    s_haptic_timer = NULL;
  }
}

static void stop_hand_lost_timer(void) {
  if (s_hand_lost_timer != NULL) {
    esp_timer_stop(s_hand_lost_timer);
    esp_timer_delete(s_hand_lost_timer);
    s_hand_lost_timer = NULL;
  }
}

static void hand_lost_cutoff_cb(void *arg) {
  // 500ms elapsed since HAND_LOST — cut off the pulsing
  ESP_LOGI(TAG, "Hand-lost cutoff: stopping haptic pulse.");
  stop_haptic_timer();
  // Also halt DRV2605L hardware playback
  if (s_drv2605_dev) {
    drv_write_reg(0x0C, 0x00);
  }
  s_hand_lost_timer = NULL;
}

static void proxim_watchdog_cb(void *arg) {
  // 500ms without PROXIM packets, stop continuous pulsing
  ESP_LOGI(TAG, "Proximity watchdog timeout, stopping haptics.");
  stop_haptic_timer();

  if (s_hand_tracked) {
    // Use same approach as HAND_LOST: fast repeating pulse for 500ms
    play_haptic_effect(1, 250);
    stop_hand_lost_timer();
    esp_timer_create_args_t args = {.callback = &hand_lost_cutoff_cb,
                                    .name = "hand_lost_cut"};
    esp_timer_create(&args, &s_hand_lost_timer);
    esp_timer_start_once(s_hand_lost_timer, 500000); // 500ms
  }

  s_current_proxim = -1;
  s_hand_tracked = false;
}

static bool s_drv_init_failed =
    false; // Tracks permanent init failure to avoid spamming logs

static void init_drv2605l_safe(void) {
  if (s_drv2605_dev != NULL)
    return;
  if (s_drv_init_failed)
    return; // Don't retry if we already diagnosed a hardware fault

  i2c_master_bus_handle_t bus_handle;
  if (bsp_get_i2c_bus_handle(&bus_handle) != ESP_OK) {
    ESP_LOGE(TAG, "DRV2605L DIAG: Failed to get I2C bus handle");
    s_drv_init_failed = true;
    return;
  }

  ESP_LOGW(TAG, "======== DRV2605L I2C DIAGNOSTIC START ========");

  // Step 1: Reset the I2C bus (sends 9 SCL clock pulses to unstick any held SDA
  // line)
  ESP_LOGI(TAG, "DRV2605L DIAG [1/4]: Resetting I2C bus (9 SCL pulses)...");
  esp_err_t reset_err = i2c_master_bus_reset(bus_handle);
  if (reset_err != ESP_OK) {
    ESP_LOGE(TAG, "DRV2605L DIAG: Bus reset failed: %s",
             esp_err_to_name(reset_err));
  } else {
    ESP_LOGI(TAG, "DRV2605L DIAG: Bus reset OK");
  }
  vTaskDelay(pdMS_TO_TICKS(50)); // Let the bus settle after reset

  // Step 2: Probe for DRV2605L at address 0x5A with retries
  ESP_LOGI(TAG, "DRV2605L DIAG [2/4]: Probing address 0x5A (DRV2605L)...");
  bool found_drv = false;
  for (int attempt = 0; attempt < 3; attempt++) {
    esp_err_t probe_err = i2c_master_probe(bus_handle, 0x5A, 500);
    if (probe_err == ESP_OK) {
      ESP_LOGI(TAG, "DRV2605L DIAG: Device FOUND at 0x5A on attempt %d!",
               attempt + 1);
      found_drv = true;
      break;
    } else {
      ESP_LOGW(TAG, "DRV2605L DIAG: Probe 0x5A attempt %d failed: %s",
               attempt + 1, esp_err_to_name(probe_err));
      // Try another bus reset between retries
      i2c_master_bus_reset(bus_handle);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  // Step 3: Full I2C bus scan to see what devices ARE reachable
  ESP_LOGI(TAG, "DRV2605L DIAG [3/4]: Scanning full I2C bus (0x08-0x77)...");
  int devices_found = 0;
  for (uint16_t addr = 0x08; addr <= 0x77; addr++) {
    esp_err_t scan_err = i2c_master_probe(bus_handle, addr, 100);
    if (scan_err == ESP_OK) {
      const char *name = "unknown";
      if (addr == 0x5A)
        name = "DRV2605L (haptic)";
      else if (addr == 0x12 || addr == 0x13)
        name = "QMA6100 (IMU)";
      else if (addr == 0x30)
        name = "OV2710 (camera)";
      else if (addr == 0x36)
        name = "OV2710 (camera alt)";
      ESP_LOGI(TAG, "  I2C SCAN: Device found at 0x%02X (%s)", addr, name);
      devices_found++;
    }
  }
  ESP_LOGI(TAG, "DRV2605L DIAG: Bus scan complete. %d device(s) found.",
           devices_found);

  // Step 4: Verdict
  if (!found_drv) {
    ESP_LOGE(TAG, "======== DRV2605L DIAGNOSTIC VERDICT ========");
    ESP_LOGE(TAG, "  DRV2605L at 0x5A is NOT responding on the I2C bus.");
    ESP_LOGE(TAG, "  The I2C bus itself is %s (found %d other device(s)).",
             devices_found > 0 ? "WORKING" : "DEAD or DISCONNECTED",
             devices_found);
    if (devices_found > 0) {
      ESP_LOGE(TAG, "  CONCLUSION: The SDA/SCL/VCC/GND wiring between the");
      ESP_LOGE(TAG, "  ESP32-P4 board and the DRV2605L breakout is broken.");
      ESP_LOGE(TAG, "  Other I2C devices on the bus are fine.");
    } else {
      ESP_LOGE(TAG, "  CONCLUSION: The entire I2C bus appears dead.");
      ESP_LOGE(TAG, "  Check main SDA (GPIO14) and SCL (GPIO13) connections.");
    }
    ESP_LOGE(TAG, "  Haptic motor will be DISABLED for this session.");
    ESP_LOGE(TAG, "==============================================");
    s_drv_init_failed = true;
    return;
  }

  // If we got here, the device responded to the probe. Proceed with normal
  // init.
  ESP_LOGI(TAG, "DRV2605L DIAG [4/4]: Device reachable! Proceeding with "
                "initialization...");

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = 0x5A,
      .scl_speed_hz = 400000,
  };
  if (i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_drv2605_dev) !=
      ESP_OK) {
    ESP_LOGE(TAG, "DRV2605L DIAG: Failed to add device to bus (driver error)");
    s_drv_init_failed = true;
    return;
  }

  ESP_LOGI(TAG, "Initializing DRV2605L...");

  // NVS storage keys for auto-calibration
  const char *nvs_ns = "drv_calib";
  nvs_handle_t nvs_handle;
  uint8_t comp = 0, bemf = 0, fb = 0, calibrated = 0;

  // Load from NVS
  if (nvs_open(nvs_ns, NVS_READONLY, &nvs_handle) == ESP_OK) {
    nvs_get_u8(nvs_handle, "calibrated", &calibrated);
    if (calibrated) {
      nvs_get_u8(nvs_handle, "comp", &comp);
      nvs_get_u8(nvs_handle, "bemf", &bemf);
      nvs_get_u8(nvs_handle, "fb", &fb);
    }
    nvs_close(nvs_handle);
  }

  // Wake up chip — verify the first write actually succeeds
  esp_err_t wake_err = drv_write_reg(0x01, 0x00);
  if (wake_err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "DRV2605L DIAG: Wake-up write failed even after successful probe!");
    ESP_LOGE(TAG, "  This suggests the device ACKs its address but NACKs "
                  "register writes.");
    ESP_LOGE(TAG, "  Possible causes: wrong VCC voltage, or chip is damaged.");
    s_drv_init_failed = true;
    // Clean up the device handle since init failed
    i2c_master_bus_rm_device(s_drv2605_dev);
    s_drv2605_dev = NULL;
    return;
  }

  if (calibrated) {
    ESP_LOGI(TAG,
             "Using saved DRV2605L Auto-Calibration: Comp=0x%02X, BEMF=0x%02X, "
             "FB=0x%02X",
             comp, bemf, fb);
    drv_write_reg(0x18, comp);
    drv_write_reg(0x19, bemf);
    drv_write_reg(0x1A, fb);
  } else {
    ESP_LOGI(TAG, "No calibration found. Running DRV2605L Auto-Calibration...");

    // Setup Rated Voltage & Clamp
    drv_write_reg(0x16, 0x7D);
    drv_write_reg(0x17, 0x7D);

    // Put in Auto-calibration mode
    drv_write_reg(0x01, 0x07);

    // Start calibration
    drv_write_reg(0x0C, 0x01); // GO

    // Poll GO register until calibration finishes
    bool success = false;
    for (int i = 0; i < 200; i++) {
      vTaskDelay(pdMS_TO_TICKS(10));
      if ((drv_read_reg(0x0C) & 0x01) == 0) {
        // Done! Check status for error bit (bit 3)
        if ((drv_read_reg(0x00) & 0x08) == 0) {
          success = true;
        }
        break;
      }
    }

    if (success) {
      comp = drv_read_reg(0x18);
      bemf = drv_read_reg(0x19);
      fb = drv_read_reg(0x1A);
      ESP_LOGI(TAG, "Calibration SUCCESS. Comp=0x%02X, BEMF=0x%02X, FB=0x%02X",
               comp, bemf, fb);

      // Save to NVS
      if (nvs_open(nvs_ns, NVS_READWRITE, &nvs_handle) == ESP_OK) {
        nvs_set_u8(nvs_handle, "comp", comp);
        nvs_set_u8(nvs_handle, "bemf", bemf);
        nvs_set_u8(nvs_handle, "fb", fb);
        nvs_set_u8(nvs_handle, "calibrated", 1);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
      }
    } else {
      ESP_LOGE(TAG, "Calibration FAILED or timed out.");
      // Fallback to manual LRA mode setup if it fails
      uint8_t current_fb = drv_read_reg(0x1A);
      drv_write_reg(0x1A, current_fb | 0x80);
    }

    // Put back in standby, then wake in active mode
    drv_write_reg(0x01, 0x00);
  }

  // Select LRA ROM library 6
  drv_write_reg(0x03, 0x06);

  // Apply Voltage
  drv_write_reg(0x16, 0x7D);
  drv_write_reg(0x17, 0x7D);

  ESP_LOGI(TAG, "======== DRV2605L INIT COMPLETE ========");
}

static void play_haptic_effect(uint8_t effect, uint32_t period_ms) {
  init_drv2605l_safe();
  stop_haptic_timer();

  if (s_drv2605_dev == NULL)
    return;

  // Stop any in-progress DRV2605L playback and let the LRA brake
  drv_write_reg(0x0C, 0x00);     // Clear GO bit — halt current waveform
  drv_write_reg(0x01, 0x00);     // Ensure mode is Internal Trigger (active)
  vTaskDelay(pdMS_TO_TICKS(10)); // Let LRA settle before new effect

  drv_write_reg(0x04, effect);
  drv_write_reg(0x05, 0x00);
  drv_write_reg(0x0C, 0x01);

  if (period_ms > 0) {
    esp_timer_create_args_t timer_args = {.callback = &haptic_timer_cb,
                                          .name = "haptic_cont"};
    esp_timer_create(&timer_args, &s_haptic_timer);
    esp_timer_start_periodic(s_haptic_timer, period_ms * 1000);
  }
}

static void fire_haptic_effect(void) {
  const haptic_test_state_t *state = &s_haptic_states[s_current_haptic_state];
  ESP_LOGI(TAG,
           "=== Haptic Test Phase %d/%d: %s ===", s_current_haptic_state + 1,
           NUM_HAPTIC_STATES, state->name);
  play_haptic_effect(state->effect, state->period_ms);
}

static void double_press_timeout_cb(void *arg) {
  // 300ms has elapsed without a second press. Safe to advance and fire!
  s_current_haptic_state = (s_current_haptic_state + 1) % NUM_HAPTIC_STATES;
  fire_haptic_effect();
}

static void udp_haptic_server_task(void *pvParameters) {
  char rx_buffer[128];
  int port = 8282;

  if (s_proxim_watchdog_timer == NULL) {
    esp_timer_create_args_t wd_args = {.callback = &proxim_watchdog_cb,
                                       .name = "proxim_wd"};
    esp_timer_create(&wd_args, &s_proxim_watchdog_timer);
  }

  while (1) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
      ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
      close(sock);
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    ESP_LOGI(TAG, "Haptic UDP server listening on port %d", port);

    while (1) {
      struct sockaddr_storage source_addr;
      socklen_t socklen = sizeof(source_addr);
      int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                         (struct sockaddr *)&source_addr, &socklen);

      if (len < 0) {
        ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        break;
      }

      rx_buffer[len] = 0; // Null-terminate

      // Parse Commands
      if (strncmp(rx_buffer, "HAND_DETECTED", 13) == 0) {
        if (s_hand_tracked) {
          continue; // Ignore duplicate packets in the burst
        }
        ESP_LOGI(TAG, "UDP Command: HAND_DETECTED");
        esp_timer_stop(s_proxim_watchdog_timer); // clear watchdog
        stop_hand_lost_timer(); // cancel any active hand-lost pulsing
        s_hand_tracked = true;
        s_current_proxim = -1;     // reset state

        // Phase 2: Force stop and brief settle delay before starting new effect
        if (s_drv2605_dev) {
          drv_write_reg(0x0C, 0x00);
        }
        vTaskDelay(pdMS_TO_TICKS(50));

        play_haptic_effect(55, 0); // Transition Hum
      } else if (strncmp(rx_buffer, "HAND_LOST", 9) == 0) {
        if (!s_hand_tracked) {
          continue; // Ignore duplicate packets in the burst
        }
        ESP_LOGI(TAG, "UDP Command: HAND_LOST");
        esp_timer_stop(s_proxim_watchdog_timer);

        // Phase 2: Stop timer and clear GO bit, then delay 100ms to let physical mass settle
        stop_haptic_timer();
        if (s_drv2605_dev) {
          drv_write_reg(0x0C, 0x00);
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        s_hand_tracked = false;
        s_current_proxim = -1; // sync state to avoid double-trigger

        // Fast repeating pulse (same as PROXIM:3) for 500ms then cut
        play_haptic_effect(1, 250);
        stop_hand_lost_timer();
        esp_timer_create_args_t args = {.callback = &hand_lost_cutoff_cb,
                                        .name = "hand_lost_cut"};
        esp_timer_create(&args, &s_hand_lost_timer);
        esp_timer_start_once(s_hand_lost_timer, 500000); // 500ms cutoff
      } else if (strncmp(rx_buffer, "PROXIM:", 7) == 0) {
        if (!s_hand_tracked) {
          continue; // Ignore proximity updates if the hand is not tracked
        }
        int dist = rx_buffer[7] - '0'; // parse 0-3

        // Restart watchdog (must stop first to reset an active esp_timer)
        esp_timer_stop(s_proxim_watchdog_timer);
        esp_timer_start_once(s_proxim_watchdog_timer, 500000); // 500ms

        if (dist != s_current_proxim) {
          int prev_proxim = s_current_proxim;
          s_current_proxim = dist;
          if (dist == 0) {
            stop_haptic_timer(); // Stop continuous effect
            if (prev_proxim > 0) {
              play_haptic_effect(
                  64,
                  0); // Lost hand effect (only if we were actively tracking)
            }
          } else if (dist == 1) {
            stop_haptic_timer(); // Overkill fix: distance 1 is COMPLETELY
                                 // silent, but keeps tracker active (>0)
          } else {
            int period = 0;
            if (dist == 2)
              period = 500; // 500ms ping
            else if (dist == 3)
              period = 250; // 250ms ping

            if (period > 0) {
              play_haptic_effect(1, period); // ROM Effect 1 (Proximity Phase)
            }
          }
        }
      }
    }

    if (sock != -1) {
      close(sock);
    }
  }
  vTaskDelete(NULL);
}

static void trigger_haptic_buzz(void) {
  init_drv2605l_safe();

  if (s_drv2605_dev) {
    if (s_double_press_timer != NULL &&
        esp_timer_is_active(s_double_press_timer)) {
      // A second press happened before the 300ms timer fired!
      // Cancel the timer so we don't advance.
      esp_timer_stop(s_double_press_timer);
      esp_timer_delete(s_double_press_timer);
      s_double_press_timer = NULL;

      if (s_current_haptic_state != -1) {
        ESP_LOGI(TAG,
                 ">>> Double-press detected! Repeating current effect... <<<");
        fire_haptic_effect();
      }
    } else {
      // First press. Start the 300ms waiting window.
      if (s_double_press_timer != NULL) {
        esp_timer_delete(s_double_press_timer);
      }

      esp_timer_create_args_t dp_timer_args = {
          .callback = &double_press_timeout_cb, .name = "double_press_wait"};
      esp_timer_create(&dp_timer_args, &s_double_press_timer);
      esp_timer_start_once(s_double_press_timer,
                           300000); // 300ms in microseconds
    }
  }
}

/* Button related variables */
static button_handle_t btns[BSP_BUTTON_NUM];

/* Knob related variables */
static int knob_step_counter = 0;
static int knob_last_direction = 0;     // 0: no direction, 1: right, -1: left
static int64_t knob_last_time = 0;      // timestamp of last rotation
static const int knob_timeout_ms = 500; // timeout in milliseconds
static int knob_step_threshold = 3;     // threshold for knob step counter
static bool s_display_suspended = false;

/* Private function prototypes */
static void btn_handler(void *arg, void *data);
static void handle_knob_rotation(int direction, void (*action_camera)(void),
                                 void (*action_main)(void),
                                 void (*action_settings)(void));

/* Helper functions for camera actions */
static void camera_decrease_magnification(void) {
  app_extra_set_magnification_factor(app_extra_get_magnification_factor() - 1);
}

static void camera_increase_magnification(void) {
  app_extra_set_magnification_factor(app_extra_get_magnification_factor() + 1);
}

/* Button handler implementation */
static void btn_handler(void *arg, void *data) {
  int button_id = (int)data;

  bsp_display_lock(0);

  if (ui_extra_handle_usb_disk_page()) {
    bsp_display_unlock();
    return;
  }

  switch (button_id) {
  case BSP_BUTTON_1:
    if (s_display_suspended) {
      bsp_display_unlock();
      return;
    }
    ui_extra_btn_menu();
    break;

  case BSP_BUTTON_2:
    /* Network Switch Button: Disconnect and switch to Hotspot */
    if (ui_extra_get_current_page() == UI_PAGE_LIVESTREAM) {
      app_livestream_switch_network();
      ESP_LOGI(TAG, "Network switch triggered by button 2");
    } else {
      /* Normal 'Up' button behavior for other pages */
      ui_extra_btn_up();
      if (ui_extra_get_current_page() == UI_PAGE_ALBUM &&
          lv_obj_has_flag(ui_PanelImageScreenAlbumDelete, LV_OBJ_FLAG_HIDDEN)) {
        app_album_prev_image();
      }
    }
    break;

  case BSP_BUTTON_3:
    ESP_LOGI(TAG,
             "BSP_BUTTON_3 (Down button) pressed. Current page: %d, Livestream "
             "page ID: %d",
             ui_extra_get_current_page(), UI_PAGE_LIVESTREAM);

    if (s_display_suspended) {
      bsp_display_unlock();
      return;
    }
    /* Trigger haptic test if in Livestream mode */
    if (ui_extra_get_current_page() == UI_PAGE_LIVESTREAM) {
      ESP_LOGI(TAG, "Triggering haptic buzz...");
      trigger_haptic_buzz();
    } else {
      /* Normal 'Down' behavior for other pages */
      ui_extra_btn_down();
      if (ui_extra_get_current_page() == UI_PAGE_ALBUM &&
          lv_obj_has_flag(ui_PanelImageScreenAlbumDelete, LV_OBJ_FLAG_HIDDEN)) {
        app_album_next_image();
      }
    }
    break;

  case BSP_BUTTON_ED:
    if (s_display_suspended) {
      bsp_display_unlock();
      return;
    }
    ui_extra_btn_encoder();
    break;

  default:
    ESP_LOGW(TAG, "Unknown button ID: %d", button_id);
    break;
  }

  bsp_display_unlock();
}

/* Helper function to handle knob rotation */
static void handle_knob_rotation(int direction, void (*action_camera)(void),
                                 void (*action_main)(void),
                                 void (*action_settings)(void)) {
  if (s_display_suspended) {
    return;
  }
  if (ui_extra_get_current_page() == UI_PAGE_ALBUM ||
      ui_extra_get_current_page() == UI_PAGE_USB_DISK) {
    return;
  }

  int64_t current_time =
      esp_timer_get_time() / 1000; // get current time in milliseconds

  // Check for timeout or direction change
  if (current_time - knob_last_time > knob_timeout_ms ||
      knob_last_direction == -direction) {
    // Timeout or direction change, reset counter
    knob_step_counter = 0;
    knob_last_direction = direction;
  }

  // Increment step counter
  knob_step_counter++;
  knob_last_time = current_time;

  // Trigger action when accumulated steps reach threshold
  if (knob_step_counter >= knob_step_threshold) {
    knob_step_counter = 0; // Reset counter

    ESP_LOGD(TAG, "Continuous rotation detected: %d steps, value %d",
             knob_step_counter, direction);

    bsp_display_lock(0);
    if (ui_extra_get_current_page() == UI_PAGE_LIVESTREAM ||
        ui_extra_get_current_page() == UI_PAGE_INTERVAL_CAM ||
        ui_extra_get_current_page() == UI_PAGE_VIDEO_MODE ||
        ui_extra_get_current_page() == UI_PAGE_AI_DETECT) {
      action_camera();
    } else if (ui_extra_get_current_page() == UI_PAGE_MAIN) {
      action_main();
    } else if (ui_extra_get_current_page() == UI_PAGE_SETTINGS) {
      action_settings();
    }
    bsp_display_unlock();
  }
}

/* Knob callbacks implementation */
static void knob_right_cb(void *arg, void *data) {
  handle_knob_rotation(-1, camera_decrease_magnification, ui_extra_btn_up,
                       ui_extra_btn_right);
}

static void knob_left_cb(void *arg, void *data) {
  handle_knob_rotation(1, camera_increase_magnification, ui_extra_btn_down,
                       ui_extra_btn_left);
}

/* Public functions implementation */

/**
 * @brief Set encoder step threshold for knob sensitivity
 *
 * @param threshold Sensitivity threshold value (higher = less sensitive)
 */
void app_control_set_knob_sensitivity(int threshold) {
  if (threshold > 0) {
    knob_step_threshold = threshold;
    ESP_LOGI(TAG, "Knob sensitivity set to %d steps", knob_step_threshold);
  }
}

/**
 * @brief Initialize application control module
 *
 * This function initializes buttons and knob controls, and registers
 * corresponding callbacks
 *
 * @return
 *      - ESP_OK: Success
 *      - Others: Fail
 */
esp_err_t app_control_init(void) {
  // Initialize the wake buttons
  const gpio_config_t config = {
      .pin_bit_mask = BIT(BSP_BUTTON_NUM1) | BIT(BSP_BUTTON_NUM2) |
                      BIT(BSP_BUTTON_NUM3) | BIT(BSP_BUTTON_ENCODER),
      .mode = GPIO_MODE_INPUT,
  };

  ESP_ERROR_CHECK(gpio_config(&config));
  ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(
      BIT(BSP_BUTTON_NUM1) | BIT(BSP_BUTTON_NUM2) | BIT(BSP_BUTTON_NUM3) |
          BIT(BSP_BUTTON_ENCODER),
      0));

  // Initialize the buttons
  ESP_ERROR_CHECK(bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM));
  ESP_ERROR_CHECK(iot_button_register_cb(btns[BSP_BUTTON_1], BUTTON_PRESS_DOWN,
                                         btn_handler, (void *)BSP_BUTTON_1));
  ESP_ERROR_CHECK(iot_button_register_cb(btns[BSP_BUTTON_2], BUTTON_PRESS_DOWN,
                                         btn_handler, (void *)BSP_BUTTON_2));
  ESP_ERROR_CHECK(iot_button_register_cb(btns[BSP_BUTTON_3], BUTTON_PRESS_DOWN,
                                         btn_handler, (void *)BSP_BUTTON_3));
  ESP_ERROR_CHECK(iot_button_register_cb(btns[BSP_BUTTON_ED], BUTTON_PRESS_UP,
                                         btn_handler, (void *)BSP_BUTTON_ED));

  // Initialize the knob
  ESP_ERROR_CHECK(bsp_knob_init());
  // Register callback functions
  ESP_ERROR_CHECK(bsp_knob_register_cb(KNOB_LEFT, knob_left_cb, NULL));
  ESP_ERROR_CHECK(bsp_knob_register_cb(KNOB_RIGHT, knob_right_cb, NULL));

  return ESP_OK;
}

/**
 * @brief Start the UDP server for haptic commands
 *
 * Must be called AFTER lwIP is initialized (e.g., after esp_netif_init /
 * esp_event_loop_create_default).
 */
void app_control_start_udp_server(void) {
  ESP_LOGI(TAG, "Starting UDP Haptic server task");
  xTaskCreate(udp_haptic_server_task, "udp_haptic_server", 4096, NULL, 5, NULL);
}