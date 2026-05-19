#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "bsp/esp-bsp.h"

#include "ui_extra.h"
#include "app_video_stream.h"
#include "app_video.h"
#include "app_album.h"
#include "app_livestream.h"
#include "app_control.h"

#include "driver/i2c_master.h"

/* Private definitions */
static const char *TAG = "app_control";

/* DRV2605L Haptic Variables */
static i2c_master_dev_handle_t s_drv2605_dev = NULL;
static esp_timer_handle_t s_haptic_timer = NULL;

typedef struct {
    uint8_t effect;
    const char* name;
    bool continuous;
} haptic_test_state_t;

static const haptic_test_state_t s_haptic_states[] = {
    {1,  "Effect 1 (Current pulse - strong click 100%)", false},
    {24, "Effect 24 (Double Click 100%)", false},
    {27, "Effect 27 (Short Double Click Strong)", false},
    {16, "Effect 16 (Long Alert)", false},
    {52, "Effect 52 (Pulsing Strong)", false},
    {55, "Effect 55 (Transition Hum)", false},
    {1,  "Effect 1 (Continuous)", true},
    {2,  "Effect 2 (Continuous - 60% click)", true},
    {3,  "Effect 3 (Continuous - 30% click)", true}
};
static const int NUM_HAPTIC_STATES = sizeof(s_haptic_states) / sizeof(s_haptic_states[0]);
static int s_current_haptic_state = -1; // starts at -1 so first press goes to 0

static esp_err_t drv_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    esp_err_t err = i2c_master_transmit(s_drv2605_dev, buf, 2, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DRV2605L I2C write error reg 0x%02X: %s", reg, esp_err_to_name(err));
    }
    return err;
}

static void haptic_timer_cb(void* arg) {
    if (s_drv2605_dev) {
        drv_write_reg(0x0C, 0x01); // GO
    }
}

static void trigger_haptic_buzz(void)
{
    // If timer is running, stop and delete it
    if (s_haptic_timer != NULL) {
        esp_timer_stop(s_haptic_timer);
        esp_timer_delete(s_haptic_timer);
        s_haptic_timer = NULL;
    }

    if (s_drv2605_dev == NULL) {
        i2c_master_bus_handle_t bus_handle;
        if (bsp_get_i2c_bus_handle(&bus_handle) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get I2C bus handle for DRV2605L");
            return;
        }

        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x5A,
            .scl_speed_hz = 100000, // standard 100kHz
        };
        if (i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_drv2605_dev) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add DRV2605L to I2C bus");
            return;
        }
        
        ESP_LOGI(TAG, "Initializing DRV2605L...");

        // 1. Take out of standby (Mode Register 0x01 = 0x00)
        if (drv_write_reg(0x01, 0x00) != ESP_OK) return;

        // 2. Select LRA ROM library (Library Selection Register 0x03 = 0x06)
        if (drv_write_reg(0x03, 0x06) != ESP_OK) return;

        uint8_t read_buf[1] = {0};
        uint8_t reg = 0x1A;

        // 3. Enable LRA Mode (0x1A: Feedback Control)
        if (i2c_master_transmit_receive(s_drv2605_dev, &reg, 1, read_buf, 1, -1) == ESP_OK) {
            drv_write_reg(0x1A, read_buf[0] | 0x80);
        } else {
            ESP_LOGE(TAG, "Failed to read Feedback Control reg 0x1A");
            return;
        }

        // 4. Set Rated Voltage (0x16) to 2.5V = 0x7D
        drv_write_reg(0x16, 0x7D);

        // 5. Set Overdrive Clamp (0x17) to 2.5V = 0x7D
        drv_write_reg(0x17, 0x7D);
        
        ESP_LOGI(TAG, "DRV2605L Configured for LRA");
    }

    if (s_drv2605_dev) {
        // Advance to next state
        s_current_haptic_state = (s_current_haptic_state + 1) % NUM_HAPTIC_STATES;
        const haptic_test_state_t* state = &s_haptic_states[s_current_haptic_state];
        
        ESP_LOGI(TAG, "=== Haptic Test Phase %d/8: %s ===", s_current_haptic_state, state->name);

        drv_write_reg(0x04, state->effect);  // sequence 1 = our current effect
        drv_write_reg(0x05, 0x00);           // sequence 2 = stop
        drv_write_reg(0x0C, 0x01);           // GO

        if (state->continuous) {
            ESP_LOGI(TAG, "Starting continuous pulse timer... Press again to stop/advance.");
            esp_timer_create_args_t timer_args = {
                .callback = &haptic_timer_cb,
                .name = "haptic_cont"
            };
            esp_timer_create(&timer_args, &s_haptic_timer);
            // 200ms period (200,000 microseconds)
            esp_timer_start_periodic(s_haptic_timer, 200000); 
        }
    }
}

/* Button related variables */
static button_handle_t btns[BSP_BUTTON_NUM];

/* Knob related variables */
static int knob_step_counter = 0;
static int knob_last_direction = 0;  // 0: no direction, 1: right, -1: left
static int64_t knob_last_time = 0;   // timestamp of last rotation
static const int knob_timeout_ms = 500;  // timeout in milliseconds
static int knob_step_threshold = 3;  // threshold for knob step counter
static bool s_display_suspended = false;

/* Private function prototypes */
static void btn_handler(void *arg, void *data);
static void handle_knob_rotation(int direction, void (*action_camera)(void), void (*action_main)(void), void (*action_settings)(void));

/* Helper functions for camera actions */
static void camera_decrease_magnification(void)
{
    app_extra_set_magnification_factor(app_extra_get_magnification_factor() - 1);
}

static void camera_increase_magnification(void)
{
    app_extra_set_magnification_factor(app_extra_get_magnification_factor() + 1);
}

/* Button handler implementation */
static void btn_handler(void *arg, void *data)
{
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
            ESP_LOGI(TAG, "BSP_BUTTON_3 (Down button) pressed. Current page: %d, Livestream page ID: %d", ui_extra_get_current_page(), UI_PAGE_LIVESTREAM);
            
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
static void handle_knob_rotation(int direction, void (*action_camera)(void), void (*action_main)(void), void (*action_settings)(void))
{
    if (s_display_suspended) {
        return;
    }
    if (ui_extra_get_current_page() == UI_PAGE_ALBUM || 
        ui_extra_get_current_page() == UI_PAGE_USB_DISK) {
        return;
    }

    int64_t current_time = esp_timer_get_time() / 1000;  // get current time in milliseconds
    
    // Check for timeout or direction change
    if (current_time - knob_last_time > knob_timeout_ms || knob_last_direction == -direction) {
        // Timeout or direction change, reset counter
        knob_step_counter = 0;
        knob_last_direction = direction;
    }
    
    // Increment step counter
    knob_step_counter++;
    knob_last_time = current_time;
    
    // Trigger action when accumulated steps reach threshold
    if (knob_step_counter >= knob_step_threshold) {
        knob_step_counter = 0;  // Reset counter
        
        ESP_LOGD(TAG, "Continuous rotation detected: %d steps, value %d", knob_step_counter, direction);
        
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
static void knob_right_cb(void *arg, void *data)
{
    handle_knob_rotation(
        -1,
        camera_decrease_magnification,
        ui_extra_btn_up,
        ui_extra_btn_right
    );
}

static void knob_left_cb(void *arg, void *data)
{
    handle_knob_rotation(
        1,
        camera_increase_magnification,
        ui_extra_btn_down,
        ui_extra_btn_left
    );
}

/* Public functions implementation */

/**
 * @brief Set encoder step threshold for knob sensitivity
 * 
 * @param threshold Sensitivity threshold value (higher = less sensitive)
 */
void app_control_set_knob_sensitivity(int threshold)
{
    if (threshold > 0) {
        knob_step_threshold = threshold;
        ESP_LOGI(TAG, "Knob sensitivity set to %d steps", knob_step_threshold);
    }
}

/**
 * @brief Initialize application control module
 * 
 * This function initializes buttons and knob controls, and registers corresponding callbacks
 * 
 * @return
 *      - ESP_OK: Success
 *      - Others: Fail
 */
esp_err_t app_control_init(void)
{
    // Initialize the wake buttons
    const gpio_config_t config = {
        .pin_bit_mask = BIT(BSP_BUTTON_NUM1) | BIT(BSP_BUTTON_NUM2) | BIT(BSP_BUTTON_NUM3) | BIT(BSP_BUTTON_ENCODER),
        .mode = GPIO_MODE_INPUT,
    };

    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(
        BIT(BSP_BUTTON_NUM1) | BIT(BSP_BUTTON_NUM2) | BIT(BSP_BUTTON_NUM3) | BIT(BSP_BUTTON_ENCODER), 0));

    // Initialize the buttons
    ESP_ERROR_CHECK(bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM));
    ESP_ERROR_CHECK(iot_button_register_cb(btns[BSP_BUTTON_1], BUTTON_PRESS_DOWN, btn_handler, (void *) BSP_BUTTON_1));
    ESP_ERROR_CHECK(iot_button_register_cb(btns[BSP_BUTTON_2], BUTTON_PRESS_DOWN, btn_handler, (void *) BSP_BUTTON_2));
    ESP_ERROR_CHECK(iot_button_register_cb(btns[BSP_BUTTON_3], BUTTON_PRESS_DOWN, btn_handler, (void *) BSP_BUTTON_3));
    ESP_ERROR_CHECK(iot_button_register_cb(btns[BSP_BUTTON_ED], BUTTON_PRESS_UP, btn_handler, (void *) BSP_BUTTON_ED));

    // Initialize the knob
    ESP_ERROR_CHECK(bsp_knob_init());
    // Register callback functions
    ESP_ERROR_CHECK(bsp_knob_register_cb(KNOB_LEFT, knob_left_cb, NULL));
    ESP_ERROR_CHECK(bsp_knob_register_cb(KNOB_RIGHT, knob_right_cb, NULL));

    return ESP_OK;
}