
#include "dl_image_define.hpp"
#include "esp_heap_caps.h"
#include "esp_imgfx_crop.h"
#include "dl_image.hpp"
#include <cstdint>
#include <stddef.h>
#define MODEL_IMG_SIZE 224
#include "sensor.h"
#include <stdio.h>
#include <algorithm>
#include "esp_camera.h"
#include "esp_log.h"
#include "sd_card.hpp"
#include <esp_system.h>
#include <string.h>
#include <vector>
#include "bsp/esp-bsp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/camera_pins.h"
#include "dl_image_draw.hpp"
#include "dl_image_color.hpp"

// Camera Module pin mapping
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sccb_sda = SIOD_GPIO_NUM,
    .pin_sccb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,          // The clock frequency of the image sensor
    .pixel_format = PIXFORMAT_GRAYSCALE,    // The pixel format of the image: PIXFORMAT_ + YUV422|GRAYSCALE|RGB565|JPEG
    .frame_size = FRAMESIZE_QVGA,      // The resolution size of the image: FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    .jpeg_quality = 5,                // The quality of the JPEG image, ranging from 0 to 63.
    .fb_count = 2,                     // The number of frame buffers to use.
    .fb_location = CAMERA_FB_IN_PSRAM, // Set the frame buffer storage location
    .grab_mode = CAMERA_GRAB_LATEST    //  The image capture mode.

};

static esp_err_t init_camera(void)
{
    // Initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE("CAM", "Camera Init Failed");
    }
    // camera settings
    sensor_t * s = esp_camera_sensor_get();
    s->set_ae_level(s, -1);      // Slightly underexpose (-2 to 2) to force a faster shutter

    return err;
}

void capture_stack_consecutive(camera_fb_t *base_pic, float alpha)
{
    float inverse_alpha = 1 - alpha;
    if (!base_pic) {
        return;
    }

    // Handle incoming frames: 
    size_t n = base_pic->len;

    camera_fb_t *incoming = esp_camera_fb_get();
    if (!incoming) {
        return;
    }
    for (size_t j = 0; j < n; ++j) {
        base_pic->buf[j] = (base_pic->buf[j] * inverse_alpha) + (incoming->buf[j] * alpha);
    }
    esp_camera_fb_return(incoming);
}

extern "C" void app_main(void)
{
    gpio_set_direction(GPIO_NUM_43, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_43, 1);
    ESP_LOGI("SD", "Mounting SD card...");
    gpio_set_direction(GPIO_NUM_43, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_43, 1);
    bool mounted = sdcard::init();
    if (!mounted) {
        ESP_LOGE("SD", "SD card init/mount failed");
        return;
    }

    if (ESP_OK != init_camera()) {
        ESP_LOGE("APP", "Camera initialization failed");
        return;
    }

    ESP_LOGI("SD", "CREATING LOGFILE...");
    bool log_initialized = sdcard::create_logfile("/sdcard/bee_traindata/log.txt");
    if (!log_initialized) {
        ESP_LOGE("SD", "Failed to create log file");
        return;
    }
    sdcard::write_log("/sdcard/bee_traindata/log.txt", "logile initialized");

    sdcard::write_log("/sdcard/bee_traindata/log.txt", "Start image capture loop");
    while (true) {
        ESP_LOGI("MEM", "Free heap at start of loop: %lu bytes", esp_get_free_heap_size());
        
        sdcard::write_log("/sdcard/bee_traindata/log.txt", "Attempting capture...");
        camera_fb_t *pic = esp_camera_fb_get();
        if (!pic) {
            sdcard::write_log("/sdcard/bee_traindata/log.txt", "Camera capture failed");
            continue;
        }

        sdcard::write_log("/sdcard/bee_traindata/log.txt", "Saving JPEG...");
        //rohes JPEG speichern
        sdcard::save_jpeg_directly(pic, "/sdcard/bee_traindata");
        sdcard::write_log("/sdcard/bee_traindata/log.txt", "JPEG saved successfully");
        esp_camera_fb_return(pic);
        sdcard::write_log("/sdcard/bee_traindata/log.txt", "Capture loop iteration complete");
        
        vTaskDelay(pdMS_TO_TICKS(5)); // perhaps remove delay entirely?
    }
    // return main buffer so camera driver can reuse it
    esp_camera_fb_return(main_pic);
}
