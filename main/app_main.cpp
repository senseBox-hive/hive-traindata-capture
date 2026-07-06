
#include "dl_image_define.hpp"
#include "esp_heap_caps.h"
#include "esp_imgfx_crop.h"
#include "dl_image.hpp"
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
    .jpeg_quality = 10,                // The quality of the JPEG image, ranging from 0 to 63.
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

void capture_averaged_image(camera_fb_t *main_pic, int num_frames)
{
    if (!main_pic || num_frames <= 0) {
        return;
    }

    // Handle grayscale frames: 1 byte per pixel
    size_t n = main_pic->len;
    std::vector<uint32_t> accumulated(n, 0);

    for (int i = 0; i < num_frames; ++i) {
        camera_fb_t *frame = esp_camera_fb_get();
        if (!frame) {
            continue;
        }

        // If frame size differs, skip this frame
        if (frame->len != n) {
            esp_camera_fb_return(frame);
            continue;
        }

        for (size_t j = 0; j < n; ++j) {
            accumulated[j] += frame->buf[j];
        }

        esp_camera_fb_return(frame);
    }

    // Average and write back into main_pic (grayscale)
    for (size_t j = 0; j < n; ++j) {
        main_pic->buf[j] = static_cast<uint8_t>(accumulated[j] / num_frames);
    }
}

extern "C" void app_main(void)
{
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

    while (true) {
        ESP_LOGI("MEM", "Free heap at start of loop: %lu bytes", esp_get_free_heap_size());
        //stack consecutive frames to capture bee trajectories

        camera_fb_t *main_pic = esp_camera_fb_get();
        if (!main_pic) {
            continue;
        }

        //capture N frames and stack on top of the main_pic
        capture_averaged_image(main_pic, 6);

        //rohes JPEG speichern (will encode when needed)
        sdcard::save_jpeg_directly(main_pic, "/sdcard/bee_traindata");

        // return main buffer so camera driver can reuse it
        esp_camera_fb_return(main_pic);
        
        vTaskDelay(pdMS_TO_TICKS(5)); // perhaps remove delay entirely?
    }

}
