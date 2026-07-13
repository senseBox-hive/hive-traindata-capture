
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

    .xclk_freq_hz = 5000000,           // The clock frequency of the image sensor
    .pixel_format = PIXFORMAT_GRAYSCALE,    // The pixel format of the image: PIXFORMAT_ + YUV422|GRAYSCALE|RGB565|JPEG
    .frame_size = FRAMESIZE_QVGA,      // The resolution size of the image: FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    .jpeg_quality = 10,                // The quality of the JPEG image, ranging from 0 to 63.
    .fb_count = 1,                     // The number of frame buffers to use.
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
    //TODO: require day-night presets
    sensor_t * s = esp_camera_sensor_get();
    s->set_exposure_ctrl(s, 0);   // 0 = disable AEC (auto exposure), 1 = enable
    s->set_aec2(s, 0);            // disable the DSP's secondary AEC algorithm

    // --- Set the manual exposure value ---
    // This is in "exposure lines", NOT milliseconds. Higher = longer exposure.
    s->set_aec_value(s, 1200);   // range roughly 0..1200 (OV2640), 0..~1500+ (OV5640)

    // --- Gain: turn off auto-gain too, or bright/dark scenes fight your exposure ---
    s->set_gain_ctrl(s, 0);      // 0 = disable AGC (auto gain)
    s->set_agc_gain(s, 0);       // manual gain, 0 = lowest (least noise)
    s->set_gainceiling(s, (gainceiling_t)0);  // cap on gain if you re-enable AGC

    s->set_whitebal(s, 0);       // auto white balance off
    s->set_awb_gain(s, 0);
    s->set_bpc(s, 0);            // black pixel correction
    s->set_wpc(s, 0);            // white pixel correction
    s->set_lenc(s, 0);           // lens correction
    s->set_dcw(s, 0);            // downsize/cropping enhancements
    // (keep these consistent so your background model stays stable)

    return err;
}

extern "C" void app_main(void)
{
    ESP_LOGI("APP", "Starting application...");
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
    sdcard::write_log("/sdcard/bee_traindata/log.txt", "Log file initialized");

    // set W and H to the camera's frame size
    const int W = 320;
    const int H = 240;
    const int DIFF_THRESH = 50; // threshold for motion detection
    const size_t framePixels = static_cast<size_t>(W) * static_cast<size_t>(H);

    uint16_t *accum = static_cast<uint16_t *>(heap_caps_calloc(framePixels, sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM));
    uint8_t *background = static_cast<uint8_t *>(heap_caps_calloc(framePixels, sizeof(uint8_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM));
    if (!accum || !background) {
        ESP_LOGE("APP", "Failed to allocate motion buffers");
        if (accum) {
            heap_caps_free(accum);
        }
        if (background) {
            heap_caps_free(background);
        }
        return;
    }

    while (true) {
        ESP_LOGI("MEM", "Free heap at start of loop: %u bytes", esp_get_free_heap_size());

        // pixel-wise accumulation of frames
        for (int i = 0; i < 5; ++i) {
            ESP_LOGI("APP", "starting frame %d capture", i);
            camera_fb_t *frame = esp_camera_fb_get();
            if (!frame) {
                ESP_LOGE("CAM", "Camera capture failed");
                continue;
            }

            // Accumulate pixel values
            ESP_LOGI("APP", "Accumulating frame %d", i);
            for (size_t j = 0; j < framePixels; ++j) {
                uint8_t f = frame->buf[j];

                // 1. update slow background (lighting drift)
                background[j] += ((int)f - background[j]) >> 5;

                // 2. motion this frame
                int d = f - background[j];
                if (d < 0) d = -d;
                uint8_t motion = (d > DIFF_THRESH) ? d : 0;

                // 3. decaying-max accumulate (the "long exposure")
                uint8_t decayed = (accum[j] * 240) >> 8;
                accum[j] = (motion > decayed) ? motion : decayed;
            }

            ESP_LOGI("APP", "Accumulated frame %d", i);
            esp_camera_fb_return(frame);
        }

        // save the accumulated motion image as a JPEG
        // Allocate a separate grayscale buffer so we don't depend on the
        // camera driver's framebuffer layout/stride.
        uint8_t *gray = static_cast<uint8_t *>(heap_caps_malloc(framePixels, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (!gray) {
            ESP_LOGE("APP", "Failed to allocate temporary gray buffer");
            continue;
        }
        for (int j = 0; j < W * H; ++j) {
            gray[j] = static_cast<uint8_t>(accum[j] & 0xFF);
        }

        bool saved = sdcard::save_grayscale_buffer(gray, W, H, "/sdcard/bee_traindata");
        if (!saved) {
            ESP_LOGE("SD", "Failed to save JPEG");
        } else {
            sdcard::write_log("/sdcard/bee_traindata/log.txt", "Saved motion image");
        }
        heap_caps_free(gray);
        
        vTaskDelay(pdMS_TO_TICKS(5)); // perhaps remove delay entirely?
    }

    heap_caps_free(accum);
    heap_caps_free(background);
}
