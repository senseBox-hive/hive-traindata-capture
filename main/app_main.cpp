
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
    .fb_count = 4,                     // The number of frame buffers to use.
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

camera_fb_t* combine_grayscale_to_rgb(const camera_fb_t *r,
                                      const camera_fb_t *g,
                                      const camera_fb_t *b)
{
    if (!r || !g || !b) {
        ESP_LOGE("APP", "Null input buffer");
        return NULL;
    }
    if (r->width != g->width || r->width != b->width ||
        r->height != g->height || r->height != b->height) {
        ESP_LOGE("APP", "Dimension mismatch");
        return NULL;
    }
    if (r->format != PIXFORMAT_GRAYSCALE ||
        g->format != PIXFORMAT_GRAYSCALE ||
        b->format != PIXFORMAT_GRAYSCALE) {
        ESP_LOGE("APP", "Inputs must be GRAYSCALE");
        return NULL;
    }

    size_t pixels  = r->width * r->height;
    size_t rgb_len = pixels * 3;

    // --- Allocate output fb ---
    camera_fb_t *out = (camera_fb_t*)heap_caps_malloc(sizeof(camera_fb_t),
                                                      MALLOC_CAP_DEFAULT);
    if (!out) return NULL;

    // Prefer PSRAM for the large pixel buffer
    out->buf = (uint8_t*)heap_caps_malloc(rgb_len, MALLOC_CAP_SPIRAM);
    if (!out->buf) {
        out->buf = (uint8_t*)heap_caps_malloc(rgb_len, MALLOC_CAP_DEFAULT);
    }
    if (!out->buf) {
        free(out);
        ESP_LOGE("APP", "Failed to allocate %u bytes", rgb_len);
        return NULL;
    }

    out->len    = rgb_len;
    out->width  = r->width;
    out->height = r->height;
    out->format = PIXFORMAT_RGB888;
    out->timestamp = r->timestamp;

    // --- Interleave channels ---
    const uint8_t *rp = r->buf;
    const uint8_t *gp = g->buf;
    const uint8_t *bp = b->buf;
    uint8_t *dst = out->buf;

    for (size_t i = 0; i < pixels; i++) {
        *dst++ = rp[i];   // R
        *dst++ = gp[i];   // G
        *dst++ = bp[i];   // B
    }

    return out;
}

void free_rgb_fb(camera_fb_t *fb) {
    if (fb) {
        if (fb->buf) free(fb->buf);
        free(fb);
    }
}


camera_fb_t* capture_rgb_stack()
{
    // capture 3 consecutive frames and combine them from the frame buffer into a single image
    camera_fb_t *red = esp_camera_fb_get();
    camera_fb_t *green = esp_camera_fb_get();
    camera_fb_t *blue = esp_camera_fb_get();

    // combine into rgb channels of base pic
    camera_fb_t *combined = combine_grayscale_to_rgb(red, green, blue);
    if (!combined) {
        ESP_LOGE("APP", "Failed to combine RGB channels");
        esp_camera_fb_return(red);
        esp_camera_fb_return(green);
        esp_camera_fb_return(blue);
        return NULL;
    }

    //return the frame buffers to the camera driver so they can be reused
    esp_camera_fb_return(red);
    esp_camera_fb_return(green);
    esp_camera_fb_return(blue);

    return combined;
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

    ESP_LOGI("MEM", "Capturing base frame for stacking...");
    // create an rgb image buffer to hold the stacked frames
    camera_fb_t *main_pic = esp_camera_fb_get();

    ESP_LOGI("MEM", "Begin Stacking loop...");
    while (true) {
        ESP_LOGI("MEM", "Free heap at start of loop: %lu bytes", esp_get_free_heap_size());
        //stack consecutive frames to capture bee trajectories
    
        if (!main_pic) {
            continue;
        }

        camera_fb_t *stacked = capture_rgb_stack();
        if (!stacked) {
            ESP_LOGE("APP", "Failed to capture RGB stack");
            continue;
        }

        //rohes JPEG speichern (will encode when needed)
        sdcard::save_jpeg_directly(stacked, "/sdcard/bee_traindata");
        free_rgb_fb(stacked);
        vTaskDelay(pdMS_TO_TICKS(5)); // perhaps remove delay entirely?
    }
    // return main buffer so camera driver can reuse it
    esp_camera_fb_return(main_pic);
}
