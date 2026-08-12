#include "sd_card.hpp"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "esp_camera.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/gpio.h"


#include <sys/stat.h>
#include <time.h>
#include <dirent.h>
#include <cstring>
#include <cstdio>
#include "ff.h" // Für FATFS Zeitstempel

#include "esp_jpeg_enc.h"
#include "dl_image_jpeg.hpp"

#include "include/sd_pins.h"  // the board-specific SD + SPI pins

#pragma GCC diagnostic warning "-Wmissing-field-initializers"

namespace sdcard {

static const char *TAG = "SDCARD";
static constexpr const char *MOUNT_POINT = "/sdcard";

static sdmmc_card_t *g_card = nullptr;
static bool g_mounted = false;

// --------- Internal helpers ----------------------------------

static void init_sd_enable_pin(void) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << SD_ENABLE);
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Active level depends on hardware
    gpio_set_level(SD_ENABLE, 0);
}

static bool mount_sdcard_spi() {
    if (g_mounted) {
        return true;
    }

    init_sd_enable_pin();
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false,
        .use_one_fat = false
    };

    ESP_LOGI(TAG, "Initializing SD card over SPI");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    // host.'slot' should be set to an sdspi device initialized by `sdspi_host_init_device()`.
    // SDSPI_HOST_DEFAULT: https://github.com/espressif/esp-idf/blob/1bbf04cb4cf54d74c1fe21ed12dbf91eb7fb1019/components/esp_driver_sdspi/include/driver/sdspi_host.h#L44
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 5000;

    constexpr spi_host_device_t SPI_HOST_ID = SPI3_HOST;
    host.slot = SPI_HOST_ID;

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num      = PIN_NUM_MOSI;
    bus_cfg.miso_io_num      = PIN_NUM_MISO;
    bus_cfg.sclk_io_num      = PIN_NUM_CLK;
    bus_cfg.quadwp_io_num    = -1;
    bus_cfg.quadhd_io_num    = -1;
    bus_cfg.max_transfer_sz  = 4000;

    ESP_LOGI(TAG, "Initializing SPI bus");
    ret = spi_bus_initialize(SPI_HOST_ID, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }

    // card select output ?
    gpio_reset_pin(PIN_NUM_CS);
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS, 1); // inactive

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    // sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    sdspi_device_config_t slot_config = {};
    slot_config.host_id      = SPI_HOST_ID;
    slot_config.gpio_cs      = PIN_NUM_CS;
    slot_config.gpio_cd      = SD_SW;
    slot_config.gpio_wp      = SDSPI_SLOT_NO_WP;
    slot_config.gpio_int     = GPIO_NUM_NC;
    slot_config.gpio_wp_polarity = SDSPI_IO_ACTIVE_LOW;
    //slot_config.duty_cycle_pos = 0;

    // spi_host_device_t host_id; ///< SPI host to use, SPIx_HOST (see spi_types.h)
    ESP_LOGI(TAG, "Mounting FAT filesystem at %s", MOUNT_POINT);
    // gpio_set_level(SD_ENABLE, 1);
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &g_card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("SD", "Failed to mount filesystem (ret == ESP_FAIL). Look into esp_vfs_fat_sdspi_mount() "
                           "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE("SD", "Failed to initialize the card (%s). Look into esp_vfs_fat_sdspi_mount() "
                           "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
            }
        return false;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, g_card);
    g_mounted = true;
    ESP_LOGI(TAG, "SD card mounted successfully");
    return true;
}

// Encode an RGB888 image to JPEG into jpeg_img.
// img.pix_type must be DL_IMAGE_PIX_TYPE_RGB888.
static jpeg_error_t encode_img_to_jpeg(const dl::image::img_t *img, dl::image::jpeg_img_t *jpeg_img, jpeg_enc_config_t cfg) {
    jpeg_enc_handle_t jpeg_enc = nullptr;
    jpeg_error_t ret = jpeg_enc_open(&cfg, &jpeg_enc);
    if (ret != JPEG_ERR_OK) {
        return ret;
    }

    const int outbuf_size = 100 * 1024; // 100 KB
    uint8_t *outbuf = static_cast<uint8_t*>(calloc(1, outbuf_size));
    if (!outbuf) {
        jpeg_enc_close(jpeg_enc);
        return JPEG_ERR_NO_MEM;
    }

    const int in_size = img->width * img->height * 3; // RGB888
    int out_len = 0;
    ret = jpeg_enc_process(jpeg_enc, static_cast<const uint8_t*>(img->data), in_size, outbuf, outbuf_size, &out_len);
    if (ret == JPEG_ERR_OK) {
        jpeg_img->data = outbuf;
        jpeg_img->data_len = out_len;
    } else {
        free(outbuf);
    }

    jpeg_enc_close(jpeg_enc);
    return ret;
}

// --------- Public API ----------------------------------

bool init() {
    return mount_sdcard_spi();
}

bool create_dir(const char *full_path) {
    if (!g_mounted) {
        ESP_LOGE(TAG, "create_dir: SD not mounted");
        return false;
    }

    struct stat st;
    if (stat(full_path, &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            ESP_LOGI(TAG, "Dir already exists: %s", full_path);
            return true;
        } else {
            ESP_LOGE(TAG, "Path exists but is not a directory: %s", full_path);
            return false;
        }
    }

    int res = mkdir(full_path, 0775);
    if (res == 0) {
        ESP_LOGI(TAG, "Created dir: %s", full_path);
        return true;
    } else {
        ESP_LOGE(TAG, "mkdir failed for %s (errno=%d)", full_path, errno);
        return false;
    }
}

bool create_logfile(const char *full_path) {
    if (!g_mounted) {
        ESP_LOGE(TAG, "create_logfile: SD not mounted");
        return false;
    }

    // S_ISREG: POSIX macro to check if it's a file
    struct stat st;
    if (stat(full_path, &st) == 0) {
        if (S_ISREG(st.st_mode)) {
            ESP_LOGI(TAG, "Logfile already exists: %s", full_path);
            return true;
        } else {
            ESP_LOGE(TAG, "Path exists but is not a logfile: %s", full_path);
            return false;
        }
    }

    FILE *logfile = fopen(full_path, "w");
    if (logfile==0) {
        ESP_LOGE(TAG, "Failed to create logfile: %s (errno=%d)", full_path, errno);
        return false;
    }
    fclose(logfile);
    return true;
}

void write_log(const char *log_path, const char *log_entry) {
    if (!g_mounted) {
        ESP_LOGE(TAG, "write_log: SD not mounted");
        return;
    }

    FILE *logfile = fopen(log_path, "a");
    if (logfile == nullptr) {
        ESP_LOGE(TAG, "Failed to open logfile for appending: %s (errno=%d)", log_path, errno);
        return;
    }
    fprintf(logfile, "%s\n", log_entry);
    ESP_LOGI(TAG,"%s",log_entry);
    fclose(logfile);
}

int count_files(const char *path) {
    int count = 0;
    DIR *dir = opendir(path);
    if (!dir) {
        ESP_LOGE("FILE_COUNT", "Failed to open directory: %s", path);
        return -1;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr) {
        // Skip current and parent directory entries
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        // Build full file path
        char full_path[256];
        // snprintf(full_path, sizeof(full_path), "%s/%s", path, entry->d_name);
        strlcpy(full_path, path, sizeof(full_path));
        strlcat(full_path, "/", sizeof(full_path));
        strlcat(full_path, entry->d_name, sizeof(full_path));
        // Get file info
        struct stat st;
        if (stat(full_path, &st) == 0) {
            if (S_ISREG(st.st_mode)) {
                count++;
            }
        }
        else {
            ESP_LOGW("FILE_COUNT", "Could not stat file: %s", full_path);
        }
    }

    closedir(dir);
    return count;
}

bool save_jpeg_directly(camera_fb_t *captureImage, const char *dir_full_path)
{
    if (!create_dir(dir_full_path)) {
        return false;
    }

    int idx = count_files(dir_full_path);
    if (idx < 0) idx = 0;

    char filepath[256];
    std::snprintf(filepath, sizeof(filepath), "%s/bee_%04d.jpg", dir_full_path, idx + 1);

    ESP_LOGI(TAG, "Saving detected JPEG: %s", filepath);

    if (!captureImage) {
        ESP_LOGE(TAG, "save_jpeg_directly: null captureImage");
        return false;
    }

    // If the frame buffer already contains JPEG compressed data, write directly
    if (captureImage->format == PIXFORMAT_JPEG) {
        FILE *fp = fopen(filepath, "wb");
        if (!fp) {
            ESP_LOGE(TAG, "Failed to create file: %s (errno=%d)", filepath, errno);
            return false;
        }
        size_t wrote = fwrite(captureImage->buf, 1, captureImage->len, fp);
        fclose(fp);
        if (wrote != captureImage->len) {
            ESP_LOGE(TAG, "Incomplete write for %s: wrote %u of %u", filepath, (unsigned)wrote, (unsigned)captureImage->len);
            return false;
        }
        ESP_LOGI(TAG, "JPEG saved as %s", filepath);
        return true;
    }

    // Otherwise convert raw frame (e.g., GRAYSCALE or RGB565) to RGB888 and encode to JPEG
    int width = captureImage->width;
    int height = captureImage->height;
    size_t pixels = static_cast<size_t>(width) * static_cast<size_t>(height);
    uint8_t *rgb_buf = static_cast<uint8_t*>(malloc(pixels * 3));
    if (!rgb_buf) {
        ESP_LOGE(TAG, "Failed to allocate rgb buffer");
        return false;
    }

    if (captureImage->format == PIXFORMAT_GRAYSCALE) {
        for (size_t i = 0; i < pixels; ++i) {
            uint8_t g = captureImage->buf[i];
            rgb_buf[3*i + 0] = g;
            rgb_buf[3*i + 1] = g;
            rgb_buf[3*i + 2] = g;
        }
    } else if (captureImage->format == PIXFORMAT_RGB565) {
        // Convert RGB565 -> RGB888
        for (size_t i = 0; i < pixels; ++i) {
            uint16_t val = static_cast<uint16_t>(captureImage->buf[2*i]) | (static_cast<uint16_t>(captureImage->buf[2*i + 1]) << 8);
            uint8_t r = ((val >> 11) & 0x1F) << 3;
            uint8_t g = ((val >> 5) & 0x3F) << 2;
            uint8_t b = (val & 0x1F) << 3;
            rgb_buf[3*i + 0] = r;
            rgb_buf[3*i + 1] = g;
            rgb_buf[3*i + 2] = b;
        }
    } else if (captureImage->format == PIXFORMAT_RGB888) {
        memcpy(rgb_buf, captureImage->buf, pixels * 3);
    } else {
        ESP_LOGW(TAG, "Unsupported frame format %d, attempting raw write", captureImage->format);
        FILE *fp = fopen(filepath, "wb");
        if (!fp) {
            ESP_LOGE(TAG, "Failed to create file: %s (errno=%d)", filepath, errno);
            free(rgb_buf);
            return false;
        }
        fwrite(captureImage->buf, 1, captureImage->len, fp);
        fclose(fp);
        free(rgb_buf);
        return true;
    }

    // Prepare image struct for encoder
    dl::image::img_t img;
    img.width = width;
    img.height = height;
    img.data = rgb_buf;
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

    dl::image::jpeg_img_t jpeg_img;
    jpeg_enc_config_t enc_cfg = {
        .width = width,
        .height = height,
        .src_type = JPEG_PIXEL_FORMAT_RGB888,
        .subsampling = JPEG_SUBSAMPLE_444,
        .quality = 80,
        .rotate = JPEG_ROTATE_0D,
        .task_enable = true,
        .hfm_task_priority = 13,
        .hfm_task_core = 1,
    };

    jpeg_error_t enc_ret = encode_img_to_jpeg(&img, &jpeg_img, enc_cfg);
    if (enc_ret != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "JPEG encoding failed (%d)", enc_ret);
        free(rgb_buf);
        return false;
    }

    FILE *fp = fopen(filepath, "wb");
    if (!fp) {
        ESP_LOGE(TAG, "Failed to create file: %s (errno=%d)", filepath, errno);
        free(jpeg_img.data);
        free(rgb_buf);
        return false;
    }
    fwrite(jpeg_img.data, 1, jpeg_img.data_len, fp);

    // Änderungsdatum setzen (aktuelles Systemdatum/Zeit) via FATFS
    // Nur möglich, wenn FF_USE_CHMOD und FF_FS_NORTC == 0 in FATFS Konfiguration
    struct tm *tm_now;
    time_t t = time(NULL);
    tm_now = localtime(&t);
    if (tm_now) {
#ifdef f_utime
        FILINFO finfo = {0};
        finfo.fdate = ((tm_now->tm_year - 80) << 9) | ((tm_now->tm_mon + 1) << 5) | tm_now->tm_mday;
        finfo.ftime = (tm_now->tm_hour << 11) | (tm_now->tm_min << 5) | (tm_now->tm_sec / 2);
        if (f_utime(filepath, &finfo) != 0) {
            ESP_LOGW(TAG, "Could not set FATFS file time: %s", filepath);
        }
#else   
        ESP_LOGW(TAG, "f_utime nicht verfügbar, Änderungsdatum kann nicht gesetzt werden: %s", filepath);
#endif
    } else {
        ESP_LOGW(TAG, "Could not get localtime for file time: %s", filepath);
    }

    fclose(fp);

    ESP_LOGI(TAG, "JPEG saved as %s", filepath);
    free(jpeg_img.data);
    free(rgb_buf);
    return true;
}

} // namespace sdcard
