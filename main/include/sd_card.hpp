#pragma once

#include "dl_image_define.hpp"
#include "dl_cls_postprocessor.hpp"  // for dl::cls::result_t
#include "esp_camera.h"

namespace sdcard {

bool init();

bool create_dir(const char *full_path);

bool create_logfile(const char *full_path);

int count_files(const char *full_path);

bool save_jpeg(const dl::image::img_t &img, const dl::cls::result_t &best, const char *dir_full_path);

bool save_jpeg_directly(camera_fb_t *captureImage, const char *dir_full_path);

} // namespace sdcard
