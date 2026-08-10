#pragma once

#include "dl_image_define.hpp"
#include "dl_cls_postprocessor.hpp"  // for dl::cls::result_t
#include "esp_camera.h"

namespace sdcard {

bool init();

bool create_dir(const char *full_path);

bool create_logfile(const char *full_path);

void write_log(const char *log_path, const char *log_entry);

int count_files(const char *full_path);

bool save_jpeg_directly(camera_fb_t *captureImage, const char *dir_full_path);

} // namespace sdcard
