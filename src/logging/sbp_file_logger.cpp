/*
 * Copyright (C) 2015-2023 Swift Navigation Inc.
 * Contact: https://support.swiftnav.com
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <libsbp/sbp.h>
#include <logging/sbp_file_logger.h>
#include <utils/utils.h>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iostream>

constexpr uint8_t FNAME_MAX_LEN = 50;

SbpFileLogger::SbpFileLogger(const std::string& file_path,
                             const LoggerPtr& logger)
    : state_(nullptr, this), logger_(logger) {
  std::string file_name(file_path);
  time_t now = time(nullptr);
  std::array<char, FNAME_MAX_LEN> fname{""};

  // Create dir
  if (!FileSystem::createDir(file_path)) {
    LOG_FATAL(logger_, "Unable to create dir: %s", file_path.c_str());
    exit(1);
  }

  struct tm local_time;

#if defined(_WIN32)
  localtime_s(&local_time, &now);
#else
  localtime_r(&now, &local_time);
#endif  // _WIN32

  std::snprintf(fname.data(), FNAME_MAX_LEN,
               "/swiftnav-%d%02d%02d-%02d%02d%02d.sbp",
               local_time.tm_year + 1900,
               local_time.tm_mon + 1,
               local_time.tm_mday,
               local_time.tm_hour,
               local_time.tm_min,
               local_time.tm_sec);

  file_name += std::string(fname.data());

  std::filesystem::path path(file_name);
  file_name = path.lexically_normal().string();

#if defined(_WIN32)
  fopen_s(&file_, file_name.c_str(), "wb");
#else
  file_ = fopen(file_name.c_str(), "wb");
#endif  // _WIN32
  if (!file_) {
    LOG_FATAL(logger_, "Unable to open the file: %s", file_name.c_str());
    exit(1);
  } else
    LOG_INFO(logger_, "Logging SBP messages to %s", file_name.c_str());
}

SbpFileLogger::~SbpFileLogger() {
  if (file_) fclose(file_);
}

void SbpFileLogger::insert(const sbp_msg_type_t msg_type,
                           const sbp_msg_t& msg) {
  state_.send_message(0, msg_type, msg);
}

s32 SbpFileLogger::write(const u8* buffer, u32 buffer_length) {
  if (file_)
    return fwrite(buffer, sizeof(uint8_t), buffer_length, file_);
  else
    return -1;
}
