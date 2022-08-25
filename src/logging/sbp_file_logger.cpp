#include <libsbp/sbp.h>
#include <logging/sbp_file_logger.h>
#include <ctime>
#include <filesystem>
#include <iostream>

SbpFileLogger::SbpFileLogger(const std::string& file_path,
                             const LoggerPtr& logger)
    : state_(nullptr, this), logger_(logger) {
  std::string file_name(file_path);
  time_t now = time(nullptr);

  struct tm local_time;

#if defined(_WIN32)
  localtime_s(&local_time, &now);
#else
  localtime_r(&now, &local_time);
#endif  // _WIN32

  file_name += "/";
  file_name += std::to_string(local_time.tm_year + 1900);
  file_name += std::to_string(local_time.tm_mon + 1);
  file_name += std::to_string(local_time.tm_mday);
  file_name += std::to_string(local_time.tm_hour);
  file_name += std::to_string(local_time.tm_min);
  file_name += std::to_string(local_time.tm_sec);
  file_name += ".sbp";

  std::filesystem::path path(file_name);
  file_name = path.lexically_normal().string();

#if defined(_WIN32)
  fopen_s(&file_, file_name.c_str(), "wb");
#else
  file_ = fopen(file_name.c_str(), "wb");
#endif  // _WIN32
  if (!file_)
    LOG_ERROR(logger_, "Unable to open the file: " << file_name);
  else
    LOG_INFO(logger_, "Now logging SBP messages to " << file_name);
}

SbpFileLogger::~SbpFileLogger() {
  if (file_) fclose(file_);
}

void SbpFileLogger::insert(const sbp_msg_type_t msg_type,
                           const sbp_msg_t& msg) {
  (void)msg_type;
  state_.send_message(0, msg_type, msg);
}

s32 SbpFileLogger::write(const u8* buffer, u32 buffer_length) {
  if (file_)
    return fwrite(buffer, sizeof(uint8_t), buffer_length, file_);
  else
    return -1;
}
