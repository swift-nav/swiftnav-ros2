#include <libsbp/sbp.h>
#include <logging/sbp_file_logger.h>
#include <ctime>
#include <iostream>

// TODO: Assure path correctness

SbpFileLogger::SbpFileLogger(const std::string& file_path)
    : state_(nullptr, this) {
  std::string file_name(file_path);
  time_t now = time(nullptr);

  struct tm local_time;

#if defined(_WIN32)
  localtime_s(&local_time, &now);
#else
  localtime_r(&now, &local_time);
#endif  // _WIN32

  file_name += std::to_string(local_time.tm_year + 1900);
  file_name += std::to_string(local_time.tm_mon + 1);
  file_name += std::to_string(local_time.tm_mday);
  file_name += std::to_string(local_time.tm_hour);
  file_name += std::to_string(local_time.tm_min);
  file_name += std::to_string(local_time.tm_sec);
  file_name += ".sbp";

#if defined(_WIN32)
  fopen_s(&file_, file_name.c_str(), "wb");
#else
  file_ = fopen(file_name.c_str(), "wb");
#endif  // _WIN32
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
