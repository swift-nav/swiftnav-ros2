#pragma once

#include <libsbp/cpp/state.h>
#include <fstream>
#include <string>

class SbpFileReader : public sbp::IReader {
 public:
  explicit SbpFileReader(const std::string &file_path);
  ~SbpFileReader();

  bool is_open() const { return file_stream_.is_open(); }
  bool eof() const;

  s32 read(u8 *buffer, u32 buffer_length) override;

 private:
  std::ifstream file_stream_;
};
