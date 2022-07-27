#pragma once

#include <libsbp/cpp/state.h>
#include <fstream>
#include <string>

class SbpFileDataSource : public sbp::IReader {
 public:
  explicit SbpFileDataSource(const std::string &file_path);
  ~SbpFileDataSource();

  bool is_open() const { return file_stream_.is_open(); }
  bool eof() const;

  s32 read(u8 *buffer, u32 buffer_length) override;

 private:
  std::ifstream file_stream_;
};
