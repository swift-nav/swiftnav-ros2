#pragma once

#include <libsbp/cpp/state.h>
#include <logging/issue_logger.h>
#include <fstream>
#include <string>

class SbpFileDataSource : public sbp::IReader {
 public:
  SbpFileDataSource() = delete;

  /**
   * @brief Construct a new Sbp File Data Source object
   *
   * @param file_path Path to the SBP file to use
   * @param logger Logging facility
   */
  SbpFileDataSource(const std::string &file_path, const LoggerPtr &logger);

  /**
   * @brief Destroy the Sbp File Data Source object
   */
  ~SbpFileDataSource();

  /**
   * @brief Methor to determine if the file is open
   *
   * @return true File opened
   * @return false File closed
   */
  bool is_open() const { return file_stream_.is_open(); }

  /**
   * @brief Method to determine if the reading operation has reached the EOF
   *
   * @return true EOF reached
   * @return false EOF not reached
   */
  bool eof() const;

  /**
   * @brief Overriden method to read from the file
   *
   * @param buffer Buffer where to put the data readed
   * @param buffer_length Number of bytes to read
   * @return Number of bytes actually readed
   */
  s32 read(u8 *buffer, u32 buffer_length) override;

 private:
  std::ifstream file_stream_; /** @brief File stream representing the file */
  LoggerPtr logger_;          /** @brief Logging facility object */
};
