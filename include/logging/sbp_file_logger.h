#pragma once

#include <libsbp/cpp/state.h>
#include <logging/issue_logger.h>
#include <cstdint>
#include <cstdio>
#include <string>

/**
 * @brief Class that handles the creation of SBP dump files
 */
class SbpFileLogger : public sbp::IWriter {
 public:
  SbpFileLogger() = delete;

  /**
   * @brief Construct a new Sbp File Logger object
   *
   * @param file_path Path where to create SBP dumps
   * @param logger Message logging facility
   */
  SbpFileLogger(const std::string& file_path, const LoggerPtr& logger);

  /**
   * @brief Destroy the Sbp File Logger object
   */
  ~SbpFileLogger();

  /**
   * @brief Method to insert a new SBP message into the file
   *
   * @param msg_type Type of SBP message
   * @param msg SBP message
   */
  void insert(const sbp_msg_type_t msg_type, const sbp_msg_t& msg);

  /**
   * @brief Overriden method to write data
   *
   * @param buffer Buffer containing the data to write
   * @param buffer_length Number of bytes to write
   * @return Number of bytes written. -1 in case of error
   */
  s32 write(const u8* buffer, u32 buffer_length) override;

 private:
  sbp::State state_; /** @brief SBP State object */
  FILE* file_;       /** @brief FILE where to write the data */
  LoggerPtr logger_; /** @brief Message logging facility */
};