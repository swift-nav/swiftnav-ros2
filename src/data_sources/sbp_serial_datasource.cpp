#include <data_sources/sbp_serial_datasource.h>
#include <sstream>
#include <vector>

/**
 * @brief Auxiliary class to split the connection string
 */
class SerialParameterSplitter {
 public:
  uint32_t speed{0U},    /** @brief Baud rate */
      data_bits{0U},     /** @brief Number of data bits */
      stop_bits{0U};     /** @brief Number of stop bits */
  char parity{'-'},      /** @brief Parity control type */
      flow_control{'-'}; /** @brief Flow control type */

 public:
  /**
   * @brief Construct a new Serial Parameter Splitter object
   *
   * @param str String to decompose inj tokens
   * @param logger Logger facility to use
   */
  SerialParameterSplitter(const std::string& str,
                          const LoggerPtr& logger) noexcept {
    split(str);
    ASSERT_COND(token_list_.size() == 5, logger, "Malformed string");
    setValues();
  }

  /**
   * @brief Method used to determine if the values in the object are valid
   *
   * @return true The values are valid
   * @return false The values are not valid
   */
  bool isValid() const noexcept {
    // Test speed
    switch (speed) {
      case 1200:
      case 2400:
      case 4800:
      case 9600:
      case 19200:
      case 38400:
      case 57600:
      case 115200:
      case 230400:
      case 460800:
      case 921600:
      break;

    default:
      return false;
      break;
    }

    // Test data bits
    if (data_bits < 7 || data_bits > 8) return false;

    // Test stop bits
    if (stop_bits < 1 || stop_bits > 2) return false;

    // Test parity
    switch (parity) {
      case 'N':
      case 'E':
      case 'O':
      case 'M':
      case 'S':
        break;

      default:
        return false;
        break;
    }

    // Test flow control
    switch (flow_control) {
      case 'N':
      case 'X':
      case 'R':
      case 'D':
        break;

      default:
        return false;
        break;
    }

    return true;
  }

 private:
  /**
   * @brief Method used to split the string into a vector of tokens
   *
   * @param str String composed by tokens
   */
  void split(const std::string& str) noexcept {
    try {
      std::stringstream ss(str);
      std::string token;

      token_list_.clear();
      while (std::getline(ss, token, '|')) {
        token_list_.emplace_back(token);
      }
    } catch (...) {
    }
  }

  /**
   * @brief Set the Values from the tokens to the corresponding variables
   */
  void setValues() noexcept {
    try {
      speed = std::stoul(token_list_[0]);
      parity = std::toupper(token_list_[1][0]);
      data_bits = std::stoul(token_list_[2]);
      stop_bits = std::stoul(token_list_[3]);
      flow_control = std::toupper(token_list_[4][0]);
      token_list_.clear();
    } catch (...) {
    }
  }

  std::vector<std::string> token_list_;
};

SbpSerialDataSource::SbpSerialDataSource(const std::string& port_name,
                                         const std::string& connection_string,
                                         const LoggerPtr& logger,
                                         const uint32_t read_timeout) noexcept
    : logger_(logger), read_timeout_(read_timeout) {
  ASSERT_COND(!port_name.empty(), logger_, "The port name should be specified");

  SerialParameterSplitter params(connection_string, logger_);
  ASSERT_COND(params.isValid(), logger_,
              "Invalid data in connection string: " << connection_string);

  sp_return result = sp_get_port_by_name(port_name.c_str(), &port_);
  ASSERT_COND(result == SP_OK, logger_, "No serial port named " << port_name);

  result = sp_open(port_, SP_MODE_READ);
  ASSERT_COND(
      result == SP_OK, logger_,
      "Cannot open port : " << sp_get_port_name(port_) << " error: " << result);
  const std::string error = setPortSettings(params);
  ASSERT_COND(error.empty(), logger_, error);
  LOG_INFO(logger_, "Port " << port_name << " opened with:\nBaud rate: "
                            << params.speed << "\nParity: " << params.parity
                            << "\nData bits: " << params.data_bits
                            << "\nStop bits: " << params.stop_bits
                            << "\nFlow control: " << params.flow_control);
}

SbpSerialDataSource::SbpSerialDataSource(SbpSerialDataSource&& rhs) noexcept {
  port_ = rhs.port_;
  rhs.port_ = nullptr;
  logger_ = rhs.logger_;
  rhs.logger_.reset();
  read_timeout_ = rhs.read_timeout_;
}

SbpSerialDataSource::~SbpSerialDataSource() { closePort(); }

s32 SbpSerialDataSource::read(u8* buffer, u32 buffer_length) {
  ASSERT_COND(port_, logger_,
              "Called read in an uninitialized SbpSerialDataSource");
  ASSERT_COND(buffer, logger_,
              "Called SbpSerialDataSource::read with a NULL buffer");

  const auto result =
      sp_blocking_read(port_, buffer, buffer_length, read_timeout_);
  if (result < 0) {
    LOG_ERROR(logger_,
              "Error (" << result << ") while reading the serial port");
    return -1;
  }

  return result;
}

bool SbpSerialDataSource::isValid() const noexcept {
  return port_ ? true : false;
}

void SbpSerialDataSource::closePort() noexcept {
  if (port_) {
    std::string port_name(sp_get_port_name(port_));
    if (sp_close(port_) != SP_OK) {
      LOG_ERROR(logger_, "Could not close " << port_name);
    } else {
      LOG_INFO(logger_, port_name << " closed");
    }

    sp_free_port(port_);
    port_ = nullptr;
  }
}

std::string SbpSerialDataSource::setPortSettings(
    const SerialParameterSplitter& params) noexcept {
  sp_return result;

  sp_flowcontrol flow_control = SP_FLOWCONTROL_NONE;
  switch (params.flow_control) {
    case 'N':
      flow_control = SP_FLOWCONTROL_NONE;
      break;

    case 'X':
      flow_control = SP_FLOWCONTROL_XONXOFF;
      break;

    case 'R':
      flow_control = SP_FLOWCONTROL_RTSCTS;
      break;

    case 'D':
      flow_control = SP_FLOWCONTROL_DTRDSR;
      break;
  }

  result = sp_set_flowcontrol(port_, flow_control);
  if (result != SP_OK)
    return (std::string("Cannot set flow control: ") + std::to_string(result));

  result = sp_set_bits(port_, params.data_bits);
  if (result != SP_OK)
    return (std::string("Cannot set data bits: ") + std::to_string(result));

  sp_parity parity = SP_PARITY_INVALID;
  switch (params.parity) {
    case 'N':
      parity = SP_PARITY_NONE;
      break;

    case 'O':
      parity = SP_PARITY_ODD;
      break;

    case 'E':
      parity = SP_PARITY_EVEN;
      break;

    case 'M':
      parity = SP_PARITY_MARK;
      break;

    case 'S':
      parity = SP_PARITY_SPACE;
      break;
  }

  result = sp_set_parity(port_, parity);
  if (result != SP_OK)
    return (std::string("Cannot set parity: ") + std::to_string(result));

  result = sp_set_stopbits(port_, params.stop_bits);
  if (result != SP_OK)
    return (std::string("Cannot set stop bits: ") + std::to_string(result));

  result = sp_set_baudrate(port_, params.speed);
  if (result != SP_OK)
    return (std::string("Cannot set baud rate: ") + std::to_string(result));

  return {};
}
