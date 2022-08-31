#include <data_sources/sbp_data_sources.h>

std::shared_ptr<SbpFileDataSource> dataSourceFactory(
    const std::string& sbp_file_path, const LoggerPtr& logger) {
  return std::make_shared<SbpFileDataSource>(sbp_file_path, logger);
}

std::shared_ptr<SbpSerialDataSource> dataSourceFactory(
    const std::string& device_name, const std::string& connection_str,
    const uint32_t read_timeout, const uint32_t write_timeout,
    const LoggerPtr& logger) {
  auto serial_port = std::make_unique<SerialPort>(
      device_name, connection_str, read_timeout, write_timeout, logger);
  return std::make_shared<SbpSerialDataSource>(logger, serial_port);
}

std::shared_ptr<SbpTCPDataSource> dataSourceFactory(const std::string& host_ip,
                                                    const uint16_t host_port,
                                                    const uint32_t timeout,
                                                    const LoggerPtr& logger) {
  return std::make_shared<SbpTCPDataSource>(host_ip, host_port, logger,
                                            timeout);
}
