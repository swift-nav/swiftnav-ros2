#include <data_sources/sbp_data_sources.h>

std::shared_ptr<SbpFileDataSource> dataSourceFactory(
    const std::string& sbp_file_path) {
  return std::make_shared<SbpFileDataSource>(sbp_file_path);
}

std::shared_ptr<SbpSerialDataSource> dataSourceFactory(
    const std::string& device_name, const std::string& connection_str,
    const uint32_t timeout, const LoggerPtr& logger) {
  return std::make_shared<SbpSerialDataSource>(device_name, connection_str,
                                               logger, timeout);
}

std::shared_ptr<SbpTCPDataSource> dataSourceFactory(const std::string& host_ip,
                                                    const uint16_t host_port,
                                                    const uint32_t timeout,
                                                    const LoggerPtr& logger) {
  return std::make_shared<SbpTCPDataSource>(host_ip, host_port, logger,
                                            timeout);
}
