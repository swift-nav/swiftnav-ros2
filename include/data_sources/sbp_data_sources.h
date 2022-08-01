#pragma once

#include <data_sources/sbp_file_datasource.h>
#include <data_sources/sbp_serial_datasource.h>
#include <data_sources/sbp_tcp_datasource.h>

#include <memory>

enum DataSources {
  INVALID_DATA_SOURCE = 0U,
  FILE_DATA_SOURCE,
  SERIAL_DATA_SOURCE,
  TCP_DATA_SOURCE,
  MAX_DATA_SOURCE
};

std::shared_ptr<SbpFileDataSource> dataSourceFactory(
    const std::string& sbp_file_path, const LoggerPtr& logger);
std::shared_ptr<SbpSerialDataSource> dataSourceFactory(
    const std::string& device_name, const std::string& connection_str,
    const uint32_t timeout, const LoggerPtr& logger);
std::shared_ptr<SbpTCPDataSource> dataSourceFactory(const std::string& host_ip,
                                                    const uint16_t host_port,
                                                    const uint32_t timeout,
                                                    const LoggerPtr& logger);