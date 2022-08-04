#pragma once

#include <data_sources/sbp_file_datasource.h>
#include <data_sources/sbp_serial_datasource.h>
#include <data_sources/sbp_tcp_datasource.h>

#include <memory>

/**
 * @brief Enumeration that lists every type of data source available
 */
enum DataSources {
  INVALID_DATA_SOURCE = 0U,
  FILE_DATA_SOURCE,
  SERIAL_DATA_SOURCE,
  TCP_DATA_SOURCE,
  MAX_DATA_SOURCE
};

/**
 * @brief Function that creates an SBP File data source
 *
 * @param sbp_file_path Path to the SBP file
 * @param logger Logging facility to use
 * @return A shared pointer to an SbpSourceFile object
 */
std::shared_ptr<SbpFileDataSource> dataSourceFactory(
    const std::string& sbp_file_path, const LoggerPtr& logger);

/**
 * @brief Function that creates an SBP Serial data source
 *
 * @param device_name Serial port name (depending on the OS format)
 * @param connection_str String used to configure the port
 * @param timeout Max time (in ms) to wait for data to come.
 * @param logger Logging facility to use
 * @return A shared pointer to an SbpSerialDataSource object
 */
std::shared_ptr<SbpSerialDataSource> dataSourceFactory(
    const std::string& device_name, const std::string& connection_str,
    const uint32_t timeout, const LoggerPtr& logger);

/**
 * @brief Function that creates an SBP TCP data source
 *
 * @param host_ip IP of the host to connect to
 * @param host_port Port of the host to connect
 * @param timeout Max time (in ms) to wait for data to come.
 * @param logger Logging facility to use
 * @return A shared pointer to an SbpTCPDataSource object
 */
std::shared_ptr<SbpTCPDataSource> dataSourceFactory(const std::string& host_ip,
                                                    const uint16_t host_port,
                                                    const uint32_t timeout,
                                                    const LoggerPtr& logger);