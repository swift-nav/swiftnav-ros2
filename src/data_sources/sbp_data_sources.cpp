/*
 * Copyright (C) 2015-2023 Swift Navigation Inc.
 * Contact: https://support.swiftnav.com
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <data_sources/sbp_data_sources.h>
#include <logging/ros_logger.h>

std::shared_ptr<SbpDataSource> dataSourceFactory(
    std::shared_ptr<Config>& config, const LoggerPtr& logger) {
  const int32_t interface = config->getInterface();

  LOG_INFO(logger, "Interface type: %d", interface);
  switch (interface) {
    case FILE_DATA_SOURCE: {
      return std::make_shared<SbpFileDataSource>(config->getFile(), logger);
    } break;

    case SERIAL_DATA_SOURCE: {
      auto serial_port = std::make_unique<SerialPort>(
          config->getDevice(), config->getConnectionString(),
          config->getReadTimeout(), config->getWriteTimeout(), logger);
      return std::make_shared<SbpSerialDataSource>(logger,
                                                   std::move(serial_port));

    } break;

    case TCP_DATA_SOURCE: {
      auto tcp = std::make_unique<TCP>(config->getIP(), config->getPort(),
                                       logger, config->getReadTimeout(),
                                       config->getWriteTimeout());
      return std::make_shared<SbpTCPDataSource>(logger, std::move(tcp));
    } break;

    default:
      LOG_FATAL(logger, "Could not open interface: %d",
                interface);
      return {};
      break;
  }
}
