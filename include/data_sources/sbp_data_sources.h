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

#pragma once

#include <data_sources/sbp_file_datasource.h>
#include <data_sources/sbp_serial_datasource.h>
#include <data_sources/sbp_tcp_datasource.h>
#include <utils/config.h>

#include <memory>

/**
 * @brief Enumeration that lists every type of data source available
 */
enum DataSources {
  INVALID_DATA_SOURCE = 0U,
  TCP_DATA_SOURCE,
  SERIAL_DATA_SOURCE,
  FILE_DATA_SOURCE,
  MAX_DATA_SOURCE
};

/**
 * @brief Function that creates an SBP data source
 *
 * @param config Node configuration
 * @param logger Logging facility to use
 * @return DataSource corresponding to the type of interface set in the
 * configuration
 */
std::shared_ptr<SbpDataSource> dataSourceFactory(
    std::shared_ptr<Config>& config, const LoggerPtr& logger);
