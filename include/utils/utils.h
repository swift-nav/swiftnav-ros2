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

#include <cstdint>
#include <ctime>

namespace TimeUtils {
/**
 * @brief Converts seconds to milliseconds
 *
 * @param seconds Number of seconds to convert
 * @return Number of milliseconds
 */
uint64_t secondsToMilliseconds(const uint64_t seconds);

/**
 * @brief Converts seconds to nanoseconds
 *
 * @param seconds Number of seconds to convert
 * @return Number of nanoseconds
 */
uint64_t secondsToNanoseconds(const uint64_t seconds);

/**
 * @brief Converts datetime information (year, month, day, hour, min, sec) to
 * Linux time
 *
 * @param utc struct containing the datetime data.
 * @return Linux time in seconds
 */
time_t utcToLinuxTime(const struct tm& utc);

}  // namespace TimeUtils

namespace Covariance {
/**
 * @brief Computes estimated horizonal error from covariance matrix
 *
 * @param cov_n_n Estimated variance of northing [m^2]
 * @param cov_n_e Covariance of northing and easting [m^2]
 * @param cov_e_e Estimated variance of easting [m^2]
 * @return double
 */
double cov2ehe(const double cov_n_n, const double cov_n_e,
               const double cov_e_e);

/**
 * @brief Computes estimated horizonal direction error from covariance matrix
 *
 * @return double
 */
double cov2ehde( const double n, const double e, const double cov_n_n, const double cov_e_e );
}  // namespace Covariance


